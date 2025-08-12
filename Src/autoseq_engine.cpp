/*
 * autoseq_engine.c – FT8 CQ/QSO auto‑sequencing engine (revised)
 *
 * This version consumes the **Decode** structure defined in decode_ft8.h, the
 * native output of *ft8_decode()* in the DX‑FT8 transceiver firmware, so no
 * wrapper conversion is required.  All logic remains the same as the QEX/WSJT‑X
 * state diagram (Tx1…Tx6) presented earlier.
 *
 */

#include <assert.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "autoseq_engine.h"
#include "qso_display.h" // For adding worked qso entry
#include "gen_ft8.h"     // For accessing CQ_Mode_Index and saving Target_*
#include "ADIF.h"        // For write_ADIF_Log()

// HAL
#ifdef HOST_HAL_MOCK
#include "host_mocks.h"
#else
#include "button.h" // For BandIndex
#include "DS3231.h" // For log_rtc_time_string
#endif

extern int Beacon_On; // TODO get rid of manual extern
extern int Skip_Tx1;  // TODO get rid of manual extern

/***** Configurable knobs *****/
int max_tx_retries;

/* For DECENDING order. Returns −1 if (a) > (b), 0 if equal, +1 if (a) < (b) */
#define CMP(a, b) (((a) < (b)) - ((a) > (b)))

/***** Identifiers for the six canonical FT8 messages *****/
typedef enum
{
    TX_UNDEF = 0,
    TX1, /*  <DXCALL> <MYCALL> <GRID>            */
    TX2, /*  <DXCALL> <MYCALL> ##                */
    TX3, /*  <DXCALL> <MYCALL> R##               */
    TX4, /*  <DXCALL> <MYCALL> RR73 (or RRR)     */
    TX5, /*  <DXCALL> <MYCALL> 73                */
    TX6  /*  CQ <MYCALL> <GRID>                  */
} tx_msg_t;

/***** High‑level auto‑sequencer states *****/
// Order that compare() can rely on
typedef enum
{
    AS_CALLING = 0,
    AS_REPLYING,
    AS_REPORT,
    AS_ROGER_REPORT,
    AS_ROGERS,
    AS_SIGNOFF,
    AS_IDLE,
} autoseq_state_t;

/***** Control‑block *****/
typedef struct
{
    autoseq_state_t state;
    tx_msg_t next_tx;
    tx_msg_t rcvd_msg_type;

    char dxcall[14];
    char dxgrid[7];

    int snr_tx; /* SNR we report to DX (‑dB) */
    int snr_rx; /* TODO: is this needed? SNR we received from DX (‑dB) */
    int retry_counter;
    int retry_limit;
    bool logged; /* true => QSO logged */
} ctx_t;
// For logging ctx in RxTxLog
static const char *queue_log_format =
    "Q s:%1ut:%1ur:%1u:%-13.13s:%-6.6s:%+3d:%+3d:%1u:%1u:%1u";

static ctx_t ctx_queue[MAX_QUEUE_SIZE];
static int queue_size = 0;

/*************** Forward declarations ****************/
static void set_state(ctx_t *ctx, autoseq_state_t s, tx_msg_t first_tx, int limit);
static void format_tx_text(tx_msg_t id, char out[MAX_MSG_LEN]);
static void parse_rcvd_msg(ctx_t *ctx, const Decode *msg);
static void on_decode(const Decode *msg);
static int compare(const void *a, const void *b);
static void pop();
static ctx_t *append();
// Internal helper called by autoseq_on_touch() and on_decode()
static bool generate_response(ctx_t *ctx, const Decode *msg, bool override);
// Ensure the queue is sorted and IDLE entries are popped
static void sort_and_clean();
static void write_worked_qso();
/******************************************************/

/* ====================================================
 *  Public API implementation
 * ==================================================== */

void autoseq_init()
{
    if (queue_size > 0)
    {
        pop();
    }
}

void autoseq_start_cq(void)
{
    if (queue_size > 0 && queue_size < MAX_QUEUE_SIZE)
    {
        // Check if CQ is already at the bottom
        if (ctx_queue[queue_size - 1].state == AS_CALLING)
        {
            return;
        }
    }
    ctx_t *ctx = append();
    strcpy(ctx->dxcall, "CQ");
    set_state(ctx, AS_CALLING, TX6, 0);
    // No need to sort, CQ is always at the bottom
}

/* === Called for selected decode (manual response) === */
// TODO dedup
void autoseq_on_touch(const Decode *msg)
{
    if (!msg)
    {
        return;
    }

    // If queue is full, remove the last one
    // Note that with MAX_QUEUE_SIZE == 1, it replaces the current TX,
    // which effectively disables the smart scheduler.
    if (queue_size == MAX_QUEUE_SIZE)
    {
        --queue_size;
    }

    ctx_t *ctx = append();
    // Check if it's addressed me
    if (strncmp(msg->call_to, Station_Call, sizeof(Station_Call)) == 0)
    {
        generate_response(ctx, msg, true);
        sort_and_clean();
        return;
    }
    // Treat it as calling CQ
    strncpy(ctx->dxcall, msg->call_from, sizeof(ctx->dxcall) - 1);
    if (msg->sequence == Seq_Locator)
    {
        strncpy(ctx->dxgrid, msg->locator, sizeof(ctx->dxgrid) - 1);
    }
    ctx->snr_tx = msg->snr;
    set_state(ctx, Skip_Tx1 ? AS_REPORT : AS_REPLYING,
              Skip_Tx1 ? TX2 : TX1, max_tx_retries);
    sort_and_clean();
}

void autoseq_on_decodes(const Decode *messages, int num_decoded)
{
    for (int i = 0; i < num_decoded; i++)
    {
        on_decode(&messages[i]);
    }
    sort_and_clean();
}

/* === Provide the message we should transmit this slot (if any) === */
bool autoseq_get_next_tx(char out_text[MAX_MSG_LEN])
{
    if (!out_text)
    {
        return false;
    }

    out_text[0] = '\0';

    if (queue_size == 0)
    {
        return false;
    }

    format_tx_text(ctx_queue[0].next_tx, out_text);

    return true;
}

/* === Populate the strings for displaying the QSO states  === */
void autoseq_get_qso_states(char lines[][MAX_LINE_LEN])
{
    if (!lines)
    {
        return;
    }

    for (int i = 0; i < MAX_QUEUE_SIZE; i++)
    {
        char *out_text = lines[i];
        out_text[0] = '\0';

        if (i >= queue_size)
        {
            continue;
        }

        const ctx_t *ctx = &ctx_queue[i];
        // IDLE state is treated as no active QSO
        if (ctx->state == AS_IDLE)
        {
            return;
        }

        const char states[][5] = {
            "CALL", // AS_CALLING
            "RPLY", // AS_REPLYING
            "RPRT", // AS_REPORT
            "RRPT", // AS_ROGER_REPORT
            "RGRS", // AS_ROGERS
            "SOFF", // AS_SIGNOFF
            "",     // AS_IDLE
        };

        snprintf(out_text, MAX_LINE_LEN,
                 "%-8.8s %.4s tried:%1u",
                 ctx->dxcall,
                 states[ctx->state],
                 ctx->retry_counter);
    }
}

/* === Slot timer / time‑out manager === */
void autoseq_tick(void)
{
    if (queue_size == 0)
    {
        return;
    }
    ctx_t *ctx = &ctx_queue[0];
    switch (ctx->state)
    {
    case AS_REPLYING:
        if (ctx->retry_counter < ctx->retry_limit)
        {
            ctx->next_tx = TX1;
            ctx->retry_counter++;
        }
        else
        {
            ctx->state = AS_IDLE;
            ctx->next_tx = TX_UNDEF;
        }
        break;
    case AS_REPORT:
        if (ctx->retry_counter < ctx->retry_limit)
        {
            ctx->next_tx = TX2;
            ctx->retry_counter++;
        }
        else
        {
            ctx->state = AS_IDLE;
            ctx->next_tx = TX_UNDEF;
        }
        break;
    case AS_ROGER_REPORT:
        if (ctx->retry_counter < ctx->retry_limit)
        {
            ctx->next_tx = TX3;
            ctx->retry_counter++;
        }
        else
        {
            ctx->state = AS_IDLE;
            ctx->next_tx = TX_UNDEF;
        }
        break;
    case AS_ROGERS:
        if (ctx->retry_counter < ctx->retry_limit)
        {
            ctx->next_tx = TX4;
            ctx->retry_counter++;
        }
        else
        {
            ctx->state = AS_IDLE;
            ctx->next_tx = TX_UNDEF;
        }
        break;
    case AS_CALLING: // CQ iscontrolled by Beacon_On, so it's only once
    case AS_SIGNOFF:
        ctx->state = AS_IDLE;
        ctx->next_tx = TX_UNDEF;
        break;
    default:
        break;
    }

    if (ctx->state == AS_IDLE)
    {
        pop();
    }
}

/* === Populate the strings for logging the ctx queue === */
void autoseq_log_ctx_queue(char lines[][53])
{
    if (!lines)
    {
        return;
    }

    for (int i = 0; i < MAX_QUEUE_SIZE; i++)
    {
        char *out_text = lines[i];
        out_text[0] = '\0';

        if (i >= queue_size)
        {
            continue;
        }
        snprintf(out_text, 53, queue_log_format,
                 ctx_queue[i].state,
                 ctx_queue[i].next_tx,
                 ctx_queue[i].rcvd_msg_type,
                 ctx_queue[i].dxcall,
                 ctx_queue[i].dxgrid,
                 ctx_queue[i].snr_tx,
                 ctx_queue[i].snr_rx,
                 ctx_queue[i].retry_counter,
                 ctx_queue[i].retry_limit,
                 ctx_queue[i].logged);
    }
}

/* ================================================================
 *                Internal helpers
 * ================================================================ */

static void set_state(ctx_t *ctx, autoseq_state_t s, tx_msg_t first_tx, int limit)
{
    ctx->state = s;
    ctx->next_tx = first_tx;
    ctx->retry_counter = 0;
    ctx->retry_limit = limit;
}

static void format_tx_text(tx_msg_t id, char out[MAX_MSG_LEN])
{
    assert(out);
    assert(queue_size > 0);
    ctx_t *ctx = &ctx_queue[0];
    assert(ctx->state != AS_IDLE);

    // Populating the global variables about the current QSO
    strncpy(Target_Call, ctx->dxcall, sizeof(Target_Call) - 1);
    strncpy(Target_Locator, ctx->dxgrid, sizeof(Target_Locator) - 1);
    Station_RSL = ctx->snr_rx;
    Target_RSL = ctx->snr_tx;

    const char *cq_str = "CQ";

    switch (id)
    {
    case TX1:
        snprintf(out, MAX_MSG_LEN, "%s %s %s", Target_Call, Station_Call, Station_Locator);
        break;
    case TX2:
        snprintf(out, MAX_MSG_LEN, "%s %s %+d", Target_Call, Station_Call, Target_RSL);
        break;
    case TX3:
        snprintf(out, MAX_MSG_LEN, "%s %s R%+d", Target_Call, Station_Call, Target_RSL);
        break;
    case TX4:
        snprintf(out, MAX_MSG_LEN, "%s %s RR73", Target_Call, Station_Call);
        if (!ctx->logged)
        {
            write_ADIF_Log();
            write_worked_qso();
            ctx->logged = true;
        }
        break;
    case TX5:
        snprintf(out, MAX_MSG_LEN, "%s %s 73", Target_Call, Station_Call);
        if (!ctx->logged)
        {
            write_ADIF_Log();
            write_worked_qso();
            ctx->logged = true;
        }
        break;
    case TX6:
        if (!free_text)
        {
            switch (CQ_Mode_Index)
            {
            case 1:
                cq_str = "CQ SOTA";
                break;
            case 2:
                cq_str = "CQ POTA";
                break;
            case 3:
                cq_str = "CQ QRP";
                break;
            default:
                break;
            }
            snprintf(out, MAX_MSG_LEN, "%s %s %s", cq_str, Station_Call, Station_Locator);
        }
        else
        {
            switch (Free_Index)
            {
            case 0:
                strncpy(out, Free_Text1, sizeof(Free_Text1) - 1);
                break;
            case 1:
                strncpy(out, Free_Text2, sizeof(Free_Text2) - 1);
                break;
            default:
                break;
            }
        }
        break;
    default:
        break;
    }
}

static void parse_rcvd_msg(ctx_t *ctx, const Decode *msg)
{
    ctx->rcvd_msg_type = TX_UNDEF;
    if (msg->sequence == Seq_Locator)
    {
        ctx->rcvd_msg_type = TX1;
        strncpy(ctx->dxgrid, msg->locator, sizeof(ctx->dxgrid) - 1);
    }
    else
    {
        if (strcmp(msg->locator, "73") == 0)
        {
            ctx->rcvd_msg_type = TX5;
        }
        else if (strcmp(msg->locator, "RR73") == 0)
        {
            ctx->rcvd_msg_type = TX4;
        }
        else if (strcmp(msg->locator, "RRR") == 0)
        {
            ctx->rcvd_msg_type = TX4;
        }
        else if (msg->locator[0] == 'R')
        {
            ctx->rcvd_msg_type = TX3;
        }
        else
        {
            ctx->rcvd_msg_type = TX2;
        }
    }
}

/* === Called for **every** new decode (auto response) === */
/* Return whether TX is needed */
void on_decode(const Decode *msg)
{
    // Not addressed us
    if (strncmp(msg->call_to, Station_Call, sizeof(Station_Call)) != 0)
    {
        return;
    }

    // Check if it matches an existing ctx
    for (int i = 0; i < queue_size; ++i)
    {
        ctx_t *ctx = &ctx_queue[i];
        if (strncmp(msg->call_from, ctx->dxcall, sizeof(ctx->dxcall)) == 0)
        {
            generate_response(ctx, msg, false);
            return;
        }
    }

    // No matching existing ctx found.
    // Check if need to enqueue a new ctx
    if (queue_size == MAX_QUEUE_SIZE)
    {
        return;
    }

    ctx_t *ctx = append();
    generate_response(ctx, msg, true);
}

// Internal helper called by and on_decode()
static bool generate_response(ctx_t *ctx, const Decode *msg, bool override)
{
    parse_rcvd_msg(ctx, msg);

    if (!msg || ctx->rcvd_msg_type == TX_UNDEF)
    {
        return false;
    }

    if (ctx->rcvd_msg_type == TX1 || ctx->rcvd_msg_type == TX2)
    {
        // Update the DX SNR
        ctx->snr_tx = msg->snr;
    }

    if (override)
    {
        strncpy(ctx->dxcall, msg->call_from, sizeof(ctx->dxcall) - 1);
        // Reset own internal state to macth rcve_msg_type
        switch (ctx->rcvd_msg_type)
        {
        case TX1:
            set_state(ctx, AS_CALLING, TX_UNDEF, 0);
            break;
        case TX2:
            set_state(ctx, AS_REPLYING, TX_UNDEF, 0);
            break;
        case TX3:
            set_state(ctx, AS_REPORT, TX_UNDEF, 0);
            break;
        case TX4:
            set_state(ctx, AS_ROGER_REPORT, TX_UNDEF, 0);
            break;
        case TX5:
            set_state(ctx, AS_ROGERS, TX_UNDEF, 0);
        // case TX6 already handled by autoseq_on_touch()
        default:
            break;
        }
    }

    if (ctx->rcvd_msg_type == TX2 || ctx->rcvd_msg_type == TX3)
    {
        ctx->snr_rx = msg->received_snr;
    }

    switch (ctx->state)
    {
    /* ------------------------------------------------ CALLING (we sent CQ) */
    case AS_CALLING:
        switch (ctx->rcvd_msg_type)
        {
        case TX1:
            set_state(ctx, AS_REPORT, TX2, max_tx_retries);
            return true;
        case TX2:
            set_state(ctx, AS_ROGER_REPORT, TX3, max_tx_retries);
            return true;
        case TX3:
            set_state(ctx, AS_ROGERS, TX4, max_tx_retries);
            return true;
        default:
            return false;
        }

    /* ------------------------------------------------ REPLYING (we sent Tx1) */
    case AS_REPLYING:
        switch (ctx->rcvd_msg_type)
        {
        // Since we sent TX1, it doesn't make sense to respond to TX1
        case TX1:
            return false;
        case TX2:
            set_state(ctx, AS_ROGER_REPORT, TX3, max_tx_retries);
            return true;
        case TX3:
            set_state(ctx, AS_ROGERS, TX4, max_tx_retries);
            return true;

        // QSO complete without signal report exchange
        case TX4:
        case TX5:
            set_state(ctx, AS_SIGNOFF, TX5, 0);
            return true;
        default:
            return false;
        }

    /* ------------------------------------------------ REPORT sent, waiting Roger */
    case AS_REPORT:
        switch (ctx->rcvd_msg_type)
        {
        // DX didn't copy our TX2 response, stay in the same state by returning false
        case TX1:
            return false;
        // Since we sent TX2, it doesn't make sense to respond to TX2
        case TX2:
            return false;
        case TX3:
            set_state(ctx, AS_ROGERS, TX4, max_tx_retries);
            return true;
        // QSO complete without signal report exchange
        case TX4:
        case TX5:
            set_state(ctx, AS_SIGNOFF, TX5, 0);
            return true;
        default:
            return false;
        }

    /* ------------------------------------------------ Roger‑Report sent */
    case AS_ROGER_REPORT:
        switch (ctx->rcvd_msg_type)
        {
        // Since we sent TX1, it doesn't make sense to respond to TX1
        // case TX1:
        //     set_state(ctx, AS_REPORT, TX2, MAX_TX_RETRY);
        //     return true;

        // DX didn't copy our TX3 response, stay in the same state by returning false
        // case TX2:
        //     set_state(ctx, AS_ROGER_REPORT, TX3, MAX_TX_RETRY);
        //     return true;

        // Since we sent TX3, it doesn't make sense to respond to TX3
        // case TX3:
        //     set_state(ctx, AS_SIGNOFF, TX4, MAX_TX_RETRY);
        //     return true;

        // QSO complete
        case TX4:
        case TX5: // Be polite, echo back 73
            set_state(ctx, AS_SIGNOFF, TX5, 0);
            return true;
        default:
            return false;
        }

    case AS_ROGERS:
        switch (ctx->rcvd_msg_type)
        {
        // QSO complete
        case TX4:
        case TX5:
            set_state(ctx, AS_IDLE, TX_UNDEF, 0);
            break;
        default:
            break;
        }
        return false;

    // Since 73 is sent only once, this should never be reached
    case AS_SIGNOFF:
        switch (ctx->rcvd_msg_type)
        {
        // DX hasn't received our TX5. Retry
        case TX4:
            break;
        default:
            set_state(ctx, AS_IDLE, TX_UNDEF, 0);
            break;
        }
        return false;

    default:
        break;
    }
    return false;
}

// CQ goes to bottom, IDLE goes to top
static int compare(const void *a, const void *b)
{
    const ctx_t *left = (const ctx_t *)a;
    const ctx_t *right = (const ctx_t *)b;
    // If both are in the same state, lower retry times wins
    if (left->state == right->state)
    {
        return CMP(right->retry_counter, left->retry_counter);
    }
    // Higher state wins
    return CMP(left->state, right->state);
}

static void pop()
{
    assert(queue_size > 0 && queue_size <= MAX_QUEUE_SIZE);
    // Shift up
    for (int i = 0; i < queue_size - 1; i++)
    {
        memcpy(&ctx_queue[i], &ctx_queue[i + 1], sizeof(ctx_t));
    }
    --queue_size;
}

static ctx_t *append()
{
    assert(queue_size < MAX_QUEUE_SIZE);
    ctx_t *ctx = &ctx_queue[queue_size++];
    ctx->dxgrid[0] = '\0';
    ctx->logged = false;
    return ctx;
}

static void sort_and_clean()
{
    qsort(ctx_queue, queue_size, sizeof(ctx_t), compare);
    // Finished QSOs (AS_IDLE) are at the top. Pop them
    while (queue_size != 0 && ctx_queue[0].state == AS_IDLE)
    {
        pop();
    }
}

static void write_worked_qso()
{
    static const char band_strs[][4] = {
        "40", "30", "20", "17", "15", "12", "10"};
    char *buf = add_worked_qso();
    // band, HH:MM, callsign, SNR
    // snprintf(buf, MAX_LINE_LEN, "%.4s %.3s %-7.7s%+02d %+02d",
    int printed = snprintf(buf, MAX_LINE_LEN, "%.4s %.3s %.12s",
                           log_rtc_time_string,
                           band_strs[BandIndex],
                           Target_Call);
    if (printed < 0)
    {
        return;
    }
    char rsl[5]; // space + sign + 2 digits + null = 5
    // Check if RX RSL would fit
    int needed = snprintf(rsl, sizeof(rsl), " %d", Station_RSL);
    if (printed + needed <= MAX_LINE_LEN - 1)
    {
        strncpy(buf + printed, rsl, needed + 1);
        printed += needed;
    }
    else
    {
        return;
    }
    // Check if TX RSL would fit
    needed = snprintf(rsl, sizeof(rsl), " %d", Target_RSL);
    if (printed + needed <= MAX_LINE_LEN - 1)
    {
        strncpy(buf + printed, rsl, needed + 1);
    }
}
