/* ====================================================
 *  Public API implementation
 * ==================================================== */

#include <stdbool.h>
#include "decode_ft8.h"
#include "qso_display.h"
#include "main.h"

// Configurable Knobs
extern int max_tx_retries;

void autoseq_init();

void autoseq_start_cq(void);

/* === Called for selected decode (manual response) === */
void autoseq_on_touch(const Decode *msg);

/* === Called for the entire decode array === */
void autoseq_on_decodes(const Decode *messages, int num_decoded);

/* === Provide the message we should transmit === */
bool autoseq_get_next_tx(char out_text[MAX_MSG_LEN]);

/* === Populate the strings for displaying the QSO states  === */
void autoseq_get_qso_states(char lines[][MAX_LINE_LEN]);

/* === Populate the strings for logging the ctx queue === */
void autoseq_log_ctx_queue(char lines[][53]);

/* === Slot timer / time‑out manager === */
void autoseq_tick(void);
