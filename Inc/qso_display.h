/* APIs for displaying QSO related information */
#ifndef QSO_DISPLAY_H_
#define QSO_DISPLAY_H_

#define MAX_LINE_LEN 22 // 21 characters + null

// Forward declaraion
typedef struct Decode Decode;

void display_messages(Decode new_decoded[], int num_decoded);
void display_queued_message(const char *);
void display_txing_message(const char *);
void display_qso_state(const char lines[][MAX_LINE_LEN]);
// Return a char buffer to caller for filling in the log string
// The buffer can hold up to 18 chars + null (MAX_LINE_LEN - 3)
char *add_worked_qso();
// Return false when last page has been displayed
bool display_worked_qsos();

#endif