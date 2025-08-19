/* Copyright (C) 2025 Paul Winwood G8KIG - All Rights Reserved. */

#ifndef PSK_INTERFACE_H
#define PSK_INTERFACE_H

/* Request that the system time is updated with the NTP time */
void requestTimeSync(void);
/* Update the system time from the NTP time */
void updateTime(void);
/* Add an PSK reporter sender record */
bool addSenderRecord(const char *callsign, const char *gridSquare, const char *software);
/* Add an PSK reporter receiver record */
bool addReceivedRecord(const char *callsign, uint32_t frequency, uint8_t snr);
/* Call the PSK reporter API with the above records */
bool sendRequest(void);

#endif 