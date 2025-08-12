/*
 * traffic_manager.h
 *
 *  Created on: Feb 29, 2020
 *      Author: user
 */

#ifndef TRAFFIC_MANAGER_H_
#define TRAFFIC_MANAGER_H_

extern int Beacon_State;

void setup_to_transmit_on_next_DSP_Flag(void);
void terminate_QSO(void);

void set_Xmit_Freq(void);
void tune_Off_sequence(void);
void tune_Off_sequence(void);
void tune_On_sequence(void);
void ft8_receive_sequence(void);
void ft8_transmit_sequence(void);
void set_FT8_Tone(uint8_t ft8_tone);
void set_Rcvr_Freq(void);

#endif /* TRAFFIC_MANAGER_H_ */
