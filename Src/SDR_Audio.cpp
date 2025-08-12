/*
 * SDR_Audio.c
 *
 *  Created on: May 10, 2016
 *      Author: user
 */

#include <stdlib.h>
#include "SDR_Audio.h"
#include "stm32746g_discovery_audio.h"
#include "wm8994.h"
#include "stm32746g_discovery_lcd.h"
#include "arm_math.h"
#include "Process_DSP.h"
#include "main.h" // for decode_flag
#include "button.h"
#ifdef __cplusplus
extern "C"
{
#endif
#include "Sine_table.h"
#ifdef __cplusplus
}
#endif

q15_t FIR_I_In[BUFFERSIZE / 4];
q15_t FIR_Q_In[BUFFERSIZE / 4];
q15_t FIR_I_Out[BUFFERSIZE / 4];
q15_t FIR_Q_Out[BUFFERSIZE / 4];

q15_t FT8_Data[2048 / 2];

uint16_t buff_offset;

int DSP_Flag;
int Xmit_Mode = 0;
int xmit_flag, ft8_xmit_counter, ft8_xmit_delay;

#define PI2 6.2831853071795864765
#define KCONV 10430.37835 // 		4096*16/PI2

static const double LO_Freq = 10000;
static const int Sample_Frequency = 32000;

static q15_t USB_Out[BUFFERSIZE / 4];
static q15_t LSB_Out[BUFFERSIZE / 4];
static q15_t in_buff[BUFFERSIZE];
static q15_t out_buff[BUFFERSIZE];

void start_audio_I2C(void)
{
	AUDIO_IO_Init();
}

void clear_Output_Buffers(void)
{
	memset(out_buff, 0, BUFFERSIZE * sizeof(q15_t));
}

void start_codec(void)
{
	uint32_t rc = wm8994_Reset(AUDIO_I2C_ADDRESS);
	HAL_Delay(100);
	rc = wm8994_Init(AUDIO_I2C_ADDRESS,
					 INPUT_DEVICE_INPUT_LINE_1 | OUTPUT_DEVICE_BOTH, 70,
					 Sample_Frequency / 2);
	(void)rc;
	HAL_Delay(100);
}

void start_duplex(void)
{
	// note, somehow there is a sneak path for setting the codec frequency see wmcodec for reference

	clear_Output_Buffers();
	BSP_AUDIO_IN_OUT_Init(INPUT_DEVICE_INPUT_LINE_1, OUTPUT_DEVICE_BOTH, 70, Sample_Frequency);
	BSP_AUDIO_IN_Record((uint16_t *)&in_buff, BUFFERSIZE);
	BSP_AUDIO_OUT_Play((uint16_t *)&out_buff, 2 * BUFFERSIZE);
}

void transfer_buffers(void)
{
	for (int j = 0; j < BUFFERSIZE / 4; j++)
	{
		int index = buff_offset + j * 2;
		out_buff[index] = in_buff[index];
		out_buff[index + 1] = in_buff[index + 1];
	}
}

void BSP_AUDIO_IN_TransferComplete_CallBack(void)
{
	buff_offset = BUFFERSIZE / 2;
	DSP_Flag = 1;
}

void BSP_AUDIO_IN_HalfTransfer_CallBack(void)
{
	buff_offset = 0;
	DSP_Flag = 1;
}

void BSP_AUDIO_OUT_TransferComplete_CallBack(void)
{
}

void BSP_AUDIO_OUT_HalfTransfer_CallBack(void)
{
}

static const float RMSConstant = 1.0 / 65485.0;
static const long NCOphzinc = (long)(KCONV * (PI2 * LO_Freq / Sample_Frequency));

int frame_counter = 0;

void I2S2_RX_ProcessBuffer(uint16_t offset)
{
	static long NCO_phz = 0;

	for (int i = 0; i < BUFFERSIZE / 4; i++)
	{
		NCO_phz += NCOphzinc;
		float temp = (q15_t)Sine_table[(NCO_phz >> 4) & 0xFFF] * RMSConstant;
		int index = i * 2 + offset;
		FIR_I_In[i] = (q15_t)(temp * in_buff[index]);
		FIR_Q_In[i] = (q15_t)(temp * in_buff[index + 1]);
	}

	Process_FIR_I_32K();
	Process_FIR_Q_32K();

	// Decimation
	uint8_t decimator = 0;
	uint16_t ft8_pos = frame_counter * 256;
	for (int i = 0; i < BUFFERSIZE / 4; i++)
	{
		USB_Out[i] = FIR_I_Out[i] - FIR_Q_Out[i];
		LSB_Out[i] = FIR_I_Out[i] + FIR_Q_Out[i];

		if (++decimator == 5)
		{
			decimator = 0;
			FT8_Data[ft8_pos++] = USB_Out[i];
		}

		int index = i * 2 + offset;
		out_buff[index] = USB_Out[i];
		out_buff[index + 1] = LSB_Out[i];
	}

	if (++frame_counter == 4)
	{
		process_FT8_FFT();
		frame_counter = 0;
	}
}
