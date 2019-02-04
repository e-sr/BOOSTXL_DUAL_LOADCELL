/* --COPYRIGHT--,BSD
 * Copyright (c) 2018, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
/*
 * ADS1220.c
 *
 */
/******************************************************************************
// ADS1220 Demo C Function Calls  
//            
 ******************************************************************************/
#include "ADS1220.h"

void ADS1220Init(ADS1220_t* ads1220,
                 uint32_t cs,
                 void (*assert_cs)(uint16_t , uint32_t),
                 void (*tx_byte)(uint16_t),
                 uint16_t (*rx_byte)(void))
{
	ads1220->cs = cs;
	ads1220->assert_cs_p = assert_cs;
	ads1220->tx_byte_p = tx_byte;
	ads1220->rx_byte_p = rx_byte;
}
/*
******************************************************************************
 higher level functions
*/
void ADS1220ReadData(ADS1220_t* ads1220)
{
    int32_t data;
   /* assert CS to start transfer */
   ads1220->assert_cs_p(1, ads1220->cs);
   /* send the command byte */
   ads1220->tx_byte_p(ADS1220_CMD_RDATA);
   /* get the conversion result */
   data = ads1220->rx_byte_p();
   data = (data << 8) | ads1220->rx_byte_p();
   data = (data << 8) | ads1220->rx_byte_p();
   /* sign extend data */
   if (data & 0x800000){//if negative
	   data |= 0xff000000;
   } 
	/* de-assert CS */
   	ads1220->raw_data=data;
	ads1220->assert_cs_p(0,ads1220->cs);
   return;
}
void ADS1220ReadRegister(ADS1220_t* ads1220, uint16_t StartAddress, uint16_t NumRegs)
{
    uint16_t i;
	/* assert CS to start transfer */
   ads1220->assert_cs_p(1, ads1220->cs);
   /* send the command byte */
   ads1220->tx_byte_p(ADS1220_CMD_RREG | (((StartAddress << 2) & 0x0c) | ((NumRegs - 1) & 0x03)));
   /* get the register content */
   for (i = StartAddress; i < StartAddress+NumRegs; i++)
   {
	   ads1220->reg_read[i] = ads1220->rx_byte_p();
	}
   	/* de-assert CS */
	ads1220->assert_cs_p(0, ads1220->cs);
	return;
}

void ADS1220WriteRegister(ADS1220_t* ads1220, uint16_t StartAddress, uint16_t NumRegs)
{
    uint16_t i;
	/* assert CS to start transfer */
	ads1220->assert_cs_p(1, ads1220->cs);
	/* send the command byte */
	ads1220->tx_byte_p(ADS1220_CMD_WREG | (((StartAddress << 2) & 0x0c) | ((NumRegs - 1) & 0x03)));
	/* get the register content */
	for (i = StartAddress; i < StartAddress + NumRegs; i++)
	{
		ads1220->tx_byte_p(ads1220->reg_write[i]);
	}
   	/* de-assert CS */
	ads1220->assert_cs_p(0, ads1220->cs);
	return;
}

void ADS1220SendCommand(ADS1220_t* ads1220, uint16_t command)
{
	ads1220->assert_cs_p(1, ads1220->cs);
	ads1220->tx_byte_p(command);
	ads1220->assert_cs_p(0, ads1220->cs);
	return;
}

