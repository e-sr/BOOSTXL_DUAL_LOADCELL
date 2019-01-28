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
 * ADS1220.h
 *
 */
/*******************************************************************************
// ADS1220 Header File 

//******************************************************************************/
#ifndef ADS1220_H_
#define ADS1220_H_
/* Error Return Values */
#define ADS1220_NO_ERROR           0
#define ADS1220_ERROR				-1
/* Command Definitions */
#define ADS1220_CMD_RDATA    	0x10
#define ADS1220_CMD_RREG     	0x20
#define ADS1220_CMD_WREG     	0x40
#define ADS1220_CMD_SYNC    	0x08
#define ADS1220_CMD_SHUTDOWN    0x02
#define ADS1220_CMD_RESET    	0x06
/* ADS1220 Register Definitions */
#define ADS1220_REG_NUMBER      4
#define ADS1220_0_REGISTER   	0x00
#define ADS1220_1_REGISTER     	0x01
#define ADS1220_2_REGISTER     	0x02
#define ADS1220_3_REGISTER    	0x03
/* ADS1220 Register 0 Definition */
/*   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0 
//--------------------------------------------------------------------------------------------
//                     MUX [3:0]                 |             GAIN[2:0]             | PGA_BYPASS
*/
/* Define MUX */
#define ADS1220_MUX_0_1   	0x00
#define ADS1220_MUX_0_2   	0x10
#define ADS1220_MUX_0_3   	0x20
#define ADS1220_MUX_1_2   	0x30
#define ADS1220_MUX_1_3   	0x40
#define ADS1220_MUX_2_3   	0x50
#define ADS1220_MUX_1_0   	0x60
#define ADS1220_MUX_3_2   	0x70
#define ADS1220_MUX_0_G		0x80
#define ADS1220_MUX_1_G   	0x90
#define ADS1220_MUX_2_G   	0xa0
#define ADS1220_MUX_3_G   	0xb0
#define ADS1220_MUX_EX_VREF 0xc0
#define ADS1220_MUX_AVDD   	0xd0
#define ADS1220_MUX_DIV2   	0xe0
/* Define GAIN */
#define ADS1220_GAIN_1      0x00
#define ADS1220_GAIN_2      0x02
#define ADS1220_GAIN_4      0x04
#define ADS1220_GAIN_8      0x06
#define ADS1220_GAIN_16     0x08
#define ADS1220_GAIN_32     0x0a
#define ADS1220_GAIN_64     0x0c
#define ADS1220_GAIN_128    0x0e
/* Define PGA_BYPASS */
#define ADS1220_PGA_BYPASS 	0x01
/* ADS1220 Register 1 Definition */
/*   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0 
//--------------------------------------------------------------------------------------------
//                DR[2:0]            |      MODE[1:0]        |     CM    |     TS    |    BCS
*/
/* Define DR (data rate) */
#define ADS1220_DR_20		0x00
#define ADS1220_DR_45		0x20
#define ADS1220_DR_90		0x40
#define ADS1220_DR_175		0x60
#define ADS1220_DR_330		0x80
#define ADS1220_DR_600		0xa0
#define ADS1220_DR_1000		0xc0
/* Define MODE of Operation */
#define ADS1220_MODE_NORMAL 0x00
#define ADS1220_MODE_DUTY	0x08
#define ADS1220_MODE_TURBO 	0x10
#define ADS1220_MODE_DCT	0x18
/* Define CM (conversion mode) */
#define ADS1220_CC			0x04
/* Define TS (temperature sensor) */
#define ADS1220_TEMP_SENSOR	0x02
/* Define BCS (burnout current source) */
#define ADS1220_BCS			0x01
/* ADS1220 Register 2 Definition */
/*   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0 
//--------------------------------------------------------------------------------------------
//         VREF[1:0]     |        50/60[1:0]     |    PSW    |             IDAC[2:0]
*/
/* Define VREF */
#define ADS1220_VREF_INT	0x00
#define ADS1220_VREF_EX_DED	0x40
#define ADS1220_VREF_EX_AIN	0x80
#define ADS1220_VREF_SUPPLY	0xc0
/* Define 50/60 (filter response) */
#define ADS1220_REJECT_OFF	0x00
#define ADS1220_REJECT_BOTH	0x10
#define ADS1220_REJECT_50	0x20
#define ADS1220_REJECT_60	0x30
/* Define PSW (low side power switch) */
#define ADS1220_PSW_SW		0x08
/* Define IDAC (IDAC current) */
#define ADS1220_IDAC_OFF	0x00
#define ADS1220_IDAC_10		0x01
#define ADS1220_IDAC_50		0x02
#define ADS1220_IDAC_100	0x03
#define ADS1220_IDAC_250	0x04
#define ADS1220_IDAC_500	0x05
#define ADS1220_IDAC_1000	0x06
#define ADS1220_IDAC_2000	0x07
/* ADS1220 Register 3 Definition */
/*   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0 
//--------------------------------------------------------------------------------------------
//               I1MUX[2:0]          |               I2MUX[2:0]          |   DRDYM   | RESERVED
*/
/* Define I1MUX (current routing) */
#define ADS1220_IDAC1_OFF	0x00
#define ADS1220_IDAC1_AIN0	0x20
#define ADS1220_IDAC1_AIN1	0x40
#define ADS1220_IDAC1_AIN2	0x60
#define ADS1220_IDAC1_AIN3	0x80
#define ADS1220_IDAC1_REFP0	0xa0
#define ADS1220_IDAC1_REFN0	0xc0
/* Define I2MUX (current routing) */
#define ADS1220_IDAC2_OFF	0x00
#define ADS1220_IDAC2_AIN0	0x04
#define ADS1220_IDAC2_AIN1	0x08
#define ADS1220_IDAC2_AIN2	0x0c
#define ADS1220_IDAC2_AIN3	0x10
#define ADS1220_IDAC2_REFP0	0x14
#define ADS1220_IDAC2_REFN0	0x18
/* define DRDYM (DOUT/DRDY behaviour) */
#define ADS1220_DRDY_MODE	0x02

/* ADS1220 low level device specification*/
typedef struct _ADS1220_t_
{
    unsigned cs;
    void (*assert_cs)(unsigned assert, unsigned cs);    /*function pointer to assert cs*/
    void (*tx_byte_p)(unsigned char);                   /*function pointer to trasmit data */
    unsigned char (*rx_byte_p)(void);                   /*function pointer to rx data */
    long raw_data;                                      /*raw converted data */
    unsigned reg_read[ADS1220_REG_NUMBER];
    unsigned reg_write[ADS1220_REG_NUMBER];

}ADS1220_t;

void ADS1220Init(ADS1220_t *ads1220,
                 unsigned cs,
                 void (*assert_cs)(unsigned assert, unsigned cs),
                 void (*tx_byte)(unsigned char),
                 unsigned char (*rx_byte)(void));

/* ADS1220 Higher Level Functions */
void ADS1220ReadData(ADS1220_t* ads1220);                               /* Read the data results */
void ADS1220ReadRegister(ADS1220_t* ads1220,int StartAddress, int NumRegs); /* Read the register(s) */
void ADS1220WriteRegister(ADS1220_t* ads1220, int StartAddress, int NumRegs); /* Write the register(s) */
void ADS1220SendCommand(ADS1220_t* ads1220, unsigned char command);                         /* Send a device Command */
#endif /*ADS1220_H_*/
