/** @file BOOSTXL_DUAL_LOAD_CELL.h
 * 
 * @brief A description of the module’s purpose. 
 *
 * @par       
 * COPYRIGHT NOTICE: (c) 2018 Barr Group.  All rights reserved.
 */
#include "ADS1220.h"
#include <stdint.h>
#include "DSP28x_Project.h"

#ifndef BOOSTXL_DUAL_LOAD_CELL_H
#define BOOSTXL_DUAL_LOAD_CELL_H
//////////////////////////////////////////////////////////////////////////////////
// SETUP BXL
// boosterpack are designed so thath you can stack 
// two of them to have 4 cell inputs
// depending of the nuber of ads you have to configure the jumper 
//  when both DRDY ADS1220 outputs goes low then DRDY goeslow
//HIGH to LOW transition on DRDY  trigger an interrupt

#define LAUNCHXL_F28069M

#define SYS_CLK_FREQ_Hz 80000000 //80 MHZ
#define LSPC_CLK_FREQ_Hz (SYS_CLK_FREQ_Hz/4)


#define BOOSTER_PACK_NUMBER 1 // 1 one single BOOSTER PACK
#define BOOSTERPACK_POSITION 1 // 0: TOP ; 1: BOTTOM
// clock select for the ADS1220 
// #define JP6_CLK_SELECT 0      // 0: J4_6 ; 1: GND (ADS1220 internal)

//to modify
#define JP1_EN_DOUT 0               // 0:auto enable; 1 GPIO control
#define JP5_DRDY_PIN_SELECT 0       // 0: J4_5 ; 1: J4_7
#define JP10_CS0_PIN_SELECT 0       // 0: J4_1 ; 1: J4_2
#define JP7_CS1_PIN_SELECT 0        // 0: J4_3 ; 1: J4_4
#define JP8_EN_DOUT_PIN_SELECT 1    // 0: J4_8 ; 1: J2_2
#define JP3_DRDY 0                  // 0(OPEN): DRDY_0 & DRDY_1; 1(CLOSE): DRDY_0 -> JP5
#define JP9_DRDY_1 0                // 0(OPEN): DRDY_1 NC; 1(CLOSE): DRDY_1 -> J1_3


//////////////////////////////////////////////////////////////////////////////////
// END SETUP BXL
/////////////////////////////////////////////////////////////////////////////////



#define BXL_D_LC_ADS1220_NUMBER BOOSTER_PACK_NUMBER*2

#if BOOSTERPACK_POSITION==BOTTOM
#define CLK_PIN J4_6
#else
#define CLK_PIN J8_6
#endif

////////////////////////////////////////////
///////////////////////////////////////////
#ifdef LAUNCHXL_F28069M
//DATA RATE SELECTION see ipynb


#define BXL_D_LC_ADS1220_DATARATE ADS1220_DR_600
#define BXL_D_LC_DATARATE 400
#define BXL_D_LC_TRUE_DATARATE 400.032003
#define BXL_D_LC_PWM_PRD 29 // PWM CLK == SYSCLK No divide
#define BXL_D_LC_PWM_DUTYCYCLE (BXL_D_LC_PWM_PRD+1)/2
#define BXL_D_LC_PWM_FREQ SYS_CLK_FREQ_Hz/BXL_PWM_PRD
#define BXL_D_LC_ADS1220_MODE ADS1220_MODE_NORMAL

#define BXL_D_LC_ADS1220_INT_CLK 0


#define BXL_D_LC_DOUT_EN 0x08000000
#define BXL_D_LC_CS0_DOUT_DISABLED 0x00000040
#define BXL_D_LC_CS0 (BXL_D_LC_DOUT_EN | BXL_D_LC_CS0_DOUT_DISABLED ) //GPIO6
#define BXL_D_LC_CS1 (0x00000001<<8)
#define BXL_D_LC_DRDY (0x00000001<<10)


#define BXL_D_LC_GPIO_DATA_REG GpioDataRegs.GPADAT.all
#if BOOSTERPACK_POSITION ==0
#error BXL position has to be bottom
#endif

void BXL_D_LC_F28069_GPIO_setup(void);
void BXL_D_LC_F28069_PWM_J8_6_setup(void);
void BXL_D_LC_F28069_DRDY_EXT_INT_setup(void);
void BXL_D_LC_F28069_DRDY_EXT_INT_enable(uint16_t en);
void BXL_D_LC_F28069_ADS1220_SYNC_START(void);

void BXL_D_LC_F28069_Spib_setup(void);
void BXL_D_LC_F28069_Spib_tx_byte(uint16_t uint8_data);
uint16_t BXL_D_LC_F28069_Spib_rx_byte(void);
void BXL_D_LC_F28069_Spib_assert_cs(uint16_t assert, uint32_t cs);
void BXL_D_LC_F28069_ADS1220_RESET(void);


#endif

//////////////////////////////////////////////////////////////////////////////////
//ADS setup
/*
CONF0 reg
analog input differential AIN0+ AIN1-: ADS1220_MUX_0_1
input gain 128 : ADS1220_GAIN_128

CONF1 reg
datarate:
turbo mode: ADS1220_MODE_TURBO
continous conversion mode: ADS1220_CC

CONF2 reg
reference on dedicated pins (ratiometric): ADS1220_VREF_EX_DED

CONF3 reg
default
*/
#define BXL_D_LC_ADS1220_CONF0_REG (ADS1220_MUX_0_1 | ADS1220_GAIN_128)
#define BXL_D_LC_ADS1220_CONF1_REG (BXL_D_LC_ADS1220_DATARATE| BXL_D_LC_ADS1220_MODE | ADS1220_CC)
#define BXL_D_LC_ADS1220_CONF2_REG ADS1220_VREF_EX_DED
#define BXL_D_LC_ADS1220_CONF3_REG 0x00

void BXL_D_LC_ADS1220_setup(ADS1220_t* ads1220);
#endif /* BOOSTXL_DUAL_LOAD_CELL_H */

/*** end of file ***/
