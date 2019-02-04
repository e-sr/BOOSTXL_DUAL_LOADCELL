#include "BOOSTXL_DUAL_LOAD_CELL.h"

void BXL_D_LC_F28069_GPIO_setup(void)
{
    EALLOW;
    //-------------------
    //ADS1220 related PIN
    //Init SPIA GPIO
    GpioCtrlRegs.GPAPUD.bit.GPIO24 = 0;  // Enable pull-up on GPIO16 (SPISIMOA)
    GpioCtrlRegs.GPAPUD.bit.GPIO25 = 0;  // Enable pull-up on GPIO17 (SPISOMIA)
    GpioCtrlRegs.GPAPUD.bit.GPIO14 = 0;  // Enable pull-up on GPIO18 (SPICLKA)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO24 = 3; // Asynch input GPIO16 (SPISIMOA)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO25 = 3; // Asynch input GPIO17 (SPISOMIA)
    GpioCtrlRegs.GPAQSEL1.bit.GPIO14 = 3; // Asynch input GPIO18 (SPICLKA)
    GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 11; // Configure GPIO16 as SPISIMOA
    GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 11; // Configure GPIO17 as SPISOMIA
    GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 11; // Configure GPIO18 as SPICLKA
    // CS and DOUT EN
    GpioCtrlRegs.GPAPUD.all &= ~(BXL_D_LC_CS0 | BXL_D_LC_CS1); //enable
    GpioCtrlRegs.GPADIR.all |= (BXL_D_LC_CS0 | BXL_D_LC_CS1); //outpud
    //CLK GPIO4 epwm
    GpioCtrlRegs.GPAPUD.bit.GPIO11 = 1;    // Disable pull-up on GPIO4 (EPWM6B)
    GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 1;   //mux to pwm
    //DRDY
    GpioCtrlRegs.GPAPUD.all &= ~(BXL_D_LC_DRDY);
    GpioCtrlRegs.GPADIR.all &= ~(BXL_D_LC_DRDY);    //input
    GpioCtrlRegs.GPAQSEL1.bit.GPIO10 = 0; // XINT1 Synch to SYSCLKOUT only
    EDIS;
    //set HIGH all CS related
    GpioDataRegs.GPADAT.all |= (BXL_D_LC_CS0 | BXL_D_LC_CS1);

    return;
}

void BXL_D_LC_F28069_Spib_setup(void)
{
    //SPICLK requirements Accordung to ADS1246 datasheet for datarate 2000sps: 488ns < SPICLK period < 520ns => 2MHz
    // ADS1246 require CLK POLARITY 0  and PHASE 1: output on the rising edge, input is latched on the falling edge
    SpibRegs.SPICCR.bit.SPISWRESET = 0;     // Reset SPI
    SpibRegs.SPICCR.bit.CLKPOLARITY = 0;
    SpibRegs.SPICCR.bit.SPICHAR = 7;        //SPIWORD 8 bit
    SpibRegs.SPICTL.bit.MASTER_SLAVE = 1;
    SpibRegs.SPICTL.bit.CLK_PHASE = 0;
    SpibRegs.SPICTL.bit.TALK = 1;
    SpibRegs.SPISTS.all = 0x0000;
    SpibRegs.SPIBRR = ((LSPC_CLK_FREQ_Hz / ADS1220_SPICLK_FREQ_HZ) - 1); // Baud rate register: SPICLK=LSPCLK/(SPIBRR+1), LSPCLK = SYSCLK/4=20MHZ, 20/(9+1)=2MHz:
    SpibRegs.SPICCR.bit.SPISWRESET = 1;      // Enable SPI
    return;

}

/* ADS1220 Initial Configuration */
void BXL_D_LC_ADS1220_setup(ADS1220_t* ads1220)
{
    ads1220->reg_write[0] = BXL_D_LC_ADS1220_CONF0_REG;
    ads1220->reg_write[1] = BXL_D_LC_ADS1220_CONF1_REG;
    ads1220->reg_write[2] = BXL_D_LC_ADS1220_CONF2_REG;
    ads1220->reg_write[3] = BXL_D_LC_ADS1220_CONF3_REG;

    /* write the register value containing the new value back to the ADS */
    ADS1220WriteRegister(ads1220, ADS1220_0_REGISTER, 0x04);
    DELAY_US(50);
    ADS1220ReadRegister(ads1220, ADS1220_0_REGISTER, 0x04);
    return;
}

void BXL_D_LC_F28069_PWM_J8_6_setup(void)
{
    //----------------------
    //init epwm3 peripheral
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;      // Stop all the TB clocks
    EDIS;
    //Timer Base submodule
    EPwm6Regs.TBCTL.bit.PHSEN = TB_DISABLE;
    EPwm6Regs.TBPHS.half.TBPHS = 0x0000;       // Phase is 0
    EPwm6Regs.TBCTR = 0x0000;                  // Clear counter
    EPwm6Regs.TBCTL.bit.HSPCLKDIV = 0x00; // divide by 1 => TBCLK = SYSCLKOUT / (HSPCLKDIV × CLKDIV) = 80/8 =10MHZ
    EPwm6Regs.TBCTL.bit.CLKDIV = 0x00; //divide by 1
    EPwm6Regs.TBPRD = (BXL_D_LC_PWM_PRD + 1); //period register frequenza TBPRD+1
    EPwm6Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;     // Count up

    //Compare submodule    //
    // Setup shadow register load immediate
    EPwm6Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm6Regs.CMPCTL.bit.LOADBMODE = CC_IMMEDIATE;
    EPwm6Regs.CMPB = BXL_D_LC_PWM_DUTYCYCLE;
    //Action Qualifier submodule
    EPwm6Regs.AQCTLB.bit.CBU = AQ_CLEAR;
    EPwm6Regs.AQCTLB.bit.ZRO = AQ_SET;
    EPwm6Regs.AQCSFRC.bit.CSFB = BXL_D_LC_ADS1220_INT_CLK; //FORCE REGISTER 0:disabled; 01:force LOW; 10:force HIGH;
    //Event Trigger submodule -> interrupt
    //EPwm6Regs.ETSEL.bit.INTSEL = ET_CTRU_CMPB; // Enable INT on CMPB event up cunting
    //EPwm3Regs.ETSEL.bit.INTEN = 1;                  // Enable INT
    //EPwm6Regs.ETPS.bit.INTPRD = ET_1ST;            // Generate INT on 1st event
    //
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;       // Start all the timers synced
    EDIS;
    //-------------------------
    return;
}

void BXL_D_LC_F28069_DRDY_EXT_INT_setup(void)
{
    // GPIO9 is XINT1
    EALLOW;
    GpioIntRegs.GPIOXINT1SEL.bit.GPIOSEL = 10;   // XINT1 is GPIO10
    EDIS;
    //
    XIntruptRegs.XINT1CR.bit.POLARITY = 0;      // Falling edge interrupt
    return;

}

void BXL_D_LC_F28069_DRDY_EXT_INT_enable(uint16_t en)
{
    XIntruptRegs.XINT1CR.bit.ENABLE = (0x0001 & en);        // Enable XINT1
    return;
}

uint16_t BXL_D_LC_F28069_Spib_rx_byte(void)
{
    uint16_t value;
    SpibRegs.SPITXBUF = 0x0000;
    while (!SpibRegs.SPISTS.bit.INT_FLAG)
        ; // wait transmission to complete
    value = SpibRegs.SPIRXBUF & 0xff;
    return value;
}

void BXL_D_LC_F28069_Spib_tx_byte(uint16_t uint8_data)
{
    SpibRegs.SPITXBUF = uint8_data << 8; //F28069 (c2000) has a 16 bit TXBUF register
    while (!SpibRegs.SPISTS.bit.INT_FLAG)
        ; // wait transmission to complete
    SpibRegs.SPIRXBUF; //read to clear SPISTS.bit.INT_FLAG!
    return;
}

void BXL_D_LC_F28069_Spib_assert_cs(uint16_t assert, uint32_t cs)
{
    if (assert)
    { //CS LOW
        DELAY_US(1);
        BXL_D_LC_GPIO_DATA_REG &= (~cs); //clear
    }
    else
    { //CS HIGH
        DELAY_US(1);
        BXL_D_LC_GPIO_DATA_REG |= cs; //set

    }
    return;
}

void BXL_D_LC_F28069_ADS1220_SYNC_START(void)
{
    BXL_D_LC_F28069_Spib_assert_cs(1, (BXL_D_LC_CS0_DOUT_DISABLED | BXL_D_LC_CS1));
    BXL_D_LC_F28069_Spib_tx_byte(ADS1220_CMD_SYNC);
    BXL_D_LC_F28069_Spib_assert_cs(0, (BXL_D_LC_CS0_DOUT_DISABLED | BXL_D_LC_CS1));
    return;
}

void BXL_D_LC_F28069_ADS1220_RESET(void)
{
    BXL_D_LC_F28069_Spib_assert_cs(1, (BXL_D_LC_CS0_DOUT_DISABLED | BXL_D_LC_CS1));
    BXL_D_LC_F28069_Spib_tx_byte(ADS1220_CMD_RESET);
    DELAY_US(100);
    BXL_D_LC_F28069_Spib_assert_cs(0, (BXL_D_LC_CS0_DOUT_DISABLED | BXL_D_LC_CS1));
    return;
}

