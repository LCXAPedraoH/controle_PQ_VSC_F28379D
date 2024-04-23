/*
 *
 * Peripheral_Setup.c
 *
 */
#include "Peripheral_Setup.h"

#define TRIG_SEL_ePWM1_SOCA 0X05

void Setup_GPIO(void){
    // pg 959 Table Mux, pg 965 Registers and  spruhm8i.pdf - Technical reference
    EALLOW;
    // LED 31 A, 2
    // LED 34 B, 1
    GpioCtrlRegs.GPAGMUX2.bit.GPIO31 = 0;
    GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO31 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO31 = 1;
    GpioCtrlRegs.GPACSEL4.bit.GPIO31 = GPIO_MUX_CPU1;

    GpioCtrlRegs.GPBGMUX1.bit.GPIO34 = 0;
    GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0;
    GpioCtrlRegs.GPBPUD.bit.GPIO34 = 1;
    GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;
    GpioCtrlRegs.GPBCSEL1.bit.GPIO34 = GPIO_MUX_CPU2;


    //===== PWM ==============================================

    //PWM 1A/B
    GpioCtrlRegs.GPAGMUX1.bit.GPIO0 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 1;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO1 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO1 = 1;
    //PWM 2A/B
    GpioCtrlRegs.GPAGMUX1.bit.GPIO2 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO3 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1;
    //PWM 3A/B
    GpioCtrlRegs.GPAGMUX1.bit.GPIO4 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO4 = 1;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO5 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO5 = 1;


    //GPIO 14 Out (debug)
    GpioCtrlRegs.GPAGMUX1.bit.GPIO14 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO14 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO14 = 1;
    EDIS;
}

void Setup_ePWM(void){
                            // pg 1978 spruhm8i.pdf - Technical reference
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    CpuSysRegs.PCLKCR2.bit.EPWM1 = 1;
    CpuSysRegs.PCLKCR2.bit.EPWM2 = 1;
    CpuSysRegs.PCLKCR2.bit.EPWM3 = 1;
//    CpuSysRegs.PCLKCR2.bit.EPWM4 = 1;
//    CpuSysRegs.PCLKCR2.bit.EPWM5 = 1;
//    CpuSysRegs.PCLKCR2.bit.EPWM6 = 1;
    EDIS;


    // ===== CONFIG PWM =====


    EPwm1Regs.TBPRD = 5000;                         // 10 kHz  (CLK_DSP/2)/(Freq*2))
    EPwm1Regs.CMPA.bit.CMPA = 0;
    EPwm1Regs.TBCTR = 0x0000;                       // Clear counter

    EPwm1Regs.TBPHS.bit.TBPHS = 0;                  // Phase is 0
    EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;     // Out pulse sincronization, master
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;         // Disable phase loading
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;  // Count up/down
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;        // Clock ratio to SYSCLKOUT
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;     // Load registers every ZERO
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD;
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;     // Load registers every ZERO
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO_PRD;

    EPwm1Regs.AQCTLA.bit.PRD = AQ_NO_ACTION;
    EPwm1Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;
    EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;            // set actions for EPWM1A
    EPwm1Regs.AQCTLA.bit.CAD = AQ_SET;
                                                    // aqui habilita o pwm complementar e seta o o complemento
    EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;       // Active Hi complementary
    EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;  // enable Dead-band module
    EPwm1Regs.DBFED.bit.DBFED = 100;                 // FED = 20 TBCLKs
    EPwm1Regs.DBRED.bit.DBRED = 100;                 // RED = 20 TBCLKs


    //======================================


    EPwm2Regs.TBPRD = EPwm1Regs.TBPRD;
    EPwm2Regs.CMPA.bit.CMPA = 0;
    EPwm2Regs.TBCTR = 0x0000;                       // Clear counter

    EPwm2Regs.TBPHS.bit.TBPHS = 0;                  // Phase is 0
    EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE; // Out pulse disable
    EPwm2Regs.TBCTL.bit.PHSEN = TB_DISABLE;         // Disable phase loading
    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;  // Count up/down
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;        // Clock ratio to SYSCLKOUT
    EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;     // Load registers every ZERO
    EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD;
    EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;     // Load registers every ZERO
    EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO_PRD;

    EPwm2Regs.AQCTLA.bit.PRD = AQ_NO_ACTION;
    EPwm2Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;
    EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;            // set actions for EPWM1A
    EPwm2Regs.AQCTLA.bit.CAD = AQ_SET;

    EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;       // Active Hi complementary
    EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;  // enable Dead-band module
    EPwm2Regs.DBFED.bit.DBFED = 100;                 // FED = 20 TBCLKs
    EPwm2Regs.DBRED.bit.DBRED = 100;                 // RED = 20 TBCLKs


    //======================================


    EPwm3Regs.TBPRD = EPwm1Regs.TBPRD;
    EPwm3Regs.CMPA.bit.CMPA = 0;
    EPwm3Regs.TBCTR = 0x0000;                       // Clear counter

    EPwm3Regs.TBPHS.bit.TBPHS = 0;                  // Phase is 0
    EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE; // Out pulse disable
    EPwm3Regs.TBCTL.bit.PHSEN = TB_DISABLE;         // Disable phase loading
    EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;  // Count up/down
    EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;        // Clock ratio to SYSCLKOUT
    EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;     // Load registers every ZERO
    EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD;
    EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;     // Load registers every ZERO
    EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO_PRD;
    EPwm3Regs.AQCTLA.bit.PRD = AQ_NO_ACTION;
    EPwm3Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;
    EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR;            // set actions for EPWM1A
    EPwm3Regs.AQCTLA.bit.CAD = AQ_SET;

    EPwm3Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;       // Active Hi complementary
    EPwm3Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;  // enable Dead-band module
    EPwm3Regs.DBFED.bit.DBFED = 100;                 // FED = 20 TBCLKs
    EPwm3Regs.DBRED.bit.DBRED = 100;                 // RED = 20 TBCLKs


    //======== Trigger ADC -----------------------------------  SERVE PRA FAZER AS AMOSTRAGENS EM 2x A f_PWM

    EPwm1Regs.ETSEL.bit.SOCAEN = 1;                 // Enable SOC on A group
    EPwm1Regs.ETSEL.bit.SOCASEL = ET_CTR_PRDZERO;   // Dispara ADC no topo
    EPwm1Regs.ETPS.bit.SOCAPRD = ET_1ST;            // Trigger on every event


    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;
}

void Setup_ADC_A(void){
                            // pg 1592 spruhm8i.pdf - Technical reference
    Uint16 acqps;

    EALLOW;
    CpuSysRegs.PCLKCR13.bit.ADC_A = 1;
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6;          // set ADCCLK divider to /4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;       // Set pulse um ciclo antes do resultado
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;          // power up the ADC
    DELAY_US(1000);                             // delay for 1ms to allow ADC time to power up

    // determine minimum acquisition window (in SYSCLKS) based on resolution

    if(ADC_RESOLUTION_12BIT == AdcaRegs.ADCCTL2.bit.RESOLUTION)
        acqps = 14;                             // 75ns
    else                                        // resolution is 16-bit
        acqps = 63;                             // 320ns


    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 14;                         //SOC0 will convert pin ADCINA14 Va (23)
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = acqps;                      //sample window is 15 SYSCLK cycles
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = TRIG_SEL_ePWM1_SOCA;      //trigger on ePWM7 SOCA

    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 3;                          //SOC1 will convert pin ADCIN2 Vb (26)
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = acqps;
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = TRIG_SEL_ePWM1_SOCA;

    AdcaRegs.ADCSOC2CTL.bit.CHSEL = 2;                          //SOC2 will convert pin ADCINA2 Vc (29)
    AdcaRegs.ADCSOC2CTL.bit.ACQPS = acqps;
    AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = TRIG_SEL_ePWM1_SOCA;

    AdcaRegs.ADCSOC3CTL.bit.CHSEL = 4;                          //SOC3 will convert pin ADCINA4 Vdc
    AdcaRegs.ADCSOC3CTL.bit.ACQPS = acqps;
    AdcaRegs.ADCSOC3CTL.bit.TRIGSEL = TRIG_SEL_ePWM1_SOCA;

//    AdcaRegs.ADCSOC4CTL.bit.CHSEL = 1;                          //SOC4 will convert pin ADCINA1 Vdc
//    AdcaRegs.ADCSOC4CTL.bit.ACQPS = acqps;
//    AdcaRegs.ADCSOC4CTL.bit.TRIGSEL = TRIG_SEL_ePWM1_SOCA;

    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 3;                      // end of SOC3 will set INT1 flag
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;                        // enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;                      // make sure INT1 flag is cleared

    EDIS;
}

void Setup_ADC_B(void){
    // pg 1592 spruhm8i.pdf - Technical reference
    Uint16 acqps;

    EALLOW;

    //write configurations
    CpuSysRegs.PCLKCR13.bit.ADC_B = 1;
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6;                          // Set ADCCLK divider to /4
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;                       // Set pulse positions to late
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;                          // power up the ADC
    DELAY_US(1000);                                             // delay for > 1ms to allow ADC time to power up

    //determine minimum acquisition window (in SYSCLKS) based on resolution
    if(ADC_RESOLUTION_12BIT == AdcbRegs.ADCCTL2.bit.RESOLUTION){
        acqps = 14;     //75ns
    }else{
        //resolution is 16-bit
        acqps = 63; //320ns
    }

    //Select the channels to convert and end of conversion flag ADCB
    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 15;                        //SOC0 will convert pin ADCINB3 Ia (25)
    AdcbRegs.ADCSOC0CTL.bit.ACQPS = acqps;                     //sample window is 100 SYSCLK cycles
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = TRIG_SEL_ePWM1_SOCA;     //trigger

    AdcbRegs.ADCSOC1CTL.bit.CHSEL = 5;                         //SOC1 will convert pin ADCINB2 Ib (28)
    AdcbRegs.ADCSOC1CTL.bit.ACQPS = acqps;
    AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = TRIG_SEL_ePWM1_SOCA;

    AdcbRegs.ADCSOC2CTL.bit.CHSEL = 4;                         //SOC2 will convert pin ADCINB5  Ic (65)
    AdcbRegs.ADCSOC2CTL.bit.ACQPS = acqps;
    AdcbRegs.ADCSOC2CTL.bit.TRIGSEL = TRIG_SEL_ePWM1_SOCA;


    //pg 1569 spruhm8i.pdf - Technical reference

    AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 2;                  // End of SOC3 will set INT1 flag
    AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1;                    // Disable INT1 flag
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;                  // Make sure INT1 flag is cleared

    EDIS;
}

void Setup_DAC(void){
    EALLOW;

    CpuSysRegs.PCLKCR16.bit.DAC_B = 1;
    DacbRegs.DACCTL.bit.SYNCSEL =       0x00;
    DacbRegs.DACCTL.bit.LOADMODE =      0x01;
    DacbRegs.DACCTL.bit.DACREFSEL =     0x01;
    DacbRegs.DACVALS.bit.DACVALS =      0; //12 bits
    DacbRegs.DACOUTEN.bit.DACOUTEN =    1;
    DacbRegs.DACLOCK.all =              0x00;


    CpuSysRegs.PCLKCR16.bit.DAC_A = 1;
    DacaRegs.DACCTL.bit.SYNCSEL =       0x00;
    DacaRegs.DACCTL.bit.LOADMODE =      0x01;
    DacaRegs.DACCTL.bit.DACREFSEL =     0x01;
    DacaRegs.DACVALS.bit.DACVALS =      0; //12 bits
    DacaRegs.DACOUTEN.bit.DACOUTEN =    1;
    DacaRegs.DACLOCK.all =              0x00;

    EDIS;

}



