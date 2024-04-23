#include "Peripheral_Setup.h"
#include "pid_reg3.h"
#include "3ph_abc_dq0.h"



// Interrupções ---------------------------------
__interrupt void isr_Cpu_Timer0(void);
__interrupt void isr_ADC(void);
//-----------------------------------------------



//PIDREG3 C_Vdc = PIDREG3_DEFAULTS;
pi_param_t C_iLd = PIDREG3_DEFAULTS;
pi_param_t C_iLq = PIDREG3_DEFAULTS;



ABC_DQ0_POS abc_dq0;        // CONVERSÃO DE VABC PARA Vdq
SPLL_3ph_SRF spll;

DQ0_ABC m_dq0_abc;         // CONVERSÃO DE Idq PARA IABC
ABC_DQ0_POS iL_abc_dq0;     // CONVERSÃO DE IABC PARA Idq



float iLa, iLb, iLc;
float Va, Vb, Vc;
float Mag_V = 400;
float Mag_I = 50;
float Vdc = 800.0;

float freqqq = 50.0;
float dut = 1;
float P_ref = 0;
float Q_ref = 0;

float duty[3];

int contador = 0, cont =100;
float filter_d[100] = {0}, filter_q[100] = {0};
float somad = 0, somaq = 0;


float id_ref, iq_ref, vd_ref, vq_ref;

float teste = 0.8;

uint8_t counter_adc = 0;

uint16_t index = 0;
uint32_t count = 0;

int main(void){



    InitSysCtrl();                          // Initialize System Control:
    DINT;                                   // Disable CPU interrupts
    InitPieCtrl();                          // Initialize the PIE control registers to their default state
    IER = 0x0000;                           // Disable CPU interrupts
    IFR = 0x0000;                           // Clear all CPU interrupt flags:

    EALLOW;
    // pg 120 spruhm8i.pdf - Technical reference
    CpuSysRegs.PCLKCR0.bit.CPUTIMER0 = 1;
    EDIS;
    InitPieVectTable();                     // Initialize the PIE vector table
    EALLOW;
    // Redirect function to interruptions
    PieVectTable.TIMER0_INT = &isr_Cpu_Timer0;
    PieVectTable.ADCA1_INT = &isr_ADC;
    EDIS;
    //pg. 102 PIE Channel Mapping spruhm8i.pdf
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;      // Timer 0
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;      // ADCA1
    IER |= M_INT1;

    InitCpuTimers();
    ConfigCpuTimer(&CpuTimer0, 200, 250000);
    CpuTimer0Regs.TCR.all = 0x4001;

    Setup_GPIO();
    Setup_ePWM();
    Setup_ADC_A();
    Setup_ADC_B();
    Setup_DAC();




    C_iLd.kp = 6.3;
    C_iLd.ki = 202;
    C_iLd.max = 400;
    C_iLd.min = -400;

    C_iLq.kp = 6.3;
    C_iLq.ki = 202;
    C_iLq.max = 400;
    C_iLq.min = -400;


    ABC_DQ0_POS_init(&abc_dq0);
    SPLL_3ph_SRF_init(freqqq,1/20000.0,&spll);

    DQ0_ABC_init(&m_dq0_abc);
    ABC_DQ0_POS_init(&iL_abc_dq0);



    EINT;                                   // Enable Global interrupt INTM
    ERTM;                                   // Enable Global realtime interrupt DBGM


    while(1){



    }
    return 0;
}



__interrupt void isr_Cpu_Timer0(void){
    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

}






__interrupt void isr_ADC(void){

    GpioDataRegs.GPASET.bit.GPIO14 = 1;



    Va = Mag_V*((float)AdcaResultRegs.ADCRESULT0 - 0x7FF)/2047;  //
    iLa = Mag_I*((float)AdcbResultRegs.ADCRESULT0 - 0x7FF)/2047;
    Vb = Mag_V*((float)AdcaResultRegs.ADCRESULT1 - 0x7FF)/2047;  //
    iLb = Mag_I*((float)AdcbResultRegs.ADCRESULT1 - 0x7FF)/2047;
    Vc = Mag_V*((float)AdcaResultRegs.ADCRESULT2 - 0x7FF)/2047;  //
    iLc = Mag_I*((float)AdcbResultRegs.ADCRESULT2 - 0x7FF)/2047;
/*
    Va = 400*((float)AdcaResultRegs.ADCRESULT0 - 0x7FF)/2047;  //
    iLa = 50*((float)AdcbResultRegs.ADCRESULT0 - 0x7FF)/2047;
    Vb = 400*((float)AdcaResultRegs.ADCRESULT1 - 0x7FF)/2047;  //
    iLb = 50*((float)AdcbResultRegs.ADCRESULT1 - 0x7FF)/2047;
    Vc = 400*((float)AdcaResultRegs.ADCRESULT2 - 0x7FF)/2047;  //
    iLc = 50*((float)AdcbResultRegs.ADCRESULT2 - 0x7FF)/2047;
*/

    // Vabc to dq
    abc_dq0.a = Va;
    abc_dq0.b = Vb;
    abc_dq0.c = Vc;

    abc_dq0.sin = __sin(spll.theta[1]); // Intrinsics for function TMU
    abc_dq0.cos = __cos(spll.theta[1]); // Intrinsics for function TMU

    ABC_DQ0_POS_FUNC(&abc_dq0);


    // PLL 3ph
    spll.v_q[0] = (abc_dq0.q/400);
    SPLL_3ph_SRF_FUNC(&spll);


  //  EALLOW;
  //  DacaRegs.DACVALS.bit.DACVALS = (uint16_t) (2047.0 * (1.0 + ampl * __sin( v_pll.theta[1] + phase)));   //essa parte é apenas para saida do pll
  //  DacbRegs.DACVALS.bit.DACVALS = (uint16_t) (2047.0 * (1.0 + __cos(spll.theta[1])));
  //  EDIS;


    // iLabc to dq
    iL_abc_dq0.a = iLa;
    iL_abc_dq0.b = iLb;
    iL_abc_dq0.c = iLc;
    iL_abc_dq0.sin = abc_dq0.sin;
    iL_abc_dq0.cos = abc_dq0.cos;
    ABC_DQ0_POS_FUNC(&iL_abc_dq0);


    id_ref = (abc_dq0.d*P_ref + abc_dq0.q*Q_ref)/(abc_dq0.d*abc_dq0.d + abc_dq0.q*abc_dq0.q)*2/3;
    iq_ref = (abc_dq0.q*P_ref - abc_dq0.d*Q_ref)/(abc_dq0.d*abc_dq0.d - abc_dq0.q*abc_dq0.q)*2/3;  // comentei pra ficar mais facil





//    filtro de potencia

    filter_d[99] = filter_d[98];
    filter_d[98] = filter_d[97];
    filter_d[97] = filter_d[96];
    filter_d[96] = filter_d[95];
    filter_d[95] = filter_d[94];
    filter_d[94] = filter_d[93];
    filter_d[93] = filter_d[92];
    filter_d[92] = filter_d[91];
    filter_d[91] = filter_d[90];
    filter_d[90] = filter_d[89];
    filter_d[89] = filter_d[88];
    filter_d[88] = filter_d[87];
    filter_d[87] = filter_d[86];
    filter_d[86] = filter_d[85];
    filter_d[85] = filter_d[84];
    filter_d[84] = filter_d[83];
    filter_d[83] = filter_d[82];
    filter_d[82] = filter_d[81];
    filter_d[81] = filter_d[80];
    filter_d[80] = filter_d[79];
    filter_d[79] = filter_d[78];
    filter_d[78] = filter_d[77];
    filter_d[77] = filter_d[76];
    filter_d[76] = filter_d[75];
    filter_d[75] = filter_d[74];
    filter_d[74] = filter_d[73];
    filter_d[73] = filter_d[72];
    filter_d[72] = filter_d[71];
    filter_d[71] = filter_d[70];
    filter_d[70] = filter_d[69];
    filter_d[69] = filter_d[68];
    filter_d[68] = filter_d[67];
    filter_d[67] = filter_d[66];
    filter_d[66] = filter_d[65];
    filter_d[65] = filter_d[64];
    filter_d[64] = filter_d[63];
    filter_d[63] = filter_d[62];
    filter_d[62] = filter_d[61];
    filter_d[61] = filter_d[60];
    filter_d[60] = filter_d[59];
    filter_d[59] = filter_d[58];
    filter_d[58] = filter_d[57];
    filter_d[57] = filter_d[56];
    filter_d[56] = filter_d[55];
    filter_d[55] = filter_d[54];
    filter_d[54] = filter_d[53];
    filter_d[53] = filter_d[52];
    filter_d[52] = filter_d[51];
    filter_d[51] = filter_d[50];
    filter_d[50] = filter_d[49];
    filter_d[49] = filter_d[48];
    filter_d[48] = filter_d[47];
    filter_d[47] = filter_d[46];
    filter_d[46] = filter_d[45];
    filter_d[45] = filter_d[44];
    filter_d[44] = filter_d[43];
    filter_d[43] = filter_d[42];
    filter_d[42] = filter_d[41];
    filter_d[41] = filter_d[40];
    filter_d[40] = filter_d[39];
    filter_d[39] = filter_d[38];
    filter_d[38] = filter_d[37];
    filter_d[37] = filter_d[36];
    filter_d[36] = filter_d[35];
    filter_d[35] = filter_d[34];
    filter_d[34] = filter_d[33];
    filter_d[33] = filter_d[32];
    filter_d[32] = filter_d[31];
    filter_d[31] = filter_d[30];
    filter_d[30] = filter_d[29];
    filter_d[29] = filter_d[28];
    filter_d[28] = filter_d[27];
    filter_d[27] = filter_d[26];
    filter_d[26] = filter_d[25];
    filter_d[25] = filter_d[24];
    filter_d[24] = filter_d[23];
    filter_d[23] = filter_d[22];
    filter_d[22] = filter_d[21];
    filter_d[21] = filter_d[20];
    filter_d[20] = filter_d[19];
    filter_d[19] = filter_d[18];
    filter_d[18] = filter_d[17];
    filter_d[17] = filter_d[16];
    filter_d[16] = filter_d[15];
    filter_d[15] = filter_d[14];
    filter_d[14] = filter_d[13];
    filter_d[13] = filter_d[12];
    filter_d[12] = filter_d[11];
    filter_d[11] = filter_d[10];
    filter_d[10] = filter_d[9];
    filter_d[9] = filter_d[8];
    filter_d[8] = filter_d[7];
    filter_d[7] = filter_d[6];
    filter_d[6] = filter_d[5];
    filter_d[5] = filter_d[4];
    filter_d[4] = filter_d[3];
    filter_d[3] = filter_d[2];
    filter_d[2] = filter_d[1];
    filter_d[1] = filter_d[0];
    filter_d[0] = id_ref/cont;

    somad = filter_d[0] +    filter_d[1] +    filter_d[2] +    filter_d[3] +    filter_d[4] +    filter_d[5] +    filter_d[6] +    filter_d[7] +    filter_d[8] +    filter_d[9] +    filter_d[10] +    filter_d[11] +    filter_d[12] +    filter_d[13] +    filter_d[14] +    filter_d[15] +    filter_d[16] +    filter_d[17] +    filter_d[18] +    filter_d[19] +    filter_d[20] +    filter_d[21] +    filter_d[22] +    filter_d[23] +    filter_d[24] +    filter_d[25] +    filter_d[26] +    filter_d[27] +    filter_d[28] +    filter_d[29] +    filter_d[30] +    filter_d[31] +    filter_d[32] +    filter_d[33] +    filter_d[34] +    filter_d[35] +    filter_d[36] +    filter_d[37] +    filter_d[38] +    filter_d[39] +    filter_d[40] +    filter_d[41] +    filter_d[42] +    filter_d[43] +    filter_d[44] +    filter_d[45] +    filter_d[46] +    filter_d[47] +    filter_d[48] +    filter_d[49] +    filter_d[50] +    filter_d[51] +    filter_d[52] +    filter_d[53] +    filter_d[54] +    filter_d[55] +    filter_d[56] +    filter_d[57] +    filter_d[58] +    filter_d[59] +    filter_d[60] +    filter_d[61] +    filter_d[62] +    filter_d[63] +    filter_d[64] +    filter_d[65] +    filter_d[66] +    filter_d[67] +    filter_d[68] +    filter_d[69] +    filter_d[70] +    filter_d[71] +    filter_d[72] +    filter_d[73] +    filter_d[74] +    filter_d[75] +    filter_d[76] +    filter_d[77] +    filter_d[78] +    filter_d[79] +    filter_d[80] +    filter_d[81] +    filter_d[82] +    filter_d[83] +    filter_d[84] +    filter_d[85] +    filter_d[86] +    filter_d[87] +    filter_d[88] +    filter_d[89] +    filter_d[90] +    filter_d[91] +    filter_d[92] +    filter_d[93] +    filter_d[94] +    filter_d[95] +    filter_d[96] +    filter_d[97] +    filter_d[98] +    filter_d[99];


    filter_q[99] = filter_q[98];
    filter_q[98] = filter_q[97];
    filter_q[97] = filter_q[96];
    filter_q[96] = filter_q[95];
    filter_q[95] = filter_q[94];
    filter_q[94] = filter_q[93];
    filter_q[93] = filter_q[92];
    filter_q[92] = filter_q[91];
    filter_q[91] = filter_q[90];
    filter_q[90] = filter_q[89];
    filter_q[89] = filter_q[88];
    filter_q[88] = filter_q[87];
    filter_q[87] = filter_q[86];
    filter_q[86] = filter_q[85];
    filter_q[85] = filter_q[84];
    filter_q[84] = filter_q[83];
    filter_q[83] = filter_q[82];
    filter_q[82] = filter_q[81];
    filter_q[81] = filter_q[80];
    filter_q[80] = filter_q[79];
    filter_q[79] = filter_q[78];
    filter_q[78] = filter_q[77];
    filter_q[77] = filter_q[76];
    filter_q[76] = filter_q[75];
    filter_q[75] = filter_q[74];
    filter_q[74] = filter_q[73];
    filter_q[73] = filter_q[72];
    filter_q[72] = filter_q[71];
    filter_q[71] = filter_q[70];
    filter_q[70] = filter_q[69];
    filter_q[69] = filter_q[68];
    filter_q[68] = filter_q[67];
    filter_q[67] = filter_q[66];
    filter_q[66] = filter_q[65];
    filter_q[65] = filter_q[64];
    filter_q[64] = filter_q[63];
    filter_q[63] = filter_q[62];
    filter_q[62] = filter_q[61];
    filter_q[61] = filter_q[60];
    filter_q[60] = filter_q[59];
    filter_q[59] = filter_q[58];
    filter_q[58] = filter_q[57];
    filter_q[57] = filter_q[56];
    filter_q[56] = filter_q[55];
    filter_q[55] = filter_q[54];
    filter_q[54] = filter_q[53];
    filter_q[53] = filter_q[52];
    filter_q[52] = filter_q[51];
    filter_q[51] = filter_q[50];
    filter_q[50] = filter_q[49];
    filter_q[49] = filter_q[48];
    filter_q[48] = filter_q[47];
    filter_q[47] = filter_q[46];
    filter_q[46] = filter_q[45];
    filter_q[45] = filter_q[44];
    filter_q[44] = filter_q[43];
    filter_q[43] = filter_q[42];
    filter_q[42] = filter_q[41];
    filter_q[41] = filter_q[40];
    filter_q[40] = filter_q[39];
    filter_q[39] = filter_q[38];
    filter_q[38] = filter_q[37];
    filter_q[37] = filter_q[36];
    filter_q[36] = filter_q[35];
    filter_q[35] = filter_q[34];
    filter_q[34] = filter_q[33];
    filter_q[33] = filter_q[32];
    filter_q[32] = filter_q[31];
    filter_q[31] = filter_q[30];
    filter_q[30] = filter_q[29];
    filter_q[29] = filter_q[28];
    filter_q[28] = filter_q[27];
    filter_q[27] = filter_q[26];
    filter_q[26] = filter_q[25];
    filter_q[25] = filter_q[24];
    filter_q[24] = filter_q[23];
    filter_q[23] = filter_q[22];
    filter_q[22] = filter_q[21];
    filter_q[21] = filter_q[20];
    filter_q[20] = filter_q[19];
    filter_q[19] = filter_q[18];
    filter_q[18] = filter_q[17];
    filter_q[17] = filter_q[16];
    filter_q[16] = filter_q[15];
    filter_q[15] = filter_q[14];
    filter_q[14] = filter_q[13];
    filter_q[13] = filter_q[12];
    filter_q[12] = filter_q[11];
    filter_q[11] = filter_q[10];
    filter_q[10] = filter_q[9];
    filter_q[9] = filter_q[8];
    filter_q[8] = filter_q[7];
    filter_q[7] = filter_q[6];
    filter_q[6] = filter_q[5];
    filter_q[5] = filter_q[4];
    filter_q[4] = filter_q[3];
    filter_q[3] = filter_q[2];
    filter_q[2] = filter_q[1];
    filter_q[1] = filter_q[0];
    filter_q[0] = iq_ref/cont;


    somaq = filter_q[0] +    filter_q[1] +    filter_q[2] +    filter_q[3] +    filter_q[4] +    filter_q[5] +    filter_q[6] +    filter_q[7] +    filter_q[8] +    filter_q[9] +    filter_q[10] +    filter_q[11] +    filter_q[12] +    filter_q[13] +    filter_q[14] +    filter_q[15] +    filter_q[16] +    filter_q[17] +    filter_q[18] +    filter_q[19] +    filter_q[20] +    filter_q[21] +    filter_q[22] +    filter_q[23] +    filter_q[24] +    filter_q[25] +    filter_q[26] +    filter_q[27] +    filter_q[28] +    filter_q[29] +    filter_q[30] +    filter_q[31] +    filter_q[32] +    filter_q[33] +    filter_q[34] +    filter_q[35] +    filter_q[36] +    filter_q[37] +    filter_q[38] +    filter_q[39] +    filter_q[40] +    filter_q[41] +    filter_q[42] +    filter_q[43] +    filter_q[44] +    filter_q[45] +    filter_q[46] +    filter_q[47] +    filter_q[48] +    filter_q[49] +    filter_q[50] +    filter_q[51] +    filter_q[52] +    filter_q[53] +    filter_q[54] +    filter_q[55] +    filter_q[56] +    filter_q[57] +    filter_q[58] +    filter_q[59] +    filter_q[60] +    filter_q[61] +    filter_q[62] +    filter_q[63] +    filter_q[64] +    filter_q[65] +    filter_q[66] +    filter_q[67] +    filter_q[68] +    filter_q[69] +    filter_q[70] +    filter_q[71] +    filter_q[72] +    filter_q[73] +    filter_q[74] +    filter_q[75] +    filter_q[76] +    filter_q[77] +    filter_q[78] +    filter_q[79] +    filter_q[80] +    filter_q[81] +    filter_q[82] +    filter_q[83] +    filter_q[84] +    filter_q[85] +    filter_q[86] +    filter_q[87] +    filter_q[88] +    filter_q[89] +    filter_q[90] +    filter_q[91] +    filter_q[92] +    filter_q[93] +    filter_q[94] +    filter_q[95] +    filter_q[96] +    filter_q[97] + filter_q[98] + filter_q[99];




    C_iLd.reference = somad;
//    C_iLd.reference = id_ref;
    C_iLd.measure = iL_abc_dq0.d;
    C_iLd.calc(&C_iLd);
    vd_ref = C_iLd.out - 314.16*(3.1/1000)*iL_abc_dq0.q; // Decoupling 2*PI*f*L

    C_iLq.reference = somaq;
//    C_iLq.reference = iq_ref;
    C_iLq.measure = iL_abc_dq0.q;
    C_iLq.calc(&C_iLq);
    vq_ref = C_iLq.out + 314.16*(3.1/1000)*iL_abc_dq0.d; // Decoupling 2*PI*f*L


    //====================================================================

    // iLdq to abc
    m_dq0_abc.d = (abc_dq0.d + vd_ref)/(Vdc/2);
    m_dq0_abc.q = (abc_dq0.q + vq_ref)/(Vdc/2);

//    m_dq0_abc.d = (vd_ref)/(Vdc/2);
//    m_dq0_abc.q = (vq_ref)/(Vdc/2);



    //=========    Saida de controle Md e Mq   ==================================

    EALLOW;
    DacaRegs.DACVALS.bit.DACVALS = (uint16_t) (2047.0 * (m_dq0_abc.d));
    DacbRegs.DACVALS.bit.DACVALS = (uint16_t) (2047.0 * (m_dq0_abc.q));
    EDIS;



    m_dq0_abc.z = 0;
    m_dq0_abc.sin = abc_dq0.sin;
    m_dq0_abc.cos = abc_dq0.cos;
    DQ0_ABC_FUNC(&m_dq0_abc);




    duty[0] = (m_dq0_abc.a);
    duty[1] = (m_dq0_abc.b);
    duty[2] = (m_dq0_abc.c);

    if(duty[0] > 0.99) duty[0] = 0.99;
    else if(duty[0] < -0.99) duty[0] = -0.99;
    if(duty[1] > 0.99) duty[1] = 0.99;
    else if(duty[1] < -0.99) duty[1] = -0.99;
    if(duty[2] > 0.99) duty[2] = 0.99;
    else if(duty[2] < -0.99) duty[2] = -0.99;


    //dutyp[0] = teste;


    //===================================


    // Phase A
    EPwm1Regs.CMPA.bit.CMPA = (uint16_t)((duty[0]+1)/2 * ((float)(EPwm1Regs.TBPRD)));
    //EPwm1Regs.CMPA.bit.CMPA = (uint16_t)((duty[0]) * ((float)(EPwm1Regs.TBPRD))); // utilizaddo para teste de pwm

    // Phase B
    EPwm2Regs.CMPA.bit.CMPA = (uint16_t)((duty[1]+1)/2 * ((float)(EPwm2Regs.TBPRD)));

    // Phase C
    EPwm3Regs.CMPA.bit.CMPA = (uint16_t)((duty[2]+1)/2* ((float)(EPwm3Regs.TBPRD)));



    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;              //clear INT1 flag
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;              //clear INT1 flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

    GpioDataRegs.GPACLEAR.bit.GPIO14 = 1;
}
