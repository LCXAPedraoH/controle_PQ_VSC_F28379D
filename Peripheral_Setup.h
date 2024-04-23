/*
 * Peripheral_Setup.h
 *

 */

#ifndef PERIPHERAL_SETUP_H_
#define PERIPHERAL_SETUP_H_
#include "F28x_Project.h"
//======= Files to CAN
#include "../inc/hw_types.h"
#include "../inc/hw_memmap.h"
#include "../inc/hw_can.h"
#include "../driverlib/can.h"
void Setup_GPIO(void);
void Setup_ePWM(void);
void Setup_ADC_A(void);
void Setup_ADC_B(void);
void Setup_DAC(void);

#define CPU_FREQ        200E6
#define LSPCLK_FREQ     CPU_FREQ/4
#define SCI_FREQ        115200
#define SCI_PRD         (LSPCLK_FREQ/(SCI_FREQ*8))-1

#endif /* PERIPHERAL_SETUP_H_ */
