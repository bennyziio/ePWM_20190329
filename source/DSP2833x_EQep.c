//###########################################################################
//
// FILE:   DSP2833x_EQep.c
//
// TITLE:  DSP2833x eQEP Initialization & Support Functions.
//
//###########################################################################
// $TI Release: F2833x/F2823x Header Files and Peripheral Examples V142 $
// $Release Date: November  1, 2016 $
// $Copyright: Copyright (C) 2007-2016 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################

//
// Included Files
//
#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File

//
// InitEQep - This function initializes the eQEP(s) to a known state.
//
void 
EQEP_Init(void)
{
	/**********************************/
	/*  M type to calculate velocity  */
	/**********************************/


    EQep1Regs.QUPRD = 150000000 / 4500;		// Unit Timer for 2000Hz at 150MHz SYSCLKOUT -- M type
    EQep1Regs.QDECCTL.bit.QSRC = 0;			// Quadrature Count mode
    EQep1Regs.QEPCTL.bit.FREE_SOFT = 2;
    EQep1Regs.QEPCTL.bit.UTE = 1;			// Unit Timer Enable
    EQep1Regs.QEPCTL.bit.PCRM = 0;			// 0 = Position counter Reset on Index Event -- M type
											// 1 = Position counter Reset on Maximum position -- T type
    EQep1Regs.QEPCTL.bit.QCLM = 1;			// 0 = Position counter latch on QPOSCNT -- T type
        									// 1 = Position counter latch on unit time out -- M type
    EQep1Regs.QPOSMAX = 4 * 8000;			// 8000 pulses @ 1 revolution
    //EQep1Regs.QPOSMAX = 0xffffffff;			// 8000 pulses @ 1 revolution
    EQep1Regs.QEPCTL.bit.QPEN = 1;			// QEP enable
    EQep1Regs.QEINT.bit.UTO = 1;			// Unit Time Out Interrupt Enable -- M type

    EQep1Regs.QCAPCTL.bit.UPPS = 2;			// 1/4 for unit position
	EQep1Regs.QCAPCTL.bit.CCPS = 4;			// 1/16 for CAP clock
	EQep1Regs.QCAPCTL.bit.CEN = 1;			// QEP Capture Enable
	//EQep1Regs.QPOSCNT = 0;


    /**********************************/
	/*  T type to calculate velocity  */
	/**********************************/

//	  EQep1Regs.QUPRD = 150000000 / 500;		// Unit Timer for 1KHz at 150MHz SYSCLKOUT -- M type
//	  EQep1Regs.QDECCTL.bit.QSRC = 0;			// Quadrature Count mode
//    EQep1Regs.QEPCTL.bit.FREE_SOFT = 2;
//    EQep1Regs.QEPCTL.bit.PCRM = 1;			// 0 = Position counter Reset on Index Event -- M type
//    										// 1 = Position counter Reset on Maximum position -- T type
//    EQep1Regs.QEPCTL.bit.QCLM = 0;			// 0 = Position counter latch on QPOSCNT -- T type
//    										// 1 = Position counter latch on unit time out -- M type
//    EQep1Regs.QPOSMAX = 32000;				// 8000 pulses @ 1 revolution
//    EQep1Regs.QEPCTL.bit.QPEN = 1;			// QEP enable
//    EQep1Regs.QCAPCTL.bit.UPPS = 2;			// 1/4 for unit position
//    EQep1Regs.QCAPCTL.bit.CCPS = 4;			// 1/16 for CAP clock
//    EQep1Regs.QCAPCTL.bit.CEN = 1;			// QEP Capture Enable
//
//    EQep1Regs.QEPCTL.bit.UTE = 1;			// Unit Timer Enable
//    EQep1Regs.QEINT.bit.UTO = 1;			// Unit Time Out Interrupt Enable -- M type
}

//
// End of file
//
