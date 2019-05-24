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
InitEQep(void)
{
    EQep1Regs.QUPRD = (150E6/2);		// Unit Timer for 2Hz at 150MHz SYSCLKOUT

    EQep1Regs.QDECCTL.bit.QSRC = 0;		// Quadrature Count mode
    EQep1Regs.QEPCTL.bit.FREE_SOFT = 2;
    EQep1Regs.QEPCTL.bit.UTE = 1;		// Unit Timer Enable
    EQep1Regs.QEPCTL.bit.QCLM = 1;		// Position counter latch on unit time out
    EQep1Regs.QEPCTL.bit.PCRM = 1;		// Position counter Reset on Maximum position
    EQep1Regs.QPOSMAX = 4 * 8000;		// 8000 pulses @ 1 revolution
    EQep1Regs.QEPCTL.bit.QPEN = 1;		// QEP enable
    EQep1Regs.QEINT.bit.UTO = 1;
}

//
// End of file
//
