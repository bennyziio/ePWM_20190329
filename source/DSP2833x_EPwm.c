//###########################################################################
//
// FILE:   DSP2833x_EPwm.c
//
// TITLE:  DSP2833x ePWM Initialization & Support Functions.
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
// InitEPwm - This function initializes the ePWM(s) to a known state.
//

void
InitEPwm(void)
{
	/************************************************************/
	/*  Initialize ePWM1                                        */
	/************************************************************/
	/*********************************************/
	/*  UP Count & Down Count - Set Timer Period */
	/*  Tpwm = (TBPRD + 1) x TBCLK               */
	/*                                           */
	/*  UPdown Count & - Set Timer Period        */
	/*  Tpwm = 2 x TBPRD x TBCLK                 */
	/*                                           */
	/*  Period = 150MHz / 4 / 1875  = 20KHz  		 */
	/*  Phase = 0                                */
	/*  Clear Counter                            */
	/*********************************************/
	EPwm1Regs.TBPRD = 1875;
	EPwm1Regs.TBPHS.half.TBPHS = 0;
	EPwm1Regs.TBCTR = 0;

	/*********************************************/
	/* Counter Up 								 */
	/*********************************************/
	EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;

	/*********************************************/
	/* Disable Phase Loading					 */
	/*********************************************/
	EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;

	/*********************************************/
	/* Time-base Counter equal to zero			 */
	/*********************************************/
	EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;

	/*********************************************/
	/*  TBCLK = SYSCLKOUT / (HSPCLKDIV x CLKDIV) */
	/*  TBCLK = (150MHz / (2 x 1 ))              */
	/*********************************************/
	EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;
	EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV2;

	/*********************************************/
	/* CMPA = SHADOW Mode, CMPB = SHADOW Mode    */
	/*********************************************/
	EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;

	/*********************************************/
	/* Load on CTR = Zero 						 */
	/*********************************************/
	EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
	EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

	/*********************************************/
	/* AQCTLB									 */
	/*********************************************/
	EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;		// CTR=CMPA when inc -> EPWM1A = 1
	EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;	// when dec -> EPWM1A = 0

	EPwm1Regs.AQCTLB.bit.CAU = AQ_SET;
	EPwm1Regs.AQCTLB.bit.CAD = AQ_CLEAR;
	EPwm1Regs.AQCTLB.bit.PRD = 0x10;	// CTR=PRD -> EPWM1B = 1
	EPwm1Regs.AQCTLB.bit.ZRO = 0x01;	// CTR=0 -> EPWM1B = 0

	/*********************************************/
	/* COMPARE Max = 1875						 */
	/*********************************************/
	EPwm1Regs.CMPA.half.CMPA = 0;
	EPwm1Regs.CMPB = 0;

	/*********************************************/
	/* DeadTime 								 */
	/*********************************************/
	/*
	EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
	EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HI;
	EPwm1Regs.DBCTL.bit.IN_MODE = DBA_RED_DBB_FED;
	EPwm1Regs.DBFED = 10;
	EPwm1Regs.DBRED = 10;
	*/

	/*********************************************/
	/* EPWM1 Interrupt Enable					 */
	/*********************************************/
	EPwm1Regs.ETSEL.bit.INTSEL = 0x2;		// 1 = (TBCTR = 0x0000), 2 = (TBCTR = TBPRD)
	EPwm1Regs.ETPS.bit.INTPRD = 0x1;		// Generate INT on 1st event
	EPwm1Regs.ETSEL.bit.INTEN = 1;		// Enable INT

	/*********************************************/
	/* FORCE									 */
	/*********************************************/
	//EPwm1Regs.AQCSFRC.bit.CSFA = FORCE_HIGH;
	//EPwm1Regs.AQCSFRC.bit.CSFB = FORCE_HIGH;



}

//
// End of file
//

