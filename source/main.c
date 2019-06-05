#include <DSP2833x_Device.h>
#include <DSP2833x_EPwm.h>
#include <DSP2833x_EPwm_defines.h>
#include <DSP2833x_EQep.h>
#include <DSP2833x_GlobalPrototypes.h>
#include <DSP2833x_PieCtrl.h>
#include <DSP2833x_PieVect.h>
#include <DSP2833x_SysCtrl.h>
#include <DSP28x_Project.h>
#include <EQEP_posspeed.h>

#define FLASH_program		1		// 1 = Flash Download, 0 = RAM Control

float32 SpeedRpm = 0;

//void POSSPEED_Init();

Uint16 FLAG_1ms = 0;
Uint16 FLAG_10ms = 0;
Uint16 FLAG_500ms = 0;
Uint16 FLAG_2000ms = 0;
Uint16 FLAG_5s = 0;

Uint16 uCount_001ms = 0;
Uint16 uCount_010ms = 0;
Uint16 uCount_500ms = 0;
Uint16 uCount_2000ms = 0;
Uint16 uCount_5s = 0;

Uint16 EPWMTimerCount = 0;

Uint16 FLAG_1ms_Counter = 0;
Uint16 FLAG_10ms_Counter = 0;
Uint16 FLAG_500ms_Counter = 0;
Uint16 FLAG_2000ms_Counter = 0;
Uint16 FLAG_5s_Counter = 0;

float32 EPwm_duty = 0;

Uint16 EPwm1_CMPA_Direction = 0;

// Prototype statements for functions found within this file.

__interrupt void epwm1_isr(void);
__interrupt void epwm2_isr(void);
__interrupt void epwm3_isr(void);

__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);

//POSSPEED qep_posspeed=POSSPEED_DEFAULTS;

void main(void)
{
	/************************************************************/
	/*  Step 1. Initialize System Control:                      */
	/*  PLL, WatchDog, enable Peripheral Clocks                 */
	/*  This function is found in the DSP2833x_SysCtrl.c file   */
	/************************************************************/
	InitSysCtrl();

	/************************************************************/
	/* Step 2. Initalize GPIO:                                  */
	/*  This function is found in the DSP2833x_Gpio.c file      */
	/************************************************************/
	InitGpio();

	/************************************************************/
	/* Disable CPU interrupts                                   */
	/************************************************************/
	DINT;

	/************************************************************/
	/* Step 3. Clear all interrupts and initialize PIE vector   */
	/*         table:                                           */
	/* Initialize the PIE control registers to their default    */
	/*         state.                                           */
	/* The default state is all PIE interrupts disabled and flag*/
	/*         are cleared.                                     */
	/* This function is found in the DSP2833x_PieCtrl.c file.   */
	/************************************************************/
	InitPieCtrl();

	/************************************************************/
	/* Disable CPU interrupts and clear all CPU interrupt flags:*/
	/************************************************************/
	IER = 0x0000;
	IFR = 0x0000;

	/************************************************************/
	/*Initialize the PIE vector table with pointers to the shell*/
	/*		Interrupt Service Routines (ISR).                   */
	/*The shell ISR routines are found in DSP2833x_DefaultIsr.c.*/
	/*This function is found in DSP2833x_PieVect.c.             */
	/************************************************************/
    InitPieVectTable();

    /************************************************************/
	/* Interrupts that are used in this example are re-mapped to*/
	/* ISR functions found within this file.                    */
    /************************************************************/
	EALLOW;  // This is needed to write to EALLOW protected registers
	PieVectTable.TINT0 = &cpu_timer0_isr;
	PieVectTable.XINT13 = &cpu_timer1_isr;
	PieVectTable.TINT2 = &cpu_timer2_isr;
	PieVectTable.EPWM1_INT = &epwm1_isr;
	PieVectTable.EPWM2_INT = &epwm2_isr;
	//PieVectTable.EPWM3_INT = &epwm3_isr;
	//PieVectTable.EQEP1_INT = &UnitTimeOut_Isr;
	EDIS;    // This is needed to disable write to EALLOW protected registers

	/************************************************************/
	/* Step 4. Initialize all the Device Peripherals:           */
	/************************************************************/
	InitEPwm();

	/************************************************************/
	/* Step 5. Initialize Timer                                 */
	/* Main Clock = 150MHz                                      */
	/* Timer0 = 1msec                                           */
	/* Timer1 = 1sec                                            */
	/************************************************************/
	InitCpuTimers();   // For this example, only initialize the Cpu Timers
	ConfigCpuTimer(&CpuTimer0, 150, 100);
	ConfigCpuTimer(&CpuTimer1, 150, 100000);

	/************************************************************/
	/* Step 6. User specific code, enable interrupts:           */
	/* Copy time critical code and Flash setup code to RAM      */
	/* The  RamfuncsLoadStart, RamfuncsLoadEnd,                 */
	/* and RamfuncsRunStart symbols are created by the linker.  */
	/* Refer to the F28335.cmd file.                            */
	/************************************************************/
	#if FLASH_program
	   MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
	#endif

	/************************************************************/
	/* Step 7. This function must reside in RAM                 */
	/************************************************************/
	InitFlash();

	/************************************************************/
	/* Step 8. Interrupt Setting                                */
	/************************************************************/
	/************************************************************/
	/* Enable CPU INT1 which is connected to Timer0:            */
	/* Enable CPU INT3 which is connected to EPWM1 ~ 5:         */
	/* Enable CPU INT13 which is connected to Timer1:           */
	/* Enable CPU INT14 which is connected to Timer2:           */
	/************************************************************/
	IER |= M_INT1;	// Timer0
	IER |= M_INT3;	// EPwm1 ~ EPwm5
	IER |= M_INT13;	// Timer1
	IER |= M_INT14;	// Timer2
	IER |= M_INT5;	// EQep1

	/************************************************************/
	/* Enable TIMER0 in the PIE:                                */
	/* 							 INTx.7 = TINT0(TIMER0)         */
	/* Enable EPWM1_INT ~ EPWM5_INT in the PIE:                 */
	/*                           INTx.1 = EPWM1_INT             */
	/*                           INTx.2 = EPWM2_INT             */
	/*                           INTx.3 = EPWM3_INT             */
	/*                           INTx.4 = EPWM4_INT             */
	/*                           INTx.5 = EPWM5_INT             */
	/* Enable TIMER1 in the PIE:                                */
	/*           TIMER1 Interrupt does not have to allocate     */
	/************************************************************/
	PieCtrlRegs.PIEIER1.bit.INTx7 = 1;		// Timer0
	PieCtrlRegs.PIEIER3.bit.INTx1 = 1;		// EPwm1
	PieCtrlRegs.PIEIER3.bit.INTx2 = 1;		// EPwm2
	PieCtrlRegs.PIEIER3.bit.INTx3 = 1;		// EPwm3
	PieCtrlRegs.PIEIER5.bit.INTx2 = 1;		// EQep1

	/************************************************************/
	/* For this example, only initialize the ePWM				*/
	/************************************************************/
	EALLOW;
	SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
	EDIS;

	EALLOW;
	SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
	EDIS;

	/************************************************************/
	/* Initialize S/W modules and Variables                     */
	/************************************************************/
	/*
	BackTicker = 0;
	IsrTicker = 0;

	PositionCounter = 0;
	RotateDirection = 0;
	UTimeOutPositionLatch = 0;
	PastUTimeOutPositionLatch = 0;
	Displacement = 0;
	*/

	POSSPEED_Init();
	EALLOW;
	GpioCtrlRegs.GPADIR.bit.GPIO4 = 1;    // GPIO4 as output simulates Index signal
	GpioDataRegs.GPACLEAR.bit.GPIO4 = 1;  // Normally low
	EDIS;

	/************************************************************/
	/* Enable global Interrupts and higher priority real-time   */
	/* debug events:                                            */
	/************************************************************/
	EINT;   // Enable Global interrupt INTM
	ERTM;   // Enable Global realtime interrupt DBGM

	/************************************************************/
	/* Start Timer                                              */
	/************************************************************/
	StartCpuTimer0();

	//qep_posspeed.init(&qep_posspeed);

	/************************************************************/
	/* Step 9. IDLE loop. Just sit and loop forever             */
	/************************************************************/
	while(1)
	{
		/*******************************/
		/* 1ms Timer Interrupt Loop    */
		/*******************************/
		if(FLAG_1ms == 1)
		{
			FLAG_1ms = 0;
			GpioDataRegs.GPATOGGLE.all = 0x0000A000;

			if(EPwm_duty == 0)
			{
				EPwm1Regs.CMPA.half.CMPA = 0;
			}
			else
			{

				EPwm1Regs.CMPA.half.CMPA = (EPwm1Regs.TBPRD * EPwm_duty) / 100;
				/****************************************************************/
				// N = 60 / PPR x Mc x Tclock[rpm]								 /
				// N = 60 / 8000 x QCPRD x 9.375MHz = SpeedRpm					 /
				// SpeedRpm = EqepCapClk / EQep1Regs.QCPRD / EncoderCount * 60;	 /
				/****************************************************************/
				//SpeedRpm = 9.375E6 / EQep1Regs.QCPRD / 8000 * 60;
				//qep_posspeed.calc(&qep_posspeed);

			}
		}
		/*******************************/
		/*  10ms Timer Interrupt Loop  */
		/*******************************/
		if(FLAG_10ms == 1)
		{
			/****************************************************************/
			// N = 60 / PPR x Mc x Tclock[rpm]								 /
			// N = 60 / 8000 x QCPRD x 9.375MHz = SpeedRpm					 /
			// SpeedRpm = EqepCapClk / EQep1Regs.QCPRD / EncoderCount * 60;	 /
			/****************************************************************/
			SpeedRpm = (9.375E6 / EQep1Regs.QCPRD / 8000) * 60;
		}

		/*******************************/
		/* 500ms Timer Interrupt Loop  */
		/*******************************/
		if(FLAG_500ms == 1)
		{
			GpioDataRegs.GPATOGGLE.all = 0x00004000;
		    FLAG_500ms = 0;
		}
	}
}

__interrupt void cpu_timer0_isr(void)
{
	CpuTimer0.InterruptCount++;

	if(uCount_001ms == 9)
	{
		uCount_001ms = 0;
		FLAG_1ms = 1;
		FLAG_1ms_Counter++;
	}
	else
	{
		uCount_001ms++;
	}

	if(uCount_010ms == 99)
	{
		uCount_010ms = 0;
		FLAG_10ms = 1;
		FLAG_10ms_Counter++;
	}
	else
	{
		uCount_010ms++;
	}

	if(uCount_500ms == 4999)
	{
		uCount_500ms = 0;
		FLAG_500ms = 1;
		FLAG_500ms_Counter++;
	}
	else
	{
		uCount_500ms++;
	}

	if(uCount_5s == 49999)
	{
		uCount_5s = 0;
		FLAG_5s = 1;
		FLAG_5s_Counter++;
	}
	else
	{
		uCount_5s++;
	}
   // Acknowledge this interrupt to receive more interrupts from group 1
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

__interrupt void cpu_timer1_isr(void)
{  EALLOW;
   CpuTimer1.InterruptCount++;

   EDIS;
}

__interrupt void cpu_timer2_isr(void)
{  EALLOW;
   CpuTimer2.InterruptCount++;
   // The CPU acknowledges the interrupt.
   EDIS;
}

__interrupt void epwm1_isr(void)
{
	EPWMTimerCount++;
    // Update the CMPA and CMPB values

	// Clear INT flag for this timer
	EPwm1Regs.ETCLR.bit.INT = 1;
	// Acknowledge this interrupt to receive more interrupts from group 3
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

__interrupt void epwm2_isr(void)
{
   // Update the CMPA and CMPB values
   //update_compare(&epwm2_info);

   // Clear INT flag for this timer
   EPwm2Regs.ETCLR.bit.INT = 1;

   // Acknowledge this interrupt to receive more interrupts from group 3
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

__interrupt void epwm3_isr(void)
{
   // Update the CMPA and CMPB values
   //update_compare(&epwm3_info);

   // Clear INT flag for this timer
   EPwm3Regs.ETCLR.bit.INT = 1;

   // Acknowledge this interrupt to receive more interrupts from group 3
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

//===========================================================================
// No more.
//===========================================================================
