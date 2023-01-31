//**********************************************************************************************************
// Based on TMS320C2000 Piccolo™ controlSTICK Examples Modified by Fernando Chandia Reyes for FULL-STEP Hybryd Motor Control. Jan 2013.
//	FILE:	Stepper_ePWM_Main.c
//			(Up, Single Edge Asymmetric Waveform, With Independent Modulation 
//			on EPWM1A, EPWM1B, EPWM2A and EPWM2B)
//
//	Description:	This program sets up the EV TIMER2 to generate complimentary 
//			PWM waveforms. The user can then observe the waveforms using an scope from 
//			ePWMs pins.
//			- In order to change the PWM frequency, the user should change
//			 the value of"period". 
//			- The duty-cycles can independently be adjusted by changing compare 
//			values (duty_cycle_A & duty_cycle_B) for ePWMs.
//			- For further details, please search for the SPRU791.PDF 
//			(TMS320x28xx, 28xxx Enhanced Pulse Width Modulator Module) at ti.com
//
//  Target: TMS320F2802x or TMS320F2803x families (Piccolo)
//
//------------------------------------------------------------------------------------
//  $TI Release:$ 	V1.1
//  $Release Date:$ 26 Oct 2009 - BL
//  $Modified by Fernando Chandia Reyes for FULL-STEP Hybryd Motor Control. Jan 2013.
//------------------------------------------------------------------------------------
//
// PLEASE READ - Useful notes about this Project

// Although this project is made up of several files, the most important ones are:
//	 "AsymmetricPWM .c",	this file
//		- Application Initialization, Peripheral config
//		- Application management
//		- Slower background code loops and Task scheduling
//	 "AsymmetricPWM-DevInit_F28xxx.c"
//		- Device Initialization, e.g. Clock, PLL, WD, GPIO mapping
//		- Peripheral clock enables
// The other files are generally used for support and defining the registers as C
// structs. In general these files will not need to be changed.
//   "F28027_RAM_AsymmetricPWM.CMD" or "F28027_FLASH_AsymmetricPWM.CMD"
//		- Allocates the program and data spaces into the device's memory map.
//   "DSP2802x_Headers_nonBIOS.cmd" and "DSP2802x_GlobalVariableDefs.c"
//		- Allocate the register structs into data memory.  These register structs are
//		  defined in the peripheral header includes (DSP2802x_Adc.h, ...) 
//
//----------------------------------------------------------------------------------

#include "DSP28x_Project.h"
#include "f2802x_epwm_defines.h" 	    // useful defines for initialization
																		 

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// FUNCTION PROTOTYPES
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void DeviceInit(void);
void InitFlash(void);
void MemCopy(Uint16 *SourceAddr, Uint16* SourceEndAddr, Uint16* DestAddr);


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// VARIABLE DECLARATIONS - GENERAL
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

// Used for running BackGround in flash and the ISR in RAM
extern Uint16 RamfuncsLoadStart, RamfuncsLoadEnd, RamfuncsRunStart;
Uint16 duty_cycle=18800;	// Set duty 50% initially

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// MAIN CODE - starts here
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void main(void)
{

//=================================
//	INITIALISATION - General
//=================================

	DeviceInit();	// Device Life support & GPIO mux settings

// Only used if running from FLASH
// Note that the variable FLASH is defined by the compiler (-d FLASH)
#ifdef FLASH		
// Copy time critical code and Flash setup code to RAM
// The  RamfuncsLoadStart, RamfuncsLoadEnd, and RamfuncsRunStart
// symbols are created by the linker. Refer to the linker files. 
	MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);

// Call Flash Initialization to setup flash waitstates
// This function must reside in RAM
	InitFlash();	// Call the flash wrapper init function
#endif //(FLASH)

//-------------------------------------------------------------

#define period 37600							  // 37600 = 100Hz when PLL is set to 0xC (60MHz) and DIV4


  // Time-base registers

	EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV4;
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count-up mode: used for symmetric PWM
   	EPwm1Regs.AQCTLA.all = 0x0006;             // Action Counter
    EPwm1Regs.TBPRD = period;                  // Set timer period, PWM frequency = 1 / period

    EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV4;
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;
    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count-up mode: used for symmetric PWM
    EPwm2Regs.AQCTLA.all = 0x0006;             // Action Counter
    EPwm2Regs.TBPRD = period;                  // Set timer period, PWM frequency = 1 / period

    EPwm1Regs.TBCTL.bit.PRDLD = TB_IMMEDIATE;  // Set Immediate load
    EPwm1Regs.TBCTR = 0;                       // Time-Base Counter Register
    EPwm1Regs.TBPHS.all = 0;                   // Time-Base Phase Register
    EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;

    EPwm2Regs.TBCTL.bit.PHSEN = TB_ENABLE;    // Enable phase loading
    EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
    EPwm2Regs.TBPHS.half.TBPHS = 28200; //Desfase en PW2 respecto a PW1: 9400=90º -> rotacion Contra Reloj, 28200=270º desfase, rotación Horaria.


   	// Setup shadow register load on ZERO

   	EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   	EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   	EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;	// load on CTR=Zero
   	EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;	// load on CTR=Zero


    // Setup shadow register load on ZERO

    EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;   // load on CTR=Zero
    EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;   // load on CTR=Zero


   	// Set Compare values

   	EPwm1Regs.CMPA.half.CMPA = duty_cycle;    // Set duty 50% initially
   	EPwm1Regs.CMPB = duty_cycle;	            // Set duty 50% initially

    // Set Compare values

    EPwm2Regs.CMPA.half.CMPA = duty_cycle;    // Set duty 50% initially
    EPwm2Regs.CMPB = duty_cycle;              // Set duty 50% initially

   	// Set actions

   	EPwm1Regs.AQCTLA.bit.ZRO = AQ_SET;            // Set PWM2A on Zero
   	EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;          // Clear PWM2A on event A, up count

   	EPwm1Regs.AQCTLB.bit.ZRO = AQ_CLEAR;          // Set PWM2B on Zero
   	EPwm1Regs.AQCTLB.bit.CBU = AQ_SET;            // Clear PWM2B on event B, up count

    // Set actions

    EPwm2Regs.AQCTLA.bit.ZRO = AQ_SET;            // Set PWM2A on Zero
    EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;          // Clear PWM2A on event A, up count

    EPwm2Regs.AQCTLB.bit.ZRO = AQ_CLEAR;          // Set PWM2B on Zero
    EPwm2Regs.AQCTLB.bit.CBU = AQ_SET;            // Clear PWM2B on event B, up count

  //=================================
  //	Forever LOOP
  //=================================
  // Just sit and loop forever:
  // No interrups needed in this example.
  // PWM pins can be observed with a scope.	

/*
for(;;)
	{
 	 EPwm1Regs.CMPA.half.CMPA = duty_cycle;      // Add duty_cycle_A to watch window
     EPwm2Regs.CMPA.half.CMPA = duty_cycle;      // Add duty_cycle_A to watch window
     EPwm1Regs.CMPB = duty_cycle;               // Add duty_cycle_B to watch window
     EPwm2Regs.CMPB = duty_cycle;               // Add duty_cycle_B to watch window
	}											  // and change its value to see the effect
*/

}
