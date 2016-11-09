/*
 * This code is a reference design for Mechanical to Electronic converter
 * for flowmeter with 3 LC sensors. An Automatic process is started once
 * the main board is powered up. The main board will then undergo an initialization
 * of Auto TSM setting, and optimize the reference voltage setting for DAC.
 *
 * When the last session of initialization reaches, LCD will display "8888".
 * At this time, user need to switch on the motor board. When the rotor disc rotated,
 * the main board will process the last session of initialization to set the optimal
 * DAC values of Extended ScanIF (ESI). Once finished. ESI will keep on counting the
 * number of rotation and display it onto the LCD.
 *
 * When measuring the current consumption, remove all jumpers of the main board
 * except for the jumper of GND above J401 and tap the current meter onto the jumper 3V3.
 * To measure the current consumption of ESI, without LCD and I2C,
 * press the black button on the main board.
 *
 * To turn on the LCD again, press the black button to toggle it.
 *
 * Texas Instruments
 *
 * Author: Thomas Kot
 * Date  : July 2014
 *
 */


#include "msp430fr6989.h"
#include "LCD.h"
#include "ScanIF.h"
#include "ESI_ESIOSC.h"

#define Time_out  8192    					// 2 sec
#define Time_to_Recal 8192  				// 2 sec for testing, 40960 for 10 sec


extern 	unsigned char  Status_flag ;

char Power_measure = 0;
signed int  rotation_counter = 0;
unsigned char ReCal_Flag ;


void Set_Clock(void);
void Port_Init(void);
void Set_Timer_A(void);
void Check_debug(void);


void Port_Init()
{
/*
 * This is to set the I/O port in the initialization.
 * The power consumption will be 0.8uA in LPM4 after setting.
 *
 */
	P1DIR=0xFF;
	P1OUT=0;

	P1DIR &= ~BIT2;
	P1REN |= BIT2;
	P1OUT |= BIT2;

	P2DIR=0xFF;
	P2OUT=0;
	P3DIR=0xFF;
	P3OUT=0;
	P4DIR=0xFF;
	P4OUT=0;
	P5DIR=0xFF;
	P5OUT=0;
	P6DIR=0xFF;
	P6OUT=0;
	P7DIR=0xFF;
	P7OUT=0;
	P8DIR=0xFF;
	P8OUT=0;
	P9DIR=0x00;
	P9OUT=0;
	P10DIR=0xFF;
	P10OUT=0;
	PJDIR=0xFF;
	PJOUT=0;

	CECTL3 = 0xFF00;

	PM5CTL0 &= ~LOCKLPM5;;    					// In initialization, the I/Os are configured before unlocking the I/O ports
	PMMCTL0_H = 0xA5;
//	PMMCTL0_L |= PMMREGOFF;
	PMMCTL0_L &= ~SVSHE;
	PMMCTL0_H = 0xEE;

}

void Set_Clock(void)
{
// use external 32KHz crystal
// MClock and SMClock from DCO 4MHz


      PJSEL0 |=  BIT4 + BIT5;					// using LFXTCLK
	  CSCTL0 = 0XA500;                          // Write in the password for system clock setting
	  CSCTL1 = 0x0006;                          // 4MHz for DCO
	  CSCTL2 = 0x0033;                          // ACLK from LFXTCLK, SMCLK and MCLK from DCO
	  CSCTL3 = 0x0000;							// ACLK div by 1, SMCLK div by 1, MCLK div by 1
	  CSCTL4 = 0x0148;							// HFXTOFF, LFXTDRIVE = 1, VLO off, SMCLK on

	  	 do
	  	  {
	  	    CSCTL5 &= ~LFXTOFFG;                // Clear XT1 fault flag
	  	    SFRIFG1 &= ~OFIFG;
	  	  }while (SFRIFG1&OFIFG);               // Test oscillator fault flag

}


void Set_Timer_A(void)
{
/*  This is the timer for triggering the run time re-calibration
 *  2 sec is set for demonstration purpose
 *  The actual timer setting depends on the application.
 */

	TA0CTL = TASSEL0 + ID0 + ID1 + TACLR; 		// Aclk divided by 8;
	TA0CCR0 = Time_to_Recal;                    // 2 sec for testing
	TA0CCTL0 = CCIE; 					  		// INT enable, INT flag cleared
	TA0CTL |= MC0;
}


void main(void)
{
	WDTCTL = WDTPW + WDTHOLD;					// disable Watchdog

	Port_Init();
	Set_Clock();

	P1IES |= BIT2;								// Set P1.2 as key input
	P1IFG &= ~BIT2;								// User can press the black button to toggle switch on/off the LCD
	P1IE  |= BIT2;

    P1SEL0 |= BIT6 | BIT7;          			// I2C pins, but not used in this firmware

//	__bis_SR_register(LPM4_bits + GIE);   		// un-remark this instruction. Engineer can measure the current consumption here. 0.7 to 0.8uA

	init_LCD();
	lcd_display_num(0,0);						// Display "0" on the LCD which indicating the rotation number detected from ESI

	ReCal_Flag = 0;                 			// Init the status flag with non-runtime calibration

	EsioscInit(ESIOSC_4MHz);       				// default setting = 4MHz
	InitScanIF();								// Initialization of ScanIf module

	Status_flag |= BIT3;						// indicating Calibration of DAC process completed


#if AFE2_enable
	 Set_Timer_A();                				// set and start timer of 10 sec INT
#endif


 	 ESIINT2 &= ~ESIIFG5;                   	// clear INT flag of Q6 of PSM
 	 ESIINT1 |= ESIIE5;							// enable INT of Q6
 	 ESICTL  |= ESIEN;            				// switch on ESI and will always on till battery drain off

	while(1)
	{

	__bis_SR_register(LPM3_bits+GIE);   		//	 wait for the ESISTOP flag

#if AFE2_enable

	if(ReCal_Flag&BIT7)
	{
	 if(ReCal_Flag&BIT6)
		{ReCal_Flag &= ~BIT7;}              	// Reset Bit7 for timer call

	  TA0CCR0 = Time_out;                   	// 2 sec for testing; generate a time out when stop rotating
	  TA0CTL |= MC0;

	  ReCalScanIF();           					// to do runtime calibration with AFE2

	  TA0CTL &= ~MC0;
	  TA0CTL |= TACLR;                      	// Reset Timer
	  TA0CCR0 = Time_to_Recal;              	// 2 sec for testing
	  TA0CTL |= MC0;							// timer re-start for ReCal.
	  TA0CCTL0 |= CCIE;

	  ReCal_Flag = 0;							// ReCal of AFE1 is done, reset all flags.

	  __bic_SR_register(GIE);					// Ensure no abnormal interrupt before entering LPM;
	  ESIINT2 &= ~ESIIFG5;                  	// clear the Q6 flag
	  ESIINT1 |= ESIIE5;						// Enable Q6 INT for in case of Time out.

	}

#endif

	}
}



#pragma vector=ESCAN_IF_VECTOR
__interrupt void ISR_ESCAN_IF(void)
{

   switch (ESIIV)
   {
   case 0x02:  if (ESIINT1&ESIIE1)

				{	ESIINT2 &= ~ESIIFG1;                 	// clear the ESISTOP flag

					if(ReCal_Flag&BIT6)
					{TA0CTL |= TACLR;                   	// Reset Timer to prevent abnormal time out.
					TA0CCTL0 &= ~CCIFG;	}

					_low_power_mode_off_on_exit();       	// exit low power mode;
				}
			  break;

   case 0x04:  break;
   case 0x06:  break;
   case 0x08:  break;

   case 0x0A: break;
   case 0x0C: if(ESIINT1&ESIIE5)
   	   	   	   	   {    ESIINT2 &= ~ESIIFG5;                // clear the Q6 flag

   	   	   	   	   	   if(ReCal_Flag&BIT6)
						{TA0CTL |= TACLR;                   // Reset Timer to prevent abnormal time out.
						TA0CCTL0 &= ~CCIFG;	}

						if(ReCal_Flag&BIT7)
						{ ReCal_Flag |= BIT6;	}           // to do runtime calibration with AFE2

						if(Status_flag&BIT3)                // Check for completion of Calibration of DAC
						{							    	// If yes, LCD is to display the rotation number

							rotation_counter = ESICNT1;     // for every complete rotation, there are 6 states change and so add +1 six times

							if (rotation_counter < 0)
								{rotation_counter = -1*rotation_counter /6;}
							else
								{rotation_counter = rotation_counter / 6;}

							lcd_display_num(rotation_counter,0);
						}


						_low_power_mode_off_on_exit();       // exit low power mode;
   	   	   	   	   }

	          break;
   case 0x0E: break;
   case 0x10: break;
   case 0x12: break;
   }

}


// Timer A0 interrupt service routine
#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer_A (void)
{
    if (ReCal_Flag&BIT6)
    {
    	ReCal_Flag |= BIT1;                           // Time out
    	ESIINT1 &= ~(ESIIE1+ESIIE5);				  // Prevent entering ESI ISR to call an extra "_low_power_mode_off_on_exit()"
    }
    else
    {
    	ReCal_Flag |= BIT7;                   		  // indicate the need to perform runtime calibration
    }

	TA0CTL &= ~MC0;									  // disable timer
	_low_power_mode_off_on_exit();       	      	  // exit low power mode from ReCal_ScanIF ;
}

// Port 1 interrupt service routine
#pragma vector=PORT1_VECTOR
__interrupt void PORT1_ISR(void)
{
    P1IFG &= ~BIT2;
    Power_measure ^= BIT0;

    if (Power_measure & BIT0)
    {LCDCCTL0 &= ~LCDON;
    ESIINT1 &= ~ESIIE5;								 // toggle INT of Q6
	TA0CTL &= ~MC0;									 // toggle on/off timer
    }
    else
    {LCDCCTL0 |= LCDON;
    ESIINT1 |= ESIIE5;								 // toggle INT of Q6
	TA0CTL |= MC0;									 // toggle on/off timer
    }

   _low_power_mode_off_on_exit();       	      	 // exit low power mode from ReCal_ScanIF ;

}

