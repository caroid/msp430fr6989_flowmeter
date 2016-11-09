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
#include "ScanIF.h"
#include "ESI_ESIOSC.h"
#include "LCD.h"

 // 3 LC sensors PSM table
 const unsigned char Table[] = {
		 128,
		 136,
		 144,
		 152,
		 160,
		 168,
		 176,
		 184,
		 128,
		 8,
		 144,
		 90,
		 160,
		 44,
		 176,
		 184,
		 128,
		 136,
		 16,
		 28,
		 160,
		 168,
		 114,
		 184,
		 128,
		 12,
		 82,
		 24,
		 160,
		 168,
		 176,
		 184,
		 128,
		 136,
		 144,
		 152,
		 32,
		 106,
		 52,
		 184,
		 128,
		 74,
		 144,
		 152,
		 36,
		 40,
		 176,
		 184,
		 128,
		 136,
		 20,
		 152,
		 98,
		 168,
		 48,
		 184,
		 128,
		 136,
		 144,
		 152,
		 160,
		 168,
		 176,
		 184
 };


/*
 * There are three channels of ESI being used for 3 LC sensors.
 * For each channel, there is a dedicated code. The variables and function calls
 * are similar for each channels. To identify the variables for each channel, see
 * the suffix of it.
 *
 * For example, each channel has its own variable to store the noise level:
 * Noise_level_0 for channel 0
 * Noise_level_1 for channel 1
 * Noise_level_2 for channel 2
 *
 */

#define Search_range  8

int AFE1_base0, AFE1_base1, AFE1_base2;
int AFE2_base0, AFE2_base1, AFE2_base2;

int AFE2_drift0 = 0;
int AFE2_drift1 = 0;
int AFE2_drift2 = 0;

int AFE2_base0_Max, AFE2_base0_Min;
int AFE2_base1_Max, AFE2_base1_Min;
int AFE2_base2_Max, AFE2_base2_Min;

#define Separation_factor   4
unsigned char  Status_flag = 0;
unsigned int  STATE_SEPARATION = 0;
unsigned int  STATE_SEPARATION_0 = 0;
unsigned int  STATE_SEPARATION_1 = 0;
unsigned int  STATE_SEPARATION_2 = 0;

unsigned int    Threshold_h2;
unsigned int	Threshold_h1;
unsigned int    Threshold_h0;
unsigned int    Threshold_l2;
unsigned int	Threshold_l1;
unsigned int    Threshold_l0;

unsigned int    Max_DAC_Ch0;
unsigned int    Max_DAC_Ch1;
unsigned int    Max_DAC_Ch2;
unsigned int    Min_DAC_Ch0;
unsigned int    Min_DAC_Ch1;
unsigned int    Min_DAC_Ch2;


#if AFE2_enable
extern unsigned char ReCal_Flag ;
extern signed int  rotation_counter;
#endif


unsigned int Noise_level = 0;
unsigned int Noise_level_0 = 0;
unsigned int Noise_level_1 = 0;
unsigned int Noise_level_2 = 0;


void FindDAC(void);
void FindDAC_Fast_Range(int , int , int, int );
void FindDAC_Fast_Successive(int , int , int, int );
void ReCalScanIF(void);
void FindTESTDAC(void);
void TSM_Auto_cal(void);
void Find_Noise_level(void);
void Set_DAC(void);

void AFE2_FindDAC_Fast_Successive(int , int , int ,  int );
void AFE2_FindDAC_Fast_Range(int , int , int ,  int );
void AFE2_FindDAC(void);

void FindDAC(void)
{
	unsigned int i;
	unsigned int DAC_BIT = 0, Prev_DAC_BIT = 0;

	DAC_BIT   = 0x0800;						// DAC Level tester, using Successive approx approach
	Prev_DAC_BIT = 0x0C00;

	ESIDAC1R0 = DAC_BIT;                 	// set as the middle point for ch0
	ESIDAC1R1 = DAC_BIT;                 	// set as the middle point for ch0

	ESIDAC1R2 = DAC_BIT;                 	// set as the middle point for ch1
	ESIDAC1R3 = DAC_BIT;                 	// set as the middle point for ch1

	ESIDAC1R4 = DAC_BIT;                 	// set as the middle point for ch2
	ESIDAC1R5 = DAC_BIT;                 	// set as the middle point for ch2

	ESIINT2 &= ~ESIIFG1;                	// clear the ESISTOP flag
	ESIINT1 |= ESIIE1;						// enable ESISTOP INT
	ESICTL  |= ESIEN;						// switch on Scan Interface.

// this for loop is to find an initial DAC value for three channels

	for(i = 0; i<12; i++)				 	// test 12 times to find out the 12bit resolution of the signal level
	{

	__bis_SR_register(LPM3_bits+GIE);   	//	 wait for the ESISTOP flag

	DAC_BIT /= 2 ;							// right shift one bit

	if (!(ESIPPU&ESIOUT0))                  // channel 0;
		{

			ESIDAC1R0 |= DAC_BIT;     		// keep the previous bit and set the next bit
			ESIDAC1R1 |= DAC_BIT;			// keep the previous bit and set the next bit

		}
	else
		{
			ESIDAC1R0 ^= Prev_DAC_BIT;		// reset the previous bit and set the next bit
			ESIDAC1R1 ^= Prev_DAC_BIT;		// reset the previous bit and set the next bit
		}

	if (!(ESIPPU&ESIOUT1))					// channel 1;
		{

			ESIDAC1R2 |= DAC_BIT;     		// keep the previous bit and set the next bit
			ESIDAC1R3 |= DAC_BIT;			// keep the previous bit and set the next bit

		}
	else
		{
			ESIDAC1R2 ^= Prev_DAC_BIT;		// reset the previous bit and set the next bit
			ESIDAC1R3 ^= Prev_DAC_BIT;		// reset the previous bit and set the next bit
		}

	if (!(ESIPPU&ESIOUT2))					// channel 2;
		{

			ESIDAC1R4 |= DAC_BIT;     		// keep the previous bit and set the next bit
			ESIDAC1R5 |= DAC_BIT;			// keep the previous bit and set the next bit

		}
	else
		{
			ESIDAC1R4 ^= Prev_DAC_BIT;		// reset the previous bit and set the next bit
			ESIDAC1R5 ^= Prev_DAC_BIT;		// reset the previous bit and set the next bit
		}


	Prev_DAC_BIT /= 2;						// right shift one bit


	}


	ESICTL &= ~ESIEN;						// switch off ESI Interface.
	ESIINT1 &= ~ESIIE1;


}


void FindDAC_Fast_Successive(int Starting_point_ch0, int Starting_point_ch1, int Starting_point_ch2, int Range_num)
{
	unsigned int i;
	unsigned int DAC_BIT;

	DAC_BIT   = 0x0001;						            // DAC Level tester, using Sucessive approx approach

	ESIDAC1R0 = Starting_point_ch0;                 	// set as the middle point
	ESIDAC1R1 = Starting_point_ch0;                 	// set as the middle point

	ESIDAC1R2 = Starting_point_ch1;                 	// set as the middle point
	ESIDAC1R3 = Starting_point_ch1;                 	// set as the middle point

	ESIDAC1R4 = Starting_point_ch2;                 	// set as the middle point
	ESIDAC1R5 = Starting_point_ch2;                 	// set as the middle point

	for(i=1; i<Range_num; i++)							// Range_num set the number of least significant bits are used
	{

	   DAC_BIT <<= 1;

	}


	ESIINT2 &= ~ESIIFG1;                				// clear the ESISTOP flag
	ESIINT1 |= ESIIE1;									// enable ESISTOP INT
	ESICTL  |= ESIEN;									// switch on Scan Interface.

// this for loop is to find an initial DAC value for three channels

	for(i = 0; i<Range_num; i++)				 		// test "Range_num" times to find out the signal level of 12 bits resolution
	{

	__bis_SR_register(LPM3_bits+GIE);   				// wait for the ESISTOP flag


	if (!(ESIPPU&ESIOUT0))                     			// channel 0;
		{

			ESIDAC1R0 += DAC_BIT;
			ESIDAC1R1 += DAC_BIT;

		}
	else
		{
			ESIDAC1R0 -= DAC_BIT;
			ESIDAC1R1 -= DAC_BIT;
		}

	if (!(ESIPPU&ESIOUT1))								// channel 1;
		{

			ESIDAC1R2 += DAC_BIT;
			ESIDAC1R3 += DAC_BIT;

		}
	else
		{
			ESIDAC1R2 -= DAC_BIT;
			ESIDAC1R3 -= DAC_BIT;
		}

	if (!(ESIPPU&ESIOUT2))                     			// channel 2;
		{

			ESIDAC1R4 += DAC_BIT;
			ESIDAC1R5 += DAC_BIT;

		}
	else
		{
			ESIDAC1R4 -= DAC_BIT;
			ESIDAC1R5 -= DAC_BIT;
		}


		DAC_BIT /= 2;									// right shift one bit

	}


		ESICTL &= ~ESIEN;								// switch off ESI Interface.
		ESIINT1 &= ~ESIIE1;

}



void FindDAC_Fast_Range(int Starting_point_ch0, int Starting_point_ch1, int Starting_point_ch2, int Range_num)
{

unsigned int Range;
unsigned int Range_status;


	ESIDAC1R0 = Starting_point_ch0;             // set DAC from the starting point
	ESIDAC1R1 = Starting_point_ch0;            	// set DAC from the starting point

	ESIDAC1R2 = Starting_point_ch1;             // set DAC from the starting point
	ESIDAC1R3 = Starting_point_ch1;            	// set DAC from the starting point

	ESIDAC1R4 = Starting_point_ch2;             // set DAC from the starting point
	ESIDAC1R5 = Starting_point_ch2;            	// set DAC from the starting point

	Range_status = 0;
	Range_status |= BIT7 ;                      // This is the searching Loop enable bit.

	Range = Range_num;

	ESIINT2 &= ~ESIIFG1;                		// clear the ESISTOP flag
	ESIINT1 |= ESIIE1;							// enable ESISTOP INT
	ESICTL  |= ESIEN;							// switch on Scan Interface.


// This loop is to find the starting point of DAC and the direction of searching by adding or subtracting a value of "Range" into or from the DAC
// Range_status is to show the direction of searching
// BIT0   indication of a searching direction of increasing for channel 0;
// BIT1   indication of a searching direction of decreasing for channel 0;
// BIT2   indication of a searching direction of increasing for channel 1;
// BIT3   indication of a searching direction of decreasing for channel 1;
// BIT8   indication of a searching direction of increasing for channel 2;
// BIT9   indication of a searching direction of decreasing for channel 2;
// BIT4   indication of completion of the loop for channel 0;
// BIT5   indication of completion of the loop for channel 1;
// BIT6   indication of completion of the loop for channel 2;

while (Range_status&BIT7)
{
		while(!((Range_status&BIT4) && (Range_status&BIT5) && (Range_status&BIT6)))
		{

					__bis_SR_register(LPM3_bits+GIE);   	//	 wait for the ESISTOP flag



				   if(!(Range_status&BIT4))
				   {
					if (ESIPPU&ESIOUT0)                     // channel 0;
						{

							if(Range_status&BIT1)			// check if there is a decreasing direction mark of channel 0.
							{ Range_status |= BIT4;}        // if yes, put a completion mark for channel 0
							else
							{ Range_status |= BIT0;}        // if no, put a direction mark pointing to increasing for channel 0.

							ESIDAC1R0 -= Range;     		// decrease the DAC by a value of "Range"
							ESIDAC1R1 -= Range;

						}
					else
						{

							if(Range_status&BIT0)			// check if there is a increasing direction mark of channel 0.
							{ Range_status |= BIT4;}        // if yes, put a completion mark for channel 0
							else
							{ Range_status |= BIT1;}        // if no, put a direction mark pointing to decreasing for channel 0.

							ESIDAC1R0 += Range;				// increase the DAC by a value of "Range"
							ESIDAC1R1 += Range;
						}
				   }


				  if(!(Range_status&BIT5))
				  {
					if (ESIPPU&ESIOUT1)						// channel 1;
						{

							if(Range_status&BIT3)			// check if there is a decreasing direction mark of channel 1.
							{ Range_status |= BIT5;}        // if yes, put a completion mark for channel 1
							else
							{ Range_status |= BIT2;}        // if no, put a direction mark pointing to increasing for channel 1.

							ESIDAC1R2 -= Range;     		// decrease the DAC by a value of "Range"
							ESIDAC1R3 -= Range;

						}
					else
						{

							if(Range_status&BIT2)			// check if there is a increasing direction mark of channel 1.
							{ Range_status |= BIT5;}        // if yes, put a completion mark for channel 1
							else
							{ Range_status |= BIT3;}        // if no, put a direction mark pointing to decreasing for channel 1.

							ESIDAC1R2 += Range;				// increase the DAC by a value of "Range"
							ESIDAC1R3 += Range;
						}
				  }


				   if(!(Range_status&BIT6))                 // The third sensor
				   {
					if (ESIPPU&ESIOUT2)                     // channel 2;
						{

							if(Range_status&BIT9)			// check if there is a decreasing direction mark of channel 2.
							{ Range_status |= BIT6;}        // if yes, put a completion mark for channel 2
							else
							{ Range_status |= BIT8;}        // if no, put a direction mark pointing to increasing for channel 2.

							ESIDAC1R4 -= Range;     		// decrease the DAC by a value of "Range"
							ESIDAC1R5 -= Range;

						}
					else
						{

							if(Range_status&BIT8)			// check if there is a increasing direction mark of channel 2.
							{ Range_status |= BIT6;}        // if yes, put a completion mark for channel 2
							else
							{ Range_status |= BIT9;}        // if no, put a direction mark pointing to decreasing for channel 2.

							ESIDAC1R4 += Range;				// increase the DAC by a value of "Range"
							ESIDAC1R5 += Range;
						}
				   }
		}

	if (Range > 1)
	{	Range_status &= ~(BIT4+BIT5+BIT6);            			// clear the flag of completion
		Range_status &= ~(BIT0+BIT1+BIT2+BIT3+BIT8+BIT9);					// optional, clear the direction flag
		Range = 1;                                 				// This is to restart the loop with searching value of one, by "+/- 1 method"
	}
	else
	{   Range_status &= ~BIT7;}									// "+/- 1 method completed, the loop end here

}


ESICTL &= ~ESIEN;												// switch off ESI Interface.
ESIINT1 &= ~ESIIE1;

}


void TSM_Auto_cal(void)
{
// constant and variable for TSM calibration
#define cycle_width 6      										// which is equal to (ESICLK / freq of LC) - 2
#define LC_Threshold_TSM_CAL 1600							    // which is the DAC level for searching the peak of LC oscillation signal
#define Ch0_finish  BIT0
#define Ch1_finish  BIT1
#define Ch2_finish  BIT2

char	 Ch0_counter = 0;
char	 Ch1_counter = 0;
char     Ch2_counter = 0;
char     Cal_status = 0;

int  DAC0_sum1,DAC0_sum2;
int  DAC1_sum1,DAC1_sum2;
int  DAC2_sum1,DAC2_sum2;
int  math_temp;


unsigned int i;

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// This module is to find the optimal timing for maximum noise margin between two peaks of the LC signal.
// Using this particular timing point, it will trigger the latch-in for the ESI comparator output.
//
// The algorithm used is to convert the LC signal with the shape of decaying sine wave into a stair-case like signal.
// The conversion is done by using a moving window and record down the maximum signal level within it.
// Each peak of the LC signal will then give out a signal level for a Tread in the stair-case.
// The optimal timing is at the point in the middle of a Tread.


			 DAC0_sum1 = DAC0_sum2 = 0;
			 DAC1_sum1 = DAC1_sum2 = 0;
			 DAC2_sum1 = DAC2_sum2 = 0;

			 Ch0_counter = 0;
			 Ch1_counter = 0;
			 Ch2_counter = 0;



	do
	{

				FindDAC();                                 	    // using 12 bit sucessive approx. method

				DAC0_sum2 = ESIDAC1R0;
				DAC1_sum2 = ESIDAC1R2;
				DAC2_sum2 = ESIDAC1R4;



	// channel 0
			 if(!(Cal_status&Ch0_finish))
			 {
					 if (DAC0_sum2 > LC_Threshold_TSM_CAL)
					 {
					 if (DAC0_sum2 > DAC0_sum1)
						 {math_temp = DAC0_sum2 - DAC0_sum1;}
					 else
						 {math_temp = DAC0_sum1 - DAC0_sum2;}

					  Ch0_counter += 1;

								 if (math_temp > 12)
								 {
									 if (Ch0_counter > cycle_width)
									 {
										 for (i= 0; i< Ch0_counter / 2 ; i++)
										 {	ESITSM3 -= 0x0800;	 }

										 Cal_status |= Ch0_finish;
									 }

									 else
									 {Ch0_counter = 0;
									  DAC0_sum1 = DAC0_sum2;
									 }
								 }
								 else
								 {
									 DAC0_sum1 = DAC0_sum2;
								 }
					 }



							if(!(Cal_status&Ch0_finish))
							{
												if (!((ESITSM3&0xF800) == 0xF800)) { ESITSM3 += 0x0800 ;}
										   else if (!((ESITSM4&0xF800) == 0xF800)) { ESITSM4 += 0x0800 ;}
										   else if (!((ESITSM5&0xF800) == 0xF800)) { ESITSM5 += 0x0800 ;}
										   else if (!((ESITSM6&0xF800) == 0xF800)) { ESITSM6 += 0x0800 ;}
										   else    {
											   	 ESITSM2 += 0x0800;
												 ESITSM3 &= 0X07FF;
												 ESITSM4 &= 0X07FF;
												 ESITSM5 &= 0X07FF;
												 ESITSM6 &= 0X07FF;

												 DAC0_sum1 = DAC0_sum2 = 0;
										         Ch0_counter = 0;
											 	   }


							 }
			  }

	// channel 1


			 if(!(Cal_status&Ch1_finish))
			 {
				if (DAC1_sum2 > LC_Threshold_TSM_CAL)
				{
				 if (DAC1_sum2 > DAC1_sum1)
					 {math_temp = DAC1_sum2 - DAC1_sum1;}
				 else
					 {math_temp = DAC1_sum1 - DAC1_sum2;}

				 Ch1_counter += 1;

				 if (math_temp > 12)
				 {
					 if (Ch1_counter > cycle_width)
					 {
						 for (i= 0; i< Ch1_counter / 2  ; i++)
						 {	ESITSM13 -= 0x0800;	 }

						 Cal_status |= Ch1_finish;
					 }
					 else
					 {Ch1_counter = 0;
					  DAC1_sum1 = DAC1_sum2;
					 }
				 }
				 else
				 {
					 DAC1_sum1 = DAC1_sum2;
				 }
				}

					   if(!(Cal_status&Ch1_finish))
					   {


								if (!((ESITSM13&0xF800) == 0xF800)) { ESITSM13 += 0x0800 ;}
						   else if (!((ESITSM14&0xF800) == 0xF800)) { ESITSM14 += 0x0800 ;}
						   else if (!((ESITSM15&0xF800) == 0xF800)) { ESITSM15 += 0x0800 ;}
						   else if (!((ESITSM16&0xF800) == 0xF800)) { ESITSM16 += 0x0800 ;}
						   else    {
								 ESITSM12 += 0x0800;
								 ESITSM13 &= 0X07FF;
								 ESITSM14 &= 0X07FF;
								 ESITSM15 &= 0X07FF;
								 ESITSM16 &= 0X07FF;

								 DAC1_sum1 = DAC1_sum2 = 0;
								 Ch1_counter = 0;
								   }

					   }
			 }


	// channel 2


			 if(!(Cal_status&Ch2_finish))
			 {
				if (DAC2_sum2 > LC_Threshold_TSM_CAL)
				{
				 if (DAC2_sum2 > DAC2_sum1)
					 {math_temp = DAC2_sum2 - DAC2_sum1;}
				 else
					 {math_temp = DAC2_sum1 - DAC2_sum2;}

				 Ch2_counter += 1;

				 if (math_temp > 12)
				 {
					 if (Ch2_counter > cycle_width)
					 {
						 for (i= 0; i< Ch2_counter / 2  ; i++)
						 {	ESITSM23 -= 0x0800;	 }

						 Cal_status |= Ch2_finish;
					 }
					 else
					 {Ch2_counter = 0;
					  DAC2_sum1 = DAC2_sum2;
					 }
				 }
				 else
				 {
					 DAC2_sum1 = DAC2_sum2;
				 }
				}

					   if(!(Cal_status&Ch2_finish))
					   {


								if (!((ESITSM23&0xF800) == 0xF800)) { ESITSM23 += 0x0800 ;}
						   else if (!((ESITSM24&0xF800) == 0xF800)) { ESITSM24 += 0x0800 ;}
						   else if (!((ESITSM25&0xF800) == 0xF800)) { ESITSM25 += 0x0800 ;}
						   else if (!((ESITSM26&0xF800) == 0xF800)) { ESITSM26 += 0x0800 ;}
						   else    {
								 ESITSM22 += 0x0800;
								 ESITSM23 &= 0X07FF;
								 ESITSM24 &= 0X07FF;
								 ESITSM25 &= 0X07FF;
								 ESITSM26 &= 0X07FF;

								 DAC2_sum1 = DAC2_sum2 = 0;
								 Ch2_counter = 0;
								   }

					   }
			 }


	}while(!(Cal_status==(Ch0_finish+Ch1_finish+Ch2_finish)))	;


// TSM Calibration competed

}



void Find_Noise_level(void)
{

unsigned int Loop_counter = 0;

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// To find the noise level of each channels

	   Min_DAC_Ch0 = 0x0FFF;					// set initial value for DAC max and min
	   Min_DAC_Ch1 = 0x0FFF; 					// this variable will record the DAC value of metal and non metal part of a rotor
	   Min_DAC_Ch2 = 0x0FFF;
	   Max_DAC_Ch0 = 0x0000;
	   Max_DAC_Ch1 = 0x0000;
	   Max_DAC_Ch2 = 0x0000;


	do {  										// do loop for detection of noise level, taking 0.5 second;

	FindDAC_Fast_Range(ESIDAC1R0, ESIDAC1R2, ESIDAC1R4, Search_range);


	if ( ESIDAC1R1 < Min_DAC_Ch0 ) {Min_DAC_Ch0 = ESIDAC1R1 ;}
	if ( ESIDAC1R0 > Max_DAC_Ch0 ) {Max_DAC_Ch0 = ESIDAC1R0 ;}
	if ( ESIDAC1R3 < Min_DAC_Ch1 ) {Min_DAC_Ch1 = ESIDAC1R3 ;}
	if ( ESIDAC1R2 > Max_DAC_Ch1 ) {Max_DAC_Ch1 = ESIDAC1R2 ;}
	if ( ESIDAC1R5 < Min_DAC_Ch2 ) {Min_DAC_Ch2 = ESIDAC1R5 ;}
	if ( ESIDAC1R4 > Max_DAC_Ch2 ) {Max_DAC_Ch2 = ESIDAC1R4 ;}

	Loop_counter++;

	}while (Loop_counter < 234);



		 Threshold_h0 = Max_DAC_Ch0 - Min_DAC_Ch0;
		 Threshold_h1 = Max_DAC_Ch1 - Min_DAC_Ch1;
		 Threshold_h2 = Max_DAC_Ch2 - Min_DAC_Ch2;

		 Noise_level_0 = Threshold_h0;
		 Noise_level_1 = Threshold_h1;
		 Noise_level_2 = Threshold_h2;

}


void Set_DAC(void)
{
unsigned int Loop_counter = 0;
	
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// when the disc is rotating, the Max and Min of signal is found and their difference is required to be larger than STATE_SEPARATION.
// After reaching the STATE_SEPARATION, it will keep rotating for one more second to ensure a complete rotation is calibrated.


 Min_DAC_Ch0 = 4096;
 Max_DAC_Ch0 = 0;
 Min_DAC_Ch1 = 4096;
 Max_DAC_Ch1 = 0;
 Min_DAC_Ch2 = 4096;
 Max_DAC_Ch2 = 0;

	// STATE_SEPARATION = Noise_level*(Separation_factor-1)+Noise_level/2;
	 STATE_SEPARATION_0 = Noise_level_0*(Separation_factor-1)+Noise_level_0/2;
	 STATE_SEPARATION_1 = Noise_level_1*(Separation_factor-1)+Noise_level_1/2;
	 STATE_SEPARATION_2 = Noise_level_2*(Separation_factor-1)+Noise_level_2/2;

	 Loop_counter = 0;

	do {  // do loop for 1 more second after valid separation detected;
	do {  // do loop for detection of valid Max-Min separation;

		FindDAC_Fast_Successive(ESIDAC1R0, ESIDAC1R2, ESIDAC1R4, 5);

	if ( ESIDAC1R1 < Min_DAC_Ch0 ) {Min_DAC_Ch0 = ESIDAC1R1 ;}
	if ( ESIDAC1R0 > Max_DAC_Ch0 ) {Max_DAC_Ch0 = ESIDAC1R0 ;}
	if ( ESIDAC1R3 < Min_DAC_Ch1 ) {Min_DAC_Ch1 = ESIDAC1R3 ;}
	if ( ESIDAC1R2 > Max_DAC_Ch1 ) {Max_DAC_Ch1 = ESIDAC1R2 ;}
	if ( ESIDAC1R5 < Min_DAC_Ch2 ) {Min_DAC_Ch2 = ESIDAC1R5 ;}
	if ( ESIDAC1R4 > Max_DAC_Ch2 ) {Max_DAC_Ch2 = ESIDAC1R4 ;}

	// To detect the a change due to rotation
	// if a separation of STATE_SEPARATION is found, this will imply that a rotation is detected
	// Keep running for 1 second to make sure the max and min of signal are found

	 Threshold_h0 = Max_DAC_Ch0 - Min_DAC_Ch0;
	 Threshold_h1 = Max_DAC_Ch1 - Min_DAC_Ch1;
	 Threshold_h2 = Max_DAC_Ch2 - Min_DAC_Ch2;

	 if (Threshold_h0 > STATE_SEPARATION_0) { Status_flag |= BIT0;}          // check for valid separation
	 if (Threshold_h1 > STATE_SEPARATION_1) { Status_flag |= BIT1;}
	 if (Threshold_h2 > STATE_SEPARATION_2) { Status_flag |= BIT2;}

	}while (!((Status_flag&BIT0)&&(Status_flag&BIT1)&&(Status_flag&BIT2)));


	Loop_counter++;
} while(Loop_counter < 468)   ;   											// 1 second for 2340Hz using Find_Fast_Successive();


	 ESIDAC1R0 = (Max_DAC_Ch0 + Min_DAC_Ch0)/2;
	 ESIDAC1R1 = ESIDAC1R0 + Noise_level_0;              					// "+" for INV version, "-" for non-INV version
	 ESIDAC1R2 = (Max_DAC_Ch1 + Min_DAC_Ch1)/2;
	 ESIDAC1R3 = ESIDAC1R2 + Noise_level_1;              					// "+" for INV version, "-" for non-INV version
	 ESIDAC1R4 = (Max_DAC_Ch2 + Min_DAC_Ch2)/2;
	 ESIDAC1R5 = ESIDAC1R4 + Noise_level_2;              					// "+" for INV version, "-" for non-INV version


	 AFE1_base0 = ESIDAC1R0;
	 AFE1_base1 = ESIDAC1R2;
	 AFE1_base2 = ESIDAC1R4;



	 ESIDAC1R0 -= Noise_level_0;                          					// "-" for INV version, "+" for non-INV version
	 ESIDAC1R2 -= Noise_level_1;											// "-" for INV version, "+" for non-INV version
	 ESIDAC1R4 -= Noise_level_2;											// "-" for INV version, "+" for non-INV version

}


void InitScanIF(void)
{
	unsigned int i;
	volatile unsigned char*  PsmRamPointer;

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	 Status_flag = 0;

//  Port pin selection for ESI, all channels for ESI should be selected, even only two channels are used.
//  The un-used pin should be floating, not be connected to any other circuit.

	P9SEL1 |= BIT0 + BIT1 + BIT2 + BIT3;
	P9SEL0 |= BIT0 + BIT1 + BIT2 + BIT3;            						// Select mux for ESI function


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//	ESI control registers setting


	ESIAFE = ESIVCC2 + ESICA1INV + ESITEN;            	                                                    // AVCC/2 enable, Excitation enable
    ESITSM = ESITSMTRG1 + ESITSMTRG0 + ESIDIV3A2; 			                                                // 1820Hz sampling rate
	ESIPSM = ESICNT2RST +ESICNT1RST + ESICNT0RST + ESICNT2EN +ESICNT1EN +ESICNT0EN;							// ALL counters reset to zero, output TSM clock signal, enable all counters // The third sensor
	ESICTL = ESIS3SEL1 + ESIS2SEL0 + ESITCH10 + ESICS ;    													// ESIOUT2 AS PPUS3 source, OUT1 for PPUS2, OUT0 for PPUS1, no test cycle, ESI not enable yet

//	__delay_cycles(1000);

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

// TSM Setting for 3 sensors
	ESITSM0 = 0x0400;     // sync with Aclk
	ESITSM1 = 0x182C;     // excitation of Ch0 for 1us
	ESITSM2 = 0x0404;     // 1 Aclk delay
	ESITSM3 = 0x0024;     // tunable delay
	ESITSM4 = 0x0024;     // tunable delay
	ESITSM5 = 0x0024;     // tunable delay
	ESITSM6 = 0x0024;     // tunable delay
	ESITSM7 = 0xC934;     // DAC on, CA on, for 26 TSM clks
	ESITSM8 = 0x4974;     // DAC on, Ca on, and latches enable, for 10 TSM clks
	ESITSM9 = 0x0400;     // internally shorted for channel 0 LC sensor, for 1 Aclk
	ESITSM10 = 0x0400;    // internally shorted for channel 0 LC sensor, for 1 Aclk
	ESITSM11 = 0x18AD;    // excitation of Ch1 for 1us
	ESITSM12 = 0x0485;    // 1 Aclk delay
	ESITSM13 = 0x00A5;    // tunable delay
	ESITSM14 = 0x00A5;    // tunable delay
	ESITSM15 = 0x00A5;    // tunable delay
	ESITSM16 = 0x00A5;    // tunable delay
	ESITSM17 = 0xC9B5;    // DAC on, CA on, for 26 TSM clks
	ESITSM18 = 0x49F5;    // DAC on, Ca on, and latches enable, for 10 TSM clks
	ESITSM19 = 0x0401;    // internally shorted for channel 1 LC sensor, for 1 Aclk
	ESITSM20 = 0x0401;    // internally shorted for channel 1 LC sensor, for 1 Aclk
	ESITSM21 = 0x182E;    // excitation of Ch2 for 1us
	ESITSM22 = 0x0406;    // 1 Aclk delay
	ESITSM23 = 0x0026;    // tunable delay
	ESITSM24 = 0x0026;    // tunable delay
	ESITSM25 = 0x0026;    // tunable delay
	ESITSM26 = 0x0026;    // tunable delay
	ESITSM27 = 0xC936;    // DAC on, CA on, for 26 TSM clks
	ESITSM28 = 0x4976;    // DAC on, Ca on, and latches enable, for 10 TSM clks
	ESITSM29 = 0x0202;    // Stop TSM

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

// Fill in ESIRAM TABLE for PSM

	PsmRamPointer = &ESIRAM0;

	for (i=0; i<64; i++)
	{
		*PsmRamPointer = Table[i];
		 PsmRamPointer +=1  ;
	}


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	TSM_Auto_cal();						// To find the optimal timing to latch-in of the comparator output
	Find_Noise_level();					// To find the signal noise level of each channels.

	lcd_display_num(8888,0);           	// "8888" on LCD indicating the completion of TSM calibration.

	Set_DAC();							// noise level found.
										// User need to switch on the motor with half-covered metal disc to finish the calibration

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//  The Initialization completed.
//	Set the ESI control registers for normal operation


	 ESIAFE = ESIVCC2 + ESICA1INV + ESITEN;             													// disable AFE2;
	 ESITSM = ESITSMTRG1 + ESITSMTRG0 + ESIDIV3B1  + ESIDIV3A1;												// ACLK divider trig for TSM sequence, Div by 50 (655Hz sampling rate)
	 ESIPSM = ESICNT2RST +ESICNT1RST + ESICNT0RST +ESICNT2EN +ESICNT1EN +ESICNT0EN;				            // ALL counters reset to zero, output TSM clock signal, enable all counters


#if AFE2_enable
	 ReCal_Flag |= BIT5;          			// indication for a call from InitScanIF

 	 ESIINT2 &= ~ESIIFG5;                   // clear INT flag of Q6 of PSM
 	 ESIINT1 |= ESIIE5;						// enable INT of Q6
 	 ESICTL  |= ESIEN;

 	__bis_SR_register(LPM3_bits+GIE);   	// wait for the ESISTOP flag

	 ReCalScanIF();                			// to find the init AFE2_base

	 ESICTL &= ~ESIEN;
	 ReCal_Flag = 0;

#endif



}



#if AFE2_enable
void ReCalScanIF(void)
{

#define delta_level 10
int New_level;
int Delta;

unsigned int Loop_counter = 0;
unsigned char Sensor_state;

int	AFE2_Min_DAC_Ch0 ;						//  value for DAC max and min
int	AFE2_Min_DAC_Ch1 ;
int	AFE2_Min_DAC_Ch2 ;
int	AFE2_Max_DAC_Ch0 ;
int	AFE2_Max_DAC_Ch1 ;
int	AFE2_Max_DAC_Ch2 ;

	ESIAFE = ESIDAC2EN + ESICA2EN + ESICA1INV + ESICA2INV + ESIVCC2 + ESITEN;            	// AVCC/2 enable, Excitation enable
	ESITSM = ESITSMTRG1 + ESITSMTRG0 + ESIDIV3A2;                                           // 1820Hz sampling rate


	AFE2_Min_DAC_Ch0 = 0 ;
	AFE2_Max_DAC_Ch0 = 0 ;
	AFE2_Min_DAC_Ch1 = 0 ;
	AFE2_Max_DAC_Ch1 = 0 ;
	AFE2_Min_DAC_Ch2 = 0 ;
	AFE2_Max_DAC_Ch2 = 0 ;

	ESIINT2 &= ~ESIIFG1;
	ESIINT1 |= ESIIE1;

do {

	if(ReCal_Flag&BIT6) {AFE2_FindDAC_Fast_Successive(AFE2_base0 + AFE2_drift0, AFE2_base1 + AFE2_drift1, AFE2_base2 + AFE2_drift2, 5);}
	else                {AFE2_FindDAC_Fast_Successive(AFE1_base0, AFE1_base1, AFE1_base2, 5);}


	TA0CCTL0 &= ~CCIE;

	Loop_counter++;

	if(ReCal_Flag&BIT6)
	{
		if (ReCal_Flag&BIT7)
		{
			AFE2_Min_DAC_Ch0 = 0 ;
			AFE2_Max_DAC_Ch0 = 0 ;
			AFE2_Min_DAC_Ch1 = 0 ;
			AFE2_Max_DAC_Ch1 = 0 ;
			AFE2_Min_DAC_Ch2 = 0 ;
			AFE2_Max_DAC_Ch2 = 0 ;

		}

		ReCal_Flag &= ~BIT7;

		// for three sensors only
				Sensor_state = (char)(ESIPPU&0x0007);
				switch(Sensor_state)
				{

				case 0x01: {
								AFE2_Max_DAC_Ch0 += ESIDAC2R0;
							}
							break;

				case 0x02: {
								AFE2_Max_DAC_Ch1 += ESIDAC2R2;
							}
							break;

				case 0x03: 	{
								AFE2_Min_DAC_Ch2 += ESIDAC2R5;
							}
							break;

				case 0x04: {
								AFE2_Max_DAC_Ch2 += ESIDAC2R4;
							}
							break;

				case 0x05: {
								AFE2_Min_DAC_Ch1 += ESIDAC2R3;
							}
							break;

				case 0x06: {
								AFE2_Min_DAC_Ch0 += ESIDAC2R1;
							}
							break;

				}

		////////////////////////////////////////////////////////////////////////
		 ESIINT1 &= ~ESIIE1;


		if (Loop_counter == 24)															   // 4 cycles
				{
				  ReCal_Flag |= BIT0;                                              	       // indication of valid calibration
				  break;
				}

		__bis_SR_register(LPM3_bits+GIE);   											   // wait for the Q6 flag

		////////////////////////////////////////////////////////////////////////


	}
	else if (ReCal_Flag&BIT5)
	{
		if (ReCal_Flag&BIT7)
		{
			AFE2_Min_DAC_Ch0 = 0 ;
			AFE2_Max_DAC_Ch0 = 0 ;
			AFE2_Min_DAC_Ch1 = 0 ;
			AFE2_Max_DAC_Ch1 = 0 ;
			AFE2_Min_DAC_Ch2 = 0 ;
			AFE2_Max_DAC_Ch2 = 0 ;

		}

		ReCal_Flag &= ~BIT7;
		// for three sensors only
				Sensor_state = (char)(ESIPPU&0x0007);
				switch(Sensor_state)
				{

				case 0x01: {
								AFE2_Max_DAC_Ch0 += ESIDAC2R0;
							}
							break;

				case 0x02: {
								AFE2_Max_DAC_Ch1 += ESIDAC2R2;
							}
							break;

				case 0x03: 	{
								AFE2_Min_DAC_Ch2 += ESIDAC2R5;
							}
							break;

				case 0x04: {
								AFE2_Max_DAC_Ch2 += ESIDAC2R4;
							}
							break;

				case 0x05: {
								AFE2_Min_DAC_Ch1 += ESIDAC2R3;
							}
							break;

				case 0x06: {
								AFE2_Min_DAC_Ch0 += ESIDAC2R1;
							}
							break;

				}


		 ESIINT1 &= ~ESIIE1;


		if (Loop_counter == 48)															   // 8 cycles
				{
				  ReCal_Flag |= BIT0;                                              	       // indication of valid calibration
				  break;
				}

		__bis_SR_register(LPM3_bits+GIE);   											   // wait for the Q6 flag

	}
	else if (ReCal_Flag&BIT7)
	{

		Sensor_state = (char)(ESIPPU&0x0007);
		switch(Sensor_state)
		{

		case 0x01: {
						AFE2_Max_DAC_Ch0 += ESIDAC2R0;
					}
					break;

		case 0x02: {
						AFE2_Max_DAC_Ch1 += ESIDAC2R2;
					}
					break;

		case 0x03: 	{
						AFE2_Min_DAC_Ch2 += ESIDAC2R5;
					}
					break;

		case 0x04: {
						AFE2_Max_DAC_Ch2 += ESIDAC2R4;
					}
					break;

		case 0x05: {
						AFE2_Min_DAC_Ch1 += ESIDAC2R3;
					}
					break;

		case 0x06: {
						AFE2_Min_DAC_Ch0 += ESIDAC2R1;
					}
					break;

		}

		if (Loop_counter == 4)															   // take 4 readings
			{
			  ReCal_Flag |= BIT0;                                              	           // indication of valid calibration
			  break;
			}


	}


	if(ReCal_Flag&BIT1)																	   // inidcation of time out from Timer A
	{   __no_operation();

		break;}

	  ESIINT2 &= ~ESIIFG1;
	  ESIINT1 |= ESIIE1;


	  TA0CCTL0 |= CCIE;

}while(Loop_counter < 54)   ;

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// if BIT0 of ReCal_Flag is set, Max & Min of two channels found. set the ESIDAC with noise margin and AFE2_offset
// otherwise, it will be time out for re-calibration when Loop_counter is 150 and no data for AFE1



if (ReCal_Flag&BIT0)
{
	if(ReCal_Flag&BIT6)
	{

	   AFE2_Max_DAC_Ch0 /= 4;
	   AFE2_Min_DAC_Ch0 /= 4;
	   AFE2_Max_DAC_Ch1 /= 4;
	   AFE2_Min_DAC_Ch1 /= 4;
	   AFE2_Max_DAC_Ch2 /= 4;
	   AFE2_Min_DAC_Ch2 /= 4;

	   AFE2_drift0 = (AFE2_Max_DAC_Ch0 + AFE2_Min_DAC_Ch0)/2 - AFE2_base0;

	   New_level  = AFE1_base0 + AFE2_drift0;

	   Delta = (ESIDAC1R0+ESIDAC1R1)/2 - New_level;

	   if (abs(Delta) < delta_level )
	   {
	   ESIDAC1R0 = New_level - Noise_level_0;                 // Noise_level, "-" for INV version, "+" for non-INV version
	   ESIDAC1R1 = New_level + Noise_level_0;				  // Noise_level, "+" for INV version, "-" for non-INV version
	   }

	   AFE2_drift1 = (AFE2_Max_DAC_Ch1 + AFE2_Min_DAC_Ch1)/2 - AFE2_base1;

	   New_level  = AFE1_base1 + AFE2_drift1;

	   Delta = (ESIDAC1R2+ESIDAC1R3)/2 - New_level;

	   if (abs(Delta) < delta_level )
	   {
	   ESIDAC1R2 = New_level - Noise_level_1;               // Noise_level, "-" for INV version, "+" for non-INV version
	   ESIDAC1R3 = New_level + Noise_level_1;               // Noise_level, "+" for INV version, "-" for non-INV version
	   }

	   AFE2_drift2 = (AFE2_Max_DAC_Ch2 + AFE2_Min_DAC_Ch2)/2 - AFE2_base2;

	   New_level  = AFE1_base2 + AFE2_drift2;

	   Delta = (ESIDAC1R4+ESIDAC1R5)/2 - New_level;

	   if (abs(Delta) < delta_level )
	   {
	   ESIDAC1R4 = New_level - Noise_level_2;               // Noise_level, "-" for INV version, "+" for non-INV version
	   ESIDAC1R5 = New_level + Noise_level_2;				// Noise_level, "+" for INV version, "-" for non-INV version
	   }

	}
	else if(ReCal_Flag&BIT5)                                // call from InitScanIF only to get AFE2 base value
			{

				AFE2_base0_Max = AFE2_Max_DAC_Ch0/8;
				AFE2_base0_Min = AFE2_Min_DAC_Ch0/8;
				AFE2_base1_Max = AFE2_Max_DAC_Ch1/8;
				AFE2_base1_Min = AFE2_Min_DAC_Ch1/8;
				AFE2_base2_Max = AFE2_Max_DAC_Ch2/8;
				AFE2_base2_Min = AFE2_Min_DAC_Ch2/8;

				AFE2_base0  = (AFE2_base0_Max + AFE2_base0_Min)/2;
				AFE2_base1  = (AFE2_base1_Max + AFE2_base1_Min)/2;
				AFE2_base2  = (AFE2_base2_Max + AFE2_base2_Min)/2;

			}
	else if(ReCal_Flag&BIT7)
			{

					Sensor_state = (char)(ESIPPU&0x0007);     // only for clockwise rotation, cutting ch0 first
							switch(Sensor_state)
							{

							case 0x01: {
										   AFE2_drift0 = AFE2_Max_DAC_Ch0 / 4 - AFE2_base0_Max;
										}
										break;

							case 0x02: {
											AFE2_drift0 = AFE2_Max_DAC_Ch1 / 4 - AFE2_base1_Max;
										}
										break;

							case 0x03: 	{
											AFE2_drift0 = AFE2_Min_DAC_Ch2 / 4 - AFE2_base2_Min ;
										}
										break;

							case 0x04: {
											AFE2_drift0 = AFE2_Max_DAC_Ch2 /4 - AFE2_base2_Max;
										}
										break;

							case 0x05: {
											AFE2_drift0 = AFE2_Min_DAC_Ch1 /4 - AFE2_base1_Min;
										}
										break;

							case 0x06: {
											AFE2_drift0 = AFE2_Min_DAC_Ch0 /4 - AFE2_base0_Min;
										}
										break;

							}


							   New_level  = AFE1_base0 + AFE2_drift0/2;

							   Delta = (ESIDAC1R0+ESIDAC1R1)/2 - New_level;

							   if (abs(Delta) < delta_level )
							   {
							   ESIDAC1R0 = New_level - Noise_level_0;                 // Noise_level, "-" for INV version, "+" for non-INV version
							   ESIDAC1R1 = New_level + Noise_level_0;				  // Noise_level, "+" for INV version, "-" for non-INV version
							   }


							   New_level  = AFE1_base1 + AFE2_drift0/2;

							   Delta = (ESIDAC1R2+ESIDAC1R3)/2 - New_level;

							   if (abs(Delta) < delta_level )
							   {
							   ESIDAC1R2 = New_level - Noise_level_1;                // Noise_level, "-" for INV version, "+" for non-INV version
							   ESIDAC1R3 = New_level + Noise_level_1;                // Noise_level, "+" for INV version, "-" for non-INV version
							   }

							   New_level  = AFE1_base2 + AFE2_drift0/2;

							   Delta = (ESIDAC1R4+ESIDAC1R5)/2 - New_level;

							   if (abs(Delta) < delta_level )
							   {
							   ESIDAC1R4 = New_level - Noise_level_2;                 // Noise_level, "-" for INV version, "+" for non-INV version
							   ESIDAC1R5 = New_level + Noise_level_2;				  // Noise_level, "+" for INV version, "-" for non-INV version
							   }


			}

}

		 ESIAFE = ESIVCC2 + ESICA1INV + ESITEN;                                  	 // disable AFE2 when completed;
		 ESITSM = ESITSMTRG1 + ESITSMTRG0 + ESIDIV3A1 + ESIDIV3B1;		             // Back to 655Hz
		 ESIINT1 &= ~ESIIE1;														 // disable ESISTOP INT

}


void AFE2_FindDAC_Fast_Range(int Starting_point_ch0, int Starting_point_ch1, int Starting_point_ch2, int Range_num)
{

unsigned int Range;
unsigned int Range_status;


	ESIDAC2R0 = Starting_point_ch0;             // set DAC from the starting point
	ESIDAC2R1 = Starting_point_ch0;            	// set DAC from the starting point

	ESIDAC2R2 = Starting_point_ch1;             // set DAC from the starting point
	ESIDAC2R3 = Starting_point_ch1;            	// set DAC from the starting point

	ESIDAC2R4 = Starting_point_ch2;             // set DAC from the starting point
	ESIDAC2R5 = Starting_point_ch2;            	// set DAC from the starting point


	Range_status = 0;
	Range_status |= BIT7;                       // This is the searching Loop enable bit.

	Range = Range_num;


// This loop is to find the starting point of DAC and the direction of searching by adding or subtracting a value of "Range" into or from the DAC
// Range_status is to show the direction of searching
// BIT0   indication of a searching direction of increasing for channel 0;
// BIT1   indication of a searching direction of decreasing for channel 0;
// BIT2   indication of a searching direction of increasing for channel 1;
// BIT3   indication of a searching direction of decreasing for channel 1;
// BIT8   indication of a searching direction of increasing for channel 2;
// BIT9   indication of a searching direction of decreasing for channel 2;
// BIT4   indication of completion of the loop for channel 0;
// BIT5   indication of completion of the loop for channel 1;
// BIT6   indication of completion of the loop for channel 2;


while (Range_status&BIT7)
{
		while(!((Range_status&BIT4) && (Range_status&BIT5) && (Range_status&BIT6)))
		{

					__bis_SR_register(LPM3_bits+GIE);   	//	 wait for the ESISTOP flag


				   if(!(Range_status&BIT4))
				   {
					if (ESIPPU&ESIOUT4)                     // channel 0;
						{

							if(Range_status&BIT1)			// check if there is a decreasing direction mark of channel 0.
							{ Range_status |= BIT4;}        // if yes, put a completion mark for channel 0
							else
							{ Range_status |= BIT0;}        // if no, put a direction mark pointing to increasing for channel 0.

							ESIDAC2R0 -= Range;     		// decrease the DAC by a value of "Range"
							ESIDAC2R1 -= Range;

						}
					else
						{

							if(Range_status&BIT0)			// check if there is a increasing direction mark of channel 0.
							{ Range_status |= BIT4;}        // if yes, put a completion mark for channel 0
							else
							{ Range_status |= BIT1;}        // if no, put a direction mark pointing to decreasing for channel 0.

							ESIDAC2R0 += Range;				// increase the DAC by a value of "Range"
							ESIDAC2R1 += Range;
						}
				   }


				  if(!(Range_status&BIT5))
				  {
					if (ESIPPU&ESIOUT5)						// channel 1;
						{

							if(Range_status&BIT3)			// check if there is a decreasing direction mark of channel 1.
							{ Range_status |= BIT5;}        // if yes, put a completion mark for channel 1
							else
							{ Range_status |= BIT2;}        // if no, put a direction mark pointing to increasing for channel 1.

							ESIDAC2R2 -= Range;     		// decrease the DAC by a value of "Range"
							ESIDAC2R3 -= Range;

						}
					else
						{

							if(Range_status&BIT2)			// check if there is a increasing direction mark of channel 1.
							{ Range_status |= BIT5;}        // if yes, put a completion mark for channel 1
							else
							{ Range_status |= BIT3;}        // if no, put a direction mark pointing to decreasing for channel 1.

							ESIDAC2R2 += Range;				// increase the DAC by a value of "Range"
							ESIDAC2R3 += Range;
						}
				  }


				   if(!(Range_status&BIT6))                 // The third sensor
				   {
					if (ESIPPU&ESIOUT6)                     // channel 2;
						{

							if(Range_status&BIT9)			// check if there is a decreasing direction mark of channel 2.
							{ Range_status |= BIT6;}        // if yes, put a completion mark for channel 2
							else
							{ Range_status |= BIT8;}        // if no, put a direction mark pointing to increasing for channel 2.

							ESIDAC2R4 -= Range;     		// decrease the DAC by a value of "Range"
							ESIDAC2R5 -= Range;

						}
					else
						{

							if(Range_status&BIT8)			// check if there is a increasing direction mark of channel 2.
							{ Range_status |= BIT6;}        // if yes, put a completion mark for channel 2
							else
							{ Range_status |= BIT9;}        // if no, put a direction mark pointing to decreasing for channel 2.

							ESIDAC2R4 += Range;				// increase the DAC by a value of "Range"
							ESIDAC2R5 += Range;
						}
				   }



		}

	if (Range > 1)
	{	Range_status &= ~(BIT4+BIT5+BIT6);            				// clear the flag of completion
		Range_status &= ~(BIT0+BIT1+BIT2+BIT3+BIT8+BIT9);			// optional, clear the direction flag
		Range = 1;                                 				    // This is to restart the loop with searching value of one, by "+/- 1 method"
	}
	else
	{   Range_status &= ~BIT7;}									    // "+/- 1 method completed, the loop end here

}


}


void AFE2_FindDAC(void)
{
	unsigned int i;
	unsigned int DAC_BIT = 0, Prev_DAC_BIT = 0;

	DAC_BIT   = 0x0800;						// DAC Level tester, using Sucessive approx approach
	Prev_DAC_BIT = 0x0C00;

	ESIDAC2R0 = DAC_BIT;                 	// set as the middle point
	ESIDAC2R1 = DAC_BIT;                 	// set as the middle point

	ESIDAC2R2 = DAC_BIT;                 	// set as the middle point
	ESIDAC2R3 = DAC_BIT;                 	// set as the middle point

	ESIDAC2R4 = DAC_BIT;                 	// set as the middle point
	ESIDAC2R5 = DAC_BIT;                 	// set as the middle point


// this for loop is to find an initial DAC value for ch0, ch1 and ch2

	for(i = 0; i<12; i++)				 	// test 12 times as 12 bit DAC
	{

	__bis_SR_register(LPM3_bits+GIE);   	//	 wait for the ESISTOP flag

	DAC_BIT /= 2 ;							// right shift one bit

	if (!(ESIPPU&ESIOUT4))                  // channel 0;
		{

			ESIDAC2R0 |= DAC_BIT;     		// keep the previous bit and set the next bit
			ESIDAC2R1 |= DAC_BIT;			// keep the previous bit and set the next bit

		}
	else
		{
			ESIDAC2R0 ^= Prev_DAC_BIT;		// reset the previous bit and set the next bit
			ESIDAC2R1 ^= Prev_DAC_BIT;		// reset the previous bit and set the next bit
		}

	if (!(ESIPPU&ESIOUT5))					// channel 1;
		{

			ESIDAC2R2 |= DAC_BIT;     		// keep the previous bit and set the next bit
			ESIDAC2R3 |= DAC_BIT;			// keep the previous bit and set the next bit

		}
	else
		{
			ESIDAC2R2 ^= Prev_DAC_BIT;		// reset the previous bit and set the next bit
			ESIDAC2R3 ^= Prev_DAC_BIT;		// reset the previous bit and set the next bit
		}


	if (!(ESIPPU&ESIOUT6))					// channel 2;
		{

			ESIDAC2R4 |= DAC_BIT;     		// keep the previous bit and set the next bit
			ESIDAC2R5 |= DAC_BIT;			// keep the previous bit and set the next bit

		}
	else
		{
			ESIDAC2R4 ^= Prev_DAC_BIT;		// reset the previous bit and set the next bit
			ESIDAC2R5 ^= Prev_DAC_BIT;		// reset the previous bit and set the next bit
		}


	Prev_DAC_BIT /= 2;						// right shift one bit


	}



}


void AFE2_FindDAC_Fast_Successive(int Starting_point_ch0, int Starting_point_ch1, int Starting_point_ch2, int Range_num)
{
	unsigned int i;
	unsigned int DAC_BIT = 0;

	DAC_BIT   = 0x0001;						            // DAC Level tester, using Sucessive approx approach

	ESIDAC2R0 = Starting_point_ch0;                 	// set as the middle point
	ESIDAC2R1 = Starting_point_ch0;                 	// set as the middle point

	ESIDAC2R2 = Starting_point_ch1;                 	// set as the middle point
	ESIDAC2R3 = Starting_point_ch1;                 	// set as the middle point

	ESIDAC2R4 = Starting_point_ch2;                 	// set as the middle point
	ESIDAC2R5 = Starting_point_ch2;                 	// set as the middle point

	for(i=1; i<Range_num; i++)							// Range_num set the number of least significant bits are used
	{

	   DAC_BIT <<= 1;

	}



// this for loop is to find an initial DAC value for ch0, ch1 and ch2
	for(i = 0; i<Range_num; i++)				 	    // test "Range_num" times to find out the signal level of 12 bits resolution
	{

	__bis_SR_register(LPM3_bits+GIE);   			    // wait for the ESISTOP flag



	if (!(ESIPPU&ESIOUT4))                     			// channel 0;
		{

			ESIDAC2R0 += DAC_BIT;
			ESIDAC2R1 += DAC_BIT;

		}
	else
		{
			ESIDAC2R0 -= DAC_BIT;
			ESIDAC2R1 -= DAC_BIT;
		}

	if (!(ESIPPU&ESIOUT5))								// channel 1;
		{

			ESIDAC2R2 += DAC_BIT;
			ESIDAC2R3 += DAC_BIT;

		}
	else
		{
			ESIDAC2R2 -= DAC_BIT;
			ESIDAC2R3 -= DAC_BIT;
		}

	if (!(ESIPPU&ESIOUT6))								// channel 2;
		{

			ESIDAC2R4 += DAC_BIT;
			ESIDAC2R5 += DAC_BIT;

		}
	else
		{
			ESIDAC2R4 -= DAC_BIT;
			ESIDAC2R5 -= DAC_BIT;
		}

		DAC_BIT /= 2;									// right shift one bit

	}


}





#endif

