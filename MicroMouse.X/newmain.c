/*
 * File:   newmain.c
 * Author: Art
 *
 * Created on October 27, 2014, 2:29 AM
 */

#include <stdio.h>
#include <stdlib.h>
#include <plib.h>
#include <p32xxxx.h>
#include <peripheral/pps.h>
#include <peripheral/adc10.h>
#include <xc.h>

// DEVCFG3
// USERID = No Setting
#pragma config FSRSSEL = PRIORITY_7     // Shadow Register Set Priority Select (SRS Priority 7)
#pragma config PMDL1WAY = ON            // Peripheral Module Disable Configuration (Allow only one reconfiguration)
#pragma config IOL1WAY = ON             // Peripheral Pin Select Configuration (Allow only one reconfiguration)
#pragma config FUSBIDIO = ON            // USB USID Selection (Controlled by the USB Module)
#pragma config FVBUSONIO = ON           // USB VBUS ON Selection (Controlled by USB Module)

// DEVCFG2
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider (2x Divider)
#pragma config FPLLMUL = MUL_20         // PLL Multiplier (20x Multiplier)
#pragma config UPLLIDIV = DIV_12        // USB PLL Input Divider (12x Divider)
#pragma config UPLLEN = OFF             // USB PLL Enable (Disabled and Bypassed)
#pragma config FPLLODIV = DIV_1         // System PLL Output Clock Divider (PLL Divide by 1)

// DEVCFG1
#pragma config FNOSC = FRCPLL           // Oscillator Selection Bits (Fast RC Osc with PLL)
//#pragma config FSOSCEN = OFF            // Secondary Oscillator Enable (Disabled)
#pragma config IESO = OFF               // Internal/External Switch Over (Disabled)
#pragma config POSCMOD = OFF             // Primary Oscillator Configuration (XT osc mode)
#pragma config OSCIOFNC = OFF           // CLKO Output Signal Active on the OSCO Pin (Disabled)
#pragma config FPBDIV = DIV_1           // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/1)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor Selection (Clock Switch Disable, FSCM Disabled)
#pragma config WDTPS = PS1              // Watchdog Timer Postscaler (1:1)
#pragma config WINDIS = OFF             // Watchdog Timer Window Enable (Watchdog Timer is in Non-Window Mode)
//#pragma config FWDTEN = OFF             // Watchdog Timer Enable (WDT Disabled (SWDTEN Bit Controls))
//#pragma config FWDTWINSZ = WISZ_25      // Watchdog Timer Window Size (Window Size is 25%)

// DEVCFG0
#pragma config DEBUG = ON               // Background Debugger Enable (Debugger is Enabled)
//#pragma config JTAGEN = ON              // JTAG Enable (JTAG Port Enabled)
#pragma config ICESEL = ICS_PGx2        // ICE/ICD Comm Channel Select (Communicate on PGEC2/PGED2)
#pragma config PWP = OFF                // Program Flash Write Protect (Disable)
#pragma config BWP = OFF                // Boot Flash Write Protect bit (Protection Disabled)
#pragma config CP = OFF                 // Code Protect (Protection Disabled)

// Configuration Bit settings
// SYSCLK = 40 MHz (8MHz Crystal/ FPLLIDIV * FPLLMUL / FPLLODIV)
// PBCLK = 40 MHz
// Primary Osc w/PLL (XT+,HS+,EC+PLL)
// WDT OFF
// Other options are don't care
//
//#pragma config FNOSC = FRCPLL, POSCMOD = HS, FPLLIDIV = DIV_2, FPLLMUL = MUL_20, FPBDIV = DIV_1, FPLLODIV = DIV_2
#pragma config FWDTEN = OFF
#pragma config FSOSCEN = OFF, JTAGEN = OFF

// frequency we're running at
#define	SYS_FREQ 80000000

int i = 0;
typedef unsigned int uint;


void adcConfigureAutoScan();
uint readpins();
void ledinit();

//Function that initializes all of the required I/O
void initIO();

void motorinit();
void setpwmR(int a);
void setpwmL(int b);
void motorRstop();
void motorLstop();
void motorRfwd();
void motorLfwd();
void motorRrev();
void motorLrev();
void PWMinit();
void encodersInit();
void timer1Init();

//unsigned short int channe10;	// conversion result as read from result buffer
//unsigned int channel12;	// conversion result as read from result buffer
//unsigned int channel13;	// conversion result as read from result buffer

uint leftspeed, rightspeed;
// volatiles for the stuff used in the ISR
volatile unsigned int DAC_value; // Vref output
//volatile int CVRCON_setup; // stores the voltage ref config register after it is set up
volatile unsigned int channel13;	// conversion result as read from result buffer


int main()
{
    //SYSTEMConfigPerformance(SYS_FREQ);
    // Configure the device for maximum performance but do not change the PBDIV
    // Given the options, this function will change the flash wait states, RAM
    // wait state and enable prefetch cache but will not change the PBDIV.
    // The PBDIV value is already set via the pragma FPBDIV option above..
    SYSTEMConfig(SYS_FREQ, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);
    PORTFbits.RF1 = 0;

    //Function that initializes all of the required I/O
    initIO();

    //Initialize all of the LED pins
    ledinit();
    adcConfigureAutoScan();
    timer1Init();
    encodersInit();
    motorinit();
    PWMinit();
    motorRstop();
    motorLstop();
    leftspeed = 310;
    rightspeed = 300;

    while(PORTDbits.RD3 == 0)
            {}

            for(i = 0; i < 5000000; i++)
            {
            }
    setpwmR(rightspeed);
    setpwmL(leftspeed);
    motorRfwd();
    motorLfwd();

    //turnright();
    //turnleft();
	while (1)
	{
                if(readpins(0) > 300)   //left
		{
                    setpwmR(100);
                    for(i=0;i<800;i++);
                    /*
                    if(leftspeed < 315)
                        leftspeed++;
                    if(rightspeed > 275)
                        rightspeed--;
                     */
                    setpwmR(rightspeed);
                    setpwmL(leftspeed);
                    motorRfwd();
                    motorLfwd();
                }
                //for(i = 0; i < 500; i++){};

                //if (channel13 < 600)
                if(readpins(2) > 600) //right
		{
                    setpwmL(100);
                    for(i=0;i<800;i++);
                    /*
                    if(leftspeed > 265)
                        leftspeed--;
                    if(rightspeed < 325)
                        rightspeed++;
                     */
                    setpwmR(rightspeed);
                    setpwmL(leftspeed);
                    motorRfwd();
                    motorLfwd();
                }
                //for(i = 0; i < 500; i++){};

                //if (channel13 < 900)
                if(readpins(1) > 900) //middle
		{
                    setpwmR(0);
                    setpwmL(0);
                    motorRstop();
                    motorLstop();
                    turnright();

                }
                //for(i = 0; i < 1500; i++){};


	}

    return (EXIT_SUCCESS);
}

void adcConfigureAutoScan()
{
        // configure and enable the ADC
        CloseADC10();	// ensure the ADC is off before setting the configuration

        // define setup parameters for OpenADC10
		    // Turn module on | output in integer | trigger mode auto | enable  autosample
	#define PARAM1  ADC_MODULE_ON | ADC_FORMAT_INTG16 | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON

	// define setup parameters for OpenADC10
			    // ADC ref external    | disable offset test    | enable scan mode | perform 2 samples | use one buffer | use MUXA mode
        // note: to read X number of pins you must set ADC_SAMPLES_PER_INT_X
	#define PARAM2  ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_ON | ADC_SAMPLES_PER_INT_3 | ADC_ALT_BUF_ON | ADC_ALT_INPUT_OFF

	// define setup parameters for OpenADC10
	// 				  use ADC internal clock | set sample time
	#define PARAM3  ADC_CONV_CLK_INTERNAL_RC | ADC_SAMPLE_TIME_11

	// define setup parameters for OpenADC10
				// set AN4 and AN5
	#define PARAM4	ENABLE_AN13_ANA|ENABLE_AN12_ANA|ENABLE_AN25_ANA

	// define setup parameters for OpenADC10
	// do not assign channels to scan
	#define PARAM5	SKIP_SCAN_ALL
        // use ground as neg ref for A
	SetChanADC10( ADC_CH0_NEG_SAMPLEA_NVREF ); // // configure to sample A13
	OpenADC10( PARAM1, PARAM2, PARAM3, PARAM4, PARAM5 ); // configure ADC using parameter define above
        AD1CSSLSET = 0x2003000;
	EnableADC10(); // Enable the ADC
}


void ledinit()
{
    LATFCLR = 0x0002;
    TRISFCLR = 0x0002;
}

// Timer 2 interrupt handler ///////
// ipl2 means "interrupt priority level 2"
// ASM output is 47 instructions for the ISR
void __ISR(_TIMER_2_VECTOR, ipl2) Timer2Handler(void)
{
    mT2ClearIntFlag();
}

void initIO()
{
    ANSELB = 0x3400; //AN0 - AN15

    ANSELGbits.ANSG6 = 0; //AN16
    ANSELGbits.ANSG7 = 0; //AN17
    ANSELGbits.ANSG8 = 0; //AN18
    ANSELGbits.ANSG9 = 0; //AN19


    ANSELEbits.ANSE2 = 0; //AN20
    ANSELEbits.ANSE4 = 0; //AN21
    ANSELEbits.ANSE5 = 0; //AN22
    ANSELEbits.ANSE6 = 0; //AN23
    ANSELEbits.ANSE7 = 0; //AN27

    ANSELDbits.ANSD1 = 0; //AN24
    ANSELDbits.ANSD2 = 0; //AN25
    ANSELDbits.ANSD3 = 0; //AN26


    //Configure analog pins IO
    TRISBbits.TRISB0 = 0;
    TRISBbits.TRISB1 = 1;
    TRISBbits.TRISB2 = 1;
    TRISBbits.TRISB3 = 0;
    TRISBbits.TRISB4 = 0;
    TRISBbits.TRISB5 = 0;
    TRISBbits.TRISB8 = 1;
    TRISBbits.TRISB9 = 0;
    TRISBbits.TRISB10 = 1;
    TRISBbits.TRISB11 = 0;
    TRISBbits.TRISB12 = 1;
    TRISBbits.TRISB13 = 1;
    TRISBbits.TRISB14 = 0;
    TRISBbits.TRISB15 = 0;


    TRISEbits.TRISE0 = 0;
    TRISEbits.TRISE1 = 0;
    TRISEbits.TRISE2 = 0;
    TRISEbits.TRISE3 = 0;
    TRISEbits.TRISE4 = 0;
    TRISEbits.TRISE5 = 0;
    TRISEbits.TRISE6 = 0;
    TRISEbits.TRISE7 = 0;


    TRISD = 0x001C;

    TRISGbits.TRISG6 = 0;
    TRISGbits.TRISG7 = 1;   //pin RG7 is configured as SPI input
    TRISGbits.TRISG8 = 0;
    TRISGbits.TRISG9 = 0;

    TRISC = 0x0000; //RC1-RC4 extra pins

/* ********************* Assign UART 2 signals onto pins using PPS *********************************** */
    PPSInput(4, U4RX, RPB2);  //Assign U2RX to pin RPD8
    PPSOutput(3, RPB0, U4TX);   //Assign U2TX to pin RPD9

/* ********************* Assign Encoders signals onto pins using PPS ************************************/
    PPSInput(3, T4CK, RPD4);  //Assign T4CK to pin RPD4
    PPSInput(2, T5CK, RPB1);  //Assign T5CK to pin RPD11

/* ********************* Assign SDI/O 2 signals onto pins using PPS *********************************** */
    PPSInput(2, SDI2, RPG7);  //Assign SDI2 to pin RPG7
    PPSOutput(1, RPG8, SDO2);   //Assign SDO2 to pin RPG8

/* ********************* Assign OC1 & OC2************************************************************** */
    PPSOutput(4, RPD1, OC1);
    PPSOutput(4, RPD5, OC2);

    //mappingMode = 1;
    //fastMode = 0;
}

void encodersInit()
{
    //T4 is input from encoder. Read from TMR4 to see number of ticks counted.
    T4CON = 0x0;                //T4 disabled
    T4CONSET = 0x0002;          //T4 External Clock Source, 1:1 prescaler
    TMR4 = 0x0;                 //Reset T4 counter
    PR4 = 0xffff;               //Set T4 period to max
    //mT4ClearIntFlag();          //clear interrupt flag
    //mT4SetIntPriority(7);       //set Timer4 Interrupt Priority
    //mT4IntEnable(1);            //enable timer4 interrupts
    T4CONSET = 0x8000;          //T4 enable

    //T5 is input from encoder. Read from TMR5 to see number of ticks counted.
    T5CON = 0x0;                //T5 disabled
    T5CONSET = 0x0002;          //T5 External Clock Source, 1:1 prescaler
    TMR5 = 0x0;                 //Reset T5
    PR5 = 0xffff;               //Set T5 period to max
    //mT5ClearIntFlag();          //clear T5 interrupt flag
    //mT5SetIntPriority(2);       //set Timer5 Interrupt Priority
    //mT5IntEnable(1);            //enable timer5 interrupts
    T5CONSET = 0x8000;          //T5 enable
}

timer1Init()
{
    OpenTimer1(T1_ON | T1_PS_1_8 | T1_SOURCE_INT, 1000);
    ConfigIntTimer1(T1_INT_ON | T1_INT_PRIOR_2);
    INTEnableSystemMultiVectoredInt();
}

void __ISR(_TIMER_1_VECTOR, ipl2) _Timer1Handler(void)
{
    //Reset the flag
    mT1ClearIntFlag();

}

void motorinit()
{
    //SET MOTOR1 GPIOS
    //LATECLR = 0x001F;
    //TRISECLR = 0x001F;

    mPORTESetBits(BIT_2);       //set standby bit
}

void motorRstop()
{
   mPORTEClearBits(BIT_0);
   mPORTEClearBits(BIT_1);
}

void motorLstop()
{
   mPORTEClearBits(BIT_3);
   mPORTEClearBits(BIT_4);
}

void motorRfwd()
{
    mPORTESetBits(BIT_1);
    mPORTEClearBits(BIT_0);
}

void motorLfwd()
{
   mPORTESetBits(BIT_3);
   mPORTEClearBits(BIT_4);
}

void motorRrev()
{
   mPORTESetBits(BIT_0);
   mPORTEClearBits(BIT_1);
}

void motorLrev()
{
   mPORTESetBits(BIT_4);
   mPORTEClearBits(BIT_3);
}

void setpwmR(int a)
{
    SetDCOC1PWM(a); // Write new duty cycle
}

void setpwmL(int b)
{
    SetDCOC2PWM(b); // Write new duty cycle
}

void PWMinit()
{
    // init OC4 & OC5 module
    OpenOC1( OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0);
    OpenOC2( OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0);

    // init Timer2 mode and period (PR2) (frequency of 1 / 20 kHz = (3999 + 1) / 80MHz * 1
    OpenTimer2( T2_ON | T2_PS_1_1 | T2_SOURCE_INT, 3999);
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);
}

void turnright()
{
    motorRstop();
    motorLstop();
    TMR4=0x0;
    TMR5=0x0;
    setpwmR(200);
    setpwmL(200);
    motorRrev();
    motorLfwd();
    while(TMR5<200);
}

void turnleft()
{
    motorRstop();
    motorLstop();
    TMR4=0x0;
    TMR5=0x0;
    setpwmR(200);
    setpwmL(200);
    motorLrev();
    motorRfwd();
    while(TMR5<300);
}

uint readpins(int PinNum)
{
    uint avalue;
    while(!IFS0bits.AD1IF); // wait until buffers contain new samples
    //AD1CON1bits.ASAM = 0;   // stop automatic sampling (essentially shut down ADC in this mode)

    if(ReadActiveBufferADC10() == 1)// check which buffers are being written to and read from the other set
    {
        switch(PinNum)
        {
            case 0:
                avalue = ADC1BUF0;
                break;
            case 1:
                avalue = ADC1BUF1;
                break;
            case 2:
                avalue = ADC1BUF2;
                break;
        }
    }
    else
    {
        switch(PinNum)
        {
            case 0:
                avalue = ADC1BUF8;
                break;
            case 1:
                avalue = ADC1BUF9;
                break;
            case 2:
                avalue = ADC1BUFA;
                break;
        }
    }
    //AD1CON1bits.ASAM = 1;           // restart automatic sampling
    mAD1ClearIntFlag();                  // clear ADC interrupt flag
    return avalue;
}





/*
 * File:   newmain.c
 * Author: Art
 *
 * Created on October 27, 2014, 2:29 AM
 */

#include <stdio.h>
#include <stdlib.h>
#include <plib.h>
#include <p32xxxx.h>
#include <peripheral/pps.h>
#include <peripheral/adc10.h>
#include <xc.h>

// DEVCFG3
// USERID = No Setting
#pragma config FSRSSEL = PRIORITY_7     // Shadow Register Set Priority Select (SRS Priority 7)
#pragma config PMDL1WAY = ON            // Peripheral Module Disable Configuration (Allow only one reconfiguration)
#pragma config IOL1WAY = ON             // Peripheral Pin Select Configuration (Allow only one reconfiguration)
#pragma config FUSBIDIO = ON            // USB USID Selection (Controlled by the USB Module)
#pragma config FVBUSONIO = ON           // USB VBUS ON Selection (Controlled by USB Module)

// DEVCFG2
//#pragma config FPLLIDIV = DIV_2         // PLL Input Divider (2x Divider)
//#pragma config FPLLMUL = MUL_20         // PLL Multiplier (20x Multiplier)
#pragma config UPLLIDIV = DIV_12        // USB PLL Input Divider (12x Divider)
#pragma config UPLLEN = OFF             // USB PLL Enable (Disabled and Bypassed)
//#pragma config FPLLODIV = DIV_1         // System PLL Output Clock Divider (PLL Divide by 1)

// DEVCFG1
//#pragma config FNOSC = FRCPLL           // Oscillator Selection Bits (Fast RC Osc with PLL)
//#pragma config FSOSCEN = OFF            // Secondary Oscillator Enable (Disabled)
#pragma config IESO = OFF               // Internal/External Switch Over (Disabled)
//#pragma config POSCMOD = OFF             // Primary Oscillator Configuration (XT osc mode)
#pragma config OSCIOFNC = OFF           // CLKO Output Signal Active on the OSCO Pin (Disabled)
//#pragma config FPBDIV = DIV_1           // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/1)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor Selection (Clock Switch Disable, FSCM Disabled)
#pragma config WDTPS = PS1              // Watchdog Timer Postscaler (1:1)
#pragma config WINDIS = OFF             // Watchdog Timer Window Enable (Watchdog Timer is in Non-Window Mode)
//#pragma config FWDTEN = OFF             // Watchdog Timer Enable (WDT Disabled (SWDTEN Bit Controls))
//#pragma config FWDTWINSZ = WISZ_25      // Watchdog Timer Window Size (Window Size is 25%)

// DEVCFG0
#pragma config DEBUG = ON               // Background Debugger Enable (Debugger is Enabled)
//#pragma config JTAGEN = ON              // JTAG Enable (JTAG Port Enabled)
#pragma config ICESEL = ICS_PGx2        // ICE/ICD Comm Channel Select (Communicate on PGEC2/PGED2)
#pragma config PWP = OFF                // Program Flash Write Protect (Disable)
#pragma config BWP = OFF                // Boot Flash Write Protect bit (Protection Disabled)
#pragma config CP = OFF                 // Code Protect (Protection Disabled)

// Configuration Bit settings
// SYSCLK = 40 MHz (8MHz Crystal/ FPLLIDIV * FPLLMUL / FPLLODIV)
// PBCLK = 40 MHz
// Primary Osc w/PLL (XT+,HS+,EC+PLL)
// WDT OFF
// Other options are don't care
//
#pragma config FNOSC = FRCPLL, POSCMOD = HS, FPLLIDIV = DIV_2, FPLLMUL = MUL_20, FPBDIV = DIV_1, FPLLODIV = DIV_2
#pragma config FWDTEN = OFF
#pragma config FSOSCEN = OFF, JTAGEN = OFF

// frequency we're running at
#define	SYS_FREQ 40000000

int i = 0;
typedef unsigned int uint;


void adcConfigureAutoScan();
uint readpins();
void ledinit();

//Function that initializes all of the required I/O
void initIO();

void motorinit();
void setpwmR(int a);
void setpwmL(int b);
void motorRstop();
void motorLstop();
void motorRfwd();
void motorLfwd();
void motorRrev();
void motorLrev();
void PWMinit();
void timersInit();
void turnright();

//unsigned short int channe10;	// conversion result as read from result buffer
//unsigned int channel12;	// conversion result as read from result buffer
//unsigned int channel13;	// conversion result as read from result buffer

uint leftspeed, rightspeed;
// volatiles for the stuff used in the ISR
volatile unsigned int DAC_value; // Vref output
//volatile int CVRCON_setup; // stores the voltage ref config register after it is set up
volatile unsigned int channel13;	// conversion result as read from result buffer


int main()
{
    //SYSTEMConfigPerformance(SYS_FREQ);
    // Configure the device for maximum performance but do not change the PBDIV
    // Given the options, this function will change the flash wait states, RAM
    // wait state and enable prefetch cache but will not change the PBDIV.
    // The PBDIV value is already set via the pragma FPBDIV option above..
    SYSTEMConfig(SYS_FREQ, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);
    PORTFbits.RF1 = 0;

    //Function that initializes all of the required I/O
    initIO();

    //Initialize all of the LED pins
    ledinit();
    adcConfigureAutoScan();
    timersInit();
    motorinit();
    PWMinit();
    motorRstop();
    motorLstop();
    leftspeed = 290;
    rightspeed = 300;

    while(PORTDbits.RD3 == 0)
            {}

            for(i = 0; i < 5000000; i++)
            {
            }
    setpwmR(rightspeed);
    setpwmL(leftspeed);
    motorRfwd();
    motorLfwd();

    //turnright();
    //turnleft();
	while (1)
	{}

    return (EXIT_SUCCESS);
}

void adcConfigureAutoScan()
{
        // configure and enable the ADC
        CloseADC10();	// ensure the ADC is off before setting the configuration

        // define setup parameters for OpenADC10
		    // Turn module on | output in integer | trigger mode auto | enable  autosample
	#define PARAM1  ADC_MODULE_ON | ADC_FORMAT_INTG16 | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON

	// define setup parameters for OpenADC10
			    // ADC ref external    | disable offset test    | enable scan mode | perform 2 samples | use one buffer | use MUXA mode
        // note: to read X number of pins you must set ADC_SAMPLES_PER_INT_X
	#define PARAM2  ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_ON | ADC_SAMPLES_PER_INT_3 | ADC_ALT_BUF_ON | ADC_ALT_INPUT_OFF

	// define setup parameters for OpenADC10
	// 				  use ADC internal clock | set sample time
	#define PARAM3  ADC_CONV_CLK_INTERNAL_RC | ADC_SAMPLE_TIME_11

	// define setup parameters for OpenADC10
				// set AN4 and AN5
	#define PARAM4	ENABLE_AN13_ANA|ENABLE_AN12_ANA|ENABLE_AN25_ANA

	// define setup parameters for OpenADC10
	// do not assign channels to scan
	#define PARAM5	SKIP_SCAN_ALL//SKIP_SCAN_AN0|SKIP_SCAN_AN1|SKIP_SCAN_AN2|SKIP_SCAN_AN3|SKIP_SCAN_AN4|SKIP_SCAN_AN5|SKIP_SCAN_AN6|SKIP_SCAN_AN7|SKIP_SCAN_AN8|SKIP_SCAN_AN9|SKIP_SCAN_AN10|SKIP_SCAN_AN11|SKIP_SCAN_AN12|SKIP_SCAN_AN14|SKIP_SCAN_AN15|SKIP_SCAN_AN16|SKIP_SCAN_AN17|SKIP_SCAN_AN18|SKIP_SCAN_AN19|SKIP_SCAN_AN20|SKIP_SCAN_AN21|SKIP_SCAN_AN22|SKIP_SCAN_AN23|SKIP_SCAN_AN24|SKIP_SCAN_AN25|SKIP_SCAN_AN26|SKIP_SCAN_AN27
        // use ground as neg ref for A
	SetChanADC10( ADC_CH0_NEG_SAMPLEA_NVREF ); // // configure to sample A13
	OpenADC10( PARAM1, PARAM2, PARAM3, PARAM4, PARAM5 ); // configure ADC using parameter define above
        AD1CSSLSET = 0x2003000;
	EnableADC10(); // Enable the ADC
}


void ledinit()
{
    LATFCLR = 0x0002;
    TRISFCLR = 0x0002;
}


void initIO()
{
    ANSELB = 0x3400; //AN0 - AN15

    ANSELGbits.ANSG6 = 0; //AN16
    ANSELGbits.ANSG7 = 0; //AN17
    ANSELGbits.ANSG8 = 0; //AN18
    ANSELGbits.ANSG9 = 0; //AN19


    ANSELEbits.ANSE2 = 0; //AN20
    ANSELEbits.ANSE4 = 0; //AN21
    ANSELEbits.ANSE5 = 0; //AN22
    ANSELEbits.ANSE6 = 0; //AN23
    ANSELEbits.ANSE7 = 0; //AN27

    ANSELDbits.ANSD1 = 0; //AN24
    ANSELDbits.ANSD2 = 0; //AN25
    ANSELDbits.ANSD3 = 0; //AN26


    //Configure analog pins IO
    TRISBbits.TRISB0 = 0;
    TRISBbits.TRISB1 = 1;
    TRISBbits.TRISB2 = 1;
    TRISBbits.TRISB3 = 0;
    TRISBbits.TRISB4 = 0;
    TRISBbits.TRISB5 = 0;
    TRISBbits.TRISB8 = 1;
    TRISBbits.TRISB9 = 0;
    TRISBbits.TRISB10 = 1;
    TRISBbits.TRISB11 = 0;
    TRISBbits.TRISB12 = 1;
    TRISBbits.TRISB13 = 1;
    TRISBbits.TRISB14 = 0;
    TRISBbits.TRISB15 = 0;


    TRISEbits.TRISE0 = 0;
    TRISEbits.TRISE1 = 0;
    TRISEbits.TRISE2 = 0;
    TRISEbits.TRISE3 = 0;
    TRISEbits.TRISE4 = 0;
    TRISEbits.TRISE5 = 0;
    TRISEbits.TRISE6 = 0;
    TRISEbits.TRISE7 = 0;


    TRISD = 0x001C;

    TRISGbits.TRISG6 = 0;
    TRISGbits.TRISG7 = 1;   //pin RG7 is configured as SPI input
    TRISGbits.TRISG8 = 0;
    TRISGbits.TRISG9 = 0;

    TRISC = 0x0000; //RC1-RC4 extra pins

/* ********************* Assign UART 2 signals onto pins using PPS *********************************** */
    PPSInput(4, U4RX, RPB2);  //Assign U2RX to pin RPD8
    PPSOutput(3, RPB0, U4TX);   //Assign U2TX to pin RPD9

/* ********************* Assign Encoders signals onto pins using PPS ************************************/
    PPSInput(3, T4CK, RPD4);  //Assign T4CK to pin RPD4
    PPSInput(2, T5CK, RPB1);  //Assign T5CK to pin RPD11

/* ********************* Assign SDI/O 2 signals onto pins using PPS *********************************** */
    PPSInput(2, SDI2, RPG7);  //Assign SDI2 to pin RPG7
    PPSOutput(1, RPG8, SDO2);   //Assign SDO2 to pin RPG8

/* ********************* Assign OC1 & OC2************************************************************** */
    PPSOutput(4, RPD1, OC1);
    PPSOutput(4, RPD5, OC2);

    //mappingMode = 1;
    //fastMode = 0;
}

void timersInit()
{
    //T4 is input from encoder. Read from TMR4 to see number of ticks counted.
    T4CON = 0x0;                //T4 disabled
    T4CONSET = 0x0002;          //T4 External Clock Source, 1:1 prescaler
    TMR4 = 0x0;                 //Reset T4 counter
    PR4 = 0xffff;               //Set T4 period to max
    //mT4ClearIntFlag();          //clear interrupt flag
    //mT4SetIntPriority(7);       //set Timer4 Interrupt Priority
    //mT4IntEnable(1);            //enable timer4 interrupts
    T4CONSET = 0x8000;          //T4 enable

    //T5 is input from encoder. Read from TMR5 to see number of ticks counted.
    T5CON = 0x0;                //T5 disabled
    T5CONSET = 0x0002;          //T5 External Clock Source, 1:1 prescaler
    TMR5 = 0x0;                 //Reset T5
    PR5 = 0xffff;               //Set T5 period to max
    //mT5ClearIntFlag();          //clear T5 interrupt flag
    //mT5SetIntPriority(2);       //set Timer5 Interrupt Priority
    //mT5IntEnable(1);            //enable timer5 interrupts
    T5CONSET = 0x8000;          //T5 enable

        //The following code snippet opens and uses Timer1,2,3 as an interrupt.
        //Open Timer1 with 1:8 prescaler (80MHz -> 10MHz), with period of 1250, therefore tick = 8kHz.
        OpenTimer1(T1_ON | T1_PS_1_8 | T1_SOURCE_INT, 1000);
        ConfigIntTimer1(T1_INT_ON | T1_INT_PRIOR_2);

        OpenTimer2(T2_ON | T2_PS_1_8 | T2_SOURCE_INT, 1000);
        ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);

        OpenTimer3(T3_ON | T3_PS_1_8 | T3_SOURCE_INT, 1000);
        ConfigIntTimer1(T3_INT_ON | T3_INT_PRIOR_2);

        INTEnableSystemMultiVectoredInt();

}

//Interrupt handler to read IR sensors
void __ISR(_TIMER_1_VECTOR, ipl2) _Timer1Handler(void)
{
    //Reset the flag
    mT1ClearIntFlag();

    if(readpins(0) > 700)   //left
		{
                    if(leftspeed < 305)
                        leftspeed++;
                    if(rightspeed > 285)
                        rightspeed--;
                    setpwmR(rightspeed);
                    setpwmL(leftspeed);
                    motorRfwd();
                    motorLfwd();
                }
                //for(i = 0; i < 500; i++){};

                //if (channel13 < 600)
                if(readpins(2) > 700) //right
		{
                    if(leftspeed > 275)
                        leftspeed--;
                    if(rightspeed < 315)
                        rightspeed++;
                    setpwmR(rightspeed);
                    setpwmL(leftspeed);
                    motorRfwd();
                    motorLfwd();
                }
                //for(i = 0; i < 500; i++){};

                //if (channel13 < 900)
                if(readpins(1) > 900) //middle
		{
                    setpwmR(0);
                    setpwmL(0);
                    motorRstop();
                    motorLstop();
                    turnright();

                }
                for(i = 0; i < 5000; i++){};


}


//Interrupt handler for PWM
void __ISR(_TIMER_2_VECTOR, ipl1) _Timer2Handler(void)
{
    //Reset the flag
    mT2ClearIntFlag();

    //Put code here

}


//Interrupt handler to
void __ISR(_TIMER_3_VECTOR, ipl3) _Timer3Handler(void)
{
    //Reset the flag
    mT3ClearIntFlag();


}


//Interrupt handler to
void __ISR(_TIMER_4_VECTOR, ipl4) _Timer4Handler(void)
{
    //Reset the flag
    mT4ClearIntFlag();

    //Put code here
}


void __ISR(_TIMER_5_VECTOR, ipl4) _Timer5Handler(void)
{
    //Reset the flag
    mT5ClearIntFlag();

    //Put code here
}

void motorinit()
{
    //SET MOTOR1 GPIOS
    //LATECLR = 0x001F;
    //TRISECLR = 0x001F;

    mPORTESetBits(BIT_2);       //set standby bit
}

void motorRstop()
{
   mPORTEClearBits(BIT_0);
   mPORTEClearBits(BIT_1);
}

void motorLstop()
{
   mPORTEClearBits(BIT_3);
   mPORTEClearBits(BIT_4);
}

void motorRfwd()
{
    mPORTESetBits(BIT_1);
    mPORTEClearBits(BIT_0);
}

void motorLfwd()
{
   mPORTESetBits(BIT_3);
   mPORTEClearBits(BIT_4);
}

void motorRrev()
{
   mPORTESetBits(BIT_0);
   mPORTEClearBits(BIT_1);
}

void motorLrev()
{
   mPORTESetBits(BIT_4);
   mPORTEClearBits(BIT_3);
}

void setpwmR(int a)
{
    SetDCOC1PWM(a); // Write new duty cycle
}

void setpwmL(int b)
{
    SetDCOC2PWM(b); // Write new duty cycle
}

void PWMinit()
{
    // init OC4 & OC5 module
    OpenOC1( OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0);
    OpenOC2( OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0);

    // init Timer2 mode and period (PR2) (frequency of 1 / 20 kHz = (3999 + 1) / 80MHz * 1
    OpenTimer2( T2_ON | T2_PS_1_1 | T2_SOURCE_INT, 3999);
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);
}

void turnright()
{
    motorRstop();
    motorLstop();
    TMR4=0x0;
    TMR5=0x0;
    setpwmR(200);
    setpwmL(200);
    motorRrev();
    motorLfwd();
    while(TMR5<300);
}

void turnleft()
{
    motorRstop();
    motorLstop();
    TMR4=0x0;
    TMR5=0x0;
    setpwmR(200);
    setpwmL(200);
    motorLrev();
    motorRfwd();
    while(TMR5<300);
}

uint readpins(int PinNum)
{
    uint avalue;
    while(!IFS0bits.AD1IF); // wait until buffers contain new samples
    //AD1CON1bits.ASAM = 0;   // stop automatic sampling (essentially shut down ADC in this mode)

    if(ReadActiveBufferADC10() == 1)// check which buffers are being written to and read from the other set
    {
        switch(PinNum)
        {
            case 0:
                avalue = ADC1BUF0;
                break;
            case 1:
                avalue = ADC1BUF1;
                break;
            case 2:
                avalue = ADC1BUF2;
                break;
        }
    }
    else
    {
        switch(PinNum)
        {
            case 0:
                avalue = ADC1BUF8;
                break;
            case 1:
                avalue = ADC1BUF9;
                break;
            case 2:
                avalue = ADC1BUFA;
                break;
        }
    }
    //AD1CON1bits.ASAM = 1;           // restart automatic sampling
    mAD1ClearIntFlag();                  // clear ADC interrupt flag
    return avalue;
}
