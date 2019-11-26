//Milestone 2
//Ryan Boylan, Robert Michaelchuck, Alexander Steel

#include <msp430.h>
// Global variables
 unsigned int value=0;
 unsigned int desiredTemp=30;
 float voltvalue=0;
 float difference=0;
 float temperature=0;
 // Function prototypes
void ConfigureAdc(void);
void Timers(void);
void LED(void);
void UART(void);

void main(void)
{
    {
        WDTCTL = WDTPW + WDTHOLD;       // Stop WDT
        BCSCTL1 = CALBC1_1MHZ;          // Set range   DCOCTL = CALDCO_1MHZ;
        BCSCTL2 &= ~(DIVS_3);           // SMCLK = DCO = 1MHz
        P1SEL |= BIT3;                  // ADC input pin P1.3
        ConfigureAdc();                 // ADC set-up function call
        Timers();                       // Configure Timers: Timer A1 is the timer used
        LED();                          // Setup P1.6, 2.1 and 2.5
        UART();                         // Setup UART to get a desired temperature

        __enable_interrupt();           // Enable interrupts.

        while(1) //Get a Temperature Reading
        {
            __delay_cycles(1000);               // Wait for ADC Ref to settle
            ADC10CTL0 |= ENC + ADC10SC;         // Sampling and conversion start
            __bis_SR_register(CPUOFF + GIE);    // Low Power Mode 0 with interrupts enabled, basically shut off processor to save power
            value = ADC10MEM;   // Assigns the value held in ADC10MEM to the integer called ADC_value
            voltvalue = 3.3 * value / 1024; //Voltage Reading from the PTAT
            temperature = ((voltvalue-0.75)/0.01) + 25; //Converted Temperature
            //'PID' control: if higher than temp, full blast, if lower than temp, off
            difference=temperature-desiredTemp;
           /* if(temperature>desiredTemp) //if temperature of the PTAT is higher than the
                TA1CCR1 = 1022; //TA1CCR1 controls P2.1, which is connected to the FET for the fan, set the fan to near full blast, full blast would be 1025
            else TA1CCR1 = 0; //Shut off Fan
            */
            if(difference>3)
                TA1CCR1 = 1025;
            else if(difference>1)
                TA1CCR1 = 800;
            else if(difference == 0)
                TA1CCR1 = 600;
            else if(difference < 0 )
                TA1CCR1 = 200;

        }
    }
}


// ADC10 interrupt service routine
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void)
{
    __bic_SR_register_on_exit(CPUOFF);        // Return to active mode }
}
//UART Interrupt
#pragma vector=USCIAB0RX_VECTOR     //Interrupt routine
__interrupt void UART_ISR(void)
{
  while (!(IFG2&UCA0TXIFG));        //Wait until a byte is ready, is USCI_A0 TX buffer ready?
  TA1CCR2 = UCA0RXBUF*4;        //Set the blue led pwm, visually shows how hot the voltage regulator should be
  desiredTemp = UCA0RXBUF;      //Get the value from UART, value is in deg C
  UCA0TXBUF = UCA0RXBUF;        //Return the input over uart
}


void LED(void)              //LED function
{
    P1DIR |= BIT6;          //Set Red LED Direction Register to output
    P1SEL |= BIT6;          //Enable pwm for Red LED
    P2DIR |= (BIT1 + BIT5); //Set Green(Bit1) and Blue(BIT5) LED Direction Register to output
    P2SEL |= (BIT1 + BIT5); //Enable pwm for Green and Blue LED

}

void UART(void)             //UART function
{
    P1SEL |= BIT1 + BIT2;   //P1.1 = RXD P1.2 = TXD
    P1SEL2 |= BIT1 + BIT2;  //P1.1 = RXD P1.2 = TXD

    UCA0CTL1 |= UCSSEL_2;   //smclk
    UCA0BR0 = 104;          //1MHz 9600 baud rate
    UCA0BR1 = 0;            //1MHz 9600 baud rate
    UCA0MCTL = UCBRS0;      //modulation UBRSx = 1
    UCA0CTL1 &= ~UCSWRST;   //initialize usci state machine

    IE2 |= UCA0RXIE;        //enable RX interrupt
}

void Timers(void)           //Timers function
{
    //timer A1
    TA1CTL |= TASSEL_2 + MC_1;     //set smclk, up mode
    TA1CCTL1 |= OUTMOD_7;   //set/reset output
    TA1CCTL2 |= OUTMOD_7;   //set/reset output
    TA1CCR0 = 1025;         //pwm period
    TA1CCR1 = 0;            //Fan PWM, also red LED
    TA1CCR2 = 0;            //blue from uart
}


// Function containing ADC set-up
void ConfigureAdc(void)
{
    ADC10CTL1 = INCH_3 + ADC10DIV_3 ;         // Channel 3, ADC10CLK/3
    ADC10CTL0 = SREF_0 + ADC10SHT_3 + ADC10ON + ADC10IE;  // Vcc & Vss as reference, Sample and hold for 64 Clock cycles, ADC on, ADC interrupt enable
    ADC10AE0 |= BIT3;                         // ADC input enable P1.3
}
