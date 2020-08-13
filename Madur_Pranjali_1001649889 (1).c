<<<<<<< HEAD
/* Name: Pranjali Madur
  student ID: 1001649889 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "tm4c123gh6pm.h"
#include "uart0.h"

#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define GREEN_LED_MASK 8

#define LDAC (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 5*4)))
#define MAX_CHARS 80
#define MAX_FIELDS 6
#define LDAC_MASK 32
#define PI 3.14159

char str[MAX_CHARS+1];
uint8_t pos[MAX_FIELDS];
uint8_t argCount, argNo, min_arg;
uint16_t raw;
char input;
float frequency = 0;
uint16_t lut[2][4096];

uint32_t deltaphi, phi=0;
uint8_t channel;
float n;
uint8_t count;
uint16_t i;
bool a=false;


// PortA masks
#define UART_TX_MASK 2
#define UART_RX_MASK 1

// PortE masks
#define AIN0_MASK 8
#define AIN1_MASK 4
#define CLOCK_M 16
#define FSS_M 32
#define TX_M 128



void initHw()
{
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    SYSCTL_GPIOHBCTL_R = 0;
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;
    SYSCTL_RCGCADC_R |= SYSCTL_RCGCADC_R0;
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;

    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOF|SYSCTL_RCGC2_GPIOA |SYSCTL_RCGC2_GPIOE| SYSCTL_RCGC2_GPIOB| SYSCTL_RCGC2_GPIOC;
    SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R2;            // turn-on SSI2 clocking

    GPIO_PORTF_DIR_R = GREEN_LED_MASK;  // make bit an output
    GPIO_PORTF_DR2R_R = GREEN_LED_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R = GREEN_LED_MASK;  // enable LED

    // Configure UART0 pins
        GPIO_PORTA_DIR_R |= UART_TX_MASK;                   // enable output on UART0 TX pin
        GPIO_PORTA_DIR_R &= ~UART_RX_MASK;                   // enable input on UART0 RX pin
        GPIO_PORTA_DR2R_R |= UART_TX_MASK;                  // set drive strength to 2mA (not needed since default configuration -- for clarity)
        GPIO_PORTA_DEN_R |= UART_TX_MASK | UART_RX_MASK;    // enable digital on UART0 pins
        GPIO_PORTA_AFSEL_R |= UART_TX_MASK | UART_RX_MASK;  // use peripheral to drive PA0, PA1
        GPIO_PORTA_PCTL_R &= ~(GPIO_PCTL_PA1_M | GPIO_PCTL_PA0_M); // clear bits 0-7
        GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;
                                                            // select UART0 to drive pins PA0 and PA1: default, added for clarity

        // Configure AIN0 as an analog input
            GPIO_PORTE_AFSEL_R |= AIN0_MASK;                 // select alternative functions for AN0 (PE0)
            GPIO_PORTE_DEN_R &= ~AIN0_MASK;                  // turn off digital operation on pin PE3
            GPIO_PORTE_AMSEL_R |= AIN0_MASK;                 // turn on analog operation on pin PE3
       // Configure AIN1 as an analog input
            GPIO_PORTE_AFSEL_R |= AIN1_MASK;                 // select alternative functions for AN1 (PE0)
            GPIO_PORTE_DEN_R &= ~AIN1_MASK;                  // turn off digital operation on pin PE2
            GPIO_PORTE_AMSEL_R |= AIN1_MASK;                 // turn on analog operation on pin PE2

    // Configure LDAC
            GPIO_PORTA_DIR_R |= LDAC_MASK;                       // make bit 2 an output
            GPIO_PORTA_DR2R_R |= LDAC_MASK;                      // set drive strength to 2mA
            GPIO_PORTA_DEN_R |= LDAC_MASK;                       // enable bit for digital
            GPIO_PORTA_PUR_R |= LDAC_MASK;

        // Configure SSI2 pins for SPI configuration
            GPIO_PORTB_DIR_R |= CLOCK_M|FSS_M|TX_M;                        // make bits 4 and 7 outputs
            GPIO_PORTB_DR2R_R |= CLOCK_M|FSS_M|TX_M;                       // set drive strength to 2mA
            GPIO_PORTB_AFSEL_R |= CLOCK_M|FSS_M|TX_M;                      // select alternative functions for MOSI, SCLK pins
            GPIO_PORTB_PCTL_R = GPIO_PCTL_PB7_SSI2TX | GPIO_PCTL_PB4_SSI2CLK|GPIO_PCTL_PB5_SSI2FSS; // map alt fns to SSI2
            GPIO_PORTB_DEN_R |= CLOCK_M|FSS_M|TX_M;                        // enable digital operation on TX, CLK pins
            GPIO_PORTB_PUR_R |= CLOCK_M|FSS_M;                        // must be enabled when SPO=1


            // Configure the SSI2 as a SPI master, mode 3, 8bit operation, 1 MHz bit rate
            SSI2_CR1_R &= ~SSI_CR1_SSE;                      // turn off SSI2 to allow re-configuration
            SSI2_CR1_R = 0;                                  // select master mode
            SSI2_CC_R = 0;                                   // select system clock as the clock source
            SSI2_CPSR_R = 10;                                // set bit rate to 1 MHz (if SR=0 in CR0)
            SSI2_CR0_R = SSI_CR0_FRF_MOTO | SSI_CR0_DSS_16; // set SR=0, mode 3 (SPH=1, SPO=1), 8-bit
            SSI2_CR1_R |= SSI_CR1_SSE;                       // turn on SSI2
            LDAC =1;
        // Configure UART0 to 115200 baud, 8N1 format
        UART0_CTL_R = 0;                                    // turn-off UART0 to allow safe programming
        UART0_CC_R = UART_CC_CS_SYSCLK;                     // use system clock (40 MHz)
        UART0_IBRD_R = 21;                                  // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
        UART0_FBRD_R = 45;                                  // round(fract(r)*64)=45
        UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN;    // configure for 8N1 w/ 16-level FIFO
        UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;
                                                            // enable TX, RX, and module
        // Configure ADC
            ADC0_CC_R = ADC_CC_CS_SYSPLL;                    // select PLL as the time base (not needed, since default value)
            ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3;                // disable sample sequencer 3 (SS3) for programming
            ADC0_EMUX_R = ADC_EMUX_EM3_PROCESSOR;            // select SS3 bit in ADCPSSI as trigger
            ADC0_SSCTL3_R = ADC_SSCTL3_END0;                 // mark first sample as the end
            ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;                 // enable SS3 for operation

}

// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
void waitMillisecond(uint32_t ms)
{
    __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*2 (speculative, so P=1)
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             NOP");                  // 1
    __asm("             B    WMS_LOOP0");       // 1*2 (speculative, so P=1)
    __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}

// Initialize UART0

void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);               // wait if uart0 tx fifo full
    UART0_DR_R = c;                                  // write character to fifo
}

void putsUart0(char* str)
{
    uint8_t i = 0;
    while (str[i] != '\0')
        putcUart0(str[i++]);
}
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE);               // wait if uart0 rx fifo empty
    return UART0_DR_R & 0xFF;                        // get character from fifo
}

void getString(char str[],uint8_t max_chars)
{
    uint8_t count=0;
    char c;
    while(1)
    {


        c=getcUart0();
        if(c==8||c==127)
        {
            if(count>0)
            {
                count--;
                //continue;
            }
//else
  //          {
    //            continue;
      //      }
        }
        else if(c==10||c==13)
        {
            str[count]=0;
            break;
        }
        else if(c>=32)
        {
            str[count++]=c;
            if(count == max_chars)
            {
                str[count] = 0;
                break;
            }
        }

       // else
         //   continue;
    }
}


void parseString(char str[], uint8_t pos[], uint8_t maxFields, uint8_t ArgCount)
{

    int i,j;
    char an_set;
    argCount=0;
    for(i=0,j=0; str[i]!='\0'; i++)
    {
        if((0x30<=str[i]&&str[i]<=0x39)||(str[i]==0x2E)||(str[i]==0x2D)||(0x41<=str[i]&&str[i]<=0x5A)||(0x61<=str[i]&&str[i]<=0x7A))
        {
            an_set='N';
        }
        else
        {
            an_set='D';
        }

        if(an_set!='D' && i==0)
        {
            pos[j]=i;
            j++;
            argCount++;
        }
        else if(an_set!='D' && i!=0)
        {
            i--;
            if((0x30<=str[i]&&str[i]<=0x39)||(str[i]==0x2E)||(str[i]==0x2D)||(0x41<=str[i]&&str[i]<=0x5A)||(0x61<=str[i]&&str[i]<=0x7A))//(an_set=='N')
            {
                i++;
            }
            else
            {
                i++;
                pos[j]=i;
                j++;
                argCount++;
            }
        }

        else if(an_set=='D')
        {
            str[i]='\0';
        }

    }

//
//    for(i=0;i<argCount;i++)
//    {
//        putsUart0("\r\n");
//        putsUart0(&str[pos[i]]);
//        putsUart0("\r\n");
//
//    }
}

char *getArgString(uint8_t argNo)
{
    if(argNo<argCount)
    {
    return &str[pos[argNo]];
    }
return "error";
}

uint32_t getArgInt(uint8_t argNo)
{
    uint32_t value;
    value = atoi(&str[pos[argNo]]);
    return value;
}

float getArgFloat(uint8_t argNo)
{
    float value;
    value = atof(&str[pos[argNo]]);
    return value;
}

bool strCmp(char str1[], char str2[])
{
  int i=0,flag=0;
  while(str1[i] != '\0' && str2[i] != '\0')
     {
         if(str1[i] != str2[i])
         {
             flag = 1;
             break;
         }
         i++;
     }
     if(flag == 0 && str1[i] == '\0' && str2[i] == '\0')
         return true;
     else
         return false;
}

bool isCommand(char str_cmd[], uint8_t minArg)
{
    if(strCmp ((str_cmd), getArgString(0)))
    {

     if(minArg<argCount)
     {
        return true;
    }
    }
    return false;
}

int16_t readAdc0Ss3()
{
    ADC0_PSSI_R |= ADC_PSSI_SS3;                     // set start bit
    while (ADC0_ACTSS_R & ADC_ACTSS_BUSY);           // wait until SS3 is not busy
    return ADC0_SSFIFO3_R;                           // get single result from the FIFO
}

void setDC(uint8_t OUT, float VOLT)
{
    //float Vout=0;
    uint16_t ssiR=0;
    //Vout=((VOLT*2.048/-10.0)+1.024);
    //ssiR=((4096*Vout)/2.048);
    ssiR=(-383.60*(VOLT))+2039.34;
    if(OUT==1)
     {
       SSI2_DR_R =12288+ssiR;
       //SSI2_DR_R =45056+ssiR;
     }
     if(OUT==2)
     {
        SSI2_DR_R =45056+ssiR;
     }
     while (SSI2_SR_R & SSI_SR_BSY);

    LDAC=0;
    LDAC=1;
}

uint32_t dphi(float freq)
{
    return ((freq*4294967296)/100000);
}
void lut_sine(uint8_t ch, float offs, float ampl)
{

   // for(i=0;i<2;i++)

        if(ch==1)
        {
        for(i=0;i<4096;i++)
            {
                lut[0][i]=-384*((ampl*(sin((2*PI*i)/4096)))+offs)+2040+12288;
            }
        }


        if(ch==2)
        {
            for(i=0;i<4096;i++)
            {
                lut[1][i]=-384*((ampl*(sin((2*PI*i)/4096)))+offs)+2040+45056;
            }
        }
}

void lut_square(uint8_t ch, float offs, float ampl)
{

   // for(i=0;i<2;i++)

        if(ch==1)
        {
        for(i=0;i<2048;i++)
            {
                lut[0][i]=(ampl*384)+offs+2040+12288;
            }
        for(i=2048;i<4096;i++)
            {
                lut[0][i]=(-ampl*384)+offs+2040+12288;
            }
        }


        if(ch==2)
        {
            for(i=0;i<2048;i++)
            {
                lut[1][i]=(ampl*384)+offs+2040+45056;
            }
            for(i=2048;i<4096;i++)
            {
                lut[1][i]=(-ampl*384)+offs+2040+45056;
            }
        }
}

void lut_triangle(uint8_t ch, float offs, float ampl)
{

   // for(i=0;i<2;i++)

        if(ch==1)
        {
        for(i=0;i<2048;i++)
            {
                lut[0][i]=-384*((ampl*i)/2048)+offs+2040+12288;
            }
        for(i=2048;i<4096;i++)
            {
                lut[0][i]=-384*(((-ampl*i)/2048)+2*ampl)+offs+2040+12288;
            }
        }


        if(ch==2)
        {
            for(i=0;i<2048;i++)
            {
                lut[1][i]=-384*((ampl*i)/2048)+offs+2040+45056;
            }
            for(i=2048;i<4096;i++)
            {
                lut[1][i]=-384*(((-ampl*i)/2048)+2*ampl)+offs+2040+45056;
            }
        }
}

void lut_sawtooth(uint8_t ch, float offs, float ampl)
{

   // for(i=0;i<2;i++)

        if(ch==1)
        {
        for(i=0;i<4096;i++)
            {
                lut[0][i]=-384*((ampl*i)/4096)+offs+2040+12288;
            }
        }


        if(ch==2)
        {
            for(i=0;i<4096;i++)
            {
                lut[1][i]=-384*((ampl*i)/4096)+offs+2040+45056;
            }
        }
}

void lut_squareduty(uint8_t ch,float offs,float ampl,float duty1)
{
    float duty2;
    duty2=4096-(4096*duty1/100);
    if(ch==1)
           {
           for(i=0;i<duty2;i++)
               {
                   lut[0][i]=(ampl*384)+offs+2040+12288;
               }
           for(i=duty2;i<4096;i++)
               {
                   lut[0][i]=(-ampl*384)+offs+2040+12288;
               }
           }


           if(ch==2)
           {
               for(i=0;i<duty2;i++)
               {
                   lut[1][i]=(ampl*384)+offs+2040+45056;
               }
               for(i=duty2;i<4096;i++)
               {
                   lut[1][i]=(-ampl*384)+offs+2040+45056;
               }
           }
}

void lut_invertsine(uint8_t ch, float offs, float ampl)
{

   // for(i=0;i<2;i++)

        if(ch==1)
        {
        for(i=0;i<4096;i++)
            {
                lut[0][i]=-384*((ampl*(sin((2*PI*i)/4096)))+offs)+2040+12288;
            }
        }


        if(ch==2)
        {
            for(i=0;i<4096;i++)
            {
                lut[1][i]=-384*((-ampl*(sin((2*PI*i)/4096)))+offs)+2040+45056;
            }
        }
}

void lut_invertsquare(uint8_t ch, float offs, float ampl)
{

   // for(i=0;i<2;i++)

        if(ch==1)
        {
        for(i=0;i<2048;i++)
            {
                lut[0][i]=(ampl*384)+offs+2040+12288;
            }
        for(i=2048;i<4096;i++)
            {
                lut[0][i]=(-ampl*384)+offs+2040+12288;
            }
        }


        if(ch==2)
        {
            for(i=0;i<2048;i++)
            {
                lut[1][i]=-(ampl*384)+offs+2040+45056;
            }
            for(i=2048;i<4096;i++)
            {
                lut[1][i]=-(-ampl*384)+offs+2040+45056;
            }
        }
}
void lut_invertsawtooth(uint8_t ch, float offs, float ampl)
{

   // for(i=0;i<2;i++)

        if(ch==1)
        {
        for(i=0;i<4096;i++)
            {
                lut[0][i]=-384*((ampl*i)/4096)+offs+2040+12288;
            }
        }


        if(ch==2)
        {
            for(i=0;i<4096;i++)
            {
                lut[1][i]=-384*((-ampl*i)/4096)+offs+2040+45056;
            }
        }
}

void lut_inverttriangle(uint8_t ch, float offs, float ampl)
{

   // for(i=0;i<2;i++)

        if(ch==1)
        {
        for(i=0;i<2048;i++)
            {
                lut[0][i]=-384*((ampl*i)/2048)+offs+2040+12288;
            }
        for(i=2048;i<4096;i++)
            {
                lut[0][i]=-384*(((-ampl*i)/2048)+2*ampl)+offs+2040+12288;
            }
        }


        if(ch==2)
        {
            for(i=0;i<2048;i++)
            {
                lut[1][i]=-384*((-ampl*i)/2048)+offs+2040+45056;
            }
            for(i=2048;i<4096;i++)
            {
                lut[1][i]=-384*(((-(-ampl)*i)/2048)+2*ampl)+offs+2040+45056;
            }
        }
}

void lut_hilbert(uint8_t ch, float offs, float ampl)
{

   // for(i=0;i<2;i++)

        if(ch==1)
        {
        for(i=0;i<4096;i++)
            {
                lut[0][i]=-384*((ampl*(sin((2*PI*i)/4096)))+offs)+2040+12288;
            }
        }


        if(ch==2)
        {
            for(i=0;i<4096;i++)
            {
                lut[1][i]=-384*((ampl*(-cos((2*PI*i)/4096)))+offs)+2040+45056;
            }
        }
}

void timer1Isr()
{
    //count = 400;
    if(a==true)
    {
        count--;
        if(count==0)
        {
            a=false;
           // lut[0][i]=0;
            TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
           // TIMER1_ICR_R = TIMER_ICR_TATOCINT;
        }
    }

    phi=phi+deltaphi;
//    if (channel==1)
//    {
    SSI2_DR_R=lut[0][phi>>20];
   // }
//    if (channel==2)
//    {
    SSI2_DR_R=lut[1][phi>>20];
   // }
    LDAC=0;
    LDAC=1;

    TIMER1_ICR_R = TIMER_ICR_TATOCINT;   //clear interrupt flag;
    //TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
}

void init_timer1()
{
    // Configure Timer 1 as the time base
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER1_TAILR_R = 400;                            // set load value to 40e6 for 1 Hz interrupt rate
    TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
    NVIC_EN0_R |= 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A)
    TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
}
//void stop_timer()
//{
//    TIMER1_CTL_R |= TIMER_CTL_TAEN;
//}

void calcCycles(float Ncycles)
{
    a = true;
    count=((100000/frequency)*Ncycles);
   // n = Ncycles;
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{

    // Initialize hardware
    initHw();
    //int i;
    GREEN_LED = 1;
    waitMillisecond(500000);
    GREEN_LED = 0;
    //bool valid = true;
    while(1)
    {
        putsUart0("\r\n");
        getString(str, MAX_CHARS);
        parseString(str,pos,MAX_FIELDS, argCount);
       // putsUart0("\r\n");
       // putsUart0(str);



          if(isCommand("reset",0))
                  {
//                      uint32_t x;
//                      x=getArgInt(1);
//                      //sprintf(str, "value:    %d\r\n", x);
//                      putsUart0(str);
//                      float y;
//                      y=getArgFloat(2);
//                      //sprintf(str, "value:    %.2f\r\n", y);
//                      putsUart0(str);
                      NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
                      //  valid = true;
                  }
          else if(isCommand("vtg",1))
          {
               float y=0,z=0;

             //char *input=getArgString(1);
//               uint32_t q=0;
//               q=getArgInt(1);
             // if(input==1)
               if(getArgInt(1)==1)
              {
                  ADC0_SSMUX3_R = 0;     //for AIN0
                  raw= readAdc0Ss3();
                  y=((raw*3.3)/4096);
                  sprintf(str, "raw:    %d\n", raw);
                  putsUart0(str);
                  sprintf(str, "voltage:    %.2f\r\n", y);
                  putsUart0(str);
              }
              if(getArgInt(1)==2)
              {
                   ADC0_SSMUX3_R = 1;     //for AIN1
                   raw= readAdc0Ss3();
                   z=((raw*3.3)/4096);
                   sprintf(str, "raw:    %d\r\n", raw);
                   putsUart0(str);
                   sprintf(str, "voltage:    %.2f\r\n", z);
                   putsUart0(str);
               }

          }
          else if(isCommand("dc",2))
               {
                   int out;
                   float volt;
                   out=getArgInt(1);
                   volt=getArgFloat(2);
                   setDC(out, volt);
               }
          else if(isCommand("sine",4))
                     {
                         float amplitude, offset;
                         channel=getArgInt(1);
                         frequency=getArgFloat(2);
                         amplitude=getArgFloat(4);
                         offset=getArgFloat(3);
                         lut_sine(channel,offset,amplitude);
                         deltaphi=dphi(frequency);
                         //init_timer1();
                     }

          else if(isCommand("run",0))
          {

              init_timer1();
              //timer1Isr();

          }
          else if(isCommand("stop",0))
          {
              SSI2_DR_R=0;
//              lut[0][i]=0;
//              TIMER1_ICR_R = TIMER_ICR_TATOCINT;
              TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
          }
          else if(isCommand("Cycles",1))
          {
              float noOfCycles;
              //ch=getArgInt(1);
              noOfCycles=getArgFloat(1);
              calcCycles(noOfCycles);
          }
          else if(isCommand("square",4))
            {
                 float amplitude, offset;
                 channel=getArgInt(1);
                 frequency=getArgFloat(2);
                 amplitude=getArgFloat(4);
                 offset=getArgFloat(3);
                 lut_square(channel,offset,amplitude);
                 deltaphi=dphi(frequency);
            }
          else if(isCommand("triangle",4))
          {
               float amplitude, offset;
               channel=getArgInt(1);
               frequency=getArgFloat(2);
               amplitude=getArgFloat(4);
               offset=getArgFloat(3);
               lut_triangle(channel,offset,amplitude);
               deltaphi=dphi(frequency);
          }
          else if(isCommand("sawtooth",4))
         {
               float amplitude, offset;
               channel=getArgInt(1);
               frequency=getArgFloat(2);
               amplitude=getArgFloat(4);
               offset=getArgFloat(3);
               lut_sawtooth(channel,offset,amplitude);
               deltaphi=dphi(frequency);
         }
          else if(isCommand("squareduty",5))
          {
               float amplitude, offset,duty;
               channel=getArgInt(1);
               frequency=getArgFloat(2);
               amplitude=getArgFloat(4);
               offset=getArgFloat(3);
               duty=getArgFloat(5);
               lut_squareduty(channel,offset,amplitude,duty);
               deltaphi=dphi(frequency);
          }
          else if(isCommand("invertsine",4))
               {
                   float amplitude, offset;
                   channel=getArgInt(1);
                   frequency=getArgFloat(2);
                   amplitude=getArgFloat(4);
                   offset=getArgFloat(3);
                   lut_invertsine(channel,offset,amplitude);
                   deltaphi=dphi(frequency);
               }
          else if(isCommand("invertsquare",4))
                     {
                         float amplitude, offset;
                         channel=getArgInt(1);
                         frequency=getArgFloat(2);
                         amplitude=getArgFloat(4);
                         offset=getArgFloat(3);
                         lut_invertsquare(channel,offset,amplitude);
                         deltaphi=dphi(frequency);
                     }
          else if(isCommand("invertsawtooth",4))
                   {
                       float amplitude, offset;
                       channel=getArgInt(1);
                       frequency=getArgFloat(2);
                       amplitude=getArgFloat(4);
                       offset=getArgFloat(3);
                       lut_invertsawtooth(channel,offset,amplitude);
                       deltaphi=dphi(frequency);
                   }
          else if(isCommand("inverttriangle",4))
                     {
                         float amplitude, offset;
                         channel=getArgInt(1);
                         frequency=getArgFloat(2);
                         amplitude=getArgFloat(4);
                         offset=getArgFloat(3);
                         lut_inverttriangle(channel,offset,amplitude);
                         deltaphi=dphi(frequency);
                     }
          else if(isCommand("hilbert",4))
                   {
                       float amplitude, offset;
                       channel=getArgInt(1);
                       frequency=getArgFloat(2);
                       amplitude=getArgFloat(4);
                       offset=getArgFloat(3);
                       lut_hilbert(channel,offset,amplitude);
                       deltaphi=dphi(frequency);
                   }
    }
 //return 0;
}


=======
/* Name: Pranjali Madur
  student ID: 1001649889 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "tm4c123gh6pm.h"
#include "uart0.h"

#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define GREEN_LED_MASK 8

#define LDAC (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 5*4)))
#define MAX_CHARS 80
#define MAX_FIELDS 6
#define LDAC_MASK 32
#define PI 3.14159

char str[MAX_CHARS+1];
uint8_t pos[MAX_FIELDS];
uint8_t argCount, argNo, min_arg;
uint16_t raw;
char input;
float frequency = 0;
uint16_t lut[2][4096];

uint32_t deltaphi, phi=0;
uint8_t channel;
float n;
uint8_t count;
uint16_t i;
bool a=false;


// PortA masks
#define UART_TX_MASK 2
#define UART_RX_MASK 1

// PortE masks
#define AIN0_MASK 8
#define AIN1_MASK 4
#define CLOCK_M 16
#define FSS_M 32
#define TX_M 128



void initHw()
{
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    SYSCTL_GPIOHBCTL_R = 0;
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;
    SYSCTL_RCGCADC_R |= SYSCTL_RCGCADC_R0;
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;

    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOF|SYSCTL_RCGC2_GPIOA |SYSCTL_RCGC2_GPIOE| SYSCTL_RCGC2_GPIOB| SYSCTL_RCGC2_GPIOC;
    SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R2;            // turn-on SSI2 clocking

    GPIO_PORTF_DIR_R = GREEN_LED_MASK;  // make bit an output
    GPIO_PORTF_DR2R_R = GREEN_LED_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R = GREEN_LED_MASK;  // enable LED

    // Configure UART0 pins
        GPIO_PORTA_DIR_R |= UART_TX_MASK;                   // enable output on UART0 TX pin
        GPIO_PORTA_DIR_R &= ~UART_RX_MASK;                   // enable input on UART0 RX pin
        GPIO_PORTA_DR2R_R |= UART_TX_MASK;                  // set drive strength to 2mA (not needed since default configuration -- for clarity)
        GPIO_PORTA_DEN_R |= UART_TX_MASK | UART_RX_MASK;    // enable digital on UART0 pins
        GPIO_PORTA_AFSEL_R |= UART_TX_MASK | UART_RX_MASK;  // use peripheral to drive PA0, PA1
        GPIO_PORTA_PCTL_R &= ~(GPIO_PCTL_PA1_M | GPIO_PCTL_PA0_M); // clear bits 0-7
        GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;
                                                            // select UART0 to drive pins PA0 and PA1: default, added for clarity

        // Configure AIN0 as an analog input
            GPIO_PORTE_AFSEL_R |= AIN0_MASK;                 // select alternative functions for AN0 (PE0)
            GPIO_PORTE_DEN_R &= ~AIN0_MASK;                  // turn off digital operation on pin PE3
            GPIO_PORTE_AMSEL_R |= AIN0_MASK;                 // turn on analog operation on pin PE3
       // Configure AIN1 as an analog input
            GPIO_PORTE_AFSEL_R |= AIN1_MASK;                 // select alternative functions for AN1 (PE0)
            GPIO_PORTE_DEN_R &= ~AIN1_MASK;                  // turn off digital operation on pin PE2
            GPIO_PORTE_AMSEL_R |= AIN1_MASK;                 // turn on analog operation on pin PE2

    // Configure LDAC
            GPIO_PORTA_DIR_R |= LDAC_MASK;                       // make bit 2 an output
            GPIO_PORTA_DR2R_R |= LDAC_MASK;                      // set drive strength to 2mA
            GPIO_PORTA_DEN_R |= LDAC_MASK;                       // enable bit for digital
            GPIO_PORTA_PUR_R |= LDAC_MASK;

        // Configure SSI2 pins for SPI configuration
            GPIO_PORTB_DIR_R |= CLOCK_M|FSS_M|TX_M;                        // make bits 4 and 7 outputs
            GPIO_PORTB_DR2R_R |= CLOCK_M|FSS_M|TX_M;                       // set drive strength to 2mA
            GPIO_PORTB_AFSEL_R |= CLOCK_M|FSS_M|TX_M;                      // select alternative functions for MOSI, SCLK pins
            GPIO_PORTB_PCTL_R = GPIO_PCTL_PB7_SSI2TX | GPIO_PCTL_PB4_SSI2CLK|GPIO_PCTL_PB5_SSI2FSS; // map alt fns to SSI2
            GPIO_PORTB_DEN_R |= CLOCK_M|FSS_M|TX_M;                        // enable digital operation on TX, CLK pins
            GPIO_PORTB_PUR_R |= CLOCK_M|FSS_M;                        // must be enabled when SPO=1


            // Configure the SSI2 as a SPI master, mode 3, 8bit operation, 1 MHz bit rate
            SSI2_CR1_R &= ~SSI_CR1_SSE;                      // turn off SSI2 to allow re-configuration
            SSI2_CR1_R = 0;                                  // select master mode
            SSI2_CC_R = 0;                                   // select system clock as the clock source
            SSI2_CPSR_R = 10;                                // set bit rate to 1 MHz (if SR=0 in CR0)
            SSI2_CR0_R = SSI_CR0_FRF_MOTO | SSI_CR0_DSS_16; // set SR=0, mode 3 (SPH=1, SPO=1), 8-bit
            SSI2_CR1_R |= SSI_CR1_SSE;                       // turn on SSI2
            LDAC =1;
        // Configure UART0 to 115200 baud, 8N1 format
        UART0_CTL_R = 0;                                    // turn-off UART0 to allow safe programming
        UART0_CC_R = UART_CC_CS_SYSCLK;                     // use system clock (40 MHz)
        UART0_IBRD_R = 21;                                  // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
        UART0_FBRD_R = 45;                                  // round(fract(r)*64)=45
        UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN;    // configure for 8N1 w/ 16-level FIFO
        UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;
                                                            // enable TX, RX, and module
        // Configure ADC
            ADC0_CC_R = ADC_CC_CS_SYSPLL;                    // select PLL as the time base (not needed, since default value)
            ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3;                // disable sample sequencer 3 (SS3) for programming
            ADC0_EMUX_R = ADC_EMUX_EM3_PROCESSOR;            // select SS3 bit in ADCPSSI as trigger
            ADC0_SSCTL3_R = ADC_SSCTL3_END0;                 // mark first sample as the end
            ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;                 // enable SS3 for operation

}

// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
void waitMillisecond(uint32_t ms)
{
    __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*2 (speculative, so P=1)
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             NOP");                  // 1
    __asm("             B    WMS_LOOP0");       // 1*2 (speculative, so P=1)
    __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}

// Initialize UART0

void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);               // wait if uart0 tx fifo full
    UART0_DR_R = c;                                  // write character to fifo
}

void putsUart0(char* str)
{
    uint8_t i = 0;
    while (str[i] != '\0')
        putcUart0(str[i++]);
}
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE);               // wait if uart0 rx fifo empty
    return UART0_DR_R & 0xFF;                        // get character from fifo
}

void getString(char str[],uint8_t max_chars)
{
    uint8_t count=0;
    char c;
    while(1)
    {


        c=getcUart0();
        if(c==8||c==127)
        {
            if(count>0)
            {
                count--;
                //continue;
            }
//else
  //          {
    //            continue;
      //      }
        }
        else if(c==10||c==13)
        {
            str[count]=0;
            break;
        }
        else if(c>=32)
        {
            str[count++]=c;
            if(count == max_chars)
            {
                str[count] = 0;
                break;
            }
        }

       // else
         //   continue;
    }
}


void parseString(char str[], uint8_t pos[], uint8_t maxFields, uint8_t ArgCount)
{

    int i,j;
    char an_set;
    argCount=0;
    for(i=0,j=0; str[i]!='\0'; i++)
    {
        if((0x30<=str[i]&&str[i]<=0x39)||(str[i]==0x2E)||(str[i]==0x2D)||(0x41<=str[i]&&str[i]<=0x5A)||(0x61<=str[i]&&str[i]<=0x7A))
        {
            an_set='N';
        }
        else
        {
            an_set='D';
        }

        if(an_set!='D' && i==0)
        {
            pos[j]=i;
            j++;
            argCount++;
        }
        else if(an_set!='D' && i!=0)
        {
            i--;
            if((0x30<=str[i]&&str[i]<=0x39)||(str[i]==0x2E)||(str[i]==0x2D)||(0x41<=str[i]&&str[i]<=0x5A)||(0x61<=str[i]&&str[i]<=0x7A))//(an_set=='N')
            {
                i++;
            }
            else
            {
                i++;
                pos[j]=i;
                j++;
                argCount++;
            }
        }

        else if(an_set=='D')
        {
            str[i]='\0';
        }

    }

//
//    for(i=0;i<argCount;i++)
//    {
//        putsUart0("\r\n");
//        putsUart0(&str[pos[i]]);
//        putsUart0("\r\n");
//
//    }
}

char *getArgString(uint8_t argNo)
{
    if(argNo<argCount)
    {
    return &str[pos[argNo]];
    }
return "error";
}

uint32_t getArgInt(uint8_t argNo)
{
    uint32_t value;
    value = atoi(&str[pos[argNo]]);
    return value;
}

float getArgFloat(uint8_t argNo)
{
    float value;
    value = atof(&str[pos[argNo]]);
    return value;
}

bool strCmp(char str1[], char str2[])
{
  int i=0,flag=0;
  while(str1[i] != '\0' && str2[i] != '\0')
     {
         if(str1[i] != str2[i])
         {
             flag = 1;
             break;
         }
         i++;
     }
     if(flag == 0 && str1[i] == '\0' && str2[i] == '\0')
         return true;
     else
         return false;
}

bool isCommand(char str_cmd[], uint8_t minArg)
{
    if(strCmp ((str_cmd), getArgString(0)))
    {

     if(minArg<argCount)
     {
        return true;
    }
    }
    return false;
}

int16_t readAdc0Ss3()
{
    ADC0_PSSI_R |= ADC_PSSI_SS3;                     // set start bit
    while (ADC0_ACTSS_R & ADC_ACTSS_BUSY);           // wait until SS3 is not busy
    return ADC0_SSFIFO3_R;                           // get single result from the FIFO
}

void setDC(uint8_t OUT, float VOLT)
{
    //float Vout=0;
    uint16_t ssiR=0;
    //Vout=((VOLT*2.048/-10.0)+1.024);
    //ssiR=((4096*Vout)/2.048);
    ssiR=(-383.60*(VOLT))+2039.34;
    if(OUT==1)
     {
       SSI2_DR_R =12288+ssiR;
       //SSI2_DR_R =45056+ssiR;
     }
     if(OUT==2)
     {
        SSI2_DR_R =45056+ssiR;
     }
     while (SSI2_SR_R & SSI_SR_BSY);

    LDAC=0;
    LDAC=1;
}

uint32_t dphi(float freq)
{
    return ((freq*4294967296)/100000);
}
void lut_sine(uint8_t ch, float offs, float ampl)
{

   // for(i=0;i<2;i++)

        if(ch==1)
        {
        for(i=0;i<4096;i++)
            {
                lut[0][i]=-384*((ampl*(sin((2*PI*i)/4096)))+offs)+2040+12288;
            }
        }


        if(ch==2)
        {
            for(i=0;i<4096;i++)
            {
                lut[1][i]=-384*((ampl*(sin((2*PI*i)/4096)))+offs)+2040+45056;
            }
        }
}

void lut_square(uint8_t ch, float offs, float ampl)
{

   // for(i=0;i<2;i++)

        if(ch==1)
        {
        for(i=0;i<2048;i++)
            {
                lut[0][i]=(ampl*384)+offs+2040+12288;
            }
        for(i=2048;i<4096;i++)
            {
                lut[0][i]=(-ampl*384)+offs+2040+12288;
            }
        }


        if(ch==2)
        {
            for(i=0;i<2048;i++)
            {
                lut[1][i]=(ampl*384)+offs+2040+45056;
            }
            for(i=2048;i<4096;i++)
            {
                lut[1][i]=(-ampl*384)+offs+2040+45056;
            }
        }
}

void lut_triangle(uint8_t ch, float offs, float ampl)
{

   // for(i=0;i<2;i++)

        if(ch==1)
        {
        for(i=0;i<2048;i++)
            {
                lut[0][i]=-384*((ampl*i)/2048)+offs+2040+12288;
            }
        for(i=2048;i<4096;i++)
            {
                lut[0][i]=-384*(((-ampl*i)/2048)+2*ampl)+offs+2040+12288;
            }
        }


        if(ch==2)
        {
            for(i=0;i<2048;i++)
            {
                lut[1][i]=-384*((ampl*i)/2048)+offs+2040+45056;
            }
            for(i=2048;i<4096;i++)
            {
                lut[1][i]=-384*(((-ampl*i)/2048)+2*ampl)+offs+2040+45056;
            }
        }
}

void lut_sawtooth(uint8_t ch, float offs, float ampl)
{

   // for(i=0;i<2;i++)

        if(ch==1)
        {
        for(i=0;i<4096;i++)
            {
                lut[0][i]=-384*((ampl*i)/4096)+offs+2040+12288;
            }
        }


        if(ch==2)
        {
            for(i=0;i<4096;i++)
            {
                lut[1][i]=-384*((ampl*i)/4096)+offs+2040+45056;
            }
        }
}

void lut_squareduty(uint8_t ch,float offs,float ampl,float duty1)
{
    float duty2;
    duty2=4096-(4096*duty1/100);
    if(ch==1)
           {
           for(i=0;i<duty2;i++)
               {
                   lut[0][i]=(ampl*384)+offs+2040+12288;
               }
           for(i=duty2;i<4096;i++)
               {
                   lut[0][i]=(-ampl*384)+offs+2040+12288;
               }
           }


           if(ch==2)
           {
               for(i=0;i<duty2;i++)
               {
                   lut[1][i]=(ampl*384)+offs+2040+45056;
               }
               for(i=duty2;i<4096;i++)
               {
                   lut[1][i]=(-ampl*384)+offs+2040+45056;
               }
           }
}

void lut_invertsine(uint8_t ch, float offs, float ampl)
{

   // for(i=0;i<2;i++)

        if(ch==1)
        {
        for(i=0;i<4096;i++)
            {
                lut[0][i]=-384*((ampl*(sin((2*PI*i)/4096)))+offs)+2040+12288;
            }
        }


        if(ch==2)
        {
            for(i=0;i<4096;i++)
            {
                lut[1][i]=-384*((-ampl*(sin((2*PI*i)/4096)))+offs)+2040+45056;
            }
        }
}

void lut_invertsquare(uint8_t ch, float offs, float ampl)
{

   // for(i=0;i<2;i++)

        if(ch==1)
        {
        for(i=0;i<2048;i++)
            {
                lut[0][i]=(ampl*384)+offs+2040+12288;
            }
        for(i=2048;i<4096;i++)
            {
                lut[0][i]=(-ampl*384)+offs+2040+12288;
            }
        }


        if(ch==2)
        {
            for(i=0;i<2048;i++)
            {
                lut[1][i]=-(ampl*384)+offs+2040+45056;
            }
            for(i=2048;i<4096;i++)
            {
                lut[1][i]=-(-ampl*384)+offs+2040+45056;
            }
        }
}
void lut_invertsawtooth(uint8_t ch, float offs, float ampl)
{

   // for(i=0;i<2;i++)

        if(ch==1)
        {
        for(i=0;i<4096;i++)
            {
                lut[0][i]=-384*((ampl*i)/4096)+offs+2040+12288;
            }
        }


        if(ch==2)
        {
            for(i=0;i<4096;i++)
            {
                lut[1][i]=-384*((-ampl*i)/4096)+offs+2040+45056;
            }
        }
}

void lut_inverttriangle(uint8_t ch, float offs, float ampl)
{

   // for(i=0;i<2;i++)

        if(ch==1)
        {
        for(i=0;i<2048;i++)
            {
                lut[0][i]=-384*((ampl*i)/2048)+offs+2040+12288;
            }
        for(i=2048;i<4096;i++)
            {
                lut[0][i]=-384*(((-ampl*i)/2048)+2*ampl)+offs+2040+12288;
            }
        }


        if(ch==2)
        {
            for(i=0;i<2048;i++)
            {
                lut[1][i]=-384*((-ampl*i)/2048)+offs+2040+45056;
            }
            for(i=2048;i<4096;i++)
            {
                lut[1][i]=-384*(((-(-ampl)*i)/2048)+2*ampl)+offs+2040+45056;
            }
        }
}

void lut_hilbert(uint8_t ch, float offs, float ampl)
{

   // for(i=0;i<2;i++)

        if(ch==1)
        {
        for(i=0;i<4096;i++)
            {
                lut[0][i]=-384*((ampl*(sin((2*PI*i)/4096)))+offs)+2040+12288;
            }
        }


        if(ch==2)
        {
            for(i=0;i<4096;i++)
            {
                lut[1][i]=-384*((ampl*(-cos((2*PI*i)/4096)))+offs)+2040+45056;
            }
        }
}

void timer1Isr()
{
    //count = 400;
    if(a==true)
    {
        count--;
        if(count==0)
        {
            a=false;
           // lut[0][i]=0;
            TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
           // TIMER1_ICR_R = TIMER_ICR_TATOCINT;
        }
    }

    phi=phi+deltaphi;
//    if (channel==1)
//    {
    SSI2_DR_R=lut[0][phi>>20];
   // }
//    if (channel==2)
//    {
    SSI2_DR_R=lut[1][phi>>20];
   // }
    LDAC=0;
    LDAC=1;

    TIMER1_ICR_R = TIMER_ICR_TATOCINT;   //clear interrupt flag;
    //TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
}

void init_timer1()
{
    // Configure Timer 1 as the time base
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER1_TAILR_R = 400;                            // set load value to 40e6 for 1 Hz interrupt rate
    TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
    NVIC_EN0_R |= 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A)
    TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
}
//void stop_timer()
//{
//    TIMER1_CTL_R |= TIMER_CTL_TAEN;
//}

void calcCycles(float Ncycles)
{
    a = true;
    count=((100000/frequency)*Ncycles);
   // n = Ncycles;
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{

    // Initialize hardware
    initHw();
    //int i;
    GREEN_LED = 1;
    waitMillisecond(500000);
    GREEN_LED = 0;
    //bool valid = true;
    while(1)
    {
        putsUart0("\r\n");
        getString(str, MAX_CHARS);
        parseString(str,pos,MAX_FIELDS, argCount);
       // putsUart0("\r\n");
       // putsUart0(str);



          if(isCommand("reset",0))
                  {
//                      uint32_t x;
//                      x=getArgInt(1);
//                      //sprintf(str, "value:    %d\r\n", x);
//                      putsUart0(str);
//                      float y;
//                      y=getArgFloat(2);
//                      //sprintf(str, "value:    %.2f\r\n", y);
//                      putsUart0(str);
                      NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
                      //  valid = true;
                  }
          else if(isCommand("vtg",1))
          {
               float y=0,z=0;

             //char *input=getArgString(1);
//               uint32_t q=0;
//               q=getArgInt(1);
             // if(input==1)
               if(getArgInt(1)==1)
              {
                  ADC0_SSMUX3_R = 0;     //for AIN0
                  raw= readAdc0Ss3();
                  y=((raw*3.3)/4096);
                  sprintf(str, "raw:    %d\n", raw);
                  putsUart0(str);
                  sprintf(str, "voltage:    %.2f\r\n", y);
                  putsUart0(str);
              }
              if(getArgInt(1)==2)
              {
                   ADC0_SSMUX3_R = 1;     //for AIN1
                   raw= readAdc0Ss3();
                   z=((raw*3.3)/4096);
                   sprintf(str, "raw:    %d\r\n", raw);
                   putsUart0(str);
                   sprintf(str, "voltage:    %.2f\r\n", z);
                   putsUart0(str);
               }

          }
          else if(isCommand("dc",2))
               {
                   int out;
                   float volt;
                   out=getArgInt(1);
                   volt=getArgFloat(2);
                   setDC(out, volt);
               }
          else if(isCommand("sine",4))
                     {
                         float amplitude, offset;
                         channel=getArgInt(1);
                         frequency=getArgFloat(2);
                         amplitude=getArgFloat(4);
                         offset=getArgFloat(3);
                         lut_sine(channel,offset,amplitude);
                         deltaphi=dphi(frequency);
                         //init_timer1();
                     }

          else if(isCommand("run",0))
          {

              init_timer1();
              //timer1Isr();

          }
          else if(isCommand("stop",0))
          {
              SSI2_DR_R=0;
//              lut[0][i]=0;
//              TIMER1_ICR_R = TIMER_ICR_TATOCINT;
              TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
          }
          else if(isCommand("Cycles",1))
          {
              float noOfCycles;
              //ch=getArgInt(1);
              noOfCycles=getArgFloat(1);
              calcCycles(noOfCycles);
          }
          else if(isCommand("square",4))
            {
                 float amplitude, offset;
                 channel=getArgInt(1);
                 frequency=getArgFloat(2);
                 amplitude=getArgFloat(4);
                 offset=getArgFloat(3);
                 lut_square(channel,offset,amplitude);
                 deltaphi=dphi(frequency);
            }
          else if(isCommand("triangle",4))
          {
               float amplitude, offset;
               channel=getArgInt(1);
               frequency=getArgFloat(2);
               amplitude=getArgFloat(4);
               offset=getArgFloat(3);
               lut_triangle(channel,offset,amplitude);
               deltaphi=dphi(frequency);
          }
          else if(isCommand("sawtooth",4))
         {
               float amplitude, offset;
               channel=getArgInt(1);
               frequency=getArgFloat(2);
               amplitude=getArgFloat(4);
               offset=getArgFloat(3);
               lut_sawtooth(channel,offset,amplitude);
               deltaphi=dphi(frequency);
         }
          else if(isCommand("squareduty",5))
          {
               float amplitude, offset,duty;
               channel=getArgInt(1);
               frequency=getArgFloat(2);
               amplitude=getArgFloat(4);
               offset=getArgFloat(3);
               duty=getArgFloat(5);
               lut_squareduty(channel,offset,amplitude,duty);
               deltaphi=dphi(frequency);
          }
          else if(isCommand("invertsine",4))
               {
                   float amplitude, offset;
                   channel=getArgInt(1);
                   frequency=getArgFloat(2);
                   amplitude=getArgFloat(4);
                   offset=getArgFloat(3);
                   lut_invertsine(channel,offset,amplitude);
                   deltaphi=dphi(frequency);
               }
          else if(isCommand("invertsquare",4))
                     {
                         float amplitude, offset;
                         channel=getArgInt(1);
                         frequency=getArgFloat(2);
                         amplitude=getArgFloat(4);
                         offset=getArgFloat(3);
                         lut_invertsquare(channel,offset,amplitude);
                         deltaphi=dphi(frequency);
                     }
          else if(isCommand("invertsawtooth",4))
                   {
                       float amplitude, offset;
                       channel=getArgInt(1);
                       frequency=getArgFloat(2);
                       amplitude=getArgFloat(4);
                       offset=getArgFloat(3);
                       lut_invertsawtooth(channel,offset,amplitude);
                       deltaphi=dphi(frequency);
                   }
          else if(isCommand("inverttriangle",4))
                     {
                         float amplitude, offset;
                         channel=getArgInt(1);
                         frequency=getArgFloat(2);
                         amplitude=getArgFloat(4);
                         offset=getArgFloat(3);
                         lut_inverttriangle(channel,offset,amplitude);
                         deltaphi=dphi(frequency);
                     }
          else if(isCommand("hilbert",4))
                   {
                       float amplitude, offset;
                       channel=getArgInt(1);
                       frequency=getArgFloat(2);
                       amplitude=getArgFloat(4);
                       offset=getArgFloat(3);
                       lut_hilbert(channel,offset,amplitude);
                       deltaphi=dphi(frequency);
                   }
    }
 //return 0;
}


>>>>>>> 3831c1058b219551adf247c0b893bcd4d1aa8f80
