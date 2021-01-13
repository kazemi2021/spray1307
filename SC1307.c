/*
;CodeVisionAVR C Compiler V3.12 Advanced
;(C) Copyright 1998-2014 Pavel Haiduc, HP InfoTech s.r.l.
;http://www.hpinfotech.com

;Build configuration    : Release
;Chip type              : ATmega8
;Program type           : Application
;Clock frequency        : 8.000000 MHz
;Memory model           : Small
;Optimize for           : Size
;(s)printf features     : int, width
;(s)scanf features      : int, width
;External RAM size      : 0
;Data Stack size        : 256 byte(s)
;Heap size              : 0 byte(s)
;Promote 'char' to 'int': Yes
;'char' is unsigned     : Yes
;8 bit enums            : Yes
;Global 'const' stored in FLASH: No
;Enhanced function parameter passing: Yes
;Enhanced core instructions: On
;Automatic register allocation for global variables: On
;Smart register allocation: On
*/

#include <mega8.h>
#include <delay.h>
#include <i2c.h>
#include <ds1307.h>


void get_time_date (void);
void set_time_date (void);
void Get_UserCode (void);
void BlinkAll(void);

#define Delay_led1  PORTB.0
#define Delay_led2  PORTB.1
#define Delay_led3  PORTB.2
#define Level_led   PORTB.3
#define Blink_led   PORTB.4
#define Relay       PORTB.5
#define Key         PIND.5


//#asm
//   .equ __i2c_port=0x15	;PORTC
//   .equ __sda_bit=1
//   .equ __scl_bit=0
//#endasm

// I2C Bus functions
#asm
   .equ __i2c_port=0x12 ;PORTD
   .equ __sda_bit=3
   .equ __scl_bit=4
#endasm

unsigned char sec=1,minute=1,hour=1;
unsigned char year=20,month=12,day=26,weekday=1;
eeprom unsigned char EE_year,EE_month,EE_day,EE_weekday,EE_minute;
unsigned int counter=0,counter2=0, second=0;
eeprom unsigned int i=1;
eeprom unsigned char Enable=0;
eeprom unsigned char Customer=0;
eeprom unsigned char On_flag=1;
unsigned char t;
unsigned char KeyFlag=0;//SensFlag=0;
unsigned char BlinkT=0;
unsigned char flag2=0;
unsigned char Level_Blink=0;
unsigned int t1ms=0;
unsigned int pulse=0;
unsigned int ResetTime=0;
unsigned int Sipher=0;

//int pulsetime=0;
//int pulseflag=0;


// External Interrupt 0 service routine
interrupt [EXT_INT0] void ext_int0_isr(void)
{

// Place your code here
pulse++;
flag2=1;           //hand detect
if(pulse>5){
    pulse=0;
 }
  
 }
 
// Timer1 overflow interrupt service routine
interrupt [TIM1_OVF] void timer1_ovf_isr(void)
{

 // Reinitialize Timer1 value   38kh frequenc generation
TCNT1H=0xFF98 >> 8;
TCNT1L=0xFF98 & 0xff;
 PORTC.4 ^= 1;
 // Place your code here
 
}

// Timer2 overflow interrupt service routine
interrupt [TIM2_OVF] void timer2_ovf_isr(void)
{

 //    PORTC.4=~ PORTC.4;
    // pulsetime++;
 
     t1ms++;
     counter++;
     counter2++;
     if(counter2 >=90){      //125000/7=17857   50 ms moudulation of 38kh
       counter2 =0;
       TIMSK ^= 4;
     }
 
 
 	   if(counter >=1780){      //125000/7=17857     1 sec
 		second++;
 		counter=0;
         BlinkT++;
          if(BlinkT>=10)
          {
          ResetTime++;
             if(ResetTime>=10)
             {
             pulse=0;
             ResetTime=0;
             }
          BlinkT=0;
          PORTD.7=~ PORTD.7;
          Blink_led=~Blink_led;
          if(Level_Blink==1)
             Level_led = ~Level_led;
          }
          }
   TCNT2=248;
 }    //80microsec

// Declare your global variables here

void main(void)
{
 // Declare your local variables here
 
 // Input/Output Ports initialization
 // Port B initialization
 // Func7=In Func6=In Func5=Out Func4=Out Func3=Out Func2=Out Func1=Out Func0=Out
  // State7=T State6=T State5=0 State4=0 State3=0 State2=0 State1=0 State0=0
  PORTB=0x00;
  DDRB=0x3F;
  // Port C initialization
  // Func6=In Func5=In Func4=In Func3=In Func2=In Func1=In Func0=In
  // State6=T State5=T State4=T State3=T State2=T State1=T State0=T
  PORTC=0x20;
  DDRC=0x10;
  
  // Port D initialization
  // Function: Bit7=Out Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
  DDRD=(1<<DDD7) | (0<<DDD6) | (0<<DDD5) | (0<<DDD4) | (0<<DDD3) | (0<<DDD2) | (0<<DDD1) | (0<<DDD0);
  // State: Bit7=0 Bit6=P Bit5=P Bit4=P Bit3=P Bit2=P Bit1=P Bit0=P
  PORTD=(0<<PORTD7) | (1<<PORTD6) | (1<<PORTD5) | (1<<PORTD4) | (1<<PORTD3) | (1<<PORTD2) | (1<<PORTD1) | (1<<PORTD0);
  
  // Timer/Counter 0 initialization
  // Port D initialization
  // Func7=Out Func6=In Func5=In Func4=In Func3=In Func2=In Func1=In Func0=In
  // State7=0 State6=P State5=P State4=T State3=T State2=T State1=T State0=T
  //PORTD=0b10000000;
  //DDRD= 0b01100000;
  
  
  // Timer/Counter 0 initialization
  // Clock source: System Clock
  // Clock value: Timer 0 Stopped
  TCCR0=0x00;
  TCNT0=0x00;
  // Timer/Counter 1 initialization
  // Clock source: System Clock
  // Clock value: 8000.000 kHz
  // Mode: CTC top=OCR1A
  // OC1A output: Disconnected
  // OC1B output: Disconnected
  // Noise Canceler: Off
  // Input Capture on Falling Edge
  // Timer Period: 0.013 ms
  // Timer1 Overflow Interrupt: On
  // Input Capture Interrupt: Off
  // Compare A Match Interrupt: Off
  // Compare B Match Interrupt: Off
  TCCR1A=(0<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (0<<WGM11) | (0<<WGM10);
  TCCR1B=(0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (1<<WGM12) | (0<<CS12) | (0<<CS11) | (1<<CS10);
  TCNT1H=0xFF;
  TCNT1H=0xFF;
  TCNT1L=0x98;
  ICR1H=0x00;
  ICR1L=0x00;
  OCR1AH=0x00;
  OCR1AL=0x67;
  OCR1BH=0x00;
  OCR1BL=0x00;
  
  
  
  // Timer/Counter 2 initialization
  // Clock source: System Clock
  // Clock value: 125.000 kHz
  // Mode: Normal top=0xFF
  // OC2 output: Disconnected
  ASSR=0x00;
  TCCR2=0x04;
  TCNT2=248;
  OCR2=0x00;
  
  // External Interrupt(s) initialization
  // INT0: On
  // INT0 Mode: Falling Edge
  // INT1: Off
  GICR|=0x40;
  MCUCR=0x02;
  GIFR=0x40;
  // Timer(s)/Counter(s) Interrupt(s) initialization
  //TIMSK=0x40;
  // Timer(s)/Counter(s) Interrupt(s) initialization
  TIMSK=(0<<OCIE2) | (1<<TOIE2) | (0<<TICIE1) | (0<<OCIE1A) | (0<<OCIE1B) | (1<<TOIE1) | (0<<TOIE0);
  // USART initialization
  // USART disabled
  UCSRB=0x00;
  // Analog Comparator initialization
  // Analog Comparator: Off
  // Analog Comparator Input Capture by Timer/Counter 1: Off
  ACSR=0x80;
  SFIOR=0x00;
  // ADC initialization
  // ADC disabled
  ADCSRA=0x00;
  
  // SPI initialization
  // SPI disabled
  SPCR=0x00;
  // TWI initialization
  // TWI disabled
  TWCR=0x00;
  // Global enable interrupts
  i2c_init();
  delay_ms(100);
  rtc_init(0,1,0);        //rs=frequency 0=1 hz   1=4096hz 2= 8192 hz 3 32768hz     sqwe=1 active 0=deactive  out=0 =1
  delay_ms(100);
  #asm("sei")
  if(Key == 0){       //go customer mode
     if(Customer==0){
        set_time_date();  // 2020/01/01 weekday=1
     }
     Customer=1;
     Relay=1;
     delay_ms(500);
     Relay=0;
     delay_ms(500);
     Relay=1;
     delay_ms(500);
     Relay=0;
     delay_ms(500);
     Relay=1;
     delay_ms(500);
     Relay=0;
     BlinkAll();
    }
  while(Customer == 1){
     if(Enable == 1) break;
     get_time_date();
     if( day+month+year == 0){        //error rtc not found
        BlinkAll();
     }
     if(On_flag == 1){
        EE_year=year;
        EE_month=month;
        EE_day=day;
  //      EE_minute = minute;
        On_flag=0;
        }
     break;
  }
  
  
  while (1)
    {
        if(Customer == 1 && Enable == 0){
          get_time_date();
  //       if( (year-EE_year)*52+weekday - EE_weekday > 6)
         if( ((year-EE_year)*12+month-EE_month)*30+ day-EE_day >= 0){
  //        if( minute - EE_minute > 1){
             Get_UserCode();
          }
        }
  
       if(i>3 || i==0) i=1;
  
       if(pulse>0){  //>0      //ms(30) //>5
          delay_ms(20);}
  
       if(Key==0 && KeyFlag==0 && t1ms > 20)
        {
          KeyFlag=1;
          i++;
          if(i>3)i=1;
        }
  
        if(Key==1){
           KeyFlag=0;
           t1ms = 0;
           }
  
        switch(i)
        {
         case 1:
          Delay_led1=1;
          Delay_led2=0;
          Delay_led3=0;
          t=5;
  
          if(flag2==1 )
          {
            second=0;
            while(second<t){
               Relay=1;
               }
            second=0;
            while(second<5){
               Relay=0;
               }
            flag2=0;
            pulse=0;
          }
  
         break;
  
         case 2:
          Delay_led1=1;
          Delay_led2=1;
          Delay_led3=0;
          t=7;
          if(flag2==1)
          {
            second=0;
            while(second<t){
               Relay=1;
               }
            second=0;
            while(second<5){
               Relay=0;
               }
            flag2=0;
            pulse=0;
          }
         break;
  
         case 3:
          Delay_led1=1;
          Delay_led2=1;
          Delay_led3=1;
          t=9;
          if(flag2==1)
          {
            second=0;
            while(second<t){
               Relay=1;
               }
            second=0;
            while(second<5){
               Relay=0;
               }
             flag2=0;
             pulse=0;
          }
         break;
  
         //default:
         //PORTA=0b10101010;
         }
  
        if(Level_Blink==0){
          if (PINC.5==0) Level_led=0;
          else Level_led=1;
        }
    }

  }

void get_time_date (void)
   {
   rtc_get_date(&weekday,&day,&month,&year);
   rtc_get_time(&hour,&minute,&sec);
   }

void set_time_date (void)
   {

   rtc_set_date(weekday,day,month,year);
   rtc_set_time(hour,minute,1);
  
   //void rtc_get_time(unsigned char *hour,unsigned char *min,unsigned char *sec);
  //void rtc_set_time(unsigned char hour,unsigned char min,unsigned char sec);
  //void rtc_get_date(unsigned char *week_day, unsigned char *day,unsigned char *month,unsigned char *year);
  //void rtc_set_date(unsigned char week_day, unsigned char day,unsigned char month,unsigned char year);
   }

void Get_UserCode (void){
       int   x,y,Sipher;
       int   s=0,f=0;
       int   i=0,j=0;
       x=day/10;
       y=day%10;
       Sipher=x*x;
       Sipher+=y*y;
       second=0;
       Level_Blink=1;
       while(1){
          if(Key == 0 && f==0){Relay=1;second=0;i++;f=1;}
          delay_ms(20);
          if(Key ==1) {Relay=0; f=0;}
          if(i ==0) {second=0;}
          if(second > 100 ) break;
       }
       Relay=1;
       delay_ms(300);
       Relay=0;
       second=0;
       while(1){
          if(Key == 0 && f==0){Relay=1;second=0;j++;f=1;}
          delay_ms(20);
          if(Key == 1) {Relay=0;f=0;}
          if(j ==0) second=0;
          if(second > 100 ) break;
       }
       Relay=1;
       delay_ms(300);
       Relay=0;
       if( Sipher == (i-1)*10+(j-1) ) {
          Level_Blink=0;
          Enable=1;
          Relay=1;
          delay_ms(3000);
          Relay=0;
          return;
       }
       else {
          Enable=0;
          Relay=0;
       }
       BlinkAll();
  }

void BlinkAll(void){
       while(1){
          Delay_led1=1;
          Delay_led2=1;
          Delay_led3=1;
          Level_led =1;
          Blink_led =1;
          delay_ms(1000);
          Delay_led1=0;
          Delay_led2=0;
          Delay_led3=0;
          Level_led =0;
          Blink_led =0;
          delay_ms(1000);
          }
  }

