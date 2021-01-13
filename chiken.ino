#include <SHT1x.h>
#include <PID_v1.h>
#include <Wire.h>
#include <EEPROM.h>
#include <TimerOne.h>
#include <avr/wdt.h>
#include "Adafruit_CCS811.h"
Adafruit_CCS811 ccs;

#define SERIAL_RX_BUFFER_SIZE 256
#define SERIAL_TX_BUFFER_SIZE 256

#define dataPin  15
#define clockPin A0
SHT1x sht1x(dataPin, clockPin);


#define Srl Serial
#define HeaterPin A2

#define SLAVE 4
#define ON    1
#define OFF   0
//#define RockTime  15        //rock time motor is 15 minute

#define KEY   7
#define DIN   8
//#define 18b20 9

#define IO_0  13  
#define IO_1  14

#define SimRx  20
#define SimTx  11
#define SimRst 18


#define HST_SCK  17
#define HST_Data 16


//Define Variables we'll be connecting to
//Define the aggressive and conservative Tuning Parameters
char RockTime;       //rock time motor is 15 minute
double MaxTemp,MaxHum;
int ExhustON,ExhustOFF;
double Setpoint, Input, Output;
unsigned int HumDrypoint,Delta;
//double aggKp=4, aggKi=0.2, aggKd=1;
//double consKp=1, consKi=0.05, consKd=0.25;
double aggKp,aggKi,aggKd;
double consKp,consKi,consKd;
//unsigned int Temp=0;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);
//****************************************

typedef struct {
     double aggKp;
     double aggKi;
     double aggKd;
     double consKp;
     double consKi;
     double consKd;
     double  Setpoint;
     double  MaxTemp;
     double  MaxHum;
     int    HumDrypoint;
     int    Delta;
     int    RockTime;      //rock time motor is 15 minute
     int    ExhustON;
     int    ExhustOFF;   
     int    Init;
   }st;

typedef union {
   st V;
   char n[sizeof(st)];
}ut;

ut EE;

  
unsigned int TimVar1_100ms=0;
unsigned int TimVar2_100ms=0;
unsigned int Rock_Time_100ms=0;
char Rock_Flag=0,Motor_Rock=0;

char Led_Heater=0;
char Led_Exhust=0;
char Led_RRack=0;
char Led_LRack=0;
char Led_Hum=0;
char Led_Light=0;
char Led_UPS=0;
char Led_Relay1=0;
char Led_Relay2=0;

unsigned char Status[4]={0,0,0,0};
unsigned char Flag2=0;
int  KeyPin = A10;
unsigned char buf[16];
void BZ(char i);
void SendCommand(char *i);
void Read_Slave(char i);
void Bilink(char i);
void Sensor_Out(unsigned char i);  
int Read_Key(char i);
void Print(char c,char r,char *n);
void SaveEEM(void);
void ReadEEM(void);
void Beep(void);
void Timer_100ms(void);
void ShowPar(void);

void setup()
{
  pinMode(HeaterPin, INPUT);    

  Srl.begin(38400);
  Srl.setTimeout(100);


  Wire.begin(); // join i2c bus (address optional for master)
  Timer1.initialize(100000);  // 100000 us = 100ms = 10Hz
  //Timer1.pwm(fanPin, (dutyCycle / 100) * 1023);
  Timer1.attachInterrupt(Timer_100ms); // to run every 0.1 seconds
  //noInterrupts();
  
  //double aggKp=4, aggKi=0.2, aggKd=1;
  //double consKp=1, consKi=0.05, consKd=0.25;
   ReadEEM();
   if(EE.V.Init!=4){
     EE.V.aggKp=16;
     EE.V.aggKi=0.2;
     EE.V.aggKd=1;
     EE.V.consKp=1;
     EE.V.consKi=0.05;
     EE.V.consKd=0.25;
     EE.V.Setpoint=37.7;
     EE.V.HumDrypoint=65;
     EE.V.Delta=10;
     EE.V.RockTime=15;          //rock time motor is 15 minute     
     EE.V.MaxTemp=38;
     EE.V.MaxHum=80;
     EE.V.ExhustON=5*60;
     EE.V.ExhustOFF=25*60;
     EE.V.Init=4;     
     SaveEEM();
  }
  
  ReadEEM();    


//*********************************
  //initialize the variables we're linked to
 // Input = analogRead(PIN_INPUT);
//  Setpoint = 100;
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
//************************************  
  delay(1000);
  SendCommand("OK");
  delay(100);
  Read_Slave(0);
  if(buf[0]=='O' && buf[1]=='K'){
      Beep();             //"  I2C is ready  "    
     }
  else{
      BZ(ON);             //" I2C Not ready  ";     
      while(1){  Srl.println("i2c not ok");}
}
  
  Srl.println("i2c ok");
  SendCommand("RESET");
  delay(1000);
  
  if(!ccs.begin()){
    Serial.println("Failed to start sensor! Please check your wiring.");
    BZ(1);delay(1000);
    BZ(0);delay(2000);
    BZ(1);delay(1000);
    BZ(0);delay(2000);
    BZ(1);delay(1000);
    BZ(0);
//    while(1);
  } 
    wdt_enable(WDTO_4S);
   
}

void loop()
{ 
  
  int Pac=0,Rock_N=0,FAN1_N=0,FAN2_N=0,FAN3_N=0,HeatCur=0,ExhustCur=0,HumCur=0;
  
  unsigned int Key,y=0; 
  double Temp_c; 
  double Humidity;
  int Co2=0;
  static char BZ_flag = 0;

  //while(!ccs.available());
  //float temp = ccs.calculateTemperature();
  //ccs.setTempOffset(temp - 25.0);
  
    Co2=-1;
    if(ccs.available()){
       //float temp = ccs.calculateTemperature();
       if(!ccs.readData()){
         Co2=(int) ccs.geteCO2();
         Serial.print("CO2: ");
         Serial.print(Co2);
       }
    }
 
  wdt_reset();
  // Read values from the sensor
  Temp_c = sht1x.read_sensor(1); 
  Humidity = sht1x.read_sensor(0);
  
  String cmd ="                                        ";
  int x[10];  
  
  if(Srl.available()) {
      cmd = Srl.readString();      
      Serial.println(cmd);
   }
  
  if ( cmd.substring(0,  5) == "UPAR=" ){
     BZ(1);
     x[0]=cmd.indexOf('=', 0);
     x[1]=cmd.indexOf(',', x[0]);
     x[2]=cmd.indexOf(',', x[1]+1);
     x[3]=cmd.indexOf(',', x[2]+1);
     x[4]=cmd.indexOf(',', x[3]+1);
     x[5]=cmd.indexOf(',', x[4]+1);
     x[6]=cmd.indexOf(',', x[5]+1);
     x[7]=cmd.indexOf(',', x[6]+1);
     x[8]=cmd.indexOf(',', x[7]+1);    
     EE.V.HumDrypoint=cmd.substring(x[0]+1,x[1]).toFloat();
     EE.V.Setpoint=cmd.substring(x[1]+1,x[2]).toFloat();      
     EE.V.MaxTemp=cmd.substring(x[2]+1,x[3]).toFloat();
     EE.V.MaxHum=cmd.substring(x[3]+1,x[4]).toFloat();
     EE.V.ExhustON=cmd.substring(x[5]+1,x[6]).toFloat();
     EE.V.ExhustOFF=cmd.substring(x[6]+1,x[7]).toFloat();
     EE.V.RockTime=cmd.substring(x[7]+1,x[8]).toInt();
     SaveEEM();
     ReadEEM();
  }
  
  if ( cmd.substring(0,  5) == "FPAR=" ){
     BZ(1);
     x[0]=cmd.indexOf('=', 0);
     x[1]=cmd.indexOf(',', x[0]);
     x[2]=cmd.indexOf(',', x[1]+1);
     x[3]=cmd.indexOf(',', x[2]+1);
     x[4]=cmd.indexOf(',', x[3]+1);
     x[5]=cmd.indexOf(',', x[4]+1);
     x[6]=cmd.indexOf(',', x[5]+1);
     x[7]=cmd.indexOf(',', x[6]+1);
     x[8]=cmd.indexOf(',', x[7]+1);    
     EE.V.aggKp=cmd.substring(x[0]+1,x[1]).toFloat();
     EE.V.aggKi=cmd.substring(x[1]+1,x[2]).toFloat();
     EE.V.aggKd=cmd.substring(x[2]+1,x[3]).toFloat();
     EE.V.consKp=cmd.substring(x[3]+1,x[4]).toFloat();
     EE.V.consKi=cmd.substring(x[5]+1,x[6]).toFloat();
     EE.V.consKd=cmd.substring(x[6]+1,x[7]).toFloat();
     EE.V.Delta=cmd.substring(x[7]+1,x[8]).toInt();
     SaveEEM();
     ReadEEM();
  }
      
 
  if(Motor_Rock==1)
     {
      if (Rock_Time_100ms > 10*60*RockTime) {
      //  if (Rock_Time_100ms > 5*10) {
        Rock_Time_100ms=0;
        if(Rock_Flag==0){
           {SendCommand("RRock=1");Led_RRack=1;}
           {SendCommand("LRock=0");Led_LRack=0;}            
           Rock_Flag=1;
        }           
        else {
           {SendCommand("RRock=0");Led_RRack=0;}
           {SendCommand("LRock=1");Led_LRack=1;}
           Rock_Flag=0;
        }        
      }      
     }
     else {
         SendCommand("RRock=0");
         SendCommand("LRock=0");                           
     }
     
  
  if ( cmd.substring(0, 4) == "Hum=")   HumDrypoint=cmd.substring(4).toInt();
  
  if( HumDrypoint > 0){
     if(Humidity < HumDrypoint) {SendCommand("Hum=1");Led_Hum=1;}
     else {SendCommand("Hum=0");Led_Hum=0;}
  }     
  else{
     if(Humidity > abs(HumDrypoint)) {SendCommand("Hum=1");Led_Hum=1;}
     else {SendCommand("Hum=0");Led_Hum=0;}
  }
     
  if ( cmd.substring(0,  6) == "aggKP=" ) {aggKp=cmd.substring(6).toFloat();SaveEEM();}
  if ( cmd.substring(0,  6) == "aggKI=" ) {aggKi=cmd.substring(6).toFloat();SaveEEM();}
  if ( cmd.substring(0,  6) == "aggKD=" ) {aggKd=cmd.substring(6).toFloat();SaveEEM();}
  if ( cmd.substring(0,  7) == "consKP="){consKp=cmd.substring(7).toFloat();SaveEEM();}
  if ( cmd.substring(0,  7) == "consKI="){consKi=cmd.substring(7).toFloat();SaveEEM();}
  if ( cmd.substring(0,  7) == "consKD="){consKd=cmd.substring(7).toFloat();SaveEEM();}
  if ( cmd.substring(0,  6) == "Delta=" ) {Delta=cmd.substring(6).toInt();SaveEEM();}

  if ( cmd == "Motor=1\r" ) {Motor_Rock=1;Rock_Time_100ms=0;}
  if ( cmd == "Motor=0\r" ) Motor_Rock=0;     
  if ( cmd.substring(0,  5) == "Temp=" ) Setpoint=cmd.substring(5).toFloat();   
  //if ( cmd.substring(1, 5) == "Temp=37.77\r" ) Setpoint=atof(cmd.substring(5, 9));   
  
  
  if ( cmd == "Exhust=1\r" ) {SendCommand("Exhust=1");Led_Exhust=1;}
  if ( cmd == "Exhust=0\r" ) {SendCommand("Exhust=0");Led_Exhust=0;}
  if (Setpoint >= MaxTemp ) {SendCommand("Exhust=1");Led_Exhust=1;}
  if (Humidity >= MaxHum ) {SendCommand("Exhust=1");Led_Exhust=1;}
  if (Setpoint < MaxTemp && Humidity < MaxHum) SendCommand("Exhust=0");
  
  
//  if ( cmd == "Hum=1\r" )    SendCommand("Hum=1");  
//  if ( cmd == "Hum=0\r" )    SendCommand("Hum=0");  

  if ( cmd == "RESET\r" ) SendCommand("RESET");
  if ( cmd == "BZ=1\r" ) BZ_flag=1;  
  if ( cmd == "BZ=0\r" ) BZ_flag=0;
  if ( cmd == "Blink=0\r" ) SendCommand("BLINK=0");  
  if ( cmd == "Blink=1\r" ) SendCommand("BLINK=1");    
  if ( cmd == "Light=1\r" )  {SendCommand("Light=1");Led_Light=1;}
  if ( cmd == "Light=0\r" )  {SendCommand("Light=0");Led_Light=0;}
  if ( cmd == "Relay1=1\r" ) {SendCommand("Relay1=1");Led_Relay1=1;}
  if ( cmd == "Relay1=0\r" ) {SendCommand("Relay1=0");Led_Relay1=0;}
  if ( cmd == "Relay2=1\r" ) {SendCommand("Relay2=1");Led_Relay2=1;}
  if ( cmd == "Relay2=0\r" ) {SendCommand("Relay2=0");Led_Relay1=0;}


  SendCommand("PowerAc");
  Read_Slave(1);
  Pac=atoi(buf);
  
//  if(buf[0]=='1') {send_sms("Power Off");call_user();}

  SendCommand("Rock_N");
  Read_Slave(1);
  Rock_N=atoi(buf);

  
  SendCommand("FAN1_N");
  Read_Slave(1);
  FAN1_N=atoi(buf);
  

  SendCommand("FAN2_N");
  Read_Slave(1);
  FAN2_N=atoi(buf);  

  
  SendCommand("FAN3_N");
  Read_Slave(1);
  FAN3_N=atoi(buf);  


  SendCommand("ADC0");
  Read_Slave(1);  
  HeatCur=atoi(buf);  

  
  SendCommand("ADC1");
  Read_Slave(1);
  ExhustCur=atoi(buf);  

  
  SendCommand("ADC2");
  Read_Slave(1);    
  HumCur=atoi(buf);

    
//**********************************
  //Input = analogRead(PIN_INPUT);
  Input =Temp_c;

  double gap = abs(Setpoint-Input);
     //distance away from setpoint
  
  if (gap < Delta)  
  { 
    //we're close to setpoint, use conservative tuning parameters
    myPID.SetTunings(consKp, consKi, consKd);
  }
  else
  {
     //we're far from setpoint, use aggressive tuning parameters
     myPID.SetTunings(aggKp, aggKi, aggKd);
  }

  myPID.Compute();
  //analogWrite(PIN_OUTPUT, Output);
  char str[50];
  sprintf(str,"Heater=%d",(int)Output);
  SendCommand(str);  

  static unsigned long SendTime=0;
  if(abs(millis()-SendTime) > 200){
     BZ(0);
     char s1[50];  
     char s2[10];
     char s3[10];     
     char s4[10];
     if(BZ_flag==1)
        SendCommand("BZ=1");
     else
        SendCommand("BZ=0");
     SendTime = millis();
     Led_Heater=digitalRead(HeaterPin);
     int Status=Led_Relay2;
     Status=Status << 1;
     Status+=Led_Relay1;
     Status=Status << 1;
     Status+=Pac;
     Status=Status << 1;
     Status+=Led_Light;
     Status=Status << 1;
     Status+=Led_Hum;
     Status=Status << 1;
     Status+=Led_LRack;
     Status=Status << 1;
     Status+=Led_RRack;
     Status=Status << 1;
     Status+=Led_Exhust;
     Status=Status << 1;          
     Status+=Led_Heater;
     Co2=5000;
     //sprintf(s1,"ATemp,Hum,Co2,Pac,Fan1_nA%s,%s,%d,%dB",     
     sprintf(s1,"A%s,%s,%d,%d,%d,%d,%d,%d,%d,%d,%d,%dB",dtostrf(Temp_c,2,2,s2),dtostrf(Humidity,2,2,s4),Co2,Pac,Rock_N,FAN1_N,FAN2_N,FAN3_N,HeatCur,ExhustCur,HumCur,Status); 
     Srl.println(s1);     
     //ShowPar();
  }
}

void SaveEEM(void){
  for(int i=0;i<sizeof(EE);i++) {EEPROM[i]=EE.n[i];delay(4);}
}

void ReadEEM(void){
  for(int i=0;i<sizeof(EE);i++) EE.n[i]=EEPROM[i];
  Setpoint=EE.V.Setpoint;
  HumDrypoint=EE.V.HumDrypoint;
  aggKp=EE.V.aggKp;  
  aggKi=EE.V.aggKi;
  aggKd=EE.V.aggKd;
  consKp=EE.V.consKp;
  consKi=EE.V.consKi;
  consKd=EE.V.consKd;
  Delta=EE.V.Delta;  
  RockTime=EE.V.RockTime;
  MaxTemp=EE.V.MaxTemp;
  MaxHum=EE.V.MaxHum;  
  ExhustON=EE.V.ExhustON;
  ExhustOFF=EE.V.ExhustOFF;
}


void SendCommand(char *str) {
  Wire.beginTransmission(SLAVE); // transmit to device #4
  Wire.write(str);        // sends five bytes  
  Wire.write(0);  
  Wire.endTransmission();    // stop transmitting  

}
void BZ(char i) {
  if(i==1)
    SendCommand("BZ=1");
 else
    SendCommand("BZ=0");
}


void Read_Slave(char i ){
  char j=0;
  //Wire.requestFrom(SLAVE, 16);
  //Wire.requestFrom(SLAVE, 16);
  Wire.requestFrom(SLAVE, 16);
  if(i==1) Wire.requestFrom(SLAVE, 16);    // request 6 bytes from slave device #4
  while(i==0){
      Wire.requestFrom(SLAVE, 16);    // request 6 bytes from slave device #4
      buf[0] = Wire.read();
      j=1;
      if(buf[0] > 0) break;
  }
  while (Wire.available()) {        // slave may send less than requested
    buf[j++] = Wire.read();         // recxeive a byte as character    
  }
}


void Bilink(char i) {
  if(i==1)
    SendCommand("BLINKON");
 else
    SendCommand("BLINKOFF");
}


void Beep(void) {
char i;
  for(i=0;i<50;i++){
     SendCommand("BZ=1");
     delay(10);
     SendCommand("BZ=0");
     delay(10);
  }
}

void Timer_100ms(void)
{
TimVar1_100ms++;
TimVar2_100ms++;
Rock_Time_100ms++;
}

void ShowPar(void){
     Serial.print("EE.V.aggKp=");
     Serial.println(EE.V.aggKi);     
     Serial.print("EE.V.aggKd=");
     Serial.println(EE.V.aggKd);
     Serial.print("EE.V.consKp=");
     Serial.println(EE.V.consKp);
     Serial.print("EE.V.consKi=");
     Serial.println(EE.V.consKi);
     Serial.print("EE.V.consKd=");
     Serial.println(EE.V.consKd);
     Serial.print("EE.V.Setpoint=");
     Serial.println(EE.V.Setpoint);
     Serial.print("EE.V.HumDrypoint=");
     Serial.println(EE.V.HumDrypoint);
     Serial.print("EE.V.Delta=");
     Serial.println(EE.V.Delta);
     Serial.print("EE.V.RockTime=");
     Serial.println(EE.V.RockTime);
     Serial.print("EE.V.MaxTemp=");
     Serial.println(EE.V.MaxTemp);
     Serial.print("EE.V.MaxHum=");
     Serial.println(EE.V.MaxHum);
     Serial.print("EE.V.ExhustON=");
     Serial.println(EE.V.ExhustON);
     Serial.print("EE.V.ExhustOFF=");
     Serial.println(EE.V.ExhustOFF);
     Serial.print("EE.V.Init=");
     Serial.println(EE.V.Init);
     Serial.println(sizeof(EE));
     
}
