// include the library header
// no font headers have to be included
#include <openGLCD.h>
#include <stdlib.h>
#include <Wire.h>
#include <OneWire.h>
#include <EEPROM.h>
//#include <LCD.h>
//#include <LiquidCrystal_I2C.h>
#include <DallasTemperature.h>
#include <SPI.h>
#include <SFE_BMP180.h>
#include <Adafruit_MAX31855.h>

/*
//LCD I2C
#define I2C_ADDR          0x27        //Define I2C Address where the PCF8574A is
#define BACKLIGHT_PIN      3
#define En_pin             2
#define Rw_pin             1
#define Rs_pin             0
#define D4_pin             4
#define D5_pin             5
#define D6_pin             6
#define D7_pin             7
*/

//PUSHBUTTON INPUT PINS
#define PUSHBUTTONPIN         2
#define PUSHBUTTON2PIN        3
#define PUSHBUTTON3PIN        4
#define PUSHBUTTON4PIN        5

//SERIAL BUS WIRE FOR DS18B20
#define ONE_WIRE_BUS          7

// Thermocouple instance with software SPI on any three digital IO pins.
#define MAXDO                 48
#define MAXCS                 46
#define MAXCLK                44

#define TEMPERATURE_PRECISION 9
#define ALARMPIN              12
#define RELAYPIN              11

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress insideThermometer, outsideThermometer, refThermometer;
SFE_BMP180 pressure;

//Thermocouple initialise
Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS, MAXDO);

/*
//Initialise the LCD
LiquidCrystal_I2C      lcd(I2C_ADDR, En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin);
*/

//FPS counting
int fpsCount=0;
long fpsTime1=0;
long fpsTime2=0;
long fpsDuration = 1000;
float duration = 3.00;
long fpsSampleInterval = 1000;
float fps=0.0;
long timeCount1 = 0;
long timeCount2 = 0;
long timeDuration = 0;

int EGTval = 50;
char EGTstr[4];
int EGTcentreX;
int EGTlimit;
int EGTmax = -500;
bool EGTalarm = false;
bool EGTalarmActive;
bool EGTalarmSilenced = false;
bool EGTalarmState = LOW;

unsigned int totalpsi;
float kpaError = -2.75; //+-3.45 max
float mBARatmo = 1014;
float PSIatmo = 14.7;
float PSIvolts = 0;
float PSIabs = 0;
float PSIval = 0;
float INHGval = 0;
char PSIstr[4];
int PSIcentreX;
float PSIlimit;
float PSImax = -500;
bool PSIalarm = false;
bool PSIalarmActive;
bool PSIalarmSilenced = false;
bool PSIalarmState = LOW;

float ECUvolts = 4.78;
int R1 = 1012;
int R2 = 0;
int totalWATER = 0;
float WATERvolts = 0;
int WATERval = 0;
char WATERstr[4];
int WATERcentreX;
int WATERlimit;
int WATERmax = -500;
bool WATERalarm = false;
bool WATERalarmActive;
bool WATERalarmSilenced = false;
bool WATERalarmState = LOW;

//Pushbutton
bool button = 0; //button off state
bool oldButton = 0; //last state read
bool button2 = 0; //button off state
bool oldButton2 = 0; //last state read
bool button3 = 0; //button off state
bool oldButton3 = 0; //last state read
bool button4 = 0; //button off state
bool oldButton4 = 0; //last state read
long lastButtonCheck = 0;
long buttonCheckInterval = 20;
int screenState = 0; //which screen is displayed
int menuItem = 0;
int menuItemY = 0;

//Voltages
//unsigned int total; //holds readings
unsigned int totalpri;
unsigned int totalaux;
unsigned int totalsolar;
//float Aref = 1.07; //change to actual Aref voltage on board
float Aref = 5.00; //change to actual Aref voltage on board
float resRatio = 24.36; //voltage divider ratio
//float resRatio = 23.0; //voltage divider ratio

//ANALOG PINS
int pri_battPin = 0; // Input from primary Battery
int aux_battPin = 1; // Input from auxiliary Battery
int solar_Pin = 2; // Input from solar Battery
int psi_pin = 3; //Intput for boost pressure
int water_pin = 4; //Input for water temp


float priBattVolt = 0; //Temporarily store the primary Voltage
float auxBattVolt = 0; //Temporarily store the secondary Voltage
float solarChargeVolt = 0; //Temporarily store the secondary Voltage

//Voltage Alarms
bool priAlarm = false;
bool priOverAlarmActive;
bool priLowAlarmActive;
bool priAlarmSilenced = false;
bool priAlarmState = LOW;
bool auxAlarm = false;
bool auxOverAlarmActive;
bool auxLowAlarmActive;
bool auxAlarmSilenced = false;
bool auxAlarmState = LOW;
bool lowVoltRelayState = LOW;
//float battLow = 11.8;
//float overCharge = 14.9;
float priBattLow;
float priOverCharge;
float auxBattLow;
float auxOverCharge;


//Screen Update
long previousFastCalcs = 0;
long fastCalcRate = 200;
long previousFastPrint = 0;
long fastPrintRate = 200;
long interval = 0;
long tempReq = 0;

bool alarmState = LOW;
long previousAlarm = 0;
long alarmInterval = 500;


//Temperatures
int tempReqState = false;
float tempC1 = 0;
float tempC2 = 0;
float tempC3 = 3;
float refHi;
float refLo;
bool refAlarm = false;
bool refHiAlarmActive;
bool refLoAlarmActive;
bool refAlarmSilenced = false;
bool refAlarmState = LOW;



void screenState0()
{
  GLCD.ClearScreen();
  GLCD.SelectFont(System5x7);
  GLCD.DrawRoundRect(0, 0, 42, 32, 5, PIXEL_ON); 
  GLCD.DrawRoundRect(43, 0, 42, 32, 5, PIXEL_ON); 
  GLCD.DrawRoundRect(86, 0, 42, 32, 5, PIXEL_ON); 
  GLCD.DrawRoundRect(0, 32, 128, 12, 3, PIXEL_ON); 
  GLCD.DrawRoundRect(0, 44, 128, 12, 3, PIXEL_ON);  

  GLCD.GotoXY(13, 2);  
  GLCD.print("EGT "); 
  GLCD.GotoXY(27, 23);  
  GLCD.write(128);
  GLCD.print("C"); 

  GLCD.GotoXY(49, 2);  
  GLCD.print("BOOST"); 

  GLCD.GotoXY(92, 2);  
  GLCD.print("WATER"); 
  GLCD.GotoXY(113, 23);  
  GLCD.write(128);
  GLCD.print("C"); 

  GLCD.SelectFont(newbasic3x5);
  GLCD.GotoXY(3, 33);  
  GLCD.print("P:"); 
  GLCD.GotoXY(45, 33);  
  GLCD.print("A:"); 
  GLCD.GotoXY(87, 33);  
  GLCD.print("S:");

  GLCD.GotoXY(2, 45);  
  GLCD.print("IN:"); 
  GLCD.GotoXY(41, 45);  
  GLCD.print("OUT:");
  GLCD.GotoXY(84, 45);  
  GLCD.print("REF:"); 

  GLCD.GotoXY(0, 57);  
  GLCD.print("MENU             MAX     SILENCE"); 
}

void screenState1()
{
  GLCD.ClearScreen();
  GLCD.SelectFont(System5x7);  
  GLCD.GotoXY(9, 0);  
  GLCD.print("EGT MAX    : "); 
  GLCD.GotoXY(9, 10);  
  GLCD.print("BOOST MAX  : "); 
  GLCD.GotoXY(9, 20);  
  GLCD.print("WATER MAX  : "); 
  GLCD.GotoXY(9, 30);
  GLCD.print("FRIDGE LO  : ");   
  GLCD.GotoXY(9, 40);
  GLCD.print("FRIDGE HI  : ");
  GLCD.GotoXY(9, 50); 
  GLCD.print("SAVE & EXIT");  
  
  GLCD.SelectFont(newbasic3x5);
  GLCD.GotoXY(0, 57);  
  GLCD.print("ENTER     -        +        NEXT"); 
}

void screenState2()
{
  GLCD.ClearScreen();
  GLCD.SelectFont(System5x7);  
  GLCD.GotoXY(9, 0);
  GLCD.print("PRI BATT LO: ");
  GLCD.GotoXY(9, 10);   
  GLCD.print("PRI BATT HI: "); 
  GLCD.GotoXY(9, 20); 
  GLCD.print("AUX BATT LO: ");
  GLCD.GotoXY(9, 30); 
  GLCD.print("AUX BATT HI: "); 
  GLCD.GotoXY(9, 40); 
  GLCD.print("USE DEFAULTS");
  GLCD.GotoXY(9, 50); 
  GLCD.print("SAVE & EXIT"); 
  
  GLCD.SelectFont(newbasic3x5);
  GLCD.GotoXY(0, 57);  
  GLCD.print("ENTER     -        +        NEXT"); 
}

void screenState3()
{
  GLCD.ClearScreen();
  GLCD.SelectFont(Verdana12_bold);
  GLCD.GotoXY(11, 1);  
  GLCD.print("PEAK READINGS");
  GLCD.SelectFont(System5x7);
  GLCD.DrawRoundRect(0, 14, 42, 32, 5, PIXEL_ON); 
  GLCD.DrawRoundRect(43, 14, 42, 32, 5, PIXEL_ON); 
  GLCD.DrawRoundRect(86, 14, 42, 32, 5, PIXEL_ON);  

  GLCD.GotoXY(13, 16);  
  GLCD.print("EGT "); 
  GLCD.GotoXY(27, 37);  
  GLCD.write(128);
  GLCD.print("C"); 

  GLCD.GotoXY(49, 16);  
  GLCD.print("BOOST"); 
  GLCD.GotoXY(64, 37);  
  GLCD.print("psi");

  GLCD.GotoXY(92, 16);  
  GLCD.print("WATER"); 
  GLCD.GotoXY(113, 37);  
  GLCD.write(128);
  GLCD.print("C"); 

  GLCD.SelectFont(newbasic3x5);
  GLCD.GotoXY(0, 57);  
  GLCD.print("MENU            ACTIVE     RESET"); 
}

void setup()
{
  // Initialize the GLCD 
  GLCD.Init();


    /*
    lcd.begin (16,2);
    //Switch on the backlight
    lcd.setBacklightPin(BACKLIGHT_PIN,POSITIVE);
    lcd.setBacklight(HIGH);
    lcd.setCursor(0, 0);// Place the Cursor on the Fist line first character position on the LCD screen
    lcd.print("WELCOME  LEWIS"); // First Line 
    */
    
    Serial.begin(115200);
    
    if (pressure.begin())
    Serial.println("BMP180 init success");
    mBARatmo = getPressure();
    PSIatmo = mBARatmo * 0.0145038;
    Serial.println(PSIatmo);

    sensors.begin();
    if (!sensors.getAddress(insideThermometer, 0)) Serial.println("Unable to find address for Device 0"); 
    if (!sensors.getAddress(outsideThermometer, 1)) Serial.println("Unable to find address for Device 1");
    if (!sensors.getAddress(refThermometer, 2)) Serial.println("Unable to find address for Device 1"); 
    sensors.setResolution(insideThermometer, TEMPERATURE_PRECISION);
    sensors.setResolution(outsideThermometer, TEMPERATURE_PRECISION);
    sensors.setResolution(refThermometer, TEMPERATURE_PRECISION);

    Serial.print("Device 0 Resolution: ");
    Serial.print(sensors.getResolution(insideThermometer), DEC); 
    Serial.println();

    Serial.print("Device 1 Resolution: ");
    Serial.print(sensors.getResolution(outsideThermometer), DEC); 
    Serial.println();

    Serial.print("Device 2 Resolution: ");
    Serial.print(sensors.getResolution(refThermometer), DEC); 
    Serial.println();

    //analogReference(INTERNAL1V1);
    //Aref = readVcc();
    //Serial.println(Aref);

    pinMode(PUSHBUTTONPIN, INPUT); //pushbutton input
    pinMode(PUSHBUTTON2PIN, INPUT); //pushbutton 2 input
    pinMode(PUSHBUTTON3PIN, INPUT); //pushbutton 3 input
    pinMode(PUSHBUTTON4PIN, INPUT); //pushbutton 4 input
    pinMode(RELAYPIN, OUTPUT); //Low voltage cutoff relay
    pinMode(ALARMPIN, OUTPUT); //alarm output
    digitalWrite(RELAYPIN,LOW);
    digitalWrite(ALARMPIN,LOW);
    

 //GET SETTINGS FROM EEPROM
    //value = EEPROM.read(address);

    /*EGTlimit = EEPROM.read(0);
    EGTlimit = EGTlimit * 10;
    
    PSIlimit = EEPROM.read(1);
    
    WATERlimit = EEPROM.read(2);
    
    refLo = EEPROM.read(3);
    refLo = ((refLo-55)/10);
    
    refHi = EEPROM.read(4);
    refHi = ((refHi-55)/10);
    
    priBattLow = EEPROM.read(5);
    priBattLow = priBattLow / 10;
    
    priOverCharge = EEPROM.read(6);
    priOverCharge = priOverCharge / 10;
    
    auxBattLow = EEPROM.read(7);
    auxBattLow = auxBattLow / 10;
    
    auxOverCharge = EEPROM.read(8);
    auxOverCharge = auxOverCharge / 10;*/

    
    EGTalarmActive = EEPROM.read(0); 
    EGTlimit = EEPROM.read(1);
    EGTlimit = EGTlimit * 10;

    PSIalarmActive = EEPROM.read(2);
    PSIlimit = EEPROM.read(3);

    WATERalarmActive = EEPROM.read(4);
    WATERlimit = EEPROM.read(5);

    refLoAlarmActive = EEPROM.read(6);
    refLo = EEPROM.read(7);
    refLo = ((refLo-55)/10);

    refHiAlarmActive = EEPROM.read(8);
    refHi = EEPROM.read(9);
    refHi = ((refHi-55)/10);

    priLowAlarmActive = EEPROM.read(10);
    priBattLow = EEPROM.read(11);
    priBattLow = priBattLow / 10;

    priOverAlarmActive = EEPROM.read(12);
    priOverCharge = EEPROM.read(13);
    priOverCharge = priOverCharge / 10;

    auxLowAlarmActive = EEPROM.read(14);
    auxBattLow = EEPROM.read(15);
    auxBattLow = auxBattLow / 10;

    auxOverAlarmActive = EEPROM.read(16);
    auxOverCharge = EEPROM.read(17);
    auxOverCharge = auxOverCharge / 10;
    
     

 // Select the font for the default text area
  GLCD.SelectFont(System5x7);

  GLCD.DrawBitmap(NISSAN,0,0);
  delay(2000);
  screenState0();
  
}

void loop()
{
  unsigned long currentMillis = millis();

//CALCS


//200ms CALCS
if (currentMillis - previousFastCalcs >= fastCalcRate) {

      
      
      if (tempReqState == false){ //EGT and Atmospheric pressure read and request DS18B20 temps, then alternate to DS18B20 temp reads. Spreads out time-consuming calcs.
        /*timeCount2=millis();
        timeDuration = timeCount2-timeCount1;
        Serial.print(" Temp request refresh time: ");
        Serial.println(timeDuration);
        timeCount1 = millis();*/
        //REQUEST TEMPS
        sensors.setWaitForConversion(false);
        sensors.requestTemperatures();
        tempReqState = true;

        //CALC AMBIENT PSI
        mBARatmo = getPressure();
        PSIatmo = mBARatmo * 0.0145038;
        //Serial.println(PSIatmo);

        //CALC EGT
        EGTval = thermocouple.readCelsius();
        //EGTval = EGTval+1;
        if (EGTval > EGTmax){
            EGTmax = EGTval;
        }

        //CALC WATER
        WATERvolts = 0;
        R2 = 0;
        totalWATER = 0; //reset total
        analogRead(water_pin);//one unused reading to clear any ghost charge
        for(int z = 0; z < 16; z++){
            totalWATER = totalWATER + analogRead(water_pin);
        }
  
        WATERvolts = (totalWATER/16)*0.00488;
        R2 = (WATERvolts*R1)/(ECUvolts-WATERvolts);
        WATERval = (1/(0.000319971+0.000497434*(log(R2))-0.000001719*pow(log(R2),3))-273.15);
        if (WATERval > WATERmax){
            WATERmax = WATERval;
        }

        //MAIN BATTERY
        totalpri = 0; //reset total
        analogRead(pri_battPin);//one unused reading to clear any ghost charge
        for(int x = 0; x < 16; x++){
          totalpri = totalpri + analogRead(pri_battPin);
        }
        priBattVolt = (totalpri/16) * Aref * resRatio / 1024;
        //priBattVolt = 22.2;
      
           
        //AUX BATTERY
        totalaux = 0; //reset total
        analogRead(aux_battPin);//one unused reading to clear any ghost charge
        for(int y = 0; y < 16; y++){
          totalaux = totalaux + analogRead(aux_battPin);
        }
        auxBattVolt = (totalaux/16) * Aref * resRatio / 1024;
        //auxBattVolt = 22.2;
  
          
        //SOLAR BATTERY
        totalsolar = 0; //reset total
        analogRead(solar_Pin);//one unused reading to clear any ghost charge
        for(int z = 0; z < 16; z++){
          totalsolar = totalsolar + analogRead(solar_Pin);
        }
        solarChargeVolt = (totalsolar/16) * Aref * resRatio / 1024;
        //solarChargeVolt = 22.2;
        //delay(13); 
        
              
      }
      else {
        /*timeCount2=millis();
        timeDuration = timeCount2-timeCount1;
        Serial.print(" Temp read refresh duration: ");
        Serial.println(timeDuration);
        timeCount1 = millis();*/
        tempC1 = sensors.getTempCByIndex(0);
        tempC2 = sensors.getTempCByIndex(1);
        tempC3 = sensors.getTempCByIndex(2);
        tempReqState = false;
      //tempC1 = 125;
      //tempC2 = -55;
        
      }
      
      
      //CALC PSI
      totalpsi = 0; //reset total
      analogRead(psi_pin);//one unused reading to clear any ghost charge
      for(int x = 0; x < 16; x++){
        totalpsi = totalpsi + analogRead(psi_pin);
      }
      //PSIvolts = (totalpsi/16) * Aref / 1024;
      //PSIabs = ((PSIvolts/0.022) +17.4)*0.145;
      PSIvolts = (totalpsi/16);
      if (tempC1>=0 and tempC1<=85){
        PSIabs = (((((float)PSIvolts/(float)1024)+0.04)/0.004)+kpaError)*0.145;
      }
      else if (tempC1<0 and tempC1>=-40){
        PSIabs = (((((float)PSIvolts/(float)1024)+0.04)/0.004)+(kpaError*(1+abs(tempC1)/20))*0.145);
      }
      else if (tempC1>85 and tempC1<=125){
        PSIabs = (((((float)PSIvolts/(float)1024)+0.04)/0.004)+(kpaError*(1+(tempC1-85)/20))*0.145);
      }
      //boost = ((((float)mapval/(float)1023+0.04)/.004)*.145)-atmpsi;
      //PSIval = PSIabs;
      PSIval = PSIabs - PSIatmo;
      INHGval = (PSIabs - PSIatmo) * 2.036;
      
      if (PSIval > PSImax){
          PSImax = PSIval;
      }
      
      //ALARMS

     //PSI ALARM
     if (PSIval >= PSIlimit and PSIalarmActive){
        if (!PSIalarmSilenced) {
          PSIalarm = true;
          //Serial.println("Boost over pressure");
        }
        else {
          PSIalarm = false;  
        }
      }
      else{
        PSIalarm = false;
        PSIalarmSilenced = false;
      }
     
     //EGT ALARM
     if (EGTval >= EGTlimit and EGTalarmActive){
        if (!EGTalarmSilenced) {
          EGTalarm = true;
          //Serial.println("EGT over temp");
        }
        else {
          EGTalarm = false;  
        }
      }
      else{
        EGTalarm = false;
        EGTalarmSilenced = false;
      }

      //WATER ALARM
     if (WATERval >= WATERlimit and WATERalarmActive){
        if (!WATERalarmSilenced) {
          WATERalarm = true;
          //Serial.println("Water over temp");
        }
        else {
          WATERalarm = false;  
        }
      }
      else{
        WATERalarm = false;
        WATERalarmSilenced = false;
      }

     //PRI ALARM
     if (priBattVolt <= priBattLow and priBattVolt >= 0.1 and priLowAlarmActive){
        if (!priAlarmSilenced) {
          priAlarm = true;
          //Serial.println("pri batt low");
        }
        else {
          priAlarm = false;  
        }
      }
      else if (priBattVolt > priOverCharge and priOverAlarmActive){
        if (!priAlarmSilenced) {
          priAlarm = true;
          //Serial.println("pri over charge");
        }
        else {
          priAlarm = false;  
        }
      }
      else{
        priAlarm = false;
        priAlarmSilenced = false;
      }

     //AUX ALARM
     if (auxBattVolt <= auxBattLow and auxBattVolt >= 0.1 and auxLowAlarmActive){
        //Serial.println("aux batt low");
        if (!auxAlarmSilenced) {
          auxAlarm = true;
          
        }
        else {
          auxAlarm = false;  
        }
      }
      else if (auxBattVolt >= auxOverCharge and auxOverAlarmActive){
        if (!auxAlarmSilenced) {
          auxAlarm = true;
          //Serial.println("aux over charge");
        }
        else {
          auxAlarm = false;  
        }
      }
      else{
        auxAlarm = false;
        auxAlarmSilenced = false;
      }

      //LOW VOLT CUTOFF
      if (auxBattVolt <= auxBattLow){
        lowVoltRelayState = HIGH;
      }
      else {
        lowVoltRelayState = LOW;
      }

     //REF ALARM
      if (tempC3 <= refLo and refLoAlarmActive){
        if (!refAlarmSilenced) {
          refAlarm = true;
          //Serial.println("fridge temp low");
        }
        else {
          refAlarm = false;  
        }
      }
      else if (tempC3 >= refHi and refHiAlarmActive){
        if (!refAlarmSilenced) {
          refAlarm = true;
          //Serial.println("fridge temp hi");
        }
        else {
          refAlarm = false;  
        }
      }
      else{
        refAlarm = false;
        refAlarmSilenced = false;
      }

      previousFastCalcs = currentMillis;
  
}

  

if (millis() - lastButtonCheck >= buttonCheckInterval){
  

    //BUTTON PRESS
    if (digitalRead(PUSHBUTTONPIN) == HIGH){ //If Pin 2 is high (at 5V)
      //button = 1 - button; //inverts value
      button = 1; //sets hi value 
    }
    
    else if (digitalRead(PUSHBUTTONPIN) == LOW){ //If Pin 2 is high (at 5V)
      button = 0; //sets lo value 
    }
    
    //BUTTON 2 PRESS
    if (digitalRead(PUSHBUTTON2PIN) == HIGH){ //If Pin 2 is high (at 5V)
      //button = 1 - button; //inverts value
      button2 = 1; //sets hi value 
    }
    
    else if (digitalRead(PUSHBUTTON2PIN) == LOW){ //If Pin 2 is high (at 5V)
      button2 = 0; //sets lo value 
    }
    
    //BUTTON 3 PRESS
    if (digitalRead(PUSHBUTTON3PIN) == HIGH){ //If Pin 2 is high (at 5V)
      //button = 1 - button; //inverts value
      button3 = 1; //sets hi value 
    }
    
    else if (digitalRead(PUSHBUTTON3PIN) == LOW){ //If Pin 2 is high (at 5V)
      button3 = 0; //sets lo value 
    }
    
    //BUTTON 4 PRESS
    if (digitalRead(PUSHBUTTON4PIN) == HIGH){ //If Pin 5 is high (at 5V)
      //button = 1 - button; //inverts value
      button4 = 1; //sets hi value 
    }
    else if (digitalRead(PUSHBUTTONPIN) == LOW){ //If Pin 5 is high (at 5V)
      button4 = 0; //sets lo value 
    }  
    
    lastButtonCheck = millis();
}


//BUTTONS
switch (screenState) {
  case 0:
                
        if(button == 1 and oldButton == 0){
          //enter functions for with button on, such as swap backlight
            screenState = 1;
            screenState1();
        }

        if(button3 == 1 and oldButton3 == 0){
          //enter functions for with button on, such as swap backlight
            screenState = 3;
            screenState3();
        }

       
        if(button4 == 1 and oldButton4 == 0){
          //enter functions for with button on, such as swap backlight
          priAlarmSilenced = true;
          auxAlarmSilenced = true;
          refAlarmSilenced = true;
          PSIalarmSilenced = true;
          EGTalarmSilenced = true;
          WATERalarmSilenced = true;
          //Serial.println("Silenced");
        }     
        
  break;

  case 1:
                
        if(button == 1 and oldButton == 0){
          //enter functions for with button on, such as swap backlight

            switch (menuItem){
              case 0: //EGT
                   EGTalarmActive = !EGTalarmActive;
              break;

              case 1: //PSI
                   PSIalarmActive = !PSIalarmActive;
              break;

              case 2: //WATER
                   WATERalarmActive = !WATERalarmActive;
              break;

              case 3: //Refridgerator Low
                   refLoAlarmActive = !refLoAlarmActive;
              break;

              case 4: //Refridgerator Hi
                   refHiAlarmActive = !refHiAlarmActive;
              break;

              case 5: //Refridgerator Hi
                  screenState = 0;
                  screenState0();
                  menuItem = 0;
                  
                  EEPROM.write(0, EGTalarmActive);
                  EEPROM.write(1, EGTlimit/10);
                  EEPROM.write(2, PSIalarmActive);
                  EEPROM.write(3, PSIlimit);
                  EEPROM.write(4, WATERalarmActive);
                  EEPROM.write(5, WATERlimit);
                  EEPROM.write(6, refLoAlarmActive);
                  EEPROM.write(7, ((refLo*10)+55));
                  EEPROM.write(8, refHiAlarmActive);
                  EEPROM.write(9, ((refHi*10)+55));
                  EEPROM.write(10, priLowAlarmActive);
                  EEPROM.write(11, priBattLow*10);
                  EEPROM.write(12, priOverAlarmActive);
                  EEPROM.write(13, priOverCharge*10);
                  EEPROM.write(14, auxLowAlarmActive);
                  EEPROM.write(15, auxBattLow*10);
                  EEPROM.write(16, auxOverAlarmActive);
                  EEPROM.write(17, auxOverCharge*10);
              break;
            }
            
            
            
        }

        if(button2 == 1 and oldButton2 == 0){
          //enter functions for with button on, such as swap backlight
            switch (menuItem){
              case 0: //EGT
                   EGTlimit = EGTlimit - 10;
              break;

              case 1: //PSI
                   PSIlimit = PSIlimit - 0.5;
              break;

              case 2: //WATER
                   WATERlimit = WATERlimit - 1;
              break;

              case 3: //Refridgerator Low
                   refLo = refLo - 0.5;
              break;

              case 4: //Refridgerator Hi
                   refHi = refHi - 0.5;
              break;
            }
        }

        if(button3 == 1 and oldButton3 == 0){
          //enter functions for with button on, such as swap backlight
            switch (menuItem){
              case 0: //EGT
                   EGTlimit = EGTlimit + 10;
              break;

              case 1: //PSI
                   PSIlimit = PSIlimit + 0.5;
              break;

              case 2: //WATER
                   WATERlimit = WATERlimit + 1;
              break;

              case 3: //Refridgerator Low
                   refLo = refLo + 0.5;
              break;

              case 4: //Refridgerator Hi
                   refHi = refHi + 0.5;
              break;
            }
           
        }      
       
        if(button4 == 1 and oldButton4 == 0){
          //enter functions for with button on, such as swap backlight
          //Serial.print("Menu Item: ");
          menuItem = menuItem + 1;
          //Serial.println(menuItem);
          if (menuItem == 6){
            menuItem = 0;
            screenState = 2;
            screenState2();
          }
        }   
  break;

  case 2:
        if(button == 1 and oldButton == 0){
          //enter functions for with button on, such as swap backlight
            switch (menuItem){
              case 0: //primary batt low
                   priLowAlarmActive = !priLowAlarmActive;
              break;

              case 1: //primary batt overcharge
                   priOverAlarmActive = !priOverAlarmActive;
              break;

              case 2: //aux batt low
                   auxLowAlarmActive = !auxLowAlarmActive;
              break;

              case 3: //aux batt overcharge
                   auxOverAlarmActive = !auxOverAlarmActive;
              break;

              case 4:

                    EGTalarmActive = true;
                    EGTlimit = 550;
                    PSIalarmActive = true;
                    PSIlimit = 14;
                    WATERalarmActive = true;
                    WATERlimit = 93;
                    refLoAlarmActive = true;
                    refLo = 1;
                    refHiAlarmActive = true;
                    refHi = 6;
                    priLowAlarmActive = true;      
                    priBattLow = 11.5;
                    priOverAlarmActive = true;
                    priOverCharge = 14.9;
                    auxLowAlarmActive = true;
                    auxBattLow = 11.5;
                    priOverAlarmActive = true;
                    auxOverCharge = 14.9;
                    
              break;

              case 5: //Refridgerator Hi
                  screenState = 0;
                  screenState0();
                  menuItem = 0;
                  
                  EEPROM.write(0, EGTalarmActive);
                  EEPROM.write(1, EGTlimit/10);
                  EEPROM.write(2, PSIalarmActive);
                  EEPROM.write(3, PSIlimit);
                  EEPROM.write(4, WATERalarmActive);
                  EEPROM.write(5, WATERlimit);
                  EEPROM.write(6, refLoAlarmActive);
                  EEPROM.write(7, ((refLo*10)+55));
                  EEPROM.write(8, refHiAlarmActive);
                  EEPROM.write(9, ((refHi*10)+55));
                  EEPROM.write(10, priLowAlarmActive);
                  EEPROM.write(11, priBattLow*10);
                  EEPROM.write(12, priOverAlarmActive);
                  EEPROM.write(13, priOverCharge*10);
                  EEPROM.write(14, auxLowAlarmActive);
                  EEPROM.write(15, auxBattLow*10);
                  EEPROM.write(16, auxOverAlarmActive);
                  EEPROM.write(17, auxOverCharge*10);
              break;
            }
            
        }

        if(button2 == 1 and oldButton2 == 0){
          //enter functions for with button on, such as swap backlight
            switch (menuItem){
              case 0: //primary batt low
                   priBattLow = priBattLow - 0.1;
              break;

              case 1: //primary batt overcharge
                   priOverCharge = priOverCharge - 0.1;
              break;

              case 2: //aux batt low
                   auxBattLow = auxBattLow - 0.1;
              break;

              case 3: //aux batt overcharge
                   auxOverCharge = auxOverCharge - 0.1;
              break;
            }
        }

        if(button3 == 1 and oldButton3 == 0){
          //enter functions for with button on, such as swap backlight
            switch (menuItem){
              case 0: //primary batt low
                   priBattLow = priBattLow + 0.1;
              break;

              case 1: //primary batt overcharge
                   priOverCharge = priOverCharge + 0.1;
              break;

              case 2: //aux batt low
                   auxBattLow = auxBattLow + 0.1;
              break;

              case 3: //aux batt overcharge
                   auxOverCharge = auxOverCharge + 0.1;
              break;

          
            }
        
           
        }      

       
        if(button4 == 1 and oldButton4 == 0){
          //enter functions for with button on, such as swap backlight
          //Serial.print("Menu Item: ");
          menuItem = menuItem + 1;
          //Serial.println(menuItem);
          if (menuItem == 6){
            menuItem = 0;
            screenState = 1;
            screenState1();
          }

        } 
  break;

  case 3:
        if(button == 1 and oldButton == 0){
          //enter functions for with button on, such as swap backlight
            screenState = 1;
            screenState1();
        }

        if(button3 == 1 and oldButton3 == 0){
          //enter functions for with button on, such as swap backlight
            screenState = 0;
            screenState0();
        }

       
        if(button4 == 1 and oldButton4 == 0){
          //enter functions for with button on, such as swap backlight
          EGTmax = -500;
          PSImax = -500;
          WATERmax = -500;
        }

  break;
}

//PRINTING

if (currentMillis - previousFastPrint >= fastPrintRate) {
  
  switch (screenState) {
  case 0:

      //TEST ALARMS
      //EGTalarm = true;
      //PSIalarm = true;
      //WATERalarm = true;

     
      //Print PSI
        GLCD.SelectFont(Verdana12_bold);
        //itoa(PSIval,PSIstr,10);
        GLCD.FillRect(44, 10, 40, 12, PIXEL_OFF);
        if (PSIval >= 0){
          dtostrf(PSIval,4,1,PSIstr);
          PSIcentreX = 43 + 42/2 - (GLCD.StringWidth(PSIstr))/2;
          GLCD.GotoXY(PSIcentreX, 10); 
          GLCD.print(PSIstr);
          GLCD.SelectFont(System5x7);
          GLCD.GotoXY(58, 23);  
          GLCD.print(" psi");
        }
        else {
          dtostrf(INHGval,4,1,PSIstr);
          PSIcentreX = 43 + 42/2 - (GLCD.StringWidth(PSIstr))/2;
          GLCD.GotoXY(PSIcentreX, 10); 
          GLCD.print(PSIstr);
          GLCD.SelectFont(System5x7);
          GLCD.GotoXY(58, 23);  
          GLCD.print("inHg");
        }

        //GLCD.SelectFont(System5x7);
        //GLCD.GotoXY(49, 23);
        if (PSIalarm == true) {
          if (PSIalarmState == LOW) {
            PSIalarmState = HIGH;  
            GLCD.DrawHLine(PSIcentreX,21,GLCD.StringWidth(PSIstr)-1,PIXEL_ON);
          }
          else {
            PSIalarmState = LOW;  
            //GLCD.DrawHLine(54,21,20,PIXEL_OFF);
          }
        }
        
      
             
     
      //Print EGT
        GLCD.SelectFont(Verdana12_bold);
        GLCD.FillRect(1, 10, 40, 12, PIXEL_OFF);
        if (!isnan(EGTval)){
          itoa(EGTval,EGTstr,10);
          EGTcentreX = 42/2 - (GLCD.StringWidth(EGTstr))/2;
          GLCD.GotoXY(EGTcentreX, 10); 
          GLCD.print(EGTstr);
        }

        //GLCD.SelectFont(System5x7);
        //GLCD.GotoXY(6, 23);
        if (EGTalarm == true) {
          if (EGTalarmState == LOW) {
            EGTalarmState = HIGH;  
            GLCD.DrawHLine(EGTcentreX,21,GLCD.StringWidth(EGTstr)-1,PIXEL_ON);
          }
          else {
            EGTalarmState = LOW;  
            //GLCD.DrawHLine(11,21,20,PIXEL_OFF);
          }
        }
       
        
      //Print WATER
        GLCD.SelectFont(Verdana12_bold);
        GLCD.FillRect(87, 10, 40, 12, PIXEL_OFF);
        if (WATERvolts > 0.2 and WATERvolts < 4.733){
          itoa(WATERval,WATERstr,10);
          WATERcentreX = 86 + 42/2 - (GLCD.StringWidth(WATERstr))/2;
          GLCD.GotoXY(WATERcentreX, 10); 
          GLCD.print(WATERstr);
        }

        //GLCD.SelectFont(System5x7);
        //GLCD.GotoXY(92, 23);
        if (WATERalarm == true) {
          if (WATERalarmState == LOW) {
            WATERalarmState = HIGH;  
            GLCD.DrawHLine(WATERcentreX,21,GLCD.StringWidth(WATERstr)-1,PIXEL_ON);
          }
          else {
            WATERalarmState = LOW;  
            //GLCD.DrawHLine(97,21,20,PIXEL_OFF);
          }
        }
      
      
        //Print Voltages
        GLCD.SelectFont(System5x7);
        GLCD.FillRect(10, 35, 30, 7, PIXEL_OFF);
        GLCD.GotoXY(11, 35); 
        if (priBattVolt < 0.1){
          GLCD.print("    ");
        }
        else if (priBattVolt > 25){
          GLCD.print(" HI ");
        }
        else{
            if (priBattVolt < 10.0){
              GLCD.print(" ");
            }
            GLCD.print(priBattVolt,1);
            GLCD.print("V");
        }
        
        if (priAlarm == true) {
          if (priAlarmState == LOW) {
            priAlarmState = HIGH;
            GLCD.DrawHLine(3,41,6,PIXEL_ON);
        }
        else {
          priAlarmState = LOW;
          GLCD.DrawHLine(3,41,6,PIXEL_OFF);
        }
      }
      else {
          GLCD.DrawHLine(3,41,6,PIXEL_OFF);
      }
      
        GLCD.FillRect(52, 35, 30, 7, PIXEL_OFF);
        GLCD.GotoXY(53, 35); 
       if (auxBattVolt < 0.1){
          GLCD.print("    ");
        }
        else if (auxBattVolt > 25){
          GLCD.print(" HI ");
        }
        else{
            if (auxBattVolt < 10.0){
              GLCD.print(" ");
            }
            GLCD.print(auxBattVolt,1); 
            GLCD.print("V"); 
        }
      
        if (auxAlarm == true) {
          if (auxAlarmState == LOW) {
            auxAlarmState = HIGH;
            GLCD.DrawHLine(45,41,6,PIXEL_ON);
        }
        else {
          auxAlarmState = LOW;
          GLCD.DrawHLine(45,41,6,PIXEL_OFF);
        }
      }
      else {
          GLCD.DrawHLine(45,41,6,PIXEL_OFF);
      }
      
      
        GLCD.FillRect(94, 35, 30, 7, PIXEL_OFF);
        GLCD.GotoXY(95, 35); 
        if (solarChargeVolt < 0.1){
          GLCD.print("    ");
        }
        else if (solarChargeVolt > 25){
          GLCD.print(" HI ");
        }
        else{
            if (solarChargeVolt < 10.0){
              GLCD.print(" ");
            }
            GLCD.print(solarChargeVolt,1);
            GLCD.print("V");
        }
      
      
        
      //PRINT TEMPS
      
        GLCD.GotoXY(13, 47); 
        if ((tempC1 < -55)||(tempC1 > 125)){
          GLCD.print("ERR");
        }
        else{
          if (tempC1 < 10.0 and tempC1 >= 0){
            GLCD.print(" ");
          }
          GLCD.print(tempC1,0);
          GLCD.write(128);
          GLCD.SelectFont(newbasic3x5);
          GLCD.print("C"); 
          GLCD.SelectFont(System5x7);
        }
          
        GLCD.GotoXY(56, 47);
        if ((tempC2 < -55)||(tempC2 > 125)){
          GLCD.print("ERR");
        }
        else{
          if (tempC2 < 10.0 and tempC2 >= 0){
            GLCD.print(" ");
          }
          GLCD.print(tempC2,0);
          GLCD.write(128);
          GLCD.SelectFont(newbasic3x5);
          GLCD.print("C"); 
          GLCD.SelectFont(System5x7);
        }
      
        GLCD.GotoXY(99, 47);
        if ((tempC3 < -126)||(tempC2 > 126)){
          GLCD.print("ERR");
        }
        else{
          if (tempC3 < 10.0 and tempC3 >= 0){
            GLCD.print(" ");
          }
          GLCD.print(tempC3,0);
          //GLCD.print(fps,2);
          GLCD.write(128);
          GLCD.SelectFont(newbasic3x5);
          GLCD.print("C");
          GLCD.SelectFont(System5x7);
        }

        if (refAlarm == true) {
          if (refAlarmState == LOW) {
            refAlarmState = HIGH;
            GLCD.DrawHLine(84,53,11,PIXEL_ON);
          }
          else {
            refAlarmState = LOW;
            GLCD.DrawHLine(84,53,11,PIXEL_OFF);
          }
        }
        else {
            GLCD.DrawHLine(84,53,11,PIXEL_OFF);
        }

        //FPS check
        /*fpsCount=fpsCount+1;
        fpsTime2 = millis();
          
            if (fpsCount == 15){
              fpsDuration = fpsTime2-fpsTime1;
              fps = (fpsDuration / 15);
              fps = 1000/fps;
              Serial.print("Frames per second: ");
              Serial.println(fps);
              fpsCount = 0;
              fpsTime1 = millis();
            }*/

        previousFastPrint = currentMillis;
 
    break;
    
  case 1:

        GLCD.SelectFont(System5x7);
        
        GLCD.FillRect(80, 0, 47, 7, PIXEL_OFF);    
        GLCD.GotoXY(80, 0);
        if (EGTalarmActive){
          GLCD.print(EGTlimit);
        }
        else{
          GLCD.print("OFF");
        }
         
        GLCD.FillRect(80, 10, 47, 7, PIXEL_OFF);  
        GLCD.GotoXY(80, 10);
        if (PSIalarmActive){  
          GLCD.print(PSIlimit,1); 
        }
        else{
          GLCD.print("OFF");
        }
          
        GLCD.FillRect(80, 20, 47, 7, PIXEL_OFF);  
        GLCD.GotoXY(80, 20);
        if (WATERalarmActive){  
          GLCD.print(WATERlimit);
        }
        else{
          GLCD.print("OFF");
        }
        
        GLCD.FillRect(80, 30, 47, 7, PIXEL_OFF);  
        GLCD.GotoXY(80, 30);  
        if (refLoAlarmActive){  
          GLCD.print(refLo,1);
        }
        else{
          GLCD.print("OFF");
        }
        
        GLCD.FillRect(80, 40, 47, 7, PIXEL_OFF);  
        GLCD.GotoXY(80, 40);
        if (refLoAlarmActive){  
          GLCD.print(refHi,1);
        }
        else{
          GLCD.print("OFF");
        }

        
          

        

        /*if (menuItem == 0){
          GLCD.FillRect(0, 0, 8, 55, PIXEL_OFF);
          GLCD.GotoXY(3, 2);  
          GLCD.print(">");
        }*/
        
        //if (menuItem == 1){
          GLCD.FillRect(0, 0, 8, 55, PIXEL_OFF);
          //GLCD.GotoXY(3, 12);
          menuItemY = (menuItem*10);
          GLCD.GotoXY(3, (menuItemY));  
          GLCD.print(">");
        //}        
        previousFastPrint = currentMillis;

    break;

    case 2:

        GLCD.SelectFont(System5x7);
        GLCD.FillRect(80, 0, 47, 7, PIXEL_OFF);  
        GLCD.GotoXY(80, 0);
        if (priLowAlarmActive){   
          GLCD.print(priBattLow,1);
        }
        else{
          GLCD.print("OFF");
        }
        
        GLCD.FillRect(80, 10, 47, 7, PIXEL_OFF);  
        GLCD.GotoXY(80, 10);
        if (priOverAlarmActive){   
          GLCD.print(priOverCharge,1);
        }
        else{
          GLCD.print("OFF");
        }
         
        GLCD.FillRect(80, 20, 47, 7, PIXEL_OFF);  
        GLCD.GotoXY(80, 20);
        if (auxLowAlarmActive){  
          GLCD.print(auxBattLow,1);
        }
        else{
          GLCD.print("OFF");
        }
         
        GLCD.FillRect(80, 30, 47, 7, PIXEL_OFF);   
        GLCD.GotoXY(80, 30);
        if (auxOverAlarmActive){ 
          GLCD.print(auxOverCharge,1);
        }
        else{
          GLCD.print("OFF");
        }
        
        if (menuItem == 4){
              GLCD.SelectFont(newbasic3x5);
              GLCD.GotoXY(0, 57);  
              GLCD.print("EXIT              YES       NEXT");
              GLCD.SelectFont(System5x7);
         }   
        
          
        

        /*if (menuItem == 0){
          GLCD.FillRect(0, 0, 8, 55, PIXEL_OFF);
          GLCD.GotoXY(3, 2);  
          GLCD.print(">");
        }*/
        
        //if (menuItem == 1){
          GLCD.FillRect(0, 0, 8, 55, PIXEL_OFF);
          //GLCD.GotoXY(3, 12);
          menuItemY = (menuItem*10);
          GLCD.GotoXY(3, (menuItemY)); 
          GLCD.print(">");
        //}        

        previousFastPrint = currentMillis;

    break;

    case 3:
         
      //Print PSI
        GLCD.SelectFont(Verdana12_bold);
        //itoa(PSIval,PSIstr,10);
        GLCD.FillRect(44, 24, 40, 12, PIXEL_OFF);
        if (PSImax > -500){
          dtostrf(PSImax,4,1,PSIstr);
          PSIcentreX = 43 + 42/2 - (GLCD.StringWidth(PSIstr))/2;
          GLCD.GotoXY(PSIcentreX, 24); 
          GLCD.print(PSIstr);
        }    
     
      //Print EGT
        GLCD.FillRect(1, 24, 40, 12, PIXEL_OFF);
        if (EGTmax > -500){
          itoa(EGTmax,EGTstr,10);
          EGTcentreX = 42/2 - (GLCD.StringWidth(EGTstr))/2;
          GLCD.GotoXY(EGTcentreX, 24); 
          GLCD.print(EGTstr);
        }
        
      //Print WATER
        GLCD.FillRect(87, 24, 40, 12, PIXEL_OFF);
        if (WATERmax > -500){
          itoa(WATERmax,WATERstr,10);
          WATERcentreX = 86 + 42/2 - (GLCD.StringWidth(WATERstr))/2;
          GLCD.GotoXY(WATERcentreX, 24); 
          GLCD.print(WATERstr);
        }

        previousFastPrint = currentMillis;

    break;
}

}


if (millis() - previousAlarm > alarmInterval) {
  //if (auxAlarm || priAlarm || refAlarm || PSIalarm || EGTalarm ||WATERalarm) {
  if (auxAlarm || priAlarm || WATERalarm) {
    //if (alarmState == LOW) {
      alarmState = HIGH;
      digitalWrite(ALARMPIN,HIGH);
      //GLCD.OffBacklight();
    //}
    /*else {
      alarmState = LOW;
      digitalWrite(ALARMPIN,LOW);
      //GLCD.OnBacklight();
    }*/
  }
  else {
      alarmState = LOW;
      digitalWrite(ALARMPIN,LOW);
      //GLCD.OnBacklight();   
  }

  //digitalWrite(RELAYPIN,lowVoltRelayState); CUT N/C RELAY if under voltage
}


oldButton = button; //button data is now old
oldButton2 = button2; //button data is now old
oldButton3 = button3; //button data is now old
oldButton4 = button4; //button data is now old

}

double getPressure()
{
  char status;
  double T,P,p0,a;

  // You must first get a temperature measurement to perform a pressure reading.
  
  // Start a temperature measurement:
  // If request is successful, the number of ms to wait is returned.
  // If request is unsuccessful, 0 is returned.

  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:

    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Use '&T' to provide the address of T to the function.
    // Function returns 1 if successful, 0 if failure.

    status = pressure.getTemperature(T);
    if (status != 0)
    {
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.

      status = pressure.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Use '&P' to provide the address of P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.

        status = pressure.getPressure(P,T);
        if (status != 0)
        {
          return(P);
        }
        else Serial.println("error retrieving pressure measurement\n");
      }
      else Serial.println("error starting pressure measurement\n");
    }
    else Serial.println("error retrieving temperature measurement\n");
  }
  else Serial.println("error starting temperature measurement\n");
}
