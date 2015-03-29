/*
 Smart Sensors Warm House controller code 
 Designed by V.Gudaitis oxyo@smartsensors.lt 2015 02 26
 
 RELEASE V.2.6 FEATURES
 
 * Code rewrited without Strings library
 * Minimal temperature bug fix
 * Weekend mode feature  
 * Remote reset feature
 * Remote diagnostics feature
 * Individual and broadcast command feature
 * Self-healing feature:
    - XRF reset (hardware change Arduino Pin5) if no sensor data received in 10 min. interval
    - Controller reset if no sensor data received 
    - Self disabling mode if no sensor data received
  
 
 Coordinator commands:
 01. R=0 - Fan speed set
 02. T=23.00 - Temperature to follow set 
 03. S=1 - Enabled/Disabled
 04. M=C - Cooling/Heating mode  
 05. O=01-23 - Offline hours set
 06. W=1 - weekend mode 
 
 Coordinator broadcast command example:
 aF0:R=4T=25.18S=1M=HA=1O=10-11W=0
 
 Individual Command example:
 aW3:R=4T=25.18S=1M=HA=1O=10-11W=0
 
 Service commands:
 06. aW0HI - Get status 
 07. aW0INFO - Get status (humman readable)
 08. aW0RESET - Controller reset
*/

#include <avr/wdt.h>
#include <EEPROM.h>
#include <DS1307.h>
#include <SPI.h>
#include <PString.h>

//Set Controller label >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

char ServiceName[3] = "W1"; //Controller ID
char CName[3] = "F1"; //Room label
char SNameT[3] = "T1"; //Temperature sensor to follow label

char CVER[] = "V.2.6"; //Firmware version info 
int maxCommandLength = 33; //Controller command length

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

//Set random delay value for broadcast answers
int rDelay = 170;

//Define main control devices
int xrfReset = 5;
int led = 8;
int relay5 = 9;
int relay4 = 10;
int relay3 = 11;
int relay2 = 12;
int relay1 = 13;

//Rename main control devices for controller purpose
int heatingON = relay5;
int coolingON = relay4;
int fan1ON = relay3;
int fan2ON = relay2;
int fan3ON = relay1;

// Init the DS1307 RTC
DS1307_RAM ramBuffer;
DS1307 rtc(2, 3);
Time tObj;
int weekDay = 0;


char currentHourStr[8+1]; //temporary array for current hour string conversion to integer
char nonWorkStart[2+1]; //temporary array for nonWorkStart hour string conversion to integer
char nonWorkEnd[2+1];
int nonWorkStartHour;
int nonWorkEndHour;

//Controller temperature level set 
char tmpData[101]; //char array for wireless temperature sensor data
char tmpStr[5+1];
char tmp1Str[2+1];
char tmp2Str[2+1];
int tmp1Value;
int tmp2Value;
byte tmpDataIndex = 0;
char tempValueBuffer[6];
PString tempValue(tempValueBuffer, sizeof(tempValueBuffer)); // Variable to store Radio Sensor temperature value
float tempData_value = 0.00; //Temperature from wireless sensor
float tempData_set = 0.00; //Temperature to follow level (first time value)

//Controller mode initialisation
boolean coolingEnabled = false; //Cooling mode flag
boolean fancoilOFF = false; //Disabled mode flag
boolean nonWorkingTime = false; //Working Hour flag
boolean timerOFF = false; //Timer Off flag
char modeT = '*'; //Temperature control mode status
int fanSpeed; // Fan speed value 
boolean weekendMode = false; //Weekend mode current status
boolean enableWeekendMode = false; //Weekend mode enable flag


// Functions ========================================================================

void(* resetFunc) (void) = 0; //declare reset function @ address 0


void xrfToReset() {
  digitalWrite(xrfReset, LOW);    // turn the XRF off by making the voltage LOW  
  delay (500); 
  digitalWrite(xrfReset, HIGH);
  delay (500);  
} 

void shortLED(){
  
  digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay (80);
  digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW  
  delay (80);
  digitalWrite(led, HIGH);   
  delay (80);
  digitalWrite(led, LOW);     
}

void getDate(){ 
  Serial.print(rtc.getDateStr());
}

void getTime(){
  Serial.print(rtc.getTimeStr());
}

// Float to String conversion function ============================================
// Usage: floatToString(buffer string, float value, precision, minimum text width)

char * floatToString(char * outstr, double val, byte precision, byte widthp){
  char temp[16]; //increase this if you need more digits than 15
  byte i;
  temp[0]='\0';
  outstr[0]='\0';
  if(val < 0.0){
    strcpy(outstr,"-\0");  //print "-" sign
    val *= -1;
  }
  if( precision == 0) {
    strcat(outstr, ltoa(round(val),temp,10));  //prints the int part
  }
  else {
    unsigned long frac, mult = 1;
    byte padding = precision-1; 
    while (precision--)
      mult *= 10;
    val += 0.5/(float)mult;      // compute rounding factor    
    strcat(outstr, ltoa(floor(val),temp,10));  //prints the integer part without rounding
    strcat(outstr, ".\0"); // print the decimal point
    frac = (val - floor(val)) * mult;
    unsigned long frac1 = frac;
    while(frac1 /= 10)
      padding--;
    while(padding--)
      strcat(outstr,"0\0");    // print padding zeros
    strcat(outstr,ltoa(frac,temp,10));  // print fraction part
  }

  // generate width space padding
  if ((widthp != 0)&&(widthp >= strlen(outstr))){
    byte J=0;
    J = widthp - strlen(outstr);
    for (i=0; i< J; i++) {
      temp[i] = ' ';
    }
    temp[i++] = '\0';
    strcat(temp,outstr);
    strcpy(outstr,temp);
  }
  return outstr;
}

// Fan speed set function =======================================================
void fanSpeedSet(int fanSpeed){ 
  
  if (fanSpeed == 0) {
       digitalWrite(fan1ON, LOW);
       digitalWrite(fan2ON, LOW);
       digitalWrite(fan3ON, LOW);
  }  
  if (fanSpeed == 1) {
       digitalWrite(fan1ON, HIGH);
       digitalWrite(fan2ON, LOW);
       digitalWrite(fan3ON, LOW);
  } 
  if (fanSpeed == 2) {
       digitalWrite(fan1ON, LOW);
       digitalWrite(fan2ON, HIGH);
       digitalWrite(fan3ON, LOW);
  } 
  if (fanSpeed == 3) {
       digitalWrite(fan1ON, LOW);
       digitalWrite(fan2ON, LOW);
       digitalWrite(fan3ON, HIGH);
  }    
  
}



//===============================================  SETUP  ==================================================
void setup() { 
        
  // Set the clock to run-mode
  rtc.halt(false);
  
  //Time set routine
  // The following lines can be commented out to use the values already stored in the DS1307
  //rtc.setTime(13, 41, 0);    // Set the time to 12:00:00 (24hr format)
  //rtc.setDate(13, 06, 2014);  // Set the date to April 02, 2013
  //rtc.setDOW(FRIDAY);        // Set Day-of-Week 
   //MONDAY	1
   //TUESDAY	2
   //WEDNESDAY	3
   //THURSDAY	4
   //FRIDAY	5
   //SATURDAY	6
   //SUNDAY	7
  
  pinMode(xrfReset, OUTPUT);
  pinMode(led, OUTPUT);
  pinMode(relay1, OUTPUT);  
  pinMode(relay2, OUTPUT); 
  pinMode(relay3, OUTPUT); 
  pinMode(relay4, OUTPUT); 
  pinMode(relay5, OUTPUT); 
  
  //LED Indication
  shortLED();  
  delay(100);
  shortLED();
  
  digitalWrite(xrfReset, HIGH);    
  
  // Setup Serial connection
  Serial.begin(1200);
  delay(1000);
    
  //Printing controller header   

  if (EEPROM.read(777) == 199) {
    Serial.print(" ");
    Serial.print(ServiceName);
    Serial.print(" ");    
    Serial.print(CName);    
    Serial.print(" OFF! ");
    Serial.print(CVER);    
    Serial.println();    
  }  
  if (EEPROM.read(777) == 200) {
    Serial.println(); 
    Serial.print(ServiceName);
    Serial.print(" ");
    Serial.print(CName);
    Serial.print(" ON ");
    Serial.print(CVER);    
    Serial.println();
  }
      
  //Initialise WDT
  wdt_enable(WDTO_8S);  //Set up the watchdog timer.  
  wdt_reset(); 
}



//===============================================  MAIN  ==================================================
void loop() {
   
  wdt_reset(); //Reset watchdog timer
  
  //Check weekday =================================================
  tObj = rtc.getTime();
  weekDay = tObj.dow;

  
  //Get Current time
  int currentHour = tObj.hour;
  int currentMin = tObj.min;
  int currentSec = tObj.sec;
  
  char currentHour_[3], currentMin_[3], currentSec_[3];
  PString currentHour_S(currentHour_, sizeof(currentHour_)); 
  PString currentMin_S(currentMin_, sizeof(currentMin_)); 
  PString currentSec_S(currentSec_, sizeof(currentSec_)); 
  
  currentHour_S.print(currentHour);
  currentMin_S.print(currentMin);
  currentSec_S.print(currentSec);  
  

  //Check weekend mode ============================================
  weekendMode = false;
  if(enableWeekendMode){
      if (weekDay == 6 || weekDay == 7) weekendMode = true; 
  } 
   
  //Check fan speed value =========================================
  fanSpeed = EEPROM.read(601);
  
  //Check working mode ============================================
  if(EEPROM.read(999) == 1){
      coolingEnabled = false; //Heating mode
  } else {
      coolingEnabled = true; //Cooling mode
  }  
   
  //Check Offline hours ===========================================
    //currentHourStr[0] = rtc.getTimeStr()[0];
    //currentHourStr[1] = rtc.getTimeStr()[1]; 
    //int currentHour = atoi(currentHourStr);
    
    int offlineHourStart = EEPROM.read(40);
    int offlineHourEnd = EEPROM.read(50);
    
    if (currentHour >= offlineHourStart) nonWorkingTime = true;
    if (currentHour + 1 >= offlineHourEnd) nonWorkingTime = false;

    
  // Check Offline hours mode Off Flag =============================================       
    if(EEPROM.read(555) == 200) {
      timerOFF == true;
      nonWorkingTime = false;   
    } else {
      timerOFF == false;    
    } 
  
  //Check if Fancoil is not disabled ==============================
    if (EEPROM.read(777) == 199){
      fancoilOFF = true;      
    } else {
      fancoilOFF = false;
    }
         
  
  //Getting last temperature sensor value from RAM
   char t_Old[5];
   t_Old[0] = rtc.peek(6);
   t_Old[1] = rtc.peek(7);
   t_Old[2] = rtc.peek(8);
   t_Old[3] = rtc.peek(9);
   t_Old[4] = rtc.peek(10);

   float tempData_Old = atof(t_Old); 
   
   int oldMin = rtc.peek(25);
   int deltaMin = currentMin - oldMin;
   
   unsigned int resetCounter = rtc.peek(27);
        
   if ( abs(deltaMin) > 10 ) { // if TX not received since 6 min
     xrfToReset();     
     rtc.poke(25, currentMin); //reset TX received time
     rtc.poke(27, resetCounter + 1); //reset counter 
     Serial.println();
     Serial.print(ServiceName);
     Serial.print(" > ");
     Serial.print(SNameT);
     Serial.print(" DEAD");   
     Serial.print(resetCounter);
     fancoilOFF = true; 
     //resetFunc(); //reset controller
     
     // If no response from T sensor after few XRF resets > reset fanCoil 
     if (resetCounter > 2) resetFunc(); //reset controller
   }
  
   // If no response from T sensor long time ago > disable controller 
   if (resetCounter > 3) {
       fancoilOFF = true; 
       //rtc.poke(27, 0); //reset counter
   }

   //Getting temperature to follow =======================================
  
   if (weekendMode) tempData_set = tempData_set - 3; 
   
 
  //Environment control routine =========================================
  
  if (!fancoilOFF) {
    
      if (timerOFF) {
        
          if (tempData_set > tempData_value && !coolingEnabled){
               modeT = 'T';  //Heating active state        
               digitalWrite(coolingON, LOW);      
               digitalWrite(heatingON, HIGH);       
               if (fanSpeed == 4) {
                 digitalWrite(fan1ON, HIGH);
                 digitalWrite(fan2ON, LOW);
                 digitalWrite(fan3ON, LOW); 
               } else {
                  fanSpeedSet(fanSpeed);      
               }         
                    
          } else if (tempData_set < tempData_value && !coolingEnabled){ 
               modeT = 't'; //Heating passive state
               digitalWrite(coolingON, LOW); 
               digitalWrite(heatingON, LOW); 
               if (fanSpeed == 4) {
                 digitalWrite(fan1ON, LOW);
                 digitalWrite(fan2ON, LOW); 
                 digitalWrite(fan3ON, LOW);
               } else {
                  fanSpeedSet(fanSpeed);      
               }                  
          }    
         
          if (tempData_set > tempData_value && coolingEnabled){
               modeT = 'c';  //Cooling passive state        
               digitalWrite(coolingON, LOW);      
               digitalWrite(heatingON, LOW);       
               if (fanSpeed == 4) {
                 digitalWrite(fan1ON, LOW);
                 digitalWrite(fan2ON, LOW);
                 digitalWrite(fan3ON, LOW); 
               } else {
                  fanSpeedSet(fanSpeed);      
               }         
                    
          } else if (tempData_set < tempData_value && coolingEnabled){ 
               modeT = 'C'; //Cooling active state
               digitalWrite(coolingON, HIGH); 
               digitalWrite(heatingON, LOW); 
               if (fanSpeed == 4) {
                 digitalWrite(fan1ON, HIGH);
                 digitalWrite(fan2ON, LOW); 
                 digitalWrite(fan3ON, LOW);
               } else {
                  fanSpeedSet(fanSpeed);      
               }                  
          }          
        
           
      } else { 
       
          if (tempData_set > tempData_value && !coolingEnabled && !nonWorkingTime){
               modeT = 'T';  //Heating active state              
               digitalWrite(coolingON, LOW);      
               digitalWrite(heatingON, HIGH);       
               if (fanSpeed == 4) {
                 digitalWrite(fan1ON, HIGH);
                 digitalWrite(fan2ON, LOW);
                 digitalWrite(fan3ON, LOW); 
               } else {
                  fanSpeedSet(fanSpeed);      
               }  
                    
          } else if (tempData_set < tempData_value && !coolingEnabled && !nonWorkingTime){ 
               modeT = 't'; //Heating passive state
               digitalWrite(coolingON, LOW); 
               digitalWrite(heatingON, LOW); 
               if (fanSpeed == 4) {
                 digitalWrite(fan1ON, LOW);
                 digitalWrite(fan2ON, LOW); 
                 digitalWrite(fan3ON, LOW);
               } else {
                  fanSpeedSet(fanSpeed);      
               } 
               
          } else if (tempData_set > tempData_value && coolingEnabled && !nonWorkingTime){ 
               modeT = 'c';  //Cooling passive state        
               digitalWrite(coolingON, LOW);      
               digitalWrite(heatingON, LOW);       
               if (fanSpeed == 4) {
                 digitalWrite(fan1ON, LOW);
                 digitalWrite(fan2ON, LOW);
                 digitalWrite(fan3ON, LOW); 
               } else {
                  fanSpeedSet(fanSpeed);      
               } 
               
          } else if (tempData_set < tempData_value && coolingEnabled && !nonWorkingTime){ 
               modeT = 'C'; //Cooling active state
               digitalWrite(coolingON, HIGH); 
               digitalWrite(heatingON, LOW); 
               if (fanSpeed == 4) {
                 digitalWrite(fan1ON, HIGH);
                 digitalWrite(fan2ON, LOW); 
                 digitalWrite(fan3ON, LOW);
               } else {
                  fanSpeedSet(fanSpeed);      
               }                          
    
                                
          } else if (nonWorkingTime) {
               modeT = 'o'; //Disabled Off hours state
               digitalWrite(coolingON, LOW); 
               digitalWrite(heatingON, LOW); 
               digitalWrite(fan3ON, LOW);
               digitalWrite(fan2ON, LOW);
               digitalWrite(fan1ON, LOW);       
          } 
    
      }
      
  } else {
  
       modeT = 'D'; //Disabled state
       digitalWrite(coolingON, LOW); 
       digitalWrite(heatingON, LOW); 
       digitalWrite(fan3ON, LOW);
       digitalWrite(fan2ON, LOW);
       digitalWrite(fan1ON, LOW);    
  }
  
 //Coordinator routine ======================================================================
    
  // Check if there's incoming serial data.
  if (Serial.available() > 0) {
    char incomingByte = Serial.read();

    // Checks for termination of the string.
    if (incomingByte == 'a') {
      
      char serialData[maxCommandLength];
      int number_of_bytes_received;
      number_of_bytes_received = Serial.readBytesUntil(13, serialData, maxCommandLength - 1);
      serialData[number_of_bytes_received] = '\0';

      //0. Getting wireless temperature sensor value  ======================================      
      if (serialData[0] == SNameT[0] && serialData[1] == SNameT[1] && serialData[2] == 'T' && serialData[3] == 'M' && serialData[4] == 'P' && serialData[5] == 'A') {         
          tmpData[tmpDataIndex++] = serialData[6];
          rtc.poke(6,serialData[6]);
          tmpData[tmpDataIndex++] = serialData[7];
          rtc.poke(7,serialData[7]);
          tmpData[tmpDataIndex++] = serialData[8];
          rtc.poke(8,serialData[8]);
          tmpData[tmpDataIndex++] = serialData[9];
          rtc.poke(9,serialData[9]);          
          tmpData[tmpDataIndex++] = serialData[10]; 
          rtc.poke(10,serialData[10]); 
          tempData_value = atof(tmpData);                   
          tmpDataIndex = 0;          
          //rtc.poke(24,currentHour);
          rtc.poke(25, tObj.min); 
          rtc.poke(27, 0); // reset reseting counter
          fancoilOFF = false;
          modeT = '*';
      } 
 
 
      // 1.Getting Coordinator Command ======================================================= 
      if ( serialData[0] == CName[0] && serialData[1] == CName[1] && serialData[2] == ':') { 
      
        char serialCommand[maxCommandLength - 4];

        for (int i=0; i< maxCommandLength; i++) {
           serialCommand[i] = serialData[i+3];
        }        

        serialCommand[29] = '\0';
                
        //1.0. Fan Speed set to auto --------------------------------------------------------------
        if (serialCommand[0] == 'R' && serialCommand[1] == '=' && serialCommand[2] == '0') {          
           if(EEPROM.read(601) != 0) EEPROM.write(601, 0);  //Fan speed set to 0 - OFF mode
           //fanSpeedSet(0);
        }  
        
        //1.1. Fan Speed set to 1 -----------------------------------------------------------------
        if (serialCommand[0] == 'R' && serialCommand[1] == '=' && serialCommand[2] == '1') {          
           if(EEPROM.read(601) != 1) EEPROM.write(601, 1);  //Fan speed set to 1 
           //fanSpeedSet(1);         
        }
        
        //1.2. Fan Speed set to 2 -----------------------------------------------------------------
         if (serialCommand[0] == 'R' && serialCommand[1] == '=' && serialCommand[2] == '2') {          
           if(EEPROM.read(601) != 2) EEPROM.write(601, 2);  //Fan speed set to 2
           //fanSpeedSet(2);          
        }

        //1.2. Fan Speed set to 3 -----------------------------------------------------------------
         if (serialCommand[0] == 'R' && serialCommand[1] == '=' && serialCommand[2] == '3') {          
           if(EEPROM.read(601) != 3) EEPROM.write(601, 3);  //Fan speed set to 3          
           //fanSpeedSet(3);          
        }
       
        //1.3. Fan Speed set to 0  ----------------------------------------------------------------
        if (serialCommand[0] == 'R' && serialCommand[1] == '=' && serialCommand[2] == '4') {          
           if(EEPROM.read(601) != 4) EEPROM.write(601, 4);  //Fan speed set to 4 - AUTO mode                      
        }        
        
        //2.0. Set-up temperature to follow -------------------------------------------------------            
        if (serialCommand[3] == 'T' && serialCommand[4] == '=') {        
                       
            //Getting full temperature string
            tmp1Str[0] = tmpStr[0] = serialCommand[5];     // String 00.00
            tmp1Str[1] = tmpStr[1] = serialCommand[6];
            tmpStr[2] = serialCommand[7];
            tmp2Str[0] = tmpStr[3] = serialCommand[8];
            tmp2Str[1] = tmpStr[4] = serialCommand[9]; 
 
            //Getting integer values to store in EEPROM
            tmp1Value = atoi(tmp1Str);         
             
            if (tmp1Value > 10) tempData_set = atof(tmpStr);        
            
                      
        } //temperature to follow routine END ------------------------------------------------------        
        
        //3.1. Disable controller ------------------------------------------------------------------
        if (serialCommand[10] == 'S' && serialCommand[11] == '=' && serialCommand[12] == '0') {         
           if(EEPROM.read(777) != 199) EEPROM.write(777, 199); //writing Disable flag in EEPROM 
           fancoilOFF = true;         
        }      
    
        //3.2. Enable Controller -------------------------------------------------------------------
        if (serialCommand[10] == 'S' && serialCommand[11] == '=' && serialCommand[12] == '1') {           
           if(EEPROM.read(777) != 200) EEPROM.write(777, 200); //writing Enable flag in EEPROM
           fancoilOFF = false;
        }        
        
        //4.1. Heating mode set -------------------------------------------------------------------
        if (serialCommand[13] == 'M' && serialCommand[14] == '=' && serialCommand[15] == 'H') {                
            if(EEPROM.read(999) != 1) EEPROM.write(999, 1);  //1 - Heating enabled
            coolingEnabled = false;
        }      
    
        //4.2. Cooling mode set -------------------------------------------------------------------
        if (serialCommand[13] == 'M' && serialCommand[14] == '=' && serialCommand[15] == 'C') {          
            if(EEPROM.read(999) != 0) EEPROM.write(999, 0);  //0 - Cooling enabled
            coolingEnabled = true;            
        }         
        
        //5.1. Timer OFF (ALL TIME WORKING) -------------------------------------------------------
        if (serialCommand[16] == 'A' && serialCommand[17] == '=' && serialCommand[18] == '1') {                 
             if(EEPROM.read(555) != 200) EEPROM.write(555, 200);
             timerOFF = true; 
        }

        //5.2. Timer ON ---------------------------------------------------------------------------       
        if (serialCommand[16] == 'A' && serialCommand[17] == '=' && serialCommand[18] == '0') {                 
             if(EEPROM.read(555) != 199) EEPROM.write(555, 199);
             timerOFF = false; 
        }  
        
        //5.3. Offline hours set ------------------------------------------------------------------        
        if (serialCommand[19] == 'O' && serialCommand[20] == '=') { 
            
            //Getting working hours string
            nonWorkStart[0] = serialCommand[21];     // String 00-00
            nonWorkStart[1] = serialCommand[22];
            nonWorkEnd[0] = serialCommand[24];
            nonWorkEnd[1] = serialCommand[25]; 

            //Getting integer values to store in EEPROM
            nonWorkStartHour = atoi(nonWorkStart);
            nonWorkEndHour = atoi(nonWorkEnd);
            
            if (EEPROM.read(40) != nonWorkStartHour) EEPROM.write(40, nonWorkStartHour); 
            if (EEPROM.read(50) != nonWorkEndHour) EEPROM.write(50, nonWorkEndHour);
       }

        //6. Weekend mode set ------------------------------------------------------------------        
        if (serialCommand[26] == 'W' && serialCommand[27] == '=' && serialCommand[28] == '0') {
          enableWeekendMode = false;         
        }       
        if (serialCommand[26] == 'W' && serialCommand[27] == '=' && serialCommand[28] == '1') {       
          enableWeekendMode = true;
        }
        
        //delay(500);
        //Serial.print(CName);
        //Serial.println(" Command:"); 
             
        
      } // END Coordinator command routine ---------------------------------------------
             
      
      
      //6. Service check ============================================================================================     
      if (serialData[0] == ServiceName[0] && serialData[1] == ServiceName[1] && serialData[2] == 'I' && serialData[3] == 'N' && serialData[4] == 'F' && serialData[5] == 'O') {
           delay(rDelay);
           Serial.println();
           Serial.print(ServiceName);
           Serial.print(" ");
           Serial.print(CVER);

           Serial.println();
           Serial.print(ServiceName);
           Serial.print(" ");
           Serial.print("TIME:");
           getTime();
           Serial.print(" ");
           Serial.print("DATE:");
           getDate();
           Serial.print(" ");
           Serial.print("WEEKDAY:");
           Serial.print(weekDay); 
           
           
           Serial.println();           
           Serial.print(ServiceName);
           Serial.print(" ");           
           Serial.print("WEEKENDMODE:");           
           if(enableWeekendMode) {
               Serial.print("ON");
               if (weekendMode) Serial.print("-active");
           } else {
               Serial.print("OFF");
           }           
           Serial.println();                      
           Serial.print(ServiceName);
           Serial.print(" ");
           Serial.print(SNameT);
           Serial.print("VAL:");           
           Serial.print(tempData_value);
           Serial.print(" ");
           Serial.print(SNameT);
           Serial.print("SET:");           
           Serial.print(tempData_set); 
           
           Serial.println(); 
           Serial.print(ServiceName);
           Serial.print(" ");           
           Serial.print("Timer:");           
           if(EEPROM.read(555) == 200) {
             Serial.print("OFF");
           } else {
             Serial.print("ON");
           }
           
           Serial.println();
           Serial.print(ServiceName);
           Serial.print(" ");           
           Serial.print("OFFH:");            
           Serial.print(EEPROM.read(40));
           Serial.print("-");
           Serial.print(EEPROM.read(50));          
           
           Serial.println();
           Serial.print(ServiceName);
           Serial.print(" ");           
           Serial.print("MODE:");           
           Serial.write(modeT);
           Serial.print(" ");
           Serial.print("FANSPEED:");           
           Serial.print(EEPROM.read(601));
           
           Serial.println(); 
           Serial.print(ServiceName);
           Serial.print(" ");           
           Serial.print("tempData_Old:");
           Serial.print(tempData_Old); 
           Serial.print(" ");
           Serial.print("deltamin:");
           Serial.print(deltaMin);  
           Serial.print(" ");
           Serial.print("resetCounter:");
           Serial.print(resetCounter);
           Serial.println();           
       }      
      
      
      //7. Diagnostics check =========================================================================================     
      if (serialData[0] == ServiceName[0] && serialData[1] == ServiceName[1] && serialData[2] == 'H' && serialData[3] == 'I') {
           delay(rDelay);
           Serial.println();
           Serial.print("_");                      
           Serial.print(ServiceName);
           Serial.print(CName);
           Serial.print("|");
           Serial.print(CVER);    
           Serial.print("|TV");           
           Serial.print(tempData_value);          
           Serial.print("|TS");           
           Serial.print(tempData_set);            
           Serial.print("|TI");           
           if(EEPROM.read(555) == 200) {
             Serial.print("0");
           } else {
             Serial.print("1");
           }          
           Serial.print("|OH");            
           Serial.print(EEPROM.read(40));
           Serial.print("-");
           Serial.print(EEPROM.read(50));          
           Serial.print("|MD");           
           Serial.write(modeT);
           Serial.print("|FS");           
           Serial.print(EEPROM.read(601));
           Serial.print("|RC");
           Serial.print(resetCounter);
           Serial.print("|TM");
           getTime();
           Serial.print("|WD");
           Serial.print(weekDay);
           Serial.print("|WM");
           if(enableWeekendMode) {              
               if (weekendMode) {
                 Serial.print("2");
               } else {
                 Serial.print("1");
               }
           } else {
               Serial.print("0");
           }          
           Serial.println();            
       }

      //8. Reset Controller =========================================================================================     
      if (serialData[0] == ServiceName[0] && serialData[1] == ServiceName[1] && serialData[2] == 'R' && serialData[3] == 'E' && serialData[4] == 'S' && serialData[5] == 'E' && serialData[6] == 'T') {
          resetFunc();   
      }      




    }
  }
}






