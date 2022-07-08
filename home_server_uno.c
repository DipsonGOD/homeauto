#include <WiFi101.h>
#include <RTCZero.h>
#include <WiFiUdp.h>
#include "arduino_secrets.h"

/*  Curtain Controller V1.2
 * 
 *  Curtain Position  \~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~/
 *                     |     |     |    |       |        |         |       |    |
 *                     |     |     |    |       |        |         |       |    |
 *                     |     |     |    |       |        |         |       |    |
 *   Position        <-Opened                                              Closed->
 *   Absolute uStep: MAXSTEPS                                                  ZERO
 *   Status:         100                           50/51                          0
 *                     |     |     |    |       |        |         |       |    |
 *                     |     |     |    |       |        |         |       |    |
 *                     |     |     |    |       |        |         |       |    |
 *     
 * 
 */

/************ Global Variables and Definitions ************/
#define DBG 1                         // Debug mode for serial momitor, leave it and no Seriall is spammed

// Motor Settings and Variables
//#define  MOTORLOCK  25    // 10* % current amplitude volume  variantion to signal a lock-up. set level higher at higher motorspeed (lower delay)
#define  MOTORSTART 1000    // ammount of usteps the slow start is working at start : 2 turns
#define  MOTORVARI  10000   // amount of usteps when the variable power kicks in : xx turns - never - disable with 10000
#define  MOTORCURRENT 990   // expected Motor current at start
#define  MAXSTEPS 5450      // MAX steps required to open/close curtain = 13.5 rotations 
#define  MINPEAK -30        // filter out peaks lower then -XX
#define  BADRAIL1 4450      // step-coubnter whith Bad rail
#define  BADRAIL2 4690      // step-coubnter whith Bad rail
int MOTORLOCK=25;

// Motor Driver IO Settings
const int DIRA=12;    // IN1 = DIRA, switches one leg Half-BridgeA  
const int BRAKEA=9;   // IN2 = DIRA xNor BRAKEA, switched one leg of BridgeA  => 1= DIRA 0= #DIRA
const int ENA=3;      // ENA = PWMA, enable signal for half Bridge A
const int DIRB=0;     // IN3 = DIRB, switches one leg of Half-BridgeB (!! ex pin 13 on Arduino UNO)
const int BRAKEB=8;   // IN3 = DIRB xNor BRAKEB, switches one leg of Half-BridgeB => 
const int ENB=11;     // ENB = PWMB, enable singnal for Half Bridge B
const int ARRAY=16;           // over current Monitoring Array Size
const int SARRAY=256;         // long term array Size - for Debugging purposes : data dump
const int Current0 = A0;      // Port for analog current measurement A0 = SenseA
const int Current1 = A1;      // Port for analog current measurement A1 = SenseB

int CurrentArray0[ARRAY];   // Current sample monitor Array
int ShadowArray[SARRAY];    // 3 Debugging Array's - Dumps all current measurements
int AverageArray0[SARRAY];
int AmplitudeArray0[SARRAY];
int CounterSh=0;            // ARRAY / SARRAY Sounters
int CounterAv=0;;
int CounterAm=0;
int CounterCu=0;
long int CounterSt=MAXSTEPS/2;      // ustep counter - to remember curtain status
int AvgCurrent=0;
int Lock;
int Sequencer[7][4] ={     // Sequencer[MotorPin][Value] 4 states
       0,    1,    1,    0,    // DIRA = IN1
       0,    0,    0,    0,    // BRAKEA = IN2 ,  high : follows DIRA, Low = IN2 = #IN1
     255,  255,  255 , 255,    // ENA PWM, 
       1,    1,    0,    0,    // DIRB = IN3
       0,    0,    0,    0,    // BRAKEB = N4, high : follows DIRB, Low :IN4 = #IN3
     255,  255,  255,  255,    // ENB PWM, 
    4800, 4800, 4800, 4800     // uSec Delay per 1/4 Phase
};

//Define ServerSettings
#define SERVER_PORT 80                // Server Port
#define VERSION "2.1.0"                     // Software verson nr
#define MAX_MISSED_DATA 2000          // MAX data missed from Client before time-out (accept short messages only)
char ssid[] = SECRET_SSID;            // your network SSID (name) - arduino_secrets.h
char pass[] = SECRET_PASS;            // your network password - arduino_secrets.h
int keyIndex = 0;                     // your network key Index number (needed only for WEP)
int status = WL_IDLE_STATUS;
WiFiServer server(SERVER_PORT);
IPAddress Myip;
long Myrssi;

// Define NTP settings
#define NTP_PACKET_SIZE 48           // NTP time stamp is in the first 48 bytes of the message
#define NTP_TIMEOUT 3                // NTP time out for parsing UDP packet 
unsigned int localPort = 2390;        // local port to listen for UDP packets
IPAddress timeServer(129,6,15,29);    //  USA NTP server server
byte packetBuffer[ NTP_PACKET_SIZE];  // buffer to hold incoming and outgoing packets
WiFiUDP Udp;                          // A UDP instance to let us send and receive packets over UDP

// Define RTC Settings
RTCZero rtc;                         // RTC Clock
unsigned long epoch= 1541062800UL;   // epoch time global variable, adapted by NTP routines
byte epoch_valid = 0;                // Flag for valid Epoch time received via NTP server
byte manual_valid = 0;               // Flag for valid manual time set
String TimeString = "00:00:00";
String DateString = "00-00-00";
byte summerTime = 1;                 // correction for summertime - see correction functions check_eusummertime()
byte utcTime = 1;                   // time is UTC time + 1 for netherlands
byte weekendCorrect =1;              // weekend rise time correct : overides calculatd rise time on Sundays
byte weekCorrect =1;
byte weekendsuHour = 9;               // Weekend Su rise time override alarm Hours
byte weekendsuMinutes = 30;           // weekend Su tise time override alarm Minutes
byte weekendsaHour = 8;               // Weekend Sa rise time override alarm Hours
byte weekendsaMinutes = 0;           // weekend Sa tise time override alarm Minutes
byte weekmofrHour = 7;               // Week rise time override alarm Hours
byte weekmofrMinutes = 00;           // week tise time override alarm Minutes

String mydays[7] = {"Monday", "Tuesday" ,"Wednesday", "Thursday", "Friday", "Saturday", "Sunday" };

// Define button variables
const int LocalLedPin = 6;                 // Local Led Pin
const int Button1Pin = A4;                 // Local Led Pin
const int Button2Pin = A5;                 // Local Led Pin
int button1 =0;
int button2 = 0;
int bb1 = 0;      // bounce memory
int bb2 =0;       // bounce memory

byte CurtainStatus=50;                    // Flag status of the Curtains: 0=closed-direction 50=unknown0 51=unknown1 100=opened-direction
int CurtainPosition=MAXSTEPS/2;           // EXACT CurtainPosition in usteps - updated every run, initialised at zero
unsigned int OpenSuccess=0;               // Counts succesfull full openings
unsigned int CloseSuccess=0;              // Counts succesfull full closings
unsigned int NoSuccess=0;                 // Counts unsuccesfull full open/close
unsigned int BlockSuccess=0;            // Counts blocks, if unsuccessfull

byte GeneralAlarm = 0;                 // General alarm action 0=void  1= morning 2= evening 3=synC 4=closebutton 5=openbutton,
byte AlarmCounter = 1;                 // Alarm counter, loops through the MyAlarm Array
byte MyAlarm [3][4] = {     // Alarm definition [AlarmNr][Value index] HH,MM,SS,TYPE
 01,30,00, 3,               // Alarm 3 : sync , in initial at 01.30 am
 07,30,00, 1,               // Alarm 1 : morning open curtains, initialy 7.30am
 17,30,00, 2                // Alarm 2 :evening close curtains, initialy 7.30 pm (17.30)
};

int Dusk2Dawn[12][2]= {
515,1012,
485,1062,
430,1114,
359,1169,
293,1221,
248,1267,
247,1280,
286,1247,
336,1183,
386,1113,
441,1046,
493,1006  
};


void setup() {   //Initialize serial and wait for port to open:
#if DBG
  Serial.begin(9600);delay(1000);
  Serial.println("Welcome to my Arduino MKR1000");
#endif
  pinMode(Button1Pin,INPUT);         // set A4 io to digital input
  pinMode(Button2Pin,INPUT);         // set A5 io to digital input
  //establish L298N motor driver pins
  pinMode(DIRA, OUTPUT); //CH A 
  pinMode(BRAKEA, OUTPUT); //brake (disable) CH A
  pinMode(DIRB, OUTPUT); //CH B 
  pinMode(BRAKEB, OUTPUT); //brake (disable) CH B
  // pin 3 PWMA out
  // pin 11 PWMB out  
  analogReference(AR_INTERNAL); // 2.23V internal reference 
  analogReadResolution(12); // 12 bit resolution is ok, gives 0-4096 value on A0 @ 2.23V reference. SenseA senses 1,65V/Amp => 2.23*(A0/4096)/1.65 =>  A0/3 mA = Motor current
  analogWrite(ENA,255);    // always high - not used
  analogWrite(ENB,255);    // always high - not used
  MotorPowerDown();
  
/*********** WIFI SETUP  **********/
  if (WiFi.status() == WL_NO_SHIELD) {   // check for the presence of the shield:
#if DBG
    Serial.println("WiFi shield not present");
#endif
    while (true);     // don't continue if no shield
  }
delay(1000);
StartMyWifi();      // Start Wifi login 

/*********** RTC SETUP  **********/
rtc.begin();
rtc.setEpoch(epoch);                             // Set Time RTC to base time
/*********** NTP Setup **********/
delay(1000);
epoch = readUnixEpochUsingNetworkTimeProtocol();// Get NTP unix time (secs since 1-1-70) UTC + 1 hour
rtc.setEpoch(epoch);                             // set time to real time - if received
/*********** Alarm Setup ************/
UpdateRTCStrings();                              // fill in time/date string  -just for info
CalculateAlarmTimes();                           // Calculate Alarm times
ResetAlarm();                                    // Re-Arm RTC Alarm to next alarm
/*********** Server SETUP  **********/
server.begin();                                  // Start HTTP Server
//InitCurtains();                                // init Curtains
}



void loop() {

 /******* Check Server and Clients ******/
CheckServerClients();

/******* Check button controller inputs ******/
button2= digitalRead(Button2Pin); // read button 2
bb2 = bb2 | button2;              // remember button2 pressed
if (GeneralAlarm == 0 ) GeneralAlarm= (bb2 & !button2)*5;     // Set Alarm5 (Open curtains) when released - if no other alarm was active
button1= digitalRead(Button1Pin); // read button 1
bb1 = bb1 | button1;              // remember button1 pressed
if (GeneralAlarm == 0 ) GeneralAlarm= (bb1 & !button1)*4;    // Set Alarm4 (Close curtains) when released - if no other alarm was active

/******* Check alarm status ******/
switch(GeneralAlarm){                                // Alarm Actions, Flags are set by RTC interrupt timer alarmMatch()
  case 1: // Morning Alarm / open curtains or button 2
    OpenCurtains();
    GeneralAlarm=0;
    ResetAlarm();                                    // Set RTC on next alarm
    break;
  case 2: // Evening Alarm / close curtains or button 1
    CloseCurtains();
    GeneralAlarm=0;
    ResetAlarm();                                    // Set RTC on next alarm
    break;
  case 3: // Sync Alarm
    StartMyWifi();                                   // Start/Check Wifi login 
    epoch = readUnixEpochUsingNetworkTimeProtocol(); // NTP unix time (= secs since 1-1-1970) UTC + corrected UTC-zone
    rtc.setEpoch(epoch);                             // Set new Time on RTC
    server.begin();                                  // Start HTTP Server
    CalculateAlarmTimes();                           // calculate new alarm times
    ResetAlarm();                                    // Set RTC on next alarm
    GeneralAlarm=0;
    break; 
  case 4:                      // Close curtains by button 1
    CloseCurtains();
    GeneralAlarm=0;
    bb1=0;
    break;
  case 5:                     // Open curtains by button 2
    OpenCurtains();
    GeneralAlarm=0;
    bb2=0;
    break;
     
  }

}


/*********** String Routine for RTC **************/

// update and Serial print the Strings that show time and date in right format for print
void UpdateRTCStrings(){
// move RTC time to TimeString global variable
TimeString = String(rtc.getHours(),DEC )+":";
if (rtc.getMinutes()<10) { TimeString= TimeString + "0"; }
TimeString = TimeString + String(rtc.getMinutes(),DEC) + ":";
if (rtc.getSeconds()<10) { TimeString= TimeString + "0"; }
TimeString = TimeString + String(rtc.getSeconds(),DEC);
// move RTC Date to DateString global variable
DateString = String(rtc.getDay(),DEC )+"-";
DateString = DateString + String(rtc.getMonth(),DEC) + "-";
DateString = DateString + "20" + String(rtc.getYear(),DEC);
#if DBG
Serial.print("Time: ");
Serial.print(DateString);Serial.print("/");Serial.println(TimeString);
#endif
}


// RTC interupt routine, set General Alarm (main loop check) and setup next alarm in the Array
void alarmMatch()
{
rtc.disableAlarm();                      // disable, no interrupts while handling Alarm status in main
GeneralAlarm = MyAlarm[AlarmCounter][3]; // set General alarm
  AlarmCounter = (AlarmCounter+1)%3;     // loop the alarm counter to next alarm
#if DBG
   UpdateRTCStrings();
   Serial.print("RTC-Alarm active, #: ");Serial.println(GeneralAlarm);
#endif
// NO OTHER ACTION IN ISR (Int Service Routine) !, DO NOT CALL RTC.xxx commands
}


// Re-Arm RTC alarm routine after event has happened
void ResetAlarm(){
  GeneralAlarm=0;
  rtc.setAlarmHours( MyAlarm[AlarmCounter][0] );   // Set hoursalarm
  rtc.setAlarmMinutes( MyAlarm[AlarmCounter][1] ); // Set minutes alarm
  rtc.setAlarmSeconds(0);                          // Set seconds alarm = 0
  rtc.enableAlarm(rtc.MATCH_HHMMSS);
  rtc.attachInterrupt(alarmMatch);                // Set Alarm Interrupt
#if DBG
   Serial.print("RTC-Alarm re-armed, next alarm #");Serial.print(MyAlarm[AlarmCounter][3]);Serial.print(" at ");
   Serial.print(rtc.getAlarmHours());Serial.print(":"); Serial.println(rtc.getAlarmMinutes());
#endif
}

// Calculation of the alarm times for sunset and sunrise - check xls file for used algorithms - very tricky :)
void CalculateAlarmTimes(){

int rise = calc_sunrise();
int set = calc_sunset();

#if DBG
UpdateRTCStrings();
Serial.print("\nDate: "); Serial.print( mydays[weekday( rtc.getEpoch() ) -1 ] );Serial.print(" ,"); Serial.print(DateString);
if(summerTime) {
   if(check_eu_summertime()&0x01) Serial.print("\nInto Summertime. "); else Serial.print("\nOut of Summertime. ");
   if((check_eu_summertime()>>4)&0x01) Serial.print("Clock changed today."); 
}
Serial.print("\nDawn-set :");Serial.print(rise);Serial.print("min. / ");Serial.print(rise/60);Serial.print(":");Serial.print(rise%60);
Serial.print("\nDusk-rise:");Serial.print(set);Serial.print("min. / ");Serial.print(set/60);Serial.print(":");Serial.print(set%60);Serial.print("\n");
#endif

    MyAlarm[1][0] = (byte) ( (rise+(check_eu_summertime()&summerTime)*60 )/60);                            // Set next Sync Alarm Hour sunrise
    MyAlarm[1][1] = (byte) ( (rise+(check_eu_summertime()&summerTime)*60 )%60);                            // Set next Sync Alarm Minute sunris
    MyAlarm[2][0] = (byte) ( (set+(check_eu_summertime()&summerTime)*60 )/60);                             // Set next Sync Alarm Hour sunset
    MyAlarm[2][1]= (byte)  ( (set+(check_eu_summertime()&summerTime)*60 )%60);                             // Set next Sync Alarm Minute sunset
    MyAlarm[0][0] = (byte) 1;                      // Set next Sync Alarm, is fixed to Hour : 1:50 midnight
    MyAlarm[0][1] = (byte) 50;                     // Set next Sync Alarm Minute

if (weekendCorrect) {   // Correct weekend Rise time - al
   if (weekday( rtc.getEpoch()) == 6 ){
      MyAlarm[1][0] = (byte) weekendsaHour;                // Set weekend alarm
      MyAlarm[1][1] = (byte) weekendsaMinutes;              // Set weekend alarm
      }
   if (weekday( rtc.getEpoch()) == 7 ){
      MyAlarm[1][0] = (byte) weekendsuHour;                // Set weekend alarm
      MyAlarm[1][1] = (byte) weekendsuMinutes;              // Set weekend alarm
      } 
   }
if( weekCorrect) { // correct week rise time, only if later!
     if ( (weekday( rtc.getEpoch()) < 6)  && (MyAlarm[1][0]<weekmofrHour) ){
     MyAlarm[1][0] = (byte) weekmofrHour;                // Set weekend alarm
     MyAlarm[1][1] = (byte) weekmofrMinutes;              // Set weekend alarm
     } 
  }
  
}



// Calculate Sun rise from Dusk2Dawn Table - minutes into Current RTC Day
int calc_sunrise(){
  return (Dusk2Dawn[rtc.getMonth()-1][0] + ((rtc.getDay()-1)*(Dusk2Dawn[rtc.getMonth()%12][0]-Dusk2Dawn[rtc.getMonth()-1][0]))/31 ); // rise time in total minutes of the day
}
// Calculate Sun Set from Dusk2Dawn Table - minutes into Current RTC Day
int calc_sunset(){
  return(Dusk2Dawn[rtc.getMonth()-1][1] + ((rtc.getDay()-1)*(Dusk2Dawn[rtc.getMonth()%12][1]-Dusk2Dawn[rtc.getMonth()-1][1]))/31);
}
// Calculates if current day is in European Summertime period : last sunday in March till last sunday in october
// Function returns : 0x00=no summertime 0x01=yes its summer timee 0x10=no summertime, it switched-exact-today! 0x11=yes its summertime-exact-today!
byte check_eu_summertime() {
int wkdy=weekday( rtc.getEpoch() );
int mn = rtc.getMonth();
int dy = rtc.getDay();

if ( (mn>=11) || (mn<=2) || ( (mn==3)&&(dy<=24) ) ) return(0x00);  // outside summertime period
if ( (mn>=4)  || ( (mn>=10)&&(dy<=24) ) ) return(0x01);           // inside summer time period
if ( mn==10 )                                                     // check last sunday of october : 25th-31st turn is om last sunday of october
  {
  if  (wkdy== 7) return(0x10);  // day7 = sunday = lastday in October, today is the turn !!
   else {
        if( (dy-24-wkdy) <=0 )return(0x01);
        else return(0x00);
        }
  } // check month 10

if ( mn==3 )                                                     // check last sunday of March : 25th-31st turn is om last sunday of october
  {
  if  (wkdy== 7) return(0x11);  // day7 = sunday = lastday in October, today is the turn !!
   else {
        if ( (dy-24-wkdy) <=0 )return(0x00);
        else return(0x01);
        }
  } // check month 3

}


/**** WIFI ROUTINES  *****/

// Login to local network  //
void StartMyWifi(){
int t=0;
if ( status != WL_CONNECTED ) { // only try if not yet connected
#if DBG
  Serial.print("\nScanning available networks... ");
  //listNetworks();
#endif
  while ( status != WL_CONNECTED && t<5) {   // attempt to connect to WiFi network:
#if DBG
       Serial.print("\nAttempting to connect to Network named: ");
       Serial.print(ssid);                // print the network name (SSID);
#endif
       status = WiFi.begin(ssid, pass);     // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
       delay(5000);                         // wait 5 seconds for connection:
       t=t=+1;                              // try-counter
  }
  if ( status == WL_CONNECTED ) {
       //server.begin();                           // start the web server outside this loop!
#if DBG
       printWiFiStatus();                        // you're connected now, so print out the status
#endif
  }
  else{                                          // no connection possible : exit without server started
#if DBG
    Serial.print("\nConnection not possible after several retries.");                   
#endif
  }
}
else{
#if DBG
    Serial.print("\nAlready connected.");                     // you're already connected
    printWiFiStatus(); 
#endif
    }
}

// LIST NETWORKS VIA SERIAL PRINT - only for DEBUG
#if DBG
void listNetworks() {
  if (Serial) {
    // Print Mac Adress
    printMacAddress();
    // scan for nearby networks:
    Serial.println("\n** Scan Networks **");
    int numSsid = WiFi.scanNetworks();
    if (numSsid == -1)
    {
      Serial.println("Couldn't get a wifi connection");
      while (true);
    }

    // print the list of networks seen:
    Serial.print("\nnumber of available networks:");
    Serial.println(numSsid);
    // print the network number and name for each network found:
    for (int thisNet = 0; thisNet < numSsid; thisNet++) {
      Serial.print(thisNet);
      Serial.print(") ");
      Serial.print(WiFi.SSID(thisNet));
      Serial.print("\tSignal: ");
      Serial.print(WiFi.RSSI(thisNet));
      Serial.print(" dBm");
      Serial.print("\tEncryption: ");
      printEncryptionType(WiFi.encryptionType(thisNet));
      Serial.flush();
    }
  }
}
#endif


// SERIALPRINT your MAc Adress - only for debug
#if DBG
void printMacAddress() {
  // the MAC address of your Wifi shield
  byte mac[6];
  WiFi.macAddress(mac);
  Serial.print("MAC: ");
  Serial.print(mac[5], HEX);
  Serial.print(":");
  Serial.print(mac[4], HEX);
  Serial.print(":");
  Serial.print(mac[3], HEX);
  Serial.print(":");
  Serial.print(mac[2], HEX);
  Serial.print(":");
  Serial.print(mac[1], HEX);
  Serial.print(":");
  Serial.println(mac[0], HEX);
}
#endif

// SERIALPRINT encryption type - only for debug
#if DBG
void printEncryptionType(int thisType) {
  // read the encryption type and print out the name:
  switch (thisType) {
    case ENC_TYPE_WEP:
      Serial.println("WEP");
      break;
    case ENC_TYPE_TKIP:
      Serial.println("WPA");
      break;
    case ENC_TYPE_CCMP:
      Serial.println("WPA2");
      break;
    case ENC_TYPE_NONE:
      Serial.println("None");
      break;
    case ENC_TYPE_AUTO:
      Serial.println("Auto");
      break;
  }
}
#endif


// SERIALPRINT Wifi Status - only for debug
#if DBG
void printWiFiStatus() {
  if (Serial) {
    // print the SSID of the network you're attached to:
    Serial.print("\nSSID: ");
    Serial.print(WiFi.SSID());

    // print your WiFi shield's IP address:
    IPAddress ip = WiFi.localIP();
    Serial.print("\nIP Address: ");
    Serial.print(ip);

    // print the received signal strength:
    long rssi = WiFi.RSSI();
    Serial.print("\nsignal strength (RSSI):");
    Serial.print(rssi);
    Serial.print(" dBm");
    // print where to go in a browser:
    Serial.print("\nTo see this page in action, open a browser to http://");
    Serial.println(ip);
  }
}
#endif


// send an NTP request to the time server at the given address, return epoch (unix time = seconds since 1-1-1970)
// Epoch time is corrected for UTC-zone[int utctime] and for summer time [ byte summerTime]
// Syncs RTC to Epoch time (old or new)
unsigned long readUnixEpochUsingNetworkTimeProtocol()
{
  unsigned long epch=0;
  int t=0;
  epoch=rtc.getEpoch();         // save current Epoch time
#if DBG
    Serial.print("Epoch rtc:");Serial.println(epoch);
#endif
  Udp.begin(localPort);
  sendNTPpacket(timeServer);    // send an NTP packet to a time server
  while (!Udp.parsePacket() ){   // wait to see if a reply is available
    delay(500);
    t++;
#if DBG
    Serial.print(".");
#endif
    if (t>NTP_TIMEOUT) break;             // max <NTP_TIMEOUT> times to parse packed, otherwise skip
  }
  if ( t<NTP_TIMEOUT ) {
    // We've received a packet, read the data from it
    Udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer
#if DBG
    Serial.println("UDP packet received from NTP Server.");
#endif

    //the timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, esxtract the two words:

    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    unsigned long secsSince1900 = highWord << 16 | lowWord;
     // now convert NTP time into everyday time for RTCZero usage
    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;
    epch = secsSince1900 - seventyYears;                                                        // UTC Greenwich time 
    if (epch>1541062800UL){                                                                     // sanity check if the receive UDP packet had sensible info
       epoch_valid = 1;                                                                         // Set global varialbel for NTP time received - one time set, will never be reset
       if ((check_eu_summertime() && summerTime )) 
            {rtc.setEpoch(epch + 3600 + 3600*utcTime);}                                          // if its summertime, and summertimeSet and alarm3 (only correct midnight) : correct summertime
       else {rtc.setEpoch(epch + 3600*utcTime);}                                                 // else set RTC to Epoch received
       epoch=rtc.getEpoch();
       } 
  else {   
       //no right NTP, but check to correct summertime
       if ( (check_eu_summertime()==0x11) && summerTime && (GeneralAlarm==3) ) epoch=epoch+3600;  // summert time starts today: add one our - only now during Alarm3 check (midnight)
       if ( (check_eu_summertime()==0x10) && summerTime && (GeneralAlarm==3) ) epoch=epoch-3600;  // summert time stops today: substract one our - only now during Alarm3 check (midnight)  
       rtc.setEpoch(epoch);
#if DBG
    Serial.println("NTP time not received");
#endif
       }
    } // end loop NTP packet received
    else {
#if DBG
       Serial.println("NTP time not received");
#endif
    }
    
#if DBG
    Serial.print("Epoch exit:");Serial.println(epoch);
#endif
  Udp.stop(); 
  return(epoch);
}


// send an NTP request to the time server at the given address
unsigned long sendNTPpacket(IPAddress & address)
{
    // set all bytes in the buffer to 0
     memset(packetBuffer, 0, NTP_PACKET_SIZE);
     // Initialize values needed to form NTP request
     // (see URL above for details on the packets
     packetBuffer[0] = 0b11100011;   // LI, Version, Mode
     packetBuffer[1] = 0;     // Stratum, or type of clock
     packetBuffer[2] = 6;     // Polling Interval
     packetBuffer[3] = 0xEC;  // Peer Clock Precision
     // 8 bytes of zero for Root Delay & Root Dispersion
     packetBuffer[12]  = 49;
     packetBuffer[13]  = 0x4E;
     packetBuffer[14]  = 49;
     packetBuffer[15]  = 52;
     // all NTP fields have been given values, now you can send a packet requesting a timestamp:
     Udp.beginPacket(address, 123); //NTP requests are to port 123
     Udp.write(packetBuffer, NTP_PACKET_SIZE);
     Udp.endPacket();
#if DBG
    Serial.println("UDP packet send to NTP Server.");
#endif
}


/*************** Curtain Functions***********/
void InitCurtains(){
#if DBG
    Serial.println("Initialising Curtains -close and open-");
#endif



}

int OpenCurtains(){
#if DBG
    Serial.print("Opening Curtains :");Serial.print(CurtainStatus); Serial.print(":");Serial.print(CounterSt);
#endif
if ( CurtainStatus!=100 || (CurtainStatus==100 && (100*CurtainPosition/MAXSTEPS)<95 ) ){  // only open when its not opened, or when it was opened less than 95%
  if (CurtainStatus==50) MotorLoop(MAXSTEPS,0);                // Unknown status: run full loop
  else MotorLoop(MAXSTEPS-CurtainPosition,0);                  // else turns MAXSTEPS-CurtPosition usteps  CCW :  only the needed steps are counted to run assuming Curtainstatus was known
 CurtainPosition = CurtainPosition + CounterSt;                // Update Absolute position
 DumpData();
   if ( (100*CounterSt/MAXSTEPS)>95) {                              // Opened with a full run, so its at position MAX
     ++OpenSuccess;                                                 // so sucesfull count
     CurtainStatus = 100;                                           // status is opened
     CurtainPosition=MAXSTEPS;                                      // set position to MAX as well, correct possible slips
     }               
   else  {                                                         // Not opened with a full run
    if (Lock>MOTORLOCK ) {
       ++BlockSuccess;                                              // if not opened fully , and it had a lock: count Blocked
       if (CurtainStatus==50){CurtainStatus=100;CurtainPosition=MAXSTEPS;} // if it had a lock and it was first run : curtain status direction open and fully at position MAX
       }
    else {
       ++NoSuccess;                                                  // if not opened fully, but no lock, count no full run success
      CurtainStatus = 100;                                            // Curtain status = unknown
      }                              
   }

}
}


// close curtains, with conditions for first runs to intialise the full open or full close position
// At start, position is unknown, so first run with a Lock means (probably) its at its end
void CloseCurtains () {
#if DBG
    Serial.print("Closing Curtains :");Serial.print(CurtainStatus); Serial.print(":");Serial.print(CounterSt);
#endif
if ( CurtainStatus!=0 || (CurtainStatus==0 && (100*CurtainPosition/MAXSTEPS)>5 ) ){  // only close when its not closed, or when it was closed less than 90%
  if (CurtainStatus==50) MotorLoop(MAXSTEPS,1);                     // Unknown status: run full loop
  else MotorLoop(CurtainPosition,1);                               // if known : turns CurtPosition usteps  CW  : only needed steps are counted to run assuming Curtainstatus was known
  CurtainPosition = CurtainPosition - CounterSt;                   // Update Absolute position
  DumpData();
   if ( (100*CounterSt/MAXSTEPS)>95) {                            // Closed with a full run, so fully closed at position = 0
     ++CloseSuccess;                                              // so sucesfull count
     CurtainStatus = 0;                                           // status is set to direction closed
     CurtainPosition=0;                                           // set position to 0 as well, correct possible slips
     }               
   else  {                                                         // Not closed with a full run
    if (Lock>MOTORLOCK ) {
       ++BlockSuccess;                                            // if not closed fully , and it had a lock: count Blocked
       if (CurtainStatus==50){CurtainStatus=0;CurtainPosition=0;} // if it had a lock and it was first run : curtain status direction closed and fully at position 0
       }
    else {
       ++NoSuccess;                                                // if not closed fully, but no lock, count no full run success
      CurtainStatus = 0;                                          // Curtain status = unknown, probably first run to middle
      }                              
   }                             
}
}


/********* SERVER ROUTINES *************/

// Check server if client is there, and sercve requests //
void CheckServerClients() {

  // Local Varaibles
  String currentLine = "";  // date line input client
  int d = 0; // client data counter
  int v = 0; // forloop counter
  byte hr,mn,sc; // hour and minute read characters
  
  Myip = WiFi.localIP();
  Myrssi = WiFi.RSSI(); // RSSI data
  WiFiClient client = server.available();   // listen for incoming clients

  if (client) {                             // if you get a client,
#if DBG
    Serial.println("new client");           // print a message out the serial port
#endif
    currentLine = "";                       // make a String to hold incoming data from the client
    d = 0;
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
#if DBG
        Serial.write(c);                     // print it out the serial monitor - debug only
#endif
        if (c == '\n') {                    // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();
            client.println("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">"); // metaview
            // the content of the HTTP response follows the header:
            client.println("<body style=\"background-color:#cccccc\">"); // set color CCS HTML5 style 
            client.print( "<p style=\"font-family:verdana; color:GhostWhite\">&nbsp<font size=1>o</font><font size=2>o</font><font size=3>O</font><font size=2>O</font><font size=1>o</font><font size=2>O</font><font size=3>O</font><font size=2>o</font><font size=1>o</font> <br>");
            client.print( "<font size=5>FoxCurt </font>&nbsp&nbsp&nbsp&nbsp<font size=3>Version ");client.print(VERSION);client.println("</font><br>");
            client.print( "<font size=2>");client.print(WiFi.SSID());client.println(" / ");client.print(Myip);client.println("</font></p>");
            client.print("<p style=\"font-family:verdana; size:2; color:#888888\">Curtain Status: ");
               if(CurtainStatus==0) client.print("Closed ("); else {if(CurtainStatus==100) client.print("Opened (");else client.print("Unknown (");}
               client.print(100*CurtainPosition/MAXSTEPS);client.print("%).<br><br>");
;
            client.print("Click <a href=\"/O\">open</a> open the curtains.<br>");
            client.print("Click <a href=\"/C\">close</a> close the curtains.<br><br>");
            client.print("Click <a href=\"/I\">info</a> to see status info.<br>");
            client.print("Click <a href=\"/H\">help</a> for Help.<br><br>");
            // The HTTP response ends with another blank line:
            client.println();
            // break out of the while loop:
#if DBG
            Serial.println("*Html Home-page send, break out*");
#endif
            break;
          }
          else {      // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        }
        else if (c != '\r') {    // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }

        // Check to see if the client request was "GET /C" or "GET /O or GET/I":
        if (currentLine.endsWith("GET /C")) {
          client.println("HTTP/1.1 200 OK");
          client.println("Content-type:text/html");
          client.println();
          client.println("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">"); // metaview
          client.println("<body style=\"background-color:#cccccc\">");
          client.print("Curtains closing.<br>");
          client.print("Click <a href=\"/\">here</a> to return to menu.<br>");
          client.print("<meta http-equiv=\"refresh\" content=\"3;url=/\" />");
          client.println();
          CloseCurtains();
#if DBG
          Serial.println("*Curtains Closed*");
#endif
          break;

        }
        if (currentLine.endsWith("GET /O")) {
          client.println("HTTP/1.1 200 OK");
          client.println("Content-type:text/html");
          client.println();
          client.println("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">"); // metaview
          client.println("<body style=\"background-color:#cccccc\">");
          client.print("Curtains opening.<br>");
          client.print("Click <a href=\"/\">here</a> to return to menu.<br>");
          client.print("<meta http-equiv=\"refresh\" content=\"3;url=/\" />");
          client.println();
          OpenCurtains();
#if DBG
          Serial.println("*Curtains Opened*");
#endif
          break;
        }

        if (currentLine.endsWith("GET /I")) {
          client.println("HTTP/1.1 200 OK");
          client.println("Content-type:text/html");
          client.println();
          client.println("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">"); // metaview
            client.println("<body style=\"background-color:#cccccc\">"); // set color CCS HTML5 style 
            client.print( "<p style=\"font-family:verdana; color:GhostWhite\">&nbsp<font size=1>o</font><font size=2>o</font><font size=3>O</font><font size=2>O</font><font size=1>o</font><font size=2>O</font><font size=3>O</font><font size=2>o</font><font size=1>o</font> <br>");
            client.print( "<font size=5>FoxCurt </font>&nbsp&nbsp&nbsp&nbsp<font size=4>Info Menu</font><br>");
            client.print("<p style=\"font-family:verdana; size:2; color:#888888\">-------------------------------------------<br>");
          client.print("IP Adress : "); client.print(Myip); client.print("<br>");
          client.print("Ssid : "); client.print(WiFi.SSID()); 
          client.print(", Rssi : "); client.print(Myrssi); client.print("dB<br>");
          UpdateRTCStrings();
          client.print("Date/Time : "); client.print( mydays[weekday( rtc.getEpoch() ) -1 ] );client.print(", "); client.print(DateString); client.print(" ");   
          client.print(TimeString); client.print("<br>");   
          if(summerTime) {
               if(check_eu_summertime()&0x01) client.print("\nInto Summertime. "); else client.print("\nOut of Summertime. ");
               if((check_eu_summertime()>>4)&0x01) client.print("Clock changed today.<br><br>"); else client.print("<br><br>");
               }
          client.print("NTP time received : "); client.print(epoch_valid); client.print(".<br>");
          client.print("Manual time set : "); client.print(manual_valid); client.print(".<br>");
          client.print("UTC-time zone : "); if (utcTime>=0 ) client.print("+"); else client.print("-");
          client.print(abs(utcTime)); client.print(" hr.<br>"); 
          client.print("Summer-time : "); if (summerTime ) client.print("used<br>"); else client.print("not used<br>");
          client.print("Weekend rise-time correct: "); client.print(weekendCorrect); client.print("<br>");
          if(weekendCorrect) {
            client.print("Saturday set at : ");
            client.print(weekendsaHour); client.print(":");client.print(weekendsaMinutes);client.print(":00<br>");
            client.print("Sunday set at : ");
            client.print(weekendsuHour); client.print(":");client.print(weekendsuMinutes);client.print(":00<br>");
          }
          client.print("Workweek rise-time correct: "); client.print(weekCorrect);  client.print("<br>");
          if(weekCorrect) {
            client.print("Monday-Friday set at : ");
            client.print(weekmofrHour); client.print(":");client.print(weekmofrMinutes);client.print(":00<br>");
          }
          client.print("<br>Curtain Status : "); 
           if(CurtainStatus==0) client.print("Closed (Position "); else {if(CurtainStatus==100) client.print("Opened (Position ");else client.print("Unknown (Position");}
           client.print(100*CurtainPosition/MAXSTEPS);client.print("%).<br>");
          client.print("Morning Alarm1 : "); client.print(MyAlarm[1][0]); client.print(":");client.print(MyAlarm[1][1]);client.print(":00 <br>");
          client.print("Evening Alarm2 : "); client.print(MyAlarm[2][0]); client.print(":");client.print(MyAlarm[2][1]);client.print(":00 <br>");
          client.print("Synchro Alarm3 : "); client.print(MyAlarm[0][0]); client.print(":");client.print(MyAlarm[0][1]);client.print(":00<br>");          
          client.print("Next Alarm : #"); client.print(MyAlarm[AlarmCounter][3]);client.print("<br><br>");
          client.print("Succes full Open : "); client.print(OpenSuccess);client.print("<br>");
          client.print("Succes full Close : "); client.print(CloseSuccess);client.print("<br>");
          client.print("No full open/close : "); client.print(BlockSuccess);client.print("<br>");
          client.print("Lockup Detected : "); client.print(NoSuccess);client.print("<br>");
          client.print("MotorLock set at: "); client.print(MOTORLOCK);client.print("<br>");
          client.print("-------------------------------------------<br>");
          client.print("<font size=1>Click <a href=\"/\">here</a> to return to menu.<br></p>");
          client.println();
#if DBG
          Serial.println("*Info Send, break out*");
#endif
          break;
        }

        if (currentLine.endsWith("GET /H")) {
          client.println("HTTP/1.1 200 OK");
          client.println("Content-type:text/html");
          client.println();
          client.println("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">"); // metaview
          client.println("<body style=\"background-color:#cccccc\">");
          client.print( "<p style=\"font-family:verdana; size:5; color:GhostWhite\">HELP menu.<br>");
          client.print("<p style=\"font-family:verdana; size:2; color:#888888\">Use HTML-command   http:\\\\"); 
          client.print(Myip); client.print("\\[command]<br><br>");
          client.print("[command] = L[x] - SetLock, R[x] - (Re)setPosition, A[x] - SetAlarmNr<br>");
          client.print("[command] = D - DumpData, X - Synctime<br><br>");
          client.print("[command] = settime[HH:MM:SS] for time set.<br>");
          client.print("[command] = setdate[DD/MM/YY] for date set.<br>");
          client.print("[command] = setweekend[HH:MM:SS] for weekend rise time override.(0=reset)<br>");          
          client.print("[command] = setutc[x] , for utc-time correct (-11..+11)<br>");
          client.print("[command] = setsummer[x] , for summertime correct On/Off (0..1)<br>");
          client.print("<br>");
          client.print("Click <a href=\"/\">here</a> to return to menu.<br>");
          client.println();
#if DBG
          Serial.println("*Help info Send, break out*");
#endif
          break;
        }
        
        if (currentLine.endsWith("GET /D")) {  // Dump last Data of the Motor Current-array to WebPage
          client.println("HTTP/1.1 200 OK");
          client.println("Content-type:text/html");
          client.println();
          client.println("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">"); // metaview
          client.println("<body style=\"background-color:#cccccc\">");
          client.print("Current;Avg;%Vol;L:"); client.print(Lock); client.print(" S:");client.print(CounterSt); client.print("<br>");
          for(v=0;v<SARRAY;++v) {
            client.print(ShadowArray[v]); client.print(";");client.print(AverageArray0[v]);client.print(";");client.print(AmplitudeArray0[v]);client.print(";<br>");
            }
          client.print(".<br>");
          client.print("Click <a href=\"/\">here</a> to return to menu.<br>");
          client.println();
#if DBG
          Serial.println("*Dump Info Send, break out*");
#endif
          break;
        }

        
        if (currentLine.endsWith("GET /R")) {
          //client.read(); client.read();// read the space %20    
          hr = ( ((byte) client.read()-48) )%2; // reAD ONE OR ZERO
          client.println("HTTP/1.1 200 OK"); 
          client.println("Content-type:text/html");
          client.println();    
          client.println("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">"); // metaview
          client.println("<body style=\"background-color:#cccccc\">");           
          if(hr==0) client.print("Reset Curtain-position to Closed(=0)<br>");  else client.print("Reset Curtain-position to Opened(=100)<br>");
          client.print("<br>Click <a href=\"/\">here</a> to return to menu.<br>");
          client.print("<meta http-equiv=\"refresh\" content=\"3;url=/\" />");
          client.println();
          CurtainPosition=hr*MAXSTEPS;
          CurtainStatus = hr*100;
#if DBG
          Serial.println("*Position ReSet, break out*");
#endif
          break;
        }

        
        if (currentLine.endsWith("GET /L")) {
          //client.read(); client.read();// read the space %20         
          hr = ( ((byte) client.read()-48)*10 + (byte) client.read()-48 )%100; // read hour AScii, and make it simple rounded up for 0-23 hr
          client.println("HTTP/1.1 200 OK"); 
          client.println("Content-type:text/html");
          client.println();  
          client.println("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">"); // metaview
          client.println("<body style=\"background-color:#cccccc\">");
          client.print("New Lock : "); client.print(hr); client.print(".<br>");
          client.print("<br>Click <a href=\"/\">here</a> to return to menu.<br>");
          client.print("<meta http-equiv=\"refresh\" content=\"3;url=/\" />");
          client.println();
          MOTORLOCK=hr;
#if DBG
          Serial.println("*Lock Set, break out*");
#endif
          break;
        }


        
        if (currentLine.endsWith("GET /A")) {
          //client.read(); client.read();// read the space %20         
          hr = ( ((byte) client.read()-48) )%4; // read alarm, value 1,2,or 3
          client.println("HTTP/1.1 200 OK"); 
          client.println("Content-type:text/html");
          client.println();  
          client.println("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">"); // metaview
          client.println("<body style=\"background-color:#cccccc\">");
          client.print("New Alarm: "); client.print(hr); client.print(".<br>");
          client.print("<br>Click <a href=\"/\">here</a> to return to menu.<br>");
          client.print("<meta http-equiv=\"refresh\" content=\"3;url=/\" />");
          client.println();
          AlarmCounter=hr%3; // index is one more looped
#if DBG
          Serial.println("*Alarm-type Set, break out*");
#endif
          break;
        }
        
        if (currentLine.endsWith("GET /settime")) {
          //client.read(); client.read();// read the space %20         
          hr = ( ((byte) client.read()-48)*10 + (byte) client.read()-48 )%24; // read hour AScii, and make it simple rounded up for 0-23 hr
          client.read(); // read the ":"
          mn = ( ((byte) client.read()-48)*10 + (byte) client.read()-48 )%60; // read minutes Ascii and make it simple rounder up for 0-59 min
          client.read(); // read the ":"
          sc = ( ((byte) client.read()-48)*10 + (byte) client.read()-48 )%60; // read Seconds Ascii and make it simple rounder up for 0-59 sec
          client.println("HTTP/1.1 200 OK"); 
          client.println("Content-type:text/html");
          client.println();    
          client.println("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">"); // metaview
          client.println("<body style=\"background-color:#cccccc\">");
          client.print("New Time Set to : "); client.print(hr); client.print("hr,");
          client.print(mn); client.print("min,"); 
          client.print(sc); client.print("sec<br>"); 
          client.print("<br>Click <a href=\"/\">here</a> to return to menu.<br>");
          client.print("<meta http-equiv=\"refresh\" content=\"5;url=/\" />");
          client.println();
          rtc.setHours(hr);rtc.setMinutes(mn);rtc.setSeconds(sc); // set new date/time and reset alarm, set global epoch and calcualte alarm times
          epoch = rtc.getEpoch();
          CalculateAlarmTimes();
          ResetAlarm();
          manual_valid=1;
#if DBG
          Serial.println("*Time Set, break out*");
#endif
          break;
        }

        if (currentLine.endsWith("GET /setdate")) {
          //client.read(); client.read();// read the space %20         
          hr = ( ((byte) client.read()-48)*10 + (byte) client.read()-48 )%32; // read day AScii, and make it simple rounded up for 31 days
          client.read(); // read the "."
          mn = ( ((byte) client.read()-48)*10 + (byte) client.read()-48 )%13; // read month Ascii and make it simple rounder up for 1-12
          client.read(); // read the "."
          sc = ( ((byte) client.read()-48)*10 + (byte) client.read()-48 )%100; // read year Ascii and make it simple rounder up 0-99
          client.println("HTTP/1.1 200 OK"); 
          client.println("Content-type:text/html");
          client.println();       
          client.println("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">"); // metaview
          client.println("<body style=\"background-color:#cccccc\">");
          client.print("New Date Set to : "); client.print(hr); client.print("-");
          client.print(mn); client.print("-"); 
          client.print(sc); client.print("<br>"); 
          client.print("<br>Click <a href=\"/\">here</a> to return to menu.<br>");
          client.print("<meta http-equiv=\"refresh\" content=\"5;url=/\" />");
          client.println();
          rtc.setDay(hr);rtc.setMonth(mn);rtc.setYear(sc); // set new date/time and reset alarm, set global epoch and calcualte alarm times
          epoch = rtc.getEpoch();
          CalculateAlarmTimes();      // set alaram - and correct time for summertime
          ResetAlarm();
#if DBG
          Serial.println("*Date Set, break out*");
#endif
          break;
        }

        
        if (currentLine.endsWith("GET /setweekendsa")) {
          //client.read(); client.read();// read the space %20         
          hr = ( ((byte) client.read()-48)*10 + (byte) client.read()-48 )%24; // read hour AScii, and make it simple rounded up for 0-23 hr
          client.read(); // read the ":"
          mn = ( ((byte) client.read()-48)*10 + (byte) client.read()-48 )%60; // read minutes Ascii and make it simple rounder up for 0-59 min
          client.read(); // read the ":"
          sc = ( ((byte) client.read()-48)*10 + (byte) client.read()-48 )%60; // read Seconds Ascii and make it simple rounder up for 0-59 sec
          client.println("HTTP/1.1 200 OK"); 
          client.println("Content-type:text/html");
          client.println();
          client.println("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">"); // metaview
          client.println("<body style=\"background-color:#cccccc\">");
          if (hr >0 ) {
            client.print("Weekend Sat rise-time Set to : "); client.print(hr); client.print("hr,");
            client.print(mn); client.print("min,"); 
            client.print(sc); client.print("sec<br>"); 
            weekendCorrect=1;
            }
          else {
            client.print("Weekend rise-time correction Off.<br>"); 
            weekendCorrect=0;           
          }
          client.print("<br>Click <a href=\"/\">here</a> to return to menu.<br>");
          client.print("<meta http-equiv=\"refresh\" content=\"5;url=/\" />");
          client.println();
          weekendsaHour = hr; weekendsaMinutes=mn;
          CalculateAlarmTimes();      // set new alarm-times
#if DBG
          Serial.println("*Weekend Sat rise-time (re)Set, break out*");
#endif
          break;
        }

        if (currentLine.endsWith("GET /setweekendsu")) {
          //client.read(); client.read();// read the space %20         
          hr = ( ((byte) client.read()-48)*10 + (byte) client.read()-48 )%24; // read hour AScii, and make it simple rounded up for 0-23 hr
          client.read(); // read the ":"
          mn = ( ((byte) client.read()-48)*10 + (byte) client.read()-48 )%60; // read minutes Ascii and make it simple rounder up for 0-59 min
          client.read(); // read the ":"
          sc = ( ((byte) client.read()-48)*10 + (byte) client.read()-48 )%60; // read Seconds Ascii and make it simple rounder up for 0-59 sec
          client.println("HTTP/1.1 200 OK"); 
          client.println("Content-type:text/html");
          client.println();
          client.println("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">"); // metaview
          client.println("<body style=\"background-color:#cccccc\">");
          if (hr >0 ) {
            client.print("Weekend Sun rise-time Set to : "); client.print(hr); client.print("hr,");
            client.print(mn); client.print("min,"); 
            client.print(sc); client.print("sec<br>"); 
            weekendCorrect=1;
            }
          else {
            client.print("Weekend rise-time correction Off.<br>"); 
            weekendCorrect=0;           
          }
          client.print("<br>Click <a href=\"/\">here</a> to return to menu.<br>");
          client.print("<meta http-equiv=\"refresh\" content=\"5;url=/\" />");
          client.println();
          weekendsuHour = hr; weekendsuMinutes=mn;
          CalculateAlarmTimes();      // set new alarm-times
#if DBG
          Serial.println("*Weekend Sun rise-time (re)Set, break out*");
#endif
          break;
        }

        if (currentLine.endsWith("GET /setweekmofr")) {
          //client.read(); client.read();// read the space %20         
          hr = ( ((byte) client.read()-48)*10 + (byte) client.read()-48 )%24; // read hour AScii, and make it simple rounded up for 0-23 hr
          client.read(); // read the ":"
          mn = ( ((byte) client.read()-48)*10 + (byte) client.read()-48 )%60; // read minutes Ascii and make it simple rounder up for 0-59 min
          client.read(); // read the ":"
          sc = ( ((byte) client.read()-48)*10 + (byte) client.read()-48 )%60; // read Seconds Ascii and make it simple rounder up for 0-59 sec
          client.println("HTTP/1.1 200 OK"); 
          client.println("Content-type:text/html");
          client.println();   
          client.println("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">"); // metaview
          client.println("<body style=\"background-color:#cccccc\">");
          if (hr >0 ) {
            client.print("Workweek Mo-Fr rise-time Set to : "); client.print(hr); client.print("hr,");
            client.print(mn); client.print("min,"); 
            client.print(sc); client.print("sec<br>"); 
            weekCorrect=1;
            }
          else {
            client.print("WorkWeek rise-time correction Off.<br>"); 
            weekCorrect=0;           
          }
          client.print("<br>Click <a href=\"/\">here</a> to return to menu.<br>");
          client.print("<meta http-equiv=\"refresh\" content=\"5;url=/\" />");
          client.println();
          weekmofrHour = hr; weekmofrMinutes=mn;
          CalculateAlarmTimes();      // set new alarm-times
#if DBG
          Serial.println("*WorkWeek rise-time (re)Set, break out*");
#endif
          break;
        }

        
        if (currentLine.endsWith("GET /setsummer")) {
          //client.read(); client.read();// read the space %20         
          hr = ( (byte) client.read()-48 )%1; // read 1 or 0
          client.println("HTTP/1.1 200 OK"); 
          client.println("Content-type:text/html");
          client.println();       
          client.println("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">"); // metaview
          client.println("<body style=\"background-color:#cccccc\">");
          client.print("Summetime correction set : "); client.print(hr); client.print(".<br>");
          client.print("<br>Click <a href=\"/\">here</a> to return to menu.<br>");
          client.print("<meta http-equiv=\"refresh\" content=\"3;url=/\" />");
          client.println();
          summerTime=hr;
#if DBG
          Serial.println("*Summertime correct Set, break out*");
#endif
          break;
        }
        

        if (currentLine.endsWith("GET /favicon")) {
#if DBG
          Serial.println("*Skipped Favicom.Ico request - fuck Chrome*");
#endif
          break;
        }

        if (currentLine.endsWith("GET /X")) {
          client.println("HTTP/1.1 200 OK"); 
          client.println("Content-type:text/html");
          client.println();       
          client.print("Starting NTP request.<br>");  
          client.print("<br>Click <a href=\"/\">here</a> to return to menu.<br>");
          client.print("<meta http-equiv=\"refresh\" content=\"3;url=/\" />");
          client.println();
          client.println("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">"); // metaview
          client.println("<body style=\"background-color:#cccccc\">");
          epoch = readUnixEpochUsingNetworkTimeProtocol();// NTP unix time (secs since 1-1-70) UTC + 1 hour 
          rtc.setEpoch(epoch);                             // Set new Time on RTC
          CalculateAlarmTimes();                           // Calculate Alarm times      
          ResetAlarm();                                    // Setup RTC Alarm for next round  
          UpdateRTCStrings();                              // fill in time/date string
#if DBG
          Serial.println("*NPT sync done, break out*");
#endif
          break;
        }
        
      }
      else {
        d++;
        if (d > MAX_MISSED_DATA) { // defined in arduino_secrets.h
#if DBG
          Serial.println("*Client missed-data time-out");
#endif
          break;   // time-out to prevent to ever waiting for misssed non-send data newclient //
        }
      }
    }
    // close the connection:
    client.stop();
#if DBG
    Serial.println("*Client disonnected*");
#endif
  }
}

// motor loop By BITBANGING the Sequence Array, 
// 1 ustep cycle, dir =1 -> CCW, dir = 0 -> CW, 400 usteps = 1 rotation
void MotorLoop(long int lps,int dir) {
long int t=0;
int v=0;
int dly; 
int tslow= MOTORSTART;
int ttslow=0;

    CounterSt=0;                   // initialises array counters
    CounterAv=0;;
    CounterAm=0;
    CounterCu=0;
    CounterSh=0;
    AvgCurrent=MOTORCURRENT;      // initialize Average
    for(t=0;t<SARRAY;++t){         // initialize arrays
      CurrentArray0[t%ARRAY]=0;
      AverageArray0[t]=0;
      AmplitudeArray0[t]=0;  
      ShadowArray[t]=0;    
      }
  
Lock=0;                  // Sense Lock up of motor, clear for start of run
dir=(dir&1)*2;           //make it binary and 0 or 2 (2 shift = half cycle step in the sequence array = 90 degrees )

for(t=1;t<lps+1;++t) {
  digitalWrite(DIRA, Sequencer[0][t%4]);           //IN1
  digitalWrite(BRAKEA, Sequencer[1][t%4]);         //IN2

  digitalWrite(DIRB, Sequencer[3][(t+dir)%4]);     //IN3
  digitalWrite(BRAKEB, Sequencer[4][(t+dir)%4]);   //IN4
  button1= digitalRead(Button1Pin); // read button 1
  button2= digitalRead(Button2Pin); // read button 2

  dly = Sequencer[6][t%4]+tslow+ttslow;    // calculate delay deviation
  if ( tslow>0 ) tslow--;                  // tslow counter for slow (re)start
  CounterSt=t;                             // keep Global track of number of usteps

  //  read current, calculate Array : est. 750 microseconds
    delayMicroseconds(dly-750);               // shorter delay due to processing part of 750us 
    Lock = (int) CalcArray(t);                 // read current , calculate array's and check for overshoot
    if ( ((CounterSt<BADRAIL1)||(CounterSt>BADRAIL2)) &&(Lock > MOTORLOCK) ) t=lps+1;   // stop loop:  if lock has value: stop, ignore Steps @(bad rails) && (t > ARRAY)
    if ( button1 || button2 ) t=lps+1;                  // stop if button pressed, Safe # steps taken
    if (t>MOTORVARI) ttslow= Lock*20;                    // slow down motor a bit when its peaking  
    }
  
  MotorPowerDown();
  while ( digitalRead(Button1Pin)==1 || digitalRead(Button2Pin)==1) ; // wait to release buttons
}




// Update Current array calculations 
int CalcArray(long int t){
int w = 0;
int tmp=0;
int vol=0;
int level=0;
  if ( (t%2)==0 )level = analogRead(Current0);   // measure Half-Bridge CoilA current, only every switching state = 0 and 2
  else level = analogRead(Current1);             // measure Half-Bridge CoilAB current, only every switching state = 1 and 3
    if(CounterSt<32){                                  // first 32 steps: fill the arrays with initial data
      CurrentArray0[CounterCu] = level  ;              // Safe current Sample                      
      for (w=0;w<CounterCu+1;++w) tmp+=CurrentArray0[w]; // Calculate Average over first ARRAY set of samples
      AvgCurrent = tmp/(CounterCu+1);        
      for(w=CounterCu;w<ARRAY;++w) {
         CurrentArray0[w]=AvgCurrent;            // fill rest ARRAY set with average
         ShadowArray[w]=AvgCurrent;                     // save current to debug array
         AverageArray0[w] = AvgCurrent;            // save Average to current measurement
         AmplitudeArray0[w] = 0;                   // set Ampl to 0
        }
      CounterCu = (CounterCu+1)%ARRAY;          // loop counter small Array
      CounterSh = (CounterSh+1)%SARRAY;         // loop counter
      CounterAv = (CounterAv+1)%SARRAY;         // loop counter
      CounterAm = (CounterAm+1)%SARRAY;         // loop counter
      vol=0;
    }
    else {
      if ( (level-AvgCurrent) < MINPEAK) CurrentArray0[CounterCu] = AvgCurrent+MINPEAK;    // Correct large minus peaks
      else CurrentArray0[CounterCu] = level  ;                                          // Safe current Sample
      
      for (w=0 ; w<ARRAY ; ++w){                                                       // loop array and calculate average of the buffer
          tmp += CurrentArray0[w];                                                     // tmp = sum samples
          vol += abs(CurrentArray0[w]-AvgCurrent);                                     // vol = sum of differences
          }
    AvgCurrent = tmp/ARRAY;                           // Calculae new Average of ARRAY    
    vol=(1000*(vol/ARRAY))/AvgCurrent ;               // scale to volume to average per array-sample relative to average (in 10* %)
    ShadowArray[CounterSh]= CurrentArray0[CounterCu];  // save current to debug array
    AverageArray0[CounterAv] = AvgCurrent;            // save Average to current measurement
    AmplitudeArray0[CounterAv] = vol;                 // save Volume to Array
      CounterCu = (CounterCu+1)%ARRAY;          // loop counter small Array
      CounterSh = (CounterSh+1)%SARRAY;         // loop counter
      CounterAv = (CounterAv+1)%SARRAY;         // loop counter
      CounterAm = (CounterAm+1)%SARRAY;         // loop counter
   }
   return (vol);  
}



// power down thew motor, disable all mosfets
void MotorPowerDown() {
  digitalWrite(DIRA, LOW);     //DISABLE CH A
  digitalWrite(BRAKEA, HIGH);   //Sets direction of CH A
  digitalWrite(DIRB, LOW);     //DISABLE CH B
  digitalWrite(BRAKEB, HIGH);   //Sets direction of CH B
}



// Dump data to serial in xls format semicolon separated
void DumpData() {
int u;
//Dump Array data Shadow
#if DBG
 Serial.print("Currrent;Avg;Ampl;");Serial.print(Lock);Serial.print(";");Serial.println(CounterSt);
  for (u=0;u<SARRAY;++u) {
  Serial.print(ShadowArray[u]); Serial.print(";"); // current 
  Serial.print(AverageArray0[u]); Serial.print(";"); // average array
  Serial.print(AmplitudeArray0[u]); Serial.println(";"); // Ampl in 10* %
  }
//Dump Monitor Array data
//  Serial.print("Lock ");Serial.print(Lock);Serial.print(";Steps ");Serial.print(CounterSt);Serial.println(".;");
//  Serial.println("Current;Average;VolumeDiff;");
//  for (u=0;u<ARRAY;++u) {
//  Serial.print(CurrentArray0[u]); Serial.print(";");      // current in mA
//  Serial.print(AverageArray0[u]); Serial.print(";");      // average in mA
//  Serial.print(AmplitudeArray0[u]); Serial.println(";"); // Volume diff in %
//  }
#endif  
}

// Calculates day of the week,  for ref : mon=day1 .. sun=day7
// input is unixtime, sec. since 1-1-1970 (this was a Thursday, day4)
int weekday(long int unixepoch)
{
int tmp;
tmp = (unixepoch/(60*60*24))%7; // days since 1-1-1970 , trunced per 7 days (value 0 - 6)
tmp = (tmp+3)%7 + 1 ;           // correct for day of the week as from Thursday (weekday4, bitcount day3
return(tmp);
}
