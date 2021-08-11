#include "Arduino.h"

#include "ArduinoLog.h"
#include "accelGyroGG.h"
#include "ultrasonicSensor.h"
#include "SIM808.h"
#include <SoftwareSerial.h>
#include <DFRobot_sim808.h>

#define PIN_TX_SIM PA8
#define PIN_RX_SIM PB10
#define DEBUG true
#define SIM_PWR PB4           ///< SIM808 PWRKEY
#define NO_FIX_GPS_DELAY 3000 ///< Delay between each GPS read when no fix is acquired
#define FIX_GPS_DELAY 10000   ///< Delay between each GPS read when a fix is acquired
#define POSITION_SIZE 128     ///< Size of the position buffer
#define SIM808_BAUDRATE 4800  ///< Control the baudrate use to communicate with the SIM808 module
#define SERIAL_BAUDRATE 9600  ///< Control the baudrate use to communicate with the SIM808 module
#define NL "\n"
const int PIN_ROSA = PB3;

#define START_SEND_GPS_SITE 1 //ATTIVA_INVIO_POSIZIONE_GPS
#define STOP_SEND_GPS_SITE 2  //DISATTIVA_INVIO_POSIZIONE_GPS
#define SEND_POSITION_CAR 3   //SEND_POSITION_CAR
char PHONE_NUMBER_WHO_HAS_TO_SEND_SMS[11] = "3206866749";

SoftwareSerial mySerial(PIN_TX_SIM, PIN_RX_SIM);
DFRobot_SIM808 sim808(&mySerial); //Connect RX,TX,PWR

// Variables definition
int first_distance, measured_distance; // variable for the distance measurement
long duration;                         // variable for the duration of sound wave travel

bool isAlarmActive;

MyGyro gyro;
String res;

String coords;

#define FRAME_LENGTH 140
char frame[FRAME_LENGTH];
char UTCdatetime[18];
char latitude[11];
char longitude[12];
//char altitude[8];
/* char speedOTG[6];
char course[6];
char fixmode[1];
char HDOP[4];
char PDOP[4];
char VDOP[4];
char satellitesinview[2];
char GNSSsatellitesused[2];
char GLONASSsatellitesused[2];
char cn0max[2];
char HPA[6];
char VPA[6]; */
String longitude_s, latitude_s;

int adaptive_delay = 1000;
int smsUnread = -1;
char buffer[300];
char phone[20];
char datetime[40];
bool gps_attached = false;
bool sendPositionInPost = false;
unsigned long previousMillis, currentMillis;

const long interval = 15000;

// --- Prototypes
String get_GPS();
static char *strtok_single(char *str, char const *delims);
String floatToString(float, byte);
void initializeBuffersForSms();
void displayInfo();
boolean haveToSendPeriodicallyPosition();
int readSmsFromMyPhone();
void sendPostData();
void call();
// --- Code!

String sendData(String command, const int timeout, boolean debug)
{
  String response = "";
  mySerial.println(command);
  long int time = millis();
  while ((time + timeout) > millis())
  {
    while (mySerial.available())
    {
      char c = mySerial.read();
      response += c;
    }
  }
  if (debug)
  {
    Serial.print(response);
  }
  return response;
}

void setup()
{
  pinMode(PIN_RX_SIM, OUTPUT);
  pinMode(PIN_TX_SIM, INPUT);
  
  mySerial.begin(SIM808_BAUDRATE);
  Serial.begin(SERIAL_BAUDRATE);
  pinMode(SIM_PWR, OUTPUT);
  

  // Initialize with log level and log output.
  Log.begin(LOG_LEVEL_VERBOSE, &Serial);

  // gyro.setupGyro();
  // setupUS();

   
  if (!sim808.checkPowerUp())
   sim808.powerUpDown(SIM_PWR);
      

  Log.notice("Initializing" NL);

  //delay(15000);

  // first_distance = getDistance();
  // gyro.readAndUpdateValues();

  //********Initialize sim808 module*************
  bool initialized = false;
  while (!initialized)
  {
    int times = 1;
    while (!(initialized = sim808.init()) && times <= 3)
    {
      delay(adaptive_delay);
      times++;
    }
    if (times > 3)
    {
      adaptive_delay *= 2;
      Log.notice("Sim808 init error - check if RX and TX are in the right place!" NL);
    }
  }
  Log.notice("Sim808 init success" NL);

  initializeBuffersForSms();

  //isAlarmActive = digitalRead(PIN_ROSA) > 0;
  isAlarmActive = false; // Scritto solo per debug
  previousMillis = millis();
}

void loop()
{
  currentMillis = millis();

  if ((currentMillis - previousMillis) >= interval)
  {
    
    previousMillis = currentMillis;
    //int smsRead = readSmsFromMyPhone();
    int smsRead = 3;
    /* Log.notice("Cod RES" NL);
     Serial.println(smsRead);*/

    switch (smsRead)
    {
    case START_SEND_GPS_SITE:
      if (!gps_attached)
        gps_attached = sim808.attachGPS();
      if (!gps_attached)
      {
        Log.notice("Error connection with GPS" NL);
        return; // Il sim non riesce a connettere il GPS!
      }
      if (!sim808_check_with_cmd("AT+SAPBR=3,1,\"APN\",\"internet.it\"", "OK", CMD))
      {
        Log.notice("Error connection with gprs" NL);
        return;
      }
      delay(1500);

      if (!sim808_check_with_cmd("AT+SAPBR=1,1", "OK", CMD))
      {
        Log.notice("Error connection with gprs 2" NL);
        return;
      }
      delay(1500);
      sendPositionInPost = true;
      break;
    case STOP_SEND_GPS_SITE:
      sendPositionInPost = false;
      sim808.detachGPS();
      sim808.close();
      sim808.disconnect();
      break;
    case SEND_POSITION_CAR:
      if (!gps_attached)
        gps_attached = sim808.attachGPS();
      if (!gps_attached)
      {
        Log.notice("Error connection with GPS" NL);
        return; // Il sim non riesce a connettere il GPS!
      }

      coords = get_GPS();
      delay(1500);

      if (coords != "")
      {
        Log.notice("Coordinate ottenute: " NL);
        Serial.println(coords);
        // ---- Invio SMS
        String msg = "Posizione auto: https://www.google.com/maps/search/?api=1&query=" + coords;

        int n = msg.length();
        char char_array[n + 1];
        strcpy(char_array, msg.c_str());
        Serial.println("INVIO FAKE SMS CON TESTO: ");
        Serial.println(char_array);
        //sim808.sendSMS(PHONE_NUMBER_WHO_HAS_TO_SEND_SMS, char_array);
      }
      else
        Log.notice("coord vuote");
      break;
    default:
      break;
    }
  }

  if (sendPositionInPost)
  {
    coords = get_GPS();
    delay(1500);
    if (coords != "")
    {
      Log.notice("Coordinate ottenute: " NL);
      Serial.println(coords);
      sendPostData();
    }
  }

  if (isAlarmActive)
  {
    measured_distance = getDistance();
    //gyro.updateAccellGyro();

    if (
        //gyro.movementDetected() ||
        (measured_distance - first_distance < sogliaDistanza))
      // casi coperti: furto nella notte, furto con jammer o botta all'auto.
      call();
  }
  delay(5000);
}

String get_GPS()
{
  String coordsToOutput = "";
  int8_t counter, answer;
  long previous;
  float latGPS;
  float longGPS;
  counter = 0;
  answer = 0;
  memset(frame, '\0', sizeof(char)*FRAME_LENGTH); // Initialize the string
  previous = millis();

  mySerial.write("AT+CGNSINF\r\n");

  do
  {
    if (mySerial.available() > 0)
    {
      frame[counter] = mySerial.read();
      counter++;
      // check if the desired answer is in the response of the module
      if (strstr(frame, "OK") != NULL)
      {
        answer = 1;
      }
    }
  } while ((answer == 0) && ((millis() - previous) < 3000));
  
  if(counter >= 6)
    frame[counter - 6] = '\0'; 
  if (DEBUG)
  {
    Serial.println("\n\n\n\n\n ----- DEBUG BUFFER dopo la lettura del fix --------- ");
    Serial.println(frame);
    Serial.println("\n----- ---------- ---------- --------- ");
  }

  if(answer) {
    strtok_single(frame, ": ");
    strtok_single(NULL, ",");                      // Gets GNSSrunstatus
    strtok_single(NULL, ",");                      // Gets Fix status
    strcpy(UTCdatetime, strtok_single(NULL, ",")); // Gets UTC date and time
    strcpy(latitude, strtok_single(NULL, ","));    // Gets latitude
    strcpy(longitude, strtok_single(NULL, ","));    // Gets longitude
    //Serial.println("FRAME AFTER STRTOK");
    //Serial.println(frame);
    /*   strcpy(altitude, strtok_single(NULL, ","));     // Gets MSL altitude
    strcpy(speedOTG, strtok_single(NULL, ","));     // Gets speed over ground
    strcpy(course, strtok_single(NULL, ","));       // Gets course over ground
    fixmode = atoi(strtok_single(NULL, ","));       // Gets Fix Mode
    strtok_single(NULL, ",");
    strcpy(HDOP, strtok_single(NULL, ",")); // Gets HDOP
    strcpy(PDOP, strtok_single(NULL, ",")); // Gets PDOP
    strcpy(VDOP, strtok_single(NULL, ",")); // Gets VDOP
    strtok_single(NULL, ",");
    strcpy(satellitesinview, strtok_single(NULL, ","));      // Gets GNSS Satellites in View
    strcpy(GNSSsatellitesused, strtok_single(NULL, ","));    // Gets GNSS Satellites used
    strcpy(GLONASSsatellitesused, strtok_single(NULL, ",")); // Gets GLONASS Satellites used
    strtok_single(NULL, ",");
    strcpy(cn0max, strtok_single(NULL, ",")); // Gets C/N0 max
    strcpy(HPA, strtok_single(NULL, ","));    // Gets HPA
    strcpy(VPA, strtok_single(NULL, "\r"));   // Gets VPA */

    if (strlen(longitude) != 0 && strlen(latitude) != 0)
    {
      Serial.println("Coord valide");
    
      longitude_s = String(longitude);
      Serial.println("Longitudine in stringa:" + longitude_s);

      latitude_s = String(latitude);
      Serial.println("latitudine in stringa:" + latitude_s);

      /*  
        longGPS = atof(longitude);
        latGPS = atof(latitude);
      */
      /*
        Serial.println("mia float latitudine");
        Serial.println(latGPS, 6);
        Serial.println("mia float latitudine");
        Serial.println(longGPS, 6);
      */

      coordsToOutput.concat(latitude_s);
      coordsToOutput.concat(",");
      coordsToOutput.concat(longitude_s);
    }
  
  }
  else
  {
    delay(5000);
    mySerial.flush();
  }
 
  return coordsToOutput;
}

/* strtok_fixed - fixed variation of strtok_single */
static char *strtok_single(char *str, char const *delims)
{
  static char *src = NULL;
  char *p, *ret = 0;

  if (str != NULL)
    src = str;

  if (src == NULL || *src == '\0') // Fix 1
    return NULL;

  ret = src; // Fix 2
  if ((p = strpbrk(src, delims)) != NULL)
  {
    *p = 0;
    src = ++p;
  }
  else
    src += strlen(src);

  return ret;
}

void call()
{
  //*********Call specified number***************

  sim808.callUp("3206866749");
  delay(15000);
  sim808.hangup();
  delay(2000);
}

void sendPostData()
{
  sendData("AT+HTTPINIT", 1000, true);
  delay(1500);

  sendData("AT+HTTPPARA=\"CID\",1", 1000, true);
  delay(1500);

  sendData("AT+HTTPPARA=\"URL\",\"http://webhook.site/dde9ac46-6f83-45dd-bf45-9525979e27cc\"", 1000, true);
  delay(1500);
  sendData("AT+HTTPPARA=\"CONTENT\",\"application/json\"", 1000, true);
  delay(1500);
  sendData("AT+HTTPDATA=\"40,20000\"", 1000, true);
  delay(1500);
  String body = "{latitude:" + latitude_s + ";longitude:" + longitude_s + "}";
  int n = body.length();
  char char_array[n + 1];
  strcpy(char_array, body.c_str());
  sendData(char_array, 1000, true);

  sendData("AT+HTTPACTION=1", 1000, true);
}

int readSmsFromMyPhone()
{
  while ((smsUnread = sim808.isSMSunread()) > 0)
  {
    sim808.readSMS(smsUnread, buffer, 300, phone, datetime);
    Log.notice("BUFFER:");
    Log.notice(buffer);

    if (strstr(PHONE_NUMBER_WHO_HAS_TO_SEND_SMS, phone) != NULL)
    {
      if (strstr("ATTIVA_INVIO_POSIZIONE_GPS", buffer) != NULL)
        return START_SEND_GPS_SITE;
      else if (strstr("DISATTIVA_INVIO_POSIZIONE_GPS", buffer) != NULL)
        return STOP_SEND_GPS_SITE;
      else
        return SEND_POSITION_CAR;
    }
    initializeBuffersForSms();
  }
  return SEND_POSITION_CAR;
}

String floatToString(float x, byte precision = 2)
{
  return String(x, precision);
}

void initializeBuffersForSms()
{
  memset(buffer, '\0', 300);
  memset(phone, '\0', 20);
  memset(datetime, '\0', 40);
}