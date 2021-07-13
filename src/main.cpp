#include "Arduino.h"

#include "ArduinoLog.h"
#include "accelGyroGG.h"
#include "ultrasonicSensor.h"
#include "SIM808.h"
#include <SoftwareSerial.h>
#include <DFRobot_sim808.h>
#include <TinyGPS++.h>

#define PIN_TX 7
#define PIN_RX 6
SoftwareSerial mySerial(PIN_TX, PIN_RX);
TinyGPSPlus gps;

DFRobot_SIM808 sim808(&mySerial); //Connect RX,TX,PWR
// Variables definition
int first_distance, measured_distance; // variable for the distance measurement
long duration;                         // variable for the duration of sound wave travel

bool isAlarmActive;
int PIN_ROSA = 10;

//GPS
char position[128];

MyGyro gyro;
String res;
#define SIM_RST 5    ///< SIM808 RESET
#define SIM_RX 6     ///< SIM808 RXD
#define SIM_TX 7     ///< SIM808 TXD
#define SIM_PWR 8    ///< SIM808 PWRKEY
#define SIM_STATUS 8 ///< SIM808 STATUS

#define NO_FIX_GPS_DELAY 3000 ///< Delay between each GPS read when no fix is acquired
#define FIX_GPS_DELAY 10000   ///< Delay between each GPS read when a fix is acquired

#define POSITION_SIZE 128    ///< Size of the position buffer
#define SIM808_BAUDRATE 4800 ///< Control the baudrate use to communicate with the SIM808 module
#define NL "\n"
String
floatToString(float x, byte precision = 2)
{
  return String(x, 6);
}
void displayInfo();
String data[5];
#define DEBUG false
String coords, coordsToOutput;

char frame[100];
char GNSSrunstatus[1];
char Fixstatus[1];
char UTCdatetime[18];
char latitude[10];
char logitude[11];
char altitude[8];
char speedOTG[6];
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
char VPA[6];
String longitude_s, latitude_s;

String get_GPS();
static char *strtok_single(char *str, char const *delims);

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
  mySerial.begin(4800);
  Serial.begin(9600);
  pinMode(SIM_PWR, OUTPUT);

  // Initialize with log level and log output.
  Log.begin(LOG_LEVEL_VERBOSE, &Serial);

  Log.notice("Aspetto per l'accensione" NL);

  // gyro.setupGyro();
  //setupUS();

  // Start logging text and formatted values
  //Log.notice("Powering on SIM808..." NL);

  // - da fare a mano -- sim808.powerOnOff(true); //power on the SIM808. Unavailable without the PWRKEY pin wired
  if (!sim808.checkPowerUp())
    sim808.powerUpDown(SIM_PWR);
  Log.notice("Initializing" NL);
  delay(15000);

  //Log.notice("Powering on SIM808's GPS..." NL);
  // first_distance = getDistance();
  // gyro.readAndUpdateValues();

  //********Initialize sim808 module*************
  while (!sim808.init())
  {
    delay(1000);
    Serial.print("Sim808 init error\r\n");
  }
  Serial.println("Sim808 init success");

  //*********Call specified number***************
  /*
  sim808.callUp("3206866749");
  delay(15000);
  sim808.hangup();
  delay(2000); 
  */
  /* 
  sendData("AT+SAPBR=3,1,\"APN\",\"internet.it\"", 1000, true);
  delay(1500);
  sendData("AT+SAPBR=1,1", 1000, true);
  delay(1500);

  sendData("AT+HTTPINIT", 1000, true);
  delay(1500);

  // AT+HTTPINIT
  sendData("AT+HTTPPARA=\"CID\",1", 1000, true);
  delay(1500);

  // AT+HTTPPARA="CID",1
  sendData("AT+HTTPPARA=\"URL\",\"http://webhook.site/dde9ac46-6f83-45dd-bf45-9525979e27cc\"", 1000, true);
  delay(1500);

  sendData("AT+HTTPACTION=0", 1000, true);
  //************* Close TCP or UDP connections **********
  sim808.close();

  //*** Disconnect wireless connection, Close Moving Scene *******
  sim808.disconnect();
 */
  Serial.print("indice ultimo msg non letto:");
  Serial.println(sim808.isSMSunread());
  int smsUnread = -1;
  char buffer[300];
  memset(buffer, 0, 300);
  char phone[20];
  memset(phone, 0, 20);

  char datetime[40];
  memset(datetime, 0, 40);

  while ((smsUnread = sim808.isSMSunread()) > 0)
  {
    sim808.readSMS(smsUnread, buffer, 300, phone, datetime);
    Serial.println("BUFFER:");
    Serial.println(buffer);
    Serial.println("Phone:");
    Serial.println(phone);
    Serial.println("Datetime:");
    Serial.println(datetime);
  }
}

void loop()
{
  //isAlarmActive = digitalRead(PIN_ROSA) > 0;

  /*  sim.readNewSms();
  if (sim.sms_text != NULL)
  {
    sim.checkIfLastSmsReceivedIsAlarm();
  }
   measured_distance = getDistance();
  gyro.updateAccellGyro();

  if (gyro.movementDetected() ||
      (measured_distance - first_distance < sogliaDistanza))
  { }

  */
  /* 
  ---- GPS ---
  sim808.attachGPS();

  coords = get_GPS();
  delay(2500);
  if (coords != "")
  {
    Log.notice("Coordinate ottenute: " NL);
    Serial.println(coords);
    // ---- Invio SMS
    String msgWithPosition = "";
    String msg = "[AUTO] FURTO AUTO RILEVATO! ATTENZIONE!\n";
    msgWithPosition =
        "https://www.google.com/maps/search/?api=1&query=" + coords;

    msg.concat(msgWithPosition);
    int n = msg.length();

    // declaring character array
    char char_array[n + 1];

    // copying the contents of the
    // string to char array
    strcpy(char_array, msg.c_str());
    sim808.sendSMS("3206866749", char_array);
    delay(60000);
  } */
}

String get_GPS()
{
  coordsToOutput = "";
  int8_t counter, answer;
  long previous;
  byte GNSSrunstatus;
  byte Fixstatus;
  float latGPS;
  float longGPS;
  byte fixmode;
  mySerial.write("AT+CGNSINF\r\n");
  counter = 0;
  answer = 0;
  memset(frame, '\0', sizeof(frame)); // Initialize the string
  previous = millis();
  // this loop waits for the NMEA string
  do
  {
    if (mySerial.available() != 0)
    {
      frame[counter] = mySerial.read();
      counter++;
      // check if the desired answer is in the response of the module
      if (strstr(frame, "OK") != NULL)
      {
        answer = 1;
      }
    }
  } while ((answer == 0) && ((millis() - previous) < 2000));
  frame[counter - 3] = '\0';
  if (true)
    Serial.println(frame);
  strtok_single(frame, ": ");
  GNSSrunstatus = atoi(strtok_single(NULL, ",")); // Gets GNSSrunstatus
  Fixstatus = atoi(strtok_single(NULL, ","));     // Gets Fix status
  strcpy(UTCdatetime, strtok_single(NULL, ","));  // Gets UTC date and time
  strcpy(latitude, strtok_single(NULL, ","));     // Gets latitude
  strcpy(logitude, strtok_single(NULL, ","));     // Gets longitude

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

  if (strlen(logitude) != 0 && strlen(latitude) != 0)
  {
    longitude_s = String(logitude);
    latitude_s = String(latitude);
    /*  longGPS = atof(logitude);
  latGPS = atof(latitude);
 */
    Serial.println("mia float latitudine");
    Serial.println(latGPS, 6);
    Serial.println("mia float latitudine");
    Serial.println(longGPS, 6);

    coordsToOutput.concat(latitude_s);
    coordsToOutput.concat(",");
    coordsToOutput.concat(longitude_s);
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
