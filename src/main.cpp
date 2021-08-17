#include "Arduino.h"

#include "ArduinoLog.h"
#include "accelGyroGG.h"
#include "ultrasonicSensor.h"
#include "SIM808.h"
#include <SoftwareSerial.h>
#include <DFRobot_sim808.h>

#define PIN_TX_SIM PB0
#define PIN_RX_SIM PB1
#define SIM_PWR PB2 ///< SIM808 PWRKEY
#define SIM_RST PB3 ///< SIM808 PWRKEY
#define DEBUG true
// #define NO_FIX_GPS_DELAY 3000 ///< Delay between each GPS read when no fix is acquired
// #define FIX_GPS_DELAY 10000   ///< Delay between each GPS read when a fix is acquired
#define POSITION_SIZE 128    ///< Size of the position buffer
#define SIM808_BAUDRATE 4800 ///< Control the baudrate use to communicate with the SIM808 module
#define SERIAL_BAUDRATE 9600 ///< Control the baudrate use to communicate with the SIM808 module
#define NL "\n"
const int PIN_ROSA = PB3;

#define START_SEND_GPS_SITE 1 //SUBMIT_GPS_SERVER
#define STOP_SEND_GPS_SITE 2  //STOP_SUBMIT_GPS_SERVER
#define SEND_POSITION_CAR 3   //SEND_POSITION_CAR

#define COMMANDS_BUFFER_SIZE 7
char PHONE_NUMBER_WHO_HAS_TO_SEND_SMS[11] = "3206866749";

SoftwareSerial mySerial(PIN_TX_SIM, PIN_RX_SIM);
DFRobot_SIM808 sim808(&mySerial); //Connect RX,TX,PWR

// Variables definition
int first_distance, measured_distance; // variable for the distance measurement
long duration;                         // variable for the duration of sound wave travel

bool isAlarmActive;
bool smsSent = false;
String coords;

#define FRAME_LENGTH 140
char frame[FRAME_LENGTH];
int GNSSrunstatus;
int Fixstatus;
char UTCdatetime[18];
char latitude[11];
char longitude[12];

String longitude_s,
    latitude_s;

int adaptive_delay = 1000;
int smsRead = -1;   // Determina quale è il messaggio letto tra quelli da calcolare
int smsUnread = -1; // Indice per l'ultimo messaggio in memoria sms
#define BUFFER_SMS_SIZE 300
char smsBuffer[BUFFER_SMS_SIZE];
char phone[20];
char datetime[40];

bool gps_attached = false;
bool sendPositionInPost = false;
unsigned long previousMillis, previousMillis2, currentMillis;

const long interval = 20000;
const long interval2 = 30000; // Intervallo di invio posizione heroku

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
bool doActionBasedOnSmsRcvd(int smsReceived);
void restartSimAndAVR();
// --- Code!

String sendData(String command, const unsigned int timeout, bool debug = true)
{
  String response = "";
  mySerial.println(command);
  long unsigned int time = millis();
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
    Serial.println(response);
  }
  return response;
}

void setup()
{
  pinMode(PIN_RX_SIM, OUTPUT);
  pinMode(PIN_TX_SIM, INPUT);
  pinMode(SIM_PWR, OUTPUT);
  pinMode(SIM_RST, OUTPUT);

  mySerial.begin(SIM808_BAUDRATE);
  Serial.begin(SERIAL_BAUDRATE);

  // Initialize with log level and log output.
  Log.begin(LOG_LEVEL_VERBOSE, &Serial);

  setupAccel();
  setupUS();

  do
  {
    Log.noticeln("Resetting Sim808");
    sim808.powerReset(SIM_PWR);
    delay(1000);
    Log.noticeln("Powering Sim808");
    sim808.powerUpDown(SIM_PWR);
    delay(10000);
  } while (!sim808.checkPowerUp());

  Log.notice("Initializing" NL);

  delay(10000);

  first_distance = getDistance();
  Serial.print("first_distance calculated: ");
  Serial.println(first_distance);

  // ********Initialize sim808 module*************
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

  mySerial.print("AT+CMGDA=\"");
  mySerial.println("DEL ALL\"");
  delay(1500);
  //sendData("AT+CMGDA=\"DEL ALL\"", 1500);

  //isAlarmActive = digitalRead(PIN_ROSA) > 0;
  isAlarmActive = true; // Scritto solo per debug
  previousMillis = millis();
  previousMillis2 = millis();
}

void loop()
{
  updateValuesGyro();
  currentMillis = millis();

  if ((currentMillis - previousMillis) >= interval) // Eseguo codice sottostante solo a intervalli di <interval> secondi
  {
    previousMillis = millis();

    if (smsRead < 0)
      smsRead = readSmsFromMyPhone(); // leggo solo se valore è -1 (può essere che ci sia già un messaggio letto precedentemente e non ancora settato a 0)

    if (!doActionBasedOnSmsRcvd(smsRead)) // If there is some sms read and any action goes wrong, restart the loop
      return;
  }

  if ((currentMillis - previousMillis2) >= interval2) // Eseguo codice sottostante solo a intervalli di <interval> secondi
  {
    previousMillis2 = millis();
    if (sendPositionInPost)
    {
      Log.notice("Invio posizione in post");

      coords = get_GPS();
      if (strstr(coords.c_str(), "ERROR"))
      {
        Log.notice("il metodo get_GPS è uscito con ERROR");
        return;
      }
      delay(2500);

      sendPostData();
    }
  }

  if (isAlarmActive)
  {
    measured_distance = getDistance();
    /*  Serial.print("measured calculated: ");
    Serial.println(measured_distance); */
    bool gyroAlarm = movementDetected();
    bool ultrasonicAlarm = ((first_distance - measured_distance) > sogliaDistanza);

    if (gyroAlarm || ultrasonicAlarm)
    {
      // casi coperti: furto nella notte, furto con jammer o botta all'auto.

      Serial.println("IN!");
      Serial.println(gyroAlarm);
      Serial.println(ultrasonicAlarm);
      delay(1500);
      call();
      delay(5000);
    }
  }
}

String get_GPS()
{
  long firstTimer, secondTimer;

  bool fixObtained = false;
  secondTimer = millis();
  String coordsToOutput = "";

  do
  {
    int8_t counter, answer;
    counter = 0;
    answer = 0;
    firstTimer = millis();
    latitude_s = "";
    longitude_s = "";
    memset(frame, '\0', sizeof(char) * FRAME_LENGTH); // Initialize the string
    Log.noticeln("Invio cgnsinf");
    mySerial.write("AT+CGNSINF\r\n");
    delay(500);
    do
    {
      if (mySerial.available() > 0)
      {
        frame[counter] = mySerial.read();
        counter++;
        // check if the desired answer is in the response of the module
        if (strstr(frame, "OK") != NULL)
          answer = 1;
      }
    } while ((answer == 0) && ((millis() - firstTimer) < 3000));

    if (counter >= 6)
      frame[counter - 6] = '\0'; // Counter - 6 perchè 6 sono i caratteri seguenti: \r \r O K \n \r

    if (DEBUG)
    {
      /*   Log.notice("answer: ");
      Serial.println(answer); */

      Serial.println("\n\n\n\n\n ----- DEBUG BUFFER after fix reading... --------- ");
      Serial.println(frame);
      Serial.println("\n----- ---------- ---------- --------- ");
    }

    if (answer) // Se ho ricevuto ok (anche se non ho fix), considero i dati.
    {

      strtok_single(frame, ": ");

      strtok_single(NULL, ",");                      // Gets GNSSrunstatus
      strtok_single(NULL, ",");                      // Gets Fix status // Possibile bug?
      strcpy(UTCdatetime, strtok_single(NULL, ",")); // Gets UTC date and time
      strcpy(latitude, strtok_single(NULL, ","));    // Gets latitude
      strcpy(longitude, strtok_single(NULL, ","));   // Gets longitude

      if (strlen(longitude) != 0 && strlen(latitude) != 0)
      {
        longitude_s = String(longitude);
        latitude_s = String(latitude);

        coordsToOutput.concat(latitude_s);
        coordsToOutput.concat(",");
        coordsToOutput.concat(longitude_s);
        fixObtained = 1;
      }
    }
    else // Risposta non ottenuta in tempo, oppure ok non ricevuto. Flusho seriale dopo attesa di 5 secondi.
    {
      Log.noticeln("ERROR: Answer without fix obtained or fix not read in valid time. Waiting a little bit then flushing serial");
      delay(5000);
      mySerial.flush();
    }
    if (!fixObtained)
    {
      Log.noticeln("Valid answer, but no fix obtained yet. Waiting 3 second before next request...");
      delay(3000); // Aspetto 5 sec prima di mandare il prossimo comando.
    }
  } while (
      !fixObtained &&
      ((millis() - secondTimer) < 300000)); // Attendo 5 minuti, appeno supero e non ho ancora ottenuto fix, resetto gps e avr

  if (!fixObtained && (millis() - secondTimer) >= 300000)
  {
    sim808.sendSMS(PHONE_NUMBER_WHO_HAS_TO_SEND_SMS, "FIX non ancora ottenuto. Riavvio loop");
    return "ERROR";
  }

  return coordsToOutput;
}

/* strtok_fixed - fixed variation of strtok_single */
static char *strtok_single(char *str, char const *delims)
{
  static char *src = NULL;
  char *p, *ret = 0;

  if (str != NULL)
    src = str; // La sorgente diventa il puntatore a carattere passato in input

  if (src == NULL || *src == '\0') // Fix 1
    return NULL;                   // Se il puntatore è null, o un delimitatore di fine stringa, ritorno null

  ret = src;                              // Altrimenti, il puntatore di src lo faccio puntare ad una stringa pari a quella in input
  if ((p = strpbrk(src, delims)) != NULL) // Se è stato trovato nella stringa in input il delim, ritorno un puntatore che parte dalla posizione della stringa dove è stato trovato
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

  sim808.callUp(PHONE_NUMBER_WHO_HAS_TO_SEND_SMS);
  delay(15000);
  sim808.hangup();
  delay(2000);
}

void sendPostData()
{
  sendData("AT+CIPSTART=\"TCP\",\"allarme-auto.herokuapp.com\",80", 3500);
  delay(1500); // Aspettare connect ok

  String commands[COMMANDS_BUFFER_SIZE];
  commands[0] = "POST / HTTP/1.1\0";
  commands[1] = "Host: allarme-auto.herokuapp.com\0";
  commands[2] = "Content-Type: application/x-www-form-urlencoded\0";
  commands[3] = "Connection: Close\0";
  String body = "latitude=" + latitude_s + "&longitude=" + longitude_s + "\0";
  int length_body = body.length();
  char buffer_composed[50];
  memset(buffer_composed, '\0', 50);
  int length_buffer_composed = sprintf(buffer_composed, "Content-Length: %d\0", length_body);
  commands[4] = String(buffer_composed);
  commands[5] = "";
  commands[6] = body;

  int sum_chars = 0;
  for (int i = 0; i < COMMANDS_BUFFER_SIZE; i++)
    sum_chars += commands[i].length() + 2; // +2 => \r \n

  // Log.notice("Lunghezza buffer: ");
  //Serial.println(sum_chars);

  char cipsend[50];
  memset(cipsend, '\0', 50);
  sprintf(cipsend, "AT+CIPSEND=%d", sum_chars);
  sendData(cipsend, 1500);

  for (int i = 0; i < COMMANDS_BUFFER_SIZE - 1; i++)
  {
    //Serial.print("Comando da inviare: ");
    //Serial.println(commands[i] + "\\r\\n");
    mySerial.println(commands[i]);
    delay(200);
  }

  sendData(commands[COMMANDS_BUFFER_SIZE - 1].c_str(), 6000, false); // Fatto a mano per leggere risposta
  mySerial.flush();
  Log.noticeln("Invio effettuato");
}

int readSmsFromMyPhone()
{
  while ((smsUnread = sim808.isSMSunread()) > 0)
  {
    sim808.readSMS(smsUnread, smsBuffer, (int)BUFFER_SMS_SIZE, phone, datetime);
    Log.notice("BUFFER sms letti:  ");
    Serial.println(smsBuffer);

    /// if (strstr(PHONE_NUMBER_WHO_HAS_TO_SEND_SMS, phone) != NULL)
    //{
    if (strstr("SUBMIT_GPS_SERVER", smsBuffer) != NULL)
      return START_SEND_GPS_SITE;
    else if (strstr("STOP_SUBMIT_GPS_SERVER", smsBuffer) != NULL)
      return STOP_SEND_GPS_SITE;
    else if (strstr("SEND_POSITION_CAR", smsBuffer) != NULL)
      return SEND_POSITION_CAR;
    // }
    initializeBuffersForSms();
  }
  return -1;
}

String floatToString(float x, byte precision = 2)
{
  return String(x, precision);
}

bool doActionBasedOnSmsRcvd(int smsReceived = -1)
{
  if (smsReceived > 0)
  {
    if (!smsSent)
    {
      char buffer_feedback[50];
      memset(buffer_feedback, '\0', 50);
      sprintf(buffer_feedback, "SMS ricevuto: %d, provvedo a quanto richiesto.", smsReceived);
      sim808.sendSMS(PHONE_NUMBER_WHO_HAS_TO_SEND_SMS, buffer_feedback);
      smsSent = true;
    }
  }

  switch (smsReceived)
  {
  case START_SEND_GPS_SITE:
    if (!gps_attached)
      gps_attached = sim808.attachGPS();
    if (!gps_attached)
    {
      Log.noticeln("Error connection with GPS" NL);
      return false; // Il sim non riesce a connettere il GPS!
    }

    if (!sim808_check_with_cmd("AT+CSTT=\"internet.it\"\r\n", "OK", CMD))
    {
      Log.noticeln("Error connection with gprs" NL);
      return false;
    }
    delay(3500);

    if (!sim808_check_with_cmd("AT+CIICR\r\n", "OK", CMD))
    {
      Log.noticeln("Error connection with gprs 2" NL);
      return false;
    }
    delay(2500);

    sendData("AT+CIFSR", 1000); // Non lo metto con il check perchè ritorna un ip in caso positivo.
    delay(1500);

    if (!sim808_check_with_cmd("AT+CIPSPRT=1\r\n", "OK", CMD))
    {
      Log.noticeln("Error connection with gprs 3" NL);
      return false;
    }
    delay(2500);

    Log.noticeln("Inizializzazione internet terminata");
    sendPositionInPost = true;
    break;
  case STOP_SEND_GPS_SITE:
    Log.noticeln("Stacco internet e GPS");
    if (!sim808.detachGPS())
    {
      Log.noticeln("Error detaching GPS.");
      return false;
    }
    gps_attached = false;
    if (!sim808.close())
    {
      Log.noticeln("Error closing TCP connection.");
      return false;
    }
    sim808.disconnect();
    sendPositionInPost = false;
    Log.noticeln("Connessione terminata, gps staccato");
    break;
  case SEND_POSITION_CAR:
    Log.noticeln("Sending Position car...");
    if (!gps_attached)
      gps_attached = sim808.attachGPS();
    if (!gps_attached)
    {
      Log.noticeln("Error connection with GPS" NL);
      return false; // Il sim non riesce a connettere il GPS!
    }

    coords = get_GPS();
    if (strstr(coords.c_str(), "ERROR"))
    {
      Log.notice("il metodo get_GPS è uscito con ERROR");
      return false;
    }
    delay(1500);

    if (coords != "")
    {
      // ---- Invio SMS
      String msg = "Posizione auto: https://www.google.com/maps/search/?api=1&query=" + coords;
      int n = msg.length();
      char char_array[n + 1];
      strcpy(char_array, msg.c_str());
      sim808.sendSMS(PHONE_NUMBER_WHO_HAS_TO_SEND_SMS, char_array);
    }
    break;
  default:
    break;
  }

  smsRead = -1; // resetto solo quando ho terminato di processare
  smsSent = false;
  return true;
}

void initializeBuffersForSms()
{
  memset(smsBuffer, '\0', (int)BUFFER_SMS_SIZE);
  memset(phone, '\0', 20);
  memset(datetime, '\0', 40);
}
