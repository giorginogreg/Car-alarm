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

#define START_SEND_GPS_SITE 1 //ATTIVA_INVIO_POSIZIONE_GPS
#define STOP_SEND_GPS_SITE 2  //DISATTIVA_INVIO_POSIZIONE_GPS
#define SEND_POSITION_CAR 3   //SEND_POSITION_CAR

#define COMMANDS_BUFFER_SIZE 6
char PHONE_NUMBER_WHO_HAS_TO_SEND_SMS[11] = "3206866749";

SoftwareSerial mySerial(PIN_TX_SIM, PIN_RX_SIM);
DFRobot_SIM808 sim808(&mySerial); //Connect RX,TX,PWR

// Variables definition
int first_distance, measured_distance; // variable for the distance measurement
long duration;                         // variable for the duration of sound wave travel

bool isAlarmActive;

MyGyro gyro;

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
bool doActionBasedOnSmsRcvd(unsigned int smsReceived);
void restartSimAndAVR();
// --- Code!

String sendData(String command, const unsigned int timeout)
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
  if (DEBUG)
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

  // gyro.setupGyro();
  // setupUS();

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

  if ((currentMillis - previousMillis) >= interval) // Eseguo codice sottostante solo a intervalli di <interval> secondi
  {
    previousMillis = millis();

    int smsRead = readSmsFromMyPhone();
    Serial.print("smsRead: ");
    Serial.println(smsRead);
    //smsRead = SEND_POSITION_CAR; // Only for debug
  }

  if (!doActionBasedOnSmsRcvd(smsRead)) // If there is some sms read and any action goes wrong, restart the loop
    return;
  if (sendPositionInPost)
  {
    Log.notice("Invio posizione in post");

    coords = get_GPS();
    delay(2500);

    sendPostData();
    sendPositionInPost = false;
  }

  if (isAlarmActive)
  {
    Log.notice("Allarme attivo!");
    //measured_distance = getDistance();
    //gyro.updateAccellGyro();

    if (
        //gyro.movementDetected() ||
        (measured_distance - first_distance < sogliaDistanza))
      // casi coperti: furto nella notte, furto con jammer o botta all'auto.
      call();
    delay(5000);
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
      Log.notice("answer: ");
      Serial.println(answer);

      Serial.println("\n\n\n\n\n ----- DEBUG BUFFER dopo la lettura del fix --------- ");
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
      Log.noticeln("Risposta senza OK o non in tempo valido");
      delay(5000);
      mySerial.flush();
    }
    if (!fixObtained)
    {
      Log.noticeln("Fix non ancora ottenuto, aspetto 3 sec prima del prossimo comando");
      delay(3000); // Aspetto 5 sec prima di mandare il prossimo comando.
    }
  } while (
      !fixObtained &&
      ((millis() - secondTimer) < 300000)); // Attendo 5 minuti, appeno supero e non ho ancora ottenuto fix, resetto gps e avr

  if (!fixObtained && (millis() - secondTimer) < 300000)
    restartSimAndAVR();

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
  sendData("AT+CSTT=\"internet.it\"", 1000);
  delay(1500);

  sendData("AT+CIICR", 1000);
  delay(1500);

  sendData("AT+CIFSR", 1000);
  delay(1500);
  sendData("AT+CIPSPRT=1", 1000);
  delay(1500);
  sendData("AT+CIPSTART=\"TCP\",\"allarme-auto.herokuapp.com\",80", 3500);
  delay(1500); // Aspettare connect ok

  String commands[COMMANDS_BUFFER_SIZE];
  commands[0] = "POST / HTTP/1.1\0";
  commands[1] = "Host: allarme-auto.herokuapp.com\0";
  commands[2] = "Content-Type: application/x-www-form-urlencoded\0";
  String body = "latitude=" + latitude_s + "&longitude=" + longitude_s + "\0";
  int length_body = body.length();
  char buffer_composed[50];
  memset(buffer_composed, '\0', 50);
  int length_buffer_composed = sprintf(buffer_composed, "Content-Length: %d\0", length_body);
  commands[3] = String(buffer_composed);
  commands[4] = "";
  commands[5] = body;

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

  sendData(commands[COMMANDS_BUFFER_SIZE - 1].c_str(), 3500); // Fatto a mano per leggere risposta

  Log.noticeln("Invio effettuato");
}

int readSmsFromMyPhone()
{
  while ((smsUnread = sim808.isSMSunread()) > 0)
  {
    sim808.readSMS(smsUnread, smsBuffer, (int)BUFFER_SMS_SIZE, phone, datetime);
    Log.notice("BUFFER:");
    Log.notice(smsBuffer);

    if (strstr(PHONE_NUMBER_WHO_HAS_TO_SEND_SMS, phone) != NULL)
    {
      if (strstr("ATTIVA_INVIO_POSIZIONE_GPS", smsBuffer) != NULL)
        return START_SEND_GPS_SITE;
      else if (strstr("DISATTIVA_INVIO_POSIZIONE_GPS", smsBuffer) != NULL)
        return STOP_SEND_GPS_SITE;
      else
        return SEND_POSITION_CAR;
    }
    initializeBuffersForSms();
  }
  return -1; // TODO: rimuovere dopo debug
}

String floatToString(float x, byte precision = 2)
{
  return String(x, precision);
}

bool doActionBasedOnSmsRcvd(unsigned int smsReceived)
{
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
    if (!sim808_check_with_cmd("AT+SAPBR=3,1,\"APN\",\"internet.it\"\r\n", "OK", CMD))
    {
      Log.noticeln("Error connection with gprs" NL);
      return false;
    }
    delay(3500);

    if (!sim808_check_with_cmd("AT+SAPBR=1,1\r\n", "OK", CMD))
    {
      Log.noticeln("Error connection with gprs 2" NL); //TODO: Anche qui, se ho sempre errore, vuol dire che magari era già avviato il sim quindi è da resettare sim e ardu
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
    if (!sim808.close())
    {
      Log.noticeln("Error closing TCP connection.");
      return false;
    }
    sim808.disconnect();
    sendPositionInPost = false;
    break;
  case SEND_POSITION_CAR:
    if (!gps_attached)
      gps_attached = sim808.attachGPS();
    if (!gps_attached)
    {
      Log.noticeln("Error connection with GPS" NL);
      return false; // Il sim non riesce a connettere il GPS!
    }

    coords = get_GPS();
    delay(1500);

    if (coords != "")
    {
      /* Log.notice("Coordinate ottenute: " NL);
      Serial.println(coords); */
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
  smsRead = -1;
  return true;
}

void initializeBuffersForSms()
{
  memset(smsBuffer, '\0', (int)BUFFER_SMS_SIZE);
  memset(phone, '\0', 20);
  memset(datetime, '\0', 40);
}

void restartSimAndAVR()
{
  Log.notice("DA IMPLEMENTARE! -- ");
  Log.noticeln("ERRORE! FIX DOPO 5 MIN NON ANCORA OTTENUTO. RIAVVIO ARDUINO E SIM");
}