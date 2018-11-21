#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
#include <SoftwareSerial.h>
#endif

#include "Statistic.h"

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#define FACTORYRESET_ENABLE         1
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"

// constants won't change. Used here to set a pin number :
const int Canal1P = 9;      // Canal 1 fase positiva
const int Canal1N = 10;      // Canal 1 fase negativa
const int CanalRele = 12;     // Canal do Rele
const int PWMm1   = 5 ;      // Controle PWM Canal 1
const int analogInMusclePin = 23;  // Analog input pin that the muscle sensor is attached to
const int interruptPin = 2;


// Variables will change :
int Canal1Pstate = LOW;     // estado inicial do canal 1
int Canal1Nstate = LOW;     // estado inicial do canal 1
int PWMm1State = 0;

boolean setupParam = false;   // estado inicial do setup de parâmetros

float ts = 0;   //case s (115)
float td = 0;  //case s (115)
float tp = 0;    //case s (115)
float th = 0;    //case c (99)
float in = 0;    //case s (115)
float lp = 0;    //case s (115)
float fr = 0;    //case s (115)


unsigned long StartTime = 0; // watch dog for data waiting loops


/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


// A small helpers
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}


void setup(void)
{
  //configuração das entradas
  pinMode(Canal1P, OUTPUT);
  pinMode(Canal1N, OUTPUT);
  pinMode(CanalRele, OUTPUT);
  pinMode(PWMm1,  OUTPUT);

  //while (!Serial);  // required for Flora & Micro
  //delay(500);

  Serial.begin(115200);


  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
    delay(500);
  }

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }
  ble.read();
  // Set module to DATA mode
  ble.setMode(BLUEFRUIT_MODE_DATA);


}

//void(* resetFunc) (void) = 0;//declare reset function at address 0


void loop(void)
{

  //Variables inicialization
  char switchSelector[1]; //characters received from Bluetooth

  Statistic measureStats;    //case r
  int aux = 0;    //case c (general purpose auxiliar variable)(100)
  int lin = 0;    //case c (general purpose auxiliar variable)(100)
  int col = 0;    //case c (general purpose auxiliar variable)(100)


  // Wait for switch instruction
  Serial.println("Awaiting instruction");
  while ( !ble.available() ) {
  }

  // Receive data instruction to switch
  while ( ble.available() ) {
    switchSelector[0] = char(ble.read());
    Serial.print(switchSelector[0]);
    ble.println(switchSelector[0]);
  }

  switch (switchSelector[0]) {

    case 'c': {  //config parameters received from app (c)

        char downloadData[128];    //case c (100)
        setupParam = false;

        StartTime = millis();
        while ( !ble.available() &&  ( millis() - StartTime < 10000)) {  //wait for data income for maximum 10 sec
        }
        StartTime = 0;

        if (!ble.available()) {  //check if data is available, otherwise reset
          Serial.println("Timeout...");
          break;
        }

        Serial.println("Awaiting parameters");
        while ( !ble.available() ) {
        }

        while (ble.available()) {
          downloadData[col] = char(ble.read());   //read, store and convert ascii decimal code to character coming in on Bluetooth
          Serial.print(downloadData[col]);
          col++;
        }

        char* valPosition;
        valPosition = strtok(downloadData, "_");
        int splitData[7];

        for (int i = 0; i < 7; i++) {
          splitData[i] = atof(valPosition);
          valPosition = strtok(NULL, "_");
        }

        ts = splitData[0];
        td = splitData[1];
        tp = splitData[2];
        th = map(splitData[3], 0, 100, 0, 1023);
        in = map(splitData[4], 1, 10, 44, 60);
        lp = splitData[5] - 32; //32 us adjustment because of uc processing time
        fr = splitData[6];

        if (th == 0) {
          ts = 2;
          tp = 5;
          td = 3;
        }

        Serial.print("\nts: ");
        Serial.println(ts);

        Serial.print("td: ");
        Serial.println(td);

        Serial.print("tp: ");
        Serial.println(tp);

        Serial.print("th: ");
        Serial.println(th);

        Serial.print("in: ");
        Serial.println(in);

        Serial.print("lp: ");
        Serial.println(lp);

        Serial.print("fr: ");
        Serial.println(fr);

        //memset(downloadData, 0, sizeof(downloadData));

        setupParam = true;
        ble.println("ck");   //ck = configuration okay
        break;
      }
    //----------------------------------------------

    case 's': {
        float incS = ceil((in / (ts * fr) ));
        float incD = ceil((in / (td * fr) ));

        float iterationS = (ts * fr);
        float iterationD = (td * fr);
        float iterationP = (tp * fr);

        float delayFr = ceil( ( ( (1 / fr) * 1000000 ) - (2 * lp) ) / 1000);

        if (!setupParam) {
          Serial.println("Parametros nao configurados...");
          break;
        }

refreshCase:
        while (ble.isConnected() && ble.read() != 'b') {

          digitalWrite(CanalRele, LOW);   // desliga o rele

          if (analogRead(analogInMusclePin) > th ) {

            digitalWrite(CanalRele, HIGH);   // liga o rele

            ble.println("1_" + String(map(analogRead(analogInMusclePin), 0, 1023, 0, 100)));
            Serial.println("estimulando");

            for (float i = 0; i < iterationS ; i++) {

              PWMm1State + incS > in ? PWMm1State = in : PWMm1State = PWMm1State + incS;  //set value max of PWM1State

              analogWrite(PWMm1, PWMm1State);


              digitalWrite(Canal1P, HIGH);   // liga o canal1P
              delayMicroseconds(lp);      // pausa por 300 microsegundos
              digitalWrite(Canal1P, LOW);    // desliga o canal1P

              digitalWrite(Canal1N, HIGH);   // liga o canal1N
              delayMicroseconds(lp);      // pausa por 300 microsegundos
              digitalWrite(Canal1N, LOW);   // desliga o canal1N

              delay(delayFr);
            }

            ble.println("1_" + String(map(analogRead(analogInMusclePin), 0, 1023, 0, 100)));

            for (float i = 0; i < iterationP ; i++) {

              analogWrite(PWMm1, in);


              digitalWrite(Canal1P, HIGH);   // liga o canal1P
              delayMicroseconds(lp);      // pausa por 300 microsegundos
              digitalWrite(Canal1P, LOW);    // desliga o canal1P

              digitalWrite(Canal1N, HIGH);   // liga o canal1N
              delayMicroseconds(lp);      // pausa por 300 microsegundos
              digitalWrite(Canal1N, LOW);   // desliga o canal1N

              delay(delayFr);
            }

            ble.println("1_" + String(map(analogRead(analogInMusclePin), 0, 1023, 0, 100)));

            for (float i = 0; i <  iterationD ; i++) {

              PWMm1State - incD < 0 ? PWMm1State = 0 : PWMm1State = PWMm1State - incD;  //set value max of PWM1State

              analogWrite(PWMm1, PWMm1State);


              digitalWrite(Canal1P, HIGH);   // liga o canal1P
              delayMicroseconds(lp);      // pausa por 300 microsegundos
              digitalWrite(Canal1P, LOW);    // desliga o canal1P

              digitalWrite(Canal1N, HIGH);   // liga o canal1N
              delayMicroseconds(lp);      // pausa por 300 microsegundos
              digitalWrite(Canal1N, LOW);   // desliga o canal1N

              delay(delayFr);

            }
            
            digitalWrite(CanalRele, LOW);   // desliga o rele
            
            th == 0 ? delay(7000) : delay(5500) ;
          }

          ble.println("0_" + String(map(analogRead(analogInMusclePin), 0, 1023, 0, 100)));
          Serial.println("nao estimulando");
        } //break while loop

        PWMm1State = LOW;
        ble.println("b");
        break;
      }
    //----------------------------------------------

    case 'f': {  //calibration Routine

        // Variables will change :
        Canal1Pstate = LOW;     // estado inicial do canal 1
        Canal1Nstate = LOW;     // estado inicial do canal 1
        PWMm1State = 0;

        setupParam = false;   // estado inicial do setup de parâmetros

        ts = 0;   //case s (115)
        td = 0;  //case s (115)
        tp = 0;    //case s (115)
        th = 0;    //case c (99)
        in = 0;    //case s (115)
        lp = 0;    //case s (115)
        fr = 0;    //case s (115)

        StartTime = 0; // watch dog for data waiting loops

        aux = 0;    //case c (general purpose auxiliar variable)(100)
        lin = 0;    //case c (general purpose auxiliar variable)(100)
        col = 0;    //case c (general purpose auxiliar variable)(100)

        /*
          ble.print("Iniciando calibracao...\n");
          measureStats.clear();    //explicitly start clean
          for (int i = 0; i < 100 ; i++) {
          measureStats.add(analogRead(analogInMusclePin));    // read the analog in value and compute to statistics
          delay(100);   //delay for reading sensor values
          }
          th = (measureStats.maximum() - measureStats.average()) / 2;
          measureStats.pop_stdev() <= 50 ? ble.print(th) : ble.print("NAK");   // check for measure quality, pass th value or error
          ble.print("\nETB\n");   //send acknowledge message
        */
        break;
      }
    //----------------------------------------------

    default:
      break;

  }
}


