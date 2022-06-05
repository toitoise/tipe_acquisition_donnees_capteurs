/********************************************************************************************************
 * 
 * Librairie du CAN l'ADS1115 :
 *      https://github.com/RobTillaart/ADS1X15
 *      https://passionelectronique.fr/tutorial-ads1115/
 *      https://cdn-learn.adafruit.com/downloads/pdf/adafruit-4-channel-adc-breakouts.pdf
 * Librairie pour DS3231:
 *      https://github.com/cvmanjoo/RTC
 * 
 *      On utilise le temps universel UTC non sujet aux changement d'heures.
 *      Cette lib retoune le temps EPOCH (secondes despuis 1970) pour tagger les echantillons
 * 
 ********************************************************************************************************/
// Version Minimaliste avec juste ADS1115+Affichage Serie+RTC
//------------------------------------------------------------


#include "ADS1X15_ST.h"
#include <Wire.h>
#include <time.h>
#include "RTC_ST.h"
#include "projet_definitions.h"

//-------------------------------------------------------
// RTC ds3231 i2c clock
//-------------------------------------------------------
static DS3231 RTC;    // Instancie l'objet RTC/DS3231
time_t epoch_RTC=0;   // Structure de temps 

//-------------------------------------------
// Variables pour l'heure et la date :
//-------------------------------------------
byte heure          = 14;     // 0 à 23
byte minute         = 54;     // 0 à 59
byte seconde        = 30;      // 0 à 59
byte numJourMois    = 7;      // 1 à 31
byte mois           = 5;      // 1 à 12
uint16_t annee      = 21;     // 0 à 99


//-------------------------------------------------------
// CAN ADS 1115
//-------------------------------------------------------
ADS1115 ADS(0x48);  // ADS1115 physiquement défini à l'adresse 0x48(default)

unsigned long current_time = 0;   
unsigned long previous_time = 0;

#include "project_own_functions.h"

//-------------------------------------------------------
//-------------------------------------------------------
//                         SETUP
//-------------------------------------------------------
//-------------------------------------------------------
#define current_gain DEFAULT_GAIN_DIFFERENTIAL
#define current_rate DEFAULT_RATE_DIFFERENTIAL

void setup() {
  Wire.begin();
  Wire.setClock(FAST_I2C_SPEED);      // Met la fréquence de l'I2C plus élevée

  // Inputs definitions
  pinMode (alertReadyPin, INPUT_PULLUP);
  
  // Initialisation de port série (usually 115200)
  Serial.begin(115200);

  // Configure l'ADS1115
  ADS.begin();
  ADS.setMode(0);                             // 0=continuous/1=single 
  // Configure le mode Continuous avec interruption générée (CF doc ads1115)
  ADS.setComparatorThresholdLow(0x0000 );   // MSB=0   for alertPin
  ADS.setComparatorThresholdHigh(0x8000 );  // MSB=1   for alertPin
  ADS.setComparatorQueConvert(0);           // trigger after One sample
  ADS.setGain(current_gain);                  
  ADS.setDataRate(current_rate);      
  ADS.requestADC_Differential_2_3(); // Preselect  MUX 

  //--------------------------------------------------------------
  // ALERT/RDY pin indicate when conversion is ready, to pin 2
  //--------------------------------------------------------------
  attachInterrupt(digitalPinToInterrupt(alertReadyPin), int_read_adc, RISING );
}

//-------------------------------------------------------
//-------------------------------------------------------
//                        LOOP
//-------------------------------------------------------
//-------------------------------------------------------
void loop() {
  //-------------------------------------------
  // Verifie si Interruption
  //-------------------------------------------
  if (DO_READ_ADC){
      // Mode Single Ended
      switch (muxStateforNextInterrupt){
        case 0:         // Channel A0-A1 (Differentiel)
          CAN_value_A0_A1=ADS.readADC_Differential_0_1(); // Récupère la valeur du CAN A0-A1
          epoch_RTC = RTC.getEpoch();                     // Récupère le temps courant
          Serial.print(epoch_RTC);Serial.print(",");      // Envoi la date sur le lien serie
          Serial.print(CAN_value_A0_A1);Serial.print(",");// envoi la valeur sur le lien serie
          muxStateforNextInterrupt=1;
          break;  
        case 1:         // Channel A2-A3    (Differentiel)
          CAN_value_A2_A3=ADS.readADC_Differential_2_3(); // Récupère la valeur du CAN A2-A3
          Serial.println(CAN_value_A2_A3);                // Envoi la veleur sur le lien serie
          muxStateforNextInterrupt=0;
          break;   
        default:
          Serial.println("ERROR: Channel incompatible avec le mode DIFFERENTIAL selectionné");
          break;
      }
    DO_READ_ADC=false;  // Remet le flag de l'intérruption à sa valeur de repos
  }
} // END LOOP



//-------------------------------------------------------
// Interrupt function to start reading CAN value
//-------------------------------------------------------
void int_read_adc(){
    DO_READ_ADC =true;
}
