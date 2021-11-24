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
 
#include "ADS1X15_ST.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <time.h>
#include "RTC_ST.h"

//-------------------------------------------------------
// RTC ds3231 i2c clock
//-------------------------------------------------------
static DS3231 RTC;
time_t epoch_RTC=0;

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
// MODE PIN for ADS conversion mode
// 1=Differential
// 0=Single-Ended
//-------------------------------------------------------
#define  SingleEndedOrDifferentialPin  3
#define  DiffMode   1
#define  SingleMode 0
short muxMode=DiffMode;

//-------------------------------------------------------
// ADS1115 PINS  (connected to interrupt pin)
//-------------------------------------------------------
#define  alertReadyPin  2

//-------------------------------------------------------
// Rotary encoder pins 
//-------------------------------------------------------
#define encoderClk    8    // CLK
#define encoderData   9    // Data
#define encoderSwitch 10    // Switch Button

//-------------------------------------------------------
// LCD 20x4 Declaration (i2c pins connexion)
//-------------------------------------------------------
LiquidCrystal_I2C lcd(0x27, 16, 2);

//-------------------------------------------------------
// CAN ADS 1115
//-------------------------------------------------------
ADS1115 ADS(0x48);  // ADS1115 physiquement défini à l'adresse 0x48(default)

/* Input multiplexer configuration **********
000 : AINP = AIN0 and AINN = AIN1   Diff A0-A1  (default) 
001 : AINP = AIN0 and AINN = AIN3   Diff A0-A3
010 : AINP = AIN1 and AINN = AIN3   Diff A1-A3
011 : AINP = AIN2 and AINN = AIN3   Diff A2-A3
100 : AINP = AIN0 and AINN = GND    A0
101 : AINP = AIN1 and AINN = GND    A1
110 : AINP = AIN2 and AINN = GND    A2
111 : AINP = AIN3 and AINN = GND    A3
********************************************/
short muxStateforNextInterrupt=0x0;

/* DateRate *********************************
DR=0 : 8 SPS
DR=1 : 16 SPS
DR=2 : 32 SPS
DR=3 : 64 SPS
********************************************/
const short sample_rate[5]      ={ 8,  16, 32, 64, 128 };
const short sample_rate_prog[5] ={ 0,   1,  2,  4,   8 };
#define sample_rate_sizeTab sizeof(sample_rate) / sizeof( short)
#define DEFAULT_RATE_SINGLE 4           // 64HZ   (4 entrées)
#define DEFAULT_RATE_DIFFERENTIAL 2     // 32HZ   (2 entrées)
short current_rate;
short saved_rate;

/* Gain interne *****************************
0 (gain = 2/3)  ± 6.144 volts 
1 (gain = 1)    ± 4.096 volts
2 (gain = 2)    ± 2.048 volts
4 (gain = 4)    ± 1.024 volts
8 (gain = 8)    ± 0.512 volts
16 (gain = 16)  ± 0.256 volts
********************************************/
//                           0       1       2       3       4       5
const float  gain_value[6]={6.144 , 4.096 , 2.048 , 1.024 , 0.512 , 0.256};  // Values to be displayed on LCD
const uint8_t gain_prog[6]={  0   ,   1   ,   2   ,   4   ,   8   ,  16  };  // Values to be programmed into ADS

#define gain_value_sizeTab sizeof(gain_value) / sizeof(float)
#define DEFAULT_GAIN 1    // 4.096
short current_gain=DEFAULT_GAIN;
short saved_gain=DEFAULT_GAIN;

uint16_t CAN_value_A0=0;              // Single Ended : only positive
uint16_t CAN_value_A1=0;              // Single Ended : only positive
uint16_t CAN_value_A2=0;              // Single Ended : only positive
uint16_t CAN_value_A3=0;              // Single Ended : only positive
int16_t  CAN_value_A0_A1=0;           // Differential : pos and neg
int16_t  CAN_value_A2_A3=0;           // Differential : pos and neg

volatile bool DO_READ_ADC    =false;           // Used inside interrupt (MUST be volatile)

// Variables traitement bouton rotatif et click
int clk_state_last      = LOW;            // Idle
int clk_state           = LOW;            // Idle
int button_state        = HIGH;           // Not pressed
unsigned long button_millis_save  = 0;
unsigned long button_millis_cur   = 0;

#define POS_SAMPLE_RATE   10            // Position sur LCD/colonne
#define POS_GAIN          10            // Position sur LCD/colonne

#define NORMAL_I2C_SPEED   400000L      // Normal Speed 400000KHz
#define DEFAULT_I2C_SPEED  600000L      // Push Freq to 600000KHz
#define FAST_I2C_SPEED     800000L      // Only used for CAN reads

//-------------------------------------------------------
// SETUP
//-------------------------------------------------------
void setup() {
  Wire.begin();

  // Inputs definitions
  pinMode (SingleEndedOrDifferentialPin,      INPUT_PULLUP);
  pinMode (encoderClk,                        INPUT);
  pinMode (encoderData,                       INPUT);
  pinMode (encoderSwitch,                     INPUT_PULLUP);
  pinMode (alertReadyPin,                     INPUT_PULLUP);
  
  // Initialisation de port série (usually 115200)
  Serial.begin(230400);

  // ADS1115
  ADS.begin();
  ADS.setGain(DEFAULT_GAIN);                  
  ADS.setMode(1);                             // 0=continuous/1=single
  
  // ALWAYS continous mode
  if (1){ // Activate Alert_Pin @ end of conversion in continous mode
    ADS.setMode(0);                           // 0=continous
    ADS.setComparatorThresholdLow(0x0000 );   // MSB=0   for alertPin
    ADS.setComparatorThresholdHigh(0x8000 );  // MSB=1   for alertPin
    ADS.setComparatorQueConvert(0);           // trigger after One sample
  }

  //--------------------------------------------------------------
  // Choose Conversion mode based on external switch
  // Either Single {A0,A1,A2,A3} or Differential {A0-A1,A2-A3} 
  //--------------------------------------------------------------
  if (digitalRead(SingleEndedOrDifferentialPin)==1) {
    // Differential mode
    current_rate=DEFAULT_RATE_DIFFERENTIAL;
    ADS.setDataRate(current_rate);      
    muxMode=DiffMode;
    muxStateforNextInterrupt=4;
    ADS.requestADC_Differential_2_3(); // Preselect  MUX 
  } else {
    // Single-Ended mode
    current_rate=DEFAULT_RATE_SINGLE;
    ADS.setDataRate(current_rate);      
    muxMode=SingleMode;
    muxStateforNextInterrupt=0;
    ADS.requestADC(0);                 // Preselect  MUX 
  }
  
  //--------------------------------------------------------------
  // ALERT/RDY pin indicate when conversion is ready, to pin 2
  //--------------------------------------------------------------
  attachInterrupt(digitalPinToInterrupt(2), int_read_adc, RISING );

  //--------------------------------------------------------------
  // initialisation de l'afficheur : Message d'accueil
  //--------------------------------------------------------------
  lcd.init();        // be aware that it reduce I2C freq 100KHz
  lcd.backlight();
  print_string_lcd("Hello, Half-God!",0,0);
  print_string_lcd("Sample Sensors",1,1);
  delay(2500);
  lcd.clear();

  //--------------------------------------------------------------
  // Prepare LCD Date display
  //--------------------------------------------------------------
  displayTime(2);         //0=lcd, 1=serial, 2=both
  lcd.setCursor(0, 1);
  if (muxMode==SingleMode){
    lcd.println("* SingleEnded  *");
  }else {
    lcd.println("* Differential *");
  }

  delay(3000);
  lcd.clear();
  //--------------------------------------------------------------
  // Prepare LCD working screen
  //--------------------------------------------------------------
  // First Line
  print_string_lcd(" Rate(Hz):      ",0,0);
  lcd.setCursor(POS_SAMPLE_RATE, 0);
  lcd.print(sample_rate[current_rate]);
  print_char_lcd('>',0,0);
  print_char_lcd('*',15,0);
  // Second line
  print_string_lcd(" Ampl(v) :      ",0,1);
  lcd.setCursor(POS_GAIN, 1);
  lcd.print(gain_value[current_gain],3);
  print_char_lcd('*',15,1);

  // END setup
  Wire.setClock(DEFAULT_I2C_SPEED);
  Serial.println("EndofSetup");
}



//-------------------------------------------------------
// LOOP
//-------------------------------------------------------
bool LCD_SELECT_SAMPLE_RATE=true;

void loop() {
  // Get Button state
  clk_state         = digitalRead(encoderClk);
  button_state      = digitalRead(encoderSwitch);
  button_millis_cur = millis();

  // check state and add debouncer
  if (button_state == LOW && (button_millis_cur - button_millis_save)>300) {
    button_millis_save=millis();
    LCD_SELECT_SAMPLE_RATE= not LCD_SELECT_SAMPLE_RATE; // bascule sur l'autre ligne
 
    if (LCD_SELECT_SAMPLE_RATE==false) {   // Switch to Gain line and validate current Rate line
      saved_rate=current_rate;
      ADS.setDataRate(sample_rate_prog[current_rate]);            // prog new data rate
      print_char_lcd('*', 15, 0);
      print_char_lcd_clear_other('>',0,1);
    }else{                          // Switch to Rate line and validate current Gain line
      saved_gain=current_gain;
      ADS.setGain(gain_prog[current_gain]);                       // prog new Gain                         
      print_char_lcd('*', 15, 1);
      print_char_lcd_clear_other('>',0,0);
    }
  
  } else {
    // Rotate button right or left
    // => ajoute un garde-fou de 7.5sec sinon on passe ici sans rotation physique : bug boutton ?
    if ((clk_state_last == LOW) && (clk_state == HIGH) && millis() > 7500) {
      // Read rotary Data pin to know which direction increase/decrease new speed
      // Act on correct LCD line
      if (LCD_SELECT_SAMPLE_RATE==true) {   // SAMPLE_RATE Line SELECTED
        if (digitalRead(encoderData) == LOW) {
          if (current_rate < (sample_rate_sizeTab-1))
            current_rate = current_rate + 1;
        } else {
            if (current_rate > 0)
              current_rate = current_rate - 1;
        }

        print_string_lcd("     ",POS_SAMPLE_RATE, 0);
        print_int_lcd(sample_rate[current_rate],POS_SAMPLE_RATE, 0);
        // Si on reselectionne la valeur déjà memorisée on affiche un caractere spécial en fin de ligne
        if (current_rate==saved_rate) {
          print_char_lcd('*',15,0);
        }else{
          print_char_lcd(' ',15,0);
        }   
      } else {  // GAIN Line SELECTED
        if (digitalRead(encoderData) == LOW) {
          if (current_gain < (gain_value_sizeTab-1))
            current_gain = current_gain + 1;
        } else {
            if (current_gain > 0)
              current_gain = current_gain - 1;
        }
        print_string_lcd("   ",POS_GAIN, 1);    // Clean lcd @ POS
        print_float_lcd(gain_value[current_gain],POS_GAIN, 1,3);
        // Si on reselectionne la valeur déjà memorisée on affiche un caractere spécial en fin de ligne
        if (current_gain==saved_gain) {
          print_char_lcd('*',15,1);
        }else{
          print_char_lcd(' ',15,1);
        }
      }
    } 
  
    clk_state_last = clk_state;       // Save CLK current state to avoid new trigger
  }
  
  //-------------------------------------------
  // After interrupt processing
  //-------------------------------------------
  if (DO_READ_ADC){
    Wire.setClock(FAST_I2C_SPEED);
    static unsigned long cv_start;
    //-----------------------------------------
    // muxMODE = 0 => Single Ended
    //         = 1 => Differential
    //-----------------------------------------
    if (muxMode==SingleMode) {
      // Mode Single Ended
      switch (muxStateforNextInterrupt){
        case 0:         // Channel A0
         //Serial.print(muxStateforNextInterrupt);
         CAN_value_A0=ADS.readADC(2);  // internally call to ADS.requestADC + read (~300us @ i2c_800KHz)
          muxStateforNextInterrupt=1;
          Wire.setClock(NORMAL_I2C_SPEED);
          epoch_RTC = RTC.getEpoch();       // Sample on A0 reading (every 4 samples)
          break;
        case 1:         // Channel A1
          //Serial.print(muxStateforNextInterrupt);
          CAN_value_A1=ADS.readADC(3);
          muxStateforNextInterrupt=2;
         break;
        case 2:         // Channel A2
          //Serial.print(muxStateforNextInterrupt);
          CAN_value_A2=ADS.readADC(0);
          muxStateforNextInterrupt=3;
          break;
        case 3:         // Channel A3
          //Serial.print(muxStateforNextInterrupt);
          CAN_value_A3=ADS.readADC(1);
          muxStateforNextInterrupt=0;
          break;
        default:
          Serial.println("ERROR: Channel incompatible avec le mode SINGLE-ENDED selectionné");
          break;
      }
    } else { 
      // Mode Single Ended
      switch (muxStateforNextInterrupt){
        case 4:         // Channel A0-A1 (Differentiel)
          //Serial.print(muxStateforNextInterrupt);
          CAN_value_A0_A1=ADS.readADC_Differential_0_1();
          muxStateforNextInterrupt=5;
          Wire.setClock(NORMAL_I2C_SPEED);
          epoch_RTC = RTC.getEpoch();     // Sample on A0-A1 reading (every 2 samples)
          break;  
        case 5:         // Channel A3    (Differentiel)
          //Serial.print(muxStateforNextInterrupt);
          CAN_value_A2_A3=ADS.readADC_Differential_2_3();
          muxStateforNextInterrupt=4;
          break;   
        default:
          Serial.println("ERROR: Channel incompatible avec le mode DIFFERENTIAL selectionné");
          break;
      }
    }


    // We have just read the A3 entry
    if (muxStateforNextInterrupt==0){ 
      Serial.print(epoch_RTC,DEC);Serial.print(" ");
      Serial.print(CAN_value_A0);Serial.print(" ");
      Serial.print(CAN_value_A1);Serial.print(" ");
      Serial.print(CAN_value_A2);Serial.print(" ");
      Serial.println(CAN_value_A3);
    }
    
    // We have just read the A2-A3 Diff entry
    if (muxStateforNextInterrupt==4){
      Serial.print(epoch_RTC);Serial.print(" ");
      Serial.print(CAN_value_A0_A1);Serial.print(" ");
      Serial.println(CAN_value_A2_A3);
    }

    Wire.setClock(DEFAULT_I2C_SPEED); // Restore normal i2c speed
    DO_READ_ADC=false;
  }
}

//-------------------------------------------------------
// Quelques fonctions pour le LCD
//-------------------------------------------------------
void print_char_lcd (char car, int col, int lg){
  lcd.setCursor(col, lg);
  lcd.print(car);
}

void print_char_lcd_clear_other (char car, int col, int lg){
  int other=1;
  if (lg==1) other=0;
  // print char      
  lcd.setCursor(col, lg);
  lcd.print(car);
  // clear other
  lcd.setCursor(col, other);
  lcd.print(' ');
}

void print_string_lcd (const String strTab, int col, int lg){
  lcd.setCursor(col, lg);
  lcd.print(strTab);
}

void print_float_lcd (const float num, int col, int lg, short decimal){
  lcd.setCursor(col, lg);
  lcd.print(num,decimal);
}

void print_int_lcd (const int num, int col, int lg){
  lcd.setCursor(col, lg);
  lcd.print(num);
}
//-------------------------------------------------------
// Interrupt function to start reading CAN value
//-------------------------------------------------------
void int_read_adc(){
    //Serial.println(millis());
    DO_READ_ADC =true;
}

//-------------------------------------------------------
// Convert binary coded decimal to normal decimal numbers
//-------------------------------------------------------
// Convert normal decimal numbers to binary coded decimal
byte decToBcd(byte val) {
  return ( (val / 10 * 16) + (val % 10) );
}
// Convert binary coded decimal to normal decimal numbers
byte bcdToDec(byte val) {
  return ( (val / 16 * 10) + (val % 16) );
}

//-------------------------------------------------------
// Affiche le temps sur le lien Serie ou LCD
//-------------------------------------------------------
void displayTime(byte displayToSerialOrLcd) {
  // retrieve data from DS3231 (takes ~1us)
  Wire.setClock(FAST_I2C_SPEED);
  annee  = RTC.getYear();
  mois   = RTC.getMonth();
  heure  = RTC.getHours();
  minute = RTC.getMinutes();
  seconde = RTC.getSeconds();
  numJourMois = RTC.getDay();
  Wire.setClock(NORMAL_I2C_SPEED);

  if (displayToSerialOrLcd==1 || displayToSerialOrLcd==2){
    Serial.print("Date is ");  
    Serial.print(annee, DEC);
    Serial.print("/");
    Serial.print(mois, DEC);
    Serial.print("/");
    Serial.print(numJourMois, DEC);
    Serial.print("__");
    Serial.print(heure, DEC);
    Serial.print(":");
    Serial.print(minute, DEC);
    Serial.print(":");
    Serial.println(seconde, DEC);
  }
  if (displayToSerialOrLcd==0 || displayToSerialOrLcd==2){
    lcd.setCursor(0,0);
    lcd.print(annee, DEC);
    lcd.print("/");
    lcd.print(mois, DEC);
    lcd.print("/");
    lcd.print(numJourMois, DEC);
    lcd.print(" ");
    lcd.print(heure, DEC);
    lcd.print(":");
    lcd.print(minute, DEC);
  }
}
