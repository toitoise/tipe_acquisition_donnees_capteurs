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
#include "projet_definitions.h"

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

short muxMode=DiffMode;

//-------------------------------------------------------
// LCD 20x4 Declaration (i2c pins connexion)
//-------------------------------------------------------
LiquidCrystal_I2C lcd(0x27, 20, 4);

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

short current_gain=DEFAULT_GAIN_SINGLE;
short saved_gain=DEFAULT_GAIN_SINGLE;

uint16_t CAN_value_A0=0;              // Single Ended : only positive
uint16_t CAN_value_A1=0;              // Single Ended : only positive
uint16_t CAN_value_A2=0;              // Single Ended : only positive
uint16_t CAN_value_A3=0;              // Single Ended : only positive
int16_t  CAN_value_A0_A1=0;           // Differential : pos and neg
int16_t  CAN_value_A2_A3=0;           // Differential : pos and neg

static String value_to_display_str;


volatile bool DO_READ_ADC    =false;           // Used inside interrupt (MUST be volatile)

// Variables traitement bouton rotatif et click
volatile static int clk_state_last      = HIGH;            // Idle
volatile static int clk_state           = HIGH;            // Idle
int button_state        = HIGH;           // Not pressed
unsigned long button_millis_save  = 0;
unsigned long button_millis_cur   = 0;

// Time updated @ each main loop iteration
unsigned long current_time = 0;   
unsigned long previous_time = 0;
unsigned long last_value_display_time = 0;

#include "project_own_functions.h"

//-------------------------------------------------------
//-------------------------------------------------------
//                         SETUP
//-------------------------------------------------------
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
  //Serial.begin(230400);
  Serial.begin(115200);

  // ADS1115
  ADS.begin();
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
    current_gain=DEFAULT_GAIN_DIFF;
    ADS.setGain(current_gain);                  
    ADS.setDataRate(current_rate);      
    muxMode=DiffMode;
    muxStateforNextInterrupt=4;
    ADS.requestADC_Differential_2_3(); // Preselect  MUX 
  } else {
    // Single-Ended mode
    current_rate=DEFAULT_RATE_SINGLE;
    ADS.setDataRate(current_rate);      
    current_gain=DEFAULT_GAIN_SINGLE;
    ADS.setGain(current_gain);  
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
  print_string_lcd("Hello, Half-God!"    ,POS_HELLO_COL, POS_HELLO_LIG);
  print_string_lcd("Module d'Acquisition",POS_MSG_COL,POS_MSG_LIG);
  delay(2500);
  lcd.clear();

  //--------------------------------------------------------------
  // Prepare LCD Date display
  //--------------------------------------------------------------
  displayTime(2);         //0=lcd, 1=serial, 2=both
  lcd.setCursor(POS_MODE_COL, POS_MODE_LIG);
  if (muxMode==SingleMode){
    lcd.println("== Single Ended == ");
  }else {
    lcd.println("== Differential == ");
  }

  delay(4000);
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
  
  // Display une lettre : S=Single D=Differential
  lcd.setCursor(19, 0);
  if (muxMode==SingleMode){
    lcd.print("S");
  }else{
    lcd.print("D");
  }

  // Third line (Chan A & B)
  lcd.setCursor(0,2);
  lcd.print("A:");
  lcd.setCursor(10,2);
  lcd.print("B:");
  
  // Fourth line (Channel C & D)
  lcd.setCursor(0,3);
  lcd.print("C:");
  lcd.setCursor(10,3);
  lcd.print("D:");

  //print_char_lcd('>',0,0);
  //print_char_lcd('*',15,0);

  // END setup
  Wire.setClock(DEFAULT_I2C_SPEED);
  //Serial.println("EndofSetup");
}

//-------------------------------------------------------
//-------------------------------------------------------
//                        LOOP
//-------------------------------------------------------
//-------------------------------------------------------
bool LCD_SELECT_SAMPLE_RATE=true;

void loop() {
  // Get Button state
  clk_state         = digitalRead(encoderClk);
  button_state      = digitalRead(encoderSwitch);
  current_time      = millis();
  button_millis_cur = current_time;

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
    //debug: Serial.println(clk_state_last);    Serial.println(clk_state);
    // Check Rotation change (rising edge)
    if ((clk_state_last == LOW) && (clk_state == HIGH) /* && millis() > 10000 */ ) {
      // Read rotary Data pin to know which direction increase/decrease new speed
      if (LCD_SELECT_SAMPLE_RATE==true) {   // SAMPLE_RATE Line SELECTED
        if (digitalRead(encoderData) == LOW) {
          if (current_rate < (sample_rate_sizeTab-1))
            current_rate = current_rate + 1;                  // Increase FREQ
        } else {
            if (current_rate > 0)
              current_rate = current_rate - 1;                // Decrease FREQ
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
           if (current_gain > 0)
            current_gain = current_gain - 1;                    // Increase GAIN
        } else {
           if (current_gain < (gain_value_sizeTab-1))
              current_gain = current_gain + 1;                  // Decrease GAIN
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
    value_to_display_str="        ";
    static bool display=false;
    //-----------------------------------------
    // muxMODE = 0 => Single Ended
    //         = 1 => Differential
    // Each channel processing take about 5ms (Reading CAN value+Display Serial and LCD), so it allows up to 128HZ datarate
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
          Serial.print(epoch_RTC,DEC);Serial.print(" ");
          Serial.print(CAN_value_A0);Serial.print(" ");
          // Only display every 2secs
          if (current_time - last_value_display_time > DISPLAY_VALUE_TIME ) {
            value_to_display_str=String(CAN_value_A0);
            format_string(value_to_display_str);
            print_string_lcd (value_to_display_str,2,2);
            last_value_display_time=current_time;
            display=true;
          }
          break;
        case 1:         // Channel A1
          //Serial.print(muxStateforNextInterrupt);
          CAN_value_A1=ADS.readADC(3);
          muxStateforNextInterrupt=2;
          Serial.print(CAN_value_A1);Serial.print(" ");
          if (display==true ) {
            value_to_display_str=String(CAN_value_A1);
            format_string(value_to_display_str);
            print_string_lcd (value_to_display_str,12,2);
          }
        break;
        case 2:         // Channel A2
         Serial.println("AA");
         Serial.println(millis());
          //Serial.print(muxStateforNextInterrupt);
          CAN_value_A2=ADS.readADC(0);
          muxStateforNextInterrupt=3;
          Serial.print(CAN_value_A1);Serial.print(" ");
         if (display==true ) {
          Serial.println("BB");
           value_to_display_str=String(CAN_value_A2);
            format_string(value_to_display_str);
            print_string_lcd (value_to_display_str,2,3);
          }
          Serial.println(millis());
          break;
        case 3:         // Channel A3
          //Serial.print(muxStateforNextInterrupt);
          CAN_value_A3=ADS.readADC(1);
          muxStateforNextInterrupt=0;
          Serial.println(CAN_value_A3);
          if (display==true ) {
            value_to_display_str=String(CAN_value_A3);
            format_string(value_to_display_str);
            print_string_lcd (value_to_display_str,12,3);
            display=false;
          }
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
          Serial.print(epoch_RTC);Serial.print(" ");
          Serial.print(CAN_value_A0_A1);Serial.print(" ");
          if (current_time - last_value_display_time > 2000 ) {
            value_to_display_str=String(CAN_value_A0_A1);
            format_string(value_to_display_str);
            print_string_lcd (value_to_display_str,2,2);
            last_value_display_time=current_time;
            display=true;
          }
          break;  
        case 5:         // Channel A3    (Differentiel)
          //Serial.print(muxStateforNextInterrupt);
          CAN_value_A2_A3=ADS.readADC_Differential_2_3();
          muxStateforNextInterrupt=4;
          Serial.println(CAN_value_A2_A3);
          if (display==true ) {
            value_to_display_str=String(CAN_value_A2_A3);
            format_string(value_to_display_str);
            print_string_lcd (value_to_display_str,2,3);
            display=false;
          }
          break;   
        default:
          Serial.println("ERROR: Channel incompatible avec le mode DIFFERENTIAL selectionné");
          break;
      }
    }

// TOO SLOW, consume too much time to print everything @ same time
if(0) {
    // We have just read the A3 entry
    if (muxStateforNextInterrupt==0){ 
      //Serial.print(epoch_RTC,DEC);Serial.print(" ");
      //Serial.print(CAN_value_A0);Serial.print(" ");
      //Serial.print(CAN_value_A1);Serial.print(" ");
      //Serial.print(CAN_value_A2);Serial.print(" ");
      //Serial.println(CAN_value_A3);
    }
    
    // We have just read the A2-A3 Diff entry
    if (muxStateforNextInterrupt==4){
      //Serial.print(epoch_RTC);Serial.print(" ");
      //Serial.print(CAN_value_A0_A1);Serial.print(" ");
      //Serial.println(CAN_value_A2_A3);
    }
}

    // Display every 2 sec on LCD
    if (current_time - last_value_display_time > 2000 ) {
        if (muxMode==SingleMode) {
//          Serial.println("first");
//          Serial.println(millis());
//          print_string_lcd ("       ",2,2);
//          print_string_lcd ("       ",12,2);
//          print_string_lcd ("       ",2,3);
//          print_string_lcd ("       ",12,3);       
//          print_int_lcd (CAN_value_A0,2,2);
//          print_int_lcd (CAN_value_A1,12,2);
//          print_int_lcd (CAN_value_A2,2,3);
//          print_int_lcd (CAN_value_A3,12,3);
//          Serial.println(millis());
        } else {
//          print_string_lcd ("       ",2,2);
//          print_string_lcd ("       ",2,3);
//          print_int_lcd (CAN_value_A0_A1,2,2);
//          print_int_lcd (CAN_value_A2_A3,2,3);
        }
//        last_value_display_time=current_time;
    }
    Wire.setClock(DEFAULT_I2C_SPEED); // Restore normal i2c speed

    DO_READ_ADC=false;
  }
  previous_time=current_time;
} // END LOOP
