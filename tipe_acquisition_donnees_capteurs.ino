/*
 * 
 * Librairie du CAN l'ADS1115 :
 *      https://github.com/RobTillaart/ADS1X15
 *      https://passionelectronique.fr/tutorial-ads1115/
 *      https://cdn-learn.adafruit.com/downloads/pdf/adafruit-4-channel-adc-breakouts.pdf
 * Librairie DS3231
 * 
 */


#include "ADS1X15_ST.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
//-------------------------------------------------------
// ADS115 PINS
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

/* DateRate *********************************
DR=0 : 8 SPS
DR=1 : 16 SPS
DR=2 : 32 SPS
DR=3 : 64 SPS
********************************************/
//                           0   1   2   3  
const short sample_rate[4]      ={ 8,  16, 32, 64};
const short sample_rate_prog[4] ={ 0,   1,  2,  4};
#define sample_rate_sizeTab sizeof(sample_rate) / sizeof( short)
#define DEFAULT_RATE 1    // 16HZ 
short current_rate=DEFAULT_RATE;
short saved_rate=DEFAULT_RATE;

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
#define DEFAULT_GAIN 3    // 16HZ 
short current_gain=DEFAULT_GAIN;
short saved_gain=DEFAULT_GAIN;

int16_t potentiel_A0=0;
int16_t potentiel_A1=0;
int16_t potentiel_A2=0;
int16_t potentiel_A3=0;
int16_t difference_potentiel_A0_A1=0;
int16_t difference_potentiel_A2_A3=0;
bool DO_READ_ADC    =false;

// Variables traitement bouton rotatif et click
int clk_state_last      = LOW;            // Idle
int clk_state           = LOW;            // Idle
int button_state        = HIGH;           // Not pressed
unsigned long button_millis_save  = 0;
unsigned long button_millis_cur   = 0;

#define POS_SAMPLE_RATE   10            // Position sur LCD/colonne
#define POS_GAIN          10            // Position sur LCD/colonne
#define DS3231_I2C_ADDRESS 0x68

//-------------------------------------------------------
// SETUP
//-------------------------------------------------------
void setup() {
  Wire.begin();

  // Inputs definitions
  pinMode (encoderClk,      INPUT);
  pinMode (encoderData,     INPUT);
  pinMode (encoderSwitch,   INPUT_PULLUP);
  pinMode (alertReadyPin,   INPUT_PULLUP);
  
  // Initialisation de port série
  Serial.begin(230400);

  // ADS1115
  ADS.begin();
  ADS.setWireClock(400000L);                  // Clock I2C @ 0.4MHz
  ADS.setGain(DEFAULT_GAIN);                  // +-1.024 
//  ADS.setMode(0);                           // 0=continous
//  ADS.setComparatorThresholdLow(0x0000 );   // MSB=0
//  ADS.setComparatorThresholdHigh(0x8000 );  // MSB=1
//  ADS.setComparatorQueConvert(0);           // trigger after One sample
  ADS.setMode(1);                             // 0=continous
  ADS.setDataRate(DEFAULT_RATE);              // 1 = 16 SPS

  ADS.readADC(0);                             // Et on fait une lecture à vide, pour envoyer tous ces paramètres

  // ALERT/RDY pin will indicate when conversion is ready, to Arduino pin 2
  attachInterrupt(digitalPinToInterrupt(2), read_adc, RISING );

  // initialisation de l'afficheur / Message d'accueil
  lcd.init(); 
  lcd.backlight();
  print_sentence_lcd("Hello, Half-God!",0,0);
  print_sentence_lcd("Sample Sensors",1,1);
  delay(5000);
  lcd.clear();
  // Prepare LCD screen
  // First Line
  print_sentence_lcd(" Rate(Hz):      ",0,0);
  lcd.setCursor(POS_SAMPLE_RATE, 0);
  lcd.print(sample_rate[current_rate]);
  print_char_lcd('>',0,0);
  print_char_lcd('*',15,0);

  // Second line
  print_sentence_lcd(" Ampl(v) :      ",0,1);
  lcd.setCursor(POS_GAIN, 1);
  lcd.print(gain_value[current_gain],3);
  print_char_lcd('*',15,1);

  // END setup
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

    Serial.print("SELECT:");
    Serial.println(LCD_SELECT_SAMPLE_RATE);

    
      if (LCD_SELECT_SAMPLE_RATE==false) {   // Switch to Gain line and validate current Rate line
        saved_rate=current_rate;
        lcd.setCursor(15, 0);
        lcd.print("*");
        print_char_lcd_clear_other('>',0,1);

        Serial.println("A");
        ADS.setDataRate(sample_rate_prog[current_rate]);            // prog new data rate

      }else{                          // Switch to Rate line and validate current Gain line

        saved_gain=current_gain;
        ADS.setGain(gain_prog[current_gain]);     // prog new Gain                         
        lcd.setCursor(15, 1);
        lcd.print("*");
        print_char_lcd_clear_other('>',0,0);
        Serial.println("B");
      }
  
  } else {
    // Rotate button right or left
    // => ajoute un garde-fou de 7.5sec sinon on passe ici sans rotation physique : bug boutton ?
    if ((clk_state_last == LOW) && (clk_state == HIGH) && millis() > 7500) {
      // Read rotary Data pin to know which direction increase/decrease new speed
      // Act on correct LCD line
      if (LCD_SELECT_SAMPLE_RATE==true) {   // SAMPLE_RATE SELECTED
        if (digitalRead(encoderData) == LOW) {
          if (current_rate < (sample_rate_sizeTab-1))
            current_rate = current_rate + 1;
        } else {
            if (current_rate > 0)
              current_rate = current_rate - 1;
        }

        lcd.setCursor(POS_SAMPLE_RATE, 0);
        lcd.print("     ");
        lcd.setCursor(POS_SAMPLE_RATE, 0);
        lcd.print(sample_rate[current_rate]);
        lcd.setCursor(15, 0);
        if (current_rate==saved_rate) {
          lcd.print("*");
        }else{
          lcd.print(" ");
        }
        
        
      } else {  // GAIN SELECTED
        if (digitalRead(encoderData) == LOW) {
          if (current_gain < (gain_value_sizeTab-1))
            current_gain = current_gain + 1;
        } else {
            if (current_gain > 0)
              current_gain = current_gain - 1;
        }
        lcd.setCursor(POS_GAIN, 1);
        lcd.print("  ");
        lcd.setCursor(POS_GAIN, 1);
        lcd.print(gain_value[current_gain],3);
        lcd.setCursor(15, 1);
        if (current_gain==saved_gain) {
          //lcd.print("*");
          print_char_lcd('*',15,1);
        }else{
          //lcd.print(" ");
          print_char_lcd(' ',15,1);
        }
      }
      // Display new speed based on rotary button direction
//      Serial.print ("B/rate,gain:" );  //debug
//      Serial.println (current_rate );  //debug
//      Serial.println (current_gain );  //debug

    } 

    // Save CLK current state to avoid new trigger
    clk_state_last = clk_state;
  }
  
  
//  if (DO_READ_ADC){
    //Serial.println(millis());
    //cv_start=micros();
 
    potentiel_A0=ADS.readADC(0);
    potentiel_A1=ADS.readADC(1);
    potentiel_A2=ADS.readADC(2);
    potentiel_A3=ADS.readADC(3);
Serial.println(micros());
    Serial.print("A0:");Serial.println(potentiel_A0);//Serial.println(ADS.toVoltage(potentiel_A0));
Serial.println(micros());
    Serial.print("A1:");Serial.println(potentiel_A1);//Serial.println(ADS.toVoltage(potentiel_A1));
Serial.println(micros());
    Serial.print("A2:");Serial.println(potentiel_A2);
    Serial.print("A3:");Serial.println(potentiel_A3);
//    Serial.print("A2:");Serial.println(ADS.toVoltage(potentiel_A2));
//    Serial.print("A3:");Serial.println(ADS.toVoltage(potentiel_A3));
        
    difference_potentiel_A0_A1 = ADS.readADC_Differential_0_1(); // read diff lane 0-1
    //ADS.readADC(0);  // read lane ain0
    //cv_end=micros();
    //Serial.println(cv_end-cv_start);
    //Serial.println(cv_end);
    DO_READ_ADC=false;
   //  Serial.println(difference_potentiel_A0_A1); 
  //float tension_volts_A0_A1 = ADS.toVoltage(difference_potentiel_A0_A1);
  //Serial.println(tension_volts_A0_A1,6);    // On limite l'affichage à 3 chiffres après la virgule
//  }

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

void print_sentence_lcd (const String carTab, int col, int lg){
  lcd.setCursor(col, lg);
  lcd.print(carTab);
}

//-------------------------------------------------------
// Interrupt function to start reading CAN value
//-------------------------------------------------------
void read_adc(){
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
// Lecture date RTC
//-------------------------------------------------------
void readDS3231time(byte *second,
                    byte *minute,
                    byte *hour,
                    byte *dayOfWeek,
                    byte *dayOfMonth,
                    byte *month,
                    byte *year) {
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0); // set DS3231 register pointer to 00h
  Wire.endTransmission();
  Wire.requestFrom(DS3231_I2C_ADDRESS, 7);
  // request seven bytes of data from DS3231 starting from register 00h
  *second     = bcdToDec(Wire.read() & 0x7f);
  *minute     = bcdToDec(Wire.read());
  *hour       = bcdToDec(Wire.read() & 0x3f);
  *dayOfWeek  = bcdToDec(Wire.read());
  *dayOfMonth = bcdToDec(Wire.read());
  *month      = bcdToDec(Wire.read());
  *year       = bcdToDec(Wire.read());
}
