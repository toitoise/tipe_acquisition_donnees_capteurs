#include "ADS1X15_ST.h"
#include "MegunoLink.h"

#define  alertReadyPin  2
 
int16_t difference_potentiel_A0_A1=0;
bool DO_READ_ADC=false;

ADS1115 ADS(0x48);  // ADS1115 physiquement défini à l'adresse 0x48, avec sa broche ADDR reliée à la masse
TimePlot MyPlot; //no channel selected

void setup() {
  // Initialisation de port série
  Serial.begin(115200);
  MyPlot.SetTitle("Hall sensor");
  MyPlot.SetXLabel("Time");
  MyPlot.SetYLabel("Amplitude");
  // Colours include: Red, Green, Blue, Yellow, Black, Magenta, Cyan, White
  // Markers include: Square, Diamond, Triangle, Circle, Cross, Plus, Star, DownwardTriangle, NoMarker
  // Line style : Solid, Dashed, Dotted, DashDot, DashDotDot
  MyPlot.SetSeriesProperties("Chan_0_1", Plot::Blue, Plot::Solid, 2, Plot::NoMarker);
  //MyPlot.SetSeriesProperties("Chan_2_3", Plot::Red , Plot::Solid, 2, Plot::Square);

  // ADS1115
  ADS.begin();
  ADS.setGain(4);     // +-1.024 
  ADS.setMode(0);     // 0=continous
  ADS.setDataRate(1); // 1 = 16 SPS
  ADS.setComparatorThresholdLow(0x0000 );  // MSB=0
  ADS.setComparatorThresholdHigh(0x8000 ); // MSB=1
  ADS.setComparatorQueConvert(0);   // trigger after One sample
  //ADS.setComparatorLatch(0);
  ADS.setWireClock(400000L);  // Clock I2C @ 0.4MHz
  //ADS.setWireClock(1000000L);  // Clock I2C @ 1MHz
  ADS.readADC(0);      // Et on fait une lecture à vide, pour envoyer tous ces paramètres

    // ALERT/RDY pin will indicate when conversion is ready, to Arduino pin 2
    pinMode(alertReadyPin,INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(2), read_adc, RISING );
    Serial.println("EndofSetup");


Serial.println(ADS.getMode(),HEX);
Serial.println(ADS.getDataRate(),HEX);
Serial.println(ADS.getComparatorQueConvert(),HEX);
}

void read_adc(){
    //Serial.println(micros());
    DO_READ_ADC =true;
}

//static unsigned long cv_start=0;
//static unsigned long cv_end=0;

void loop() {
  if (DO_READ_ADC){
    //Serial.println(millis());
    //cv_start=micros();
    difference_potentiel_A0_A1 = ADS.readADC_Differential_0_1(); // read diff lane 0-1
    //ADS.readADC(0);  // read lane ain0
    //cv_end=micros();
    //Serial.println(cv_end-cv_start);
    //Serial.println(cv_end);
    DO_READ_ADC=false;
  }
//  Serial.println(difference_potentiel_A0_A1); 
  MyPlot.SendData(F("Chan_0_1"),difference_potentiel_A0_A1); // Sinewave = series name, dY = data to plot
  //MyPlot.SendData(F("Chan_2_3"),dY2); // By wrapping strings in F("") we can save ram by storing strings in program memory

  //float tension_volts_A0_A1 = ADS.toVoltage(difference_potentiel_A0_A1);
  //Serial.print(tension_volts_A0_A1,6);    // On limite l'affichage à 3 chiffres après la virgule
  //Serial.println("aaaa");
  
}
