
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
    lcd.setCursor(POS_DATE_COL, POS_DATE_LIG);
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

// Pour formater une string a la taille constante de 8 caracteres pour le display des valeurs CAN sur LCD
// Ca permet de ne pas avoir des carateres rémanents si on passe d'une grosse valeur a une petite.
void format_string(String & str){
  switch (str.length()) {
  case 1 :
    str+="       ";
    return str;
  case 2 :
    str+="      ";
    return str;
  case 3 :
    str+="     ";
    return str;
  case 4 :
    str+="    ";
    return str;
  case 5 :
    str+="   ";
    return str;
  case 6 :
    str+="  ";
    return str;
  case 7 :
    str+=" ";
    return str;
  default:
    return str;
  }

}
