

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
  // retrieve data from DS3231 
  annee  = RTC.getYear();
  mois   = RTC.getMonth();
  heure  = RTC.getHours();
  minute = RTC.getMinutes();
  seconde = RTC.getSeconds();
  numJourMois = RTC.getDay();

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
