#include <SoftwareSerial.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

String incomingString;

LiquidCrystal_I2C lcd(0x27,20,4);

SoftwareSerial lora(2,3);

void setup()
{
  Serial.begin(9600);
  lora.begin(9600);
  lora.setTimeout(250);
  lcd.init();
  lcd.setCursor(3,0); 
  lcd.backlight(); 
}

void loop()
{
  if (lora.available()) {
    
    incomingString = lora.readString();
    Serial.println(incomingString);

    char dataArray[30]; 
    incomingString.toCharArray(dataArray,30);
    char* data = strtok(dataArray, ",");
    data = strtok(NULL, ",");
    char*latitude = strtok(NULL, ",");
    Serial.print("Lat: ");Serial.println(latitude);
    lcd.setCursor(1,0);
    lcd.print("LAT: ");lcd.print(latitude);

    char*longitude = strtok(NULL, ",");
    Serial.print("Long: ");Serial.println(longitude);
    lcd.setCursor(1,1);
    lcd.print("LON: ");lcd.print(longitude);
  }
}
