#include <SoftwareSerial.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <string.h>

String incomingString; //Global String 

LiquidCrystal_I2C lcd(0x27,20,4); 

SoftwareSerial lora(2,3); 
/* LORA RYLR998 Wiring

  TXD to Nano 2
  RXD to ESP 3

*/

void setup()
{
  Serial.begin(9600);
  lora.begin(9600);
  lora.setTimeout(250);
  lcd.init();
  lcd.setCursor(3,0); 
  lcd.backlight(); 
}

void loop(){ 
  char*latitude="";
  char*longitude="";

  if (lora.available()) {
    
    incomingString = lora.readString(); //reads transmitted string
    Serial.print(incomingString);

    char dataArray[50]; 
    incomingString.toCharArray(dataArray,incomingString.length());
    char* data = strtok(dataArray, ","); // skips Fix (connection); why didn't I use this earlier :(
    data = strtok(NULL, ","); //skips length

    char*connection = strtok(NULL, ","); //reads if connection (that YG or NG)
    Serial.print("Connection: ");Serial.println(connection);

    if (strcmp(connection,"NG")==0){ //compares string and return 0 if equal (C String Function)
      lcd.setCursor(15,0);
      lcd.print("N");
      lcd.setCursor(15,1); //NG = No GPS
      lcd.print("G");
      latitude = strtok(NULL, ",");
      longitude = strtok(NULL, ",");
      Serial.println("NG, Skipped Printing");
      Serial.println("----------------------");
    }
    else{
      lcd.setCursor(15,0);
      lcd.print("Y");
      lcd.setCursor(15,1);
      lcd.print("G");
      latitude = strtok(NULL, ","); //reads latitude (after YG/NG)
      Serial.print("Lat: ");Serial.println(latitude);
      lcd.setCursor(0,0);
      lcd.print("LAT ");lcd.print(latitude); //Displays Lat

      longitude = strtok(NULL, ",");//reads longitude (after latitude)
      Serial.print("Long: ");Serial.println(longitude);
      lcd.setCursor(0,1);
      lcd.print("LON ");lcd.print(longitude);//Displays Long
      Serial.println("----------------------");
    }
  }
  
}
