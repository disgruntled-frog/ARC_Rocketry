#include <Adafruit_GPS.h>

String lora_RX_address = "2"; 
//Each Transmitter/Reciever has its own Address(ID).
//RX is Reciever


HardwareSerial GPSSerial(2);
Adafruit_GPS GPS(&GPSSerial);

HardwareSerial lora(1);
/* LORA RYLR998 Wiring

  TXD to ESP D4
  RXD to ESP D5

*/

#define GPSECHO false // true = echo raw NMEA for debugging

uint32_t timer = 0;

// -------------------- Helper: convert NMEA to decimal degrees --------------------
float convertToDecimal(float nmeaValue) {
  int deg = int(nmeaValue / 100);        // degrees
  float min = nmeaValue - deg * 100;     // minutes
  return deg + min / 60.0;
}

void setup() {
  Serial.begin(115200); //Serial Baud Rate set to 115200
  Serial.println("ESP32 GPS Decimal Degrees Test");

  GPSSerial.begin(9600, SERIAL_8N1, 16, 17);// GPS TX -> ESP32 RX2,   GPS RX -> ESP32 TX2

  lora.begin(9600, SERIAL_8N1, 5, 4);



  // Configure GPS output and update rate
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // RMC + GGA sentences
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);    // 1 Hz
  GPS.sendCommand(PGCMD_ANTENNA);               // antenna status

  delay(1000);
  GPSSerial.println(PMTK_Q_RELEASE);           // request firmware version
}

void loop() {
  // Read GPS characters
  char c = GPS.read(); 
  if (GPSECHO && c) Serial.write(c);

  // Check for a full new NMEA sentence
  if (GPS.newNMEAreceived()) {
    GPS.parse(GPS.lastNMEA());
  }

  // Print GPS info every second
  if (millis() - timer > 1000) {
    timer = millis();

    Serial.print("Fix: "); Serial.println(GPS.fix); //Prints if fixed (aka if connected to SAT)
    
    float lat = convertToDecimal(GPS.latitude);
    if (GPS.lat == 'S') lat = -lat; //South = -lattitude

    float lon = convertToDecimal(GPS.longitude);
    if (GPS.lon == 'W') lon = -lon; //West = -longitude

    Serial.print("Latitude: "); Serial.println(lat, 6); //Prints to Serial for debugging
    Serial.print("Longitude: "); Serial.println(lon, 6);
    Serial.print("Altitude (m): "); Serial.println(GPS.altitude);
    Serial.print("Speed (knots): "); Serial.println(GPS.speed);
    Serial.print("Satellites: "); Serial.println((int)GPS.satellites);

    String msg = String(lat,6) + "," + String(lon,6); //String(lat/lon, # of decimal Points(rounds up) )

    double length =msg.length(); 
    // To transmit requires: Reciever Address, message length, and message.
    lora.println("AT+SEND=" + lora_RX_address + ","+ length +","+msg); 
    //AT+SEND is the transmit command for the RYLR
    Serial.println("msg sent");
    Serial.println("----------------------");    
  }
}
