/*
    Author:      Sam Manley
    Date:        10-26-25
    Description: Testing XTSD SD Card. Writes and Reads a file, compares to ensure proper write.
                 Next step is reading an I2C sensor, logs data as CSV, and times write time.
    Notes:       - Use this write speed to help with timing of master loop
                 - There will be a seperate sketch for streaming telemetry data off SD card to computer through ESP32
                 - This Code is intended to be used with default SPI and CS/SS Pins
*/

#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "Wire.h"


// Functions Declarations
void listDir(fs::FS &fs, const char *dirname, uint8_t levels);
void createDir(fs::FS &fs, const char *path);
void removeDir(fs::FS &fs, const char *path);
void readFile(fs::FS &fs, const char *path);
void writeFile(fs::FS &fs, const char *path, const char *message);
void appendFile(fs::FS &fs, const char *path, const char *message);
void renameFile(fs::FS &fs, const char *path1, const char *path2);
void deleteFile(fs::FS &fs, const char *path);
void testFileIO(fs::FS &fs, const char *path);
void timeWrite(fs::FS &fs, const char *path, int num_write_blocks = 1);


void setup() {
  // Here we will:
  // - Read and Write a test file, and report the time and if its the same contents (BERT)
  Serial.begin(115200);
  Wire.begin();
  
  if (!SD.begin()){
    Serial.println("SD Card Failed to Begin!");
    while(1);
  } else {
    Serial.println("SD Card Initialized!");
  }
  listDir(SD,"/",4);            // List all files in SD card with a depth of 4
  createDir(SD,"/Telemetry");   // Creating Telemetry Dir (kinda unnecessary since we only are using it for this but could be useful)
  timeWrite(SD,"/TEST.txt");
  deleteFile(SD,"/TEST.txt");

}

void loop() {


}

/*******************************************************************************/
/******************* Function Definitions **************************************/ 
/*******************************************************************************/

void listDir(fs::FS &fs, const char *dirname, uint8_t levels) {
  Serial.printf("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if (!root) {
    Serial.println("Failed to open directory");
    return;
  }
  if (!root.isDirectory()) {
    Serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if (levels) {
        listDir(fs, file.path(), levels - 1);
      }
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("  SIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}

void createDir(fs::FS &fs, const char *path) {
  Serial.printf("Creating Dir: %s\n", path);
  if (fs.mkdir(path)) {
    Serial.println("Dir created");
  } else {
    Serial.println("mkdir failed");
  }
}

void removeDir(fs::FS &fs, const char *path) {
  Serial.printf("Removing Dir: %s\n", path);
  if (fs.rmdir(path)) {
    Serial.println("Dir removed");
  } else {
    Serial.println("rmdir failed");
  }
}

void readFile(fs::FS &fs, const char *path) {
  Serial.printf("Reading file: %s\n", path);

  File file = fs.open(path);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }

  Serial.print("Read from file: ");
  while (file.available()) {
    Serial.write(file.read());
  }
  file.close();
}

void writeFile(fs::FS &fs, const char *path, const char *message) {
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

void appendFile(fs::FS &fs, const char *path, const char *message) {
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (file.print(message)) {
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}

void renameFile(fs::FS &fs, const char *path1, const char *path2) {
  Serial.printf("Renaming file %s to %s\n", path1, path2);
  if (fs.rename(path1, path2)) {
    Serial.println("File renamed");
  } else {
    Serial.println("Rename failed");
  }
}

void deleteFile(fs::FS &fs, const char *path) {
  Serial.printf("Deleting file: %s\n", path);
  if (fs.remove(path)) {
    Serial.println("File deleted");
  } else {
    Serial.println("Delete failed");
  }
}
void testFileIO(fs::FS &fs, const char *path) {
  // Give Test File Path To Read From

  File file = fs.open(path);
  static uint8_t buf[512];
  size_t len = 0;
  uint32_t start = millis();
  uint32_t end = start;
  if (file) {
    len = file.size();
    size_t flen = len;
    start = millis();
    while (len) {
      size_t toRead = len;
      if (toRead > 512) {
        toRead = 512;
      }
      file.read(buf, toRead);
      len -= toRead;
    }
    end = millis() - start;
    Serial.printf("%u bytes read for %lu ms\n", flen, end);
    file.close();
  } else {
    Serial.println("Failed to open file for reading");
  }

  file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }

  size_t i;
  start = millis();
  //for (i = 0; i < 2048; i++) {
  file.write(buf, 512);
  //}
  end = millis() - start;
  Serial.printf("%u bytes written for %lu ms\n", 512, end);
  file.close();
}

void timeWrite(fs::FS &fs, const char *path, int num_write_blocks){
  // For such tight timing constraints, and small testing size, I am using micros() for this


  static uint8_t buf[512];
  uint32_t start = micros();
  uint32_t end = start;

  File file = fs.open(path, FILE_WRITE);  // Open Test File
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }

  if (num_write_blocks == 1){
    // Isolating this so we can null the time it takes to set up for loop if there is any
    // This will run then the next will, the time difference will be the time it takes for the for loop
    start = micros();
    file.write(buf, 512);
    end = micros() - start;
    Serial.printf("%u WRITTEN in %lu us\n", num_write_blocks*512, end);
  }

  int i;
  start = micros();
  for (i=0; i<num_write_blocks; i++){
    file.write(buf, 512);
  }
  end = micros() - start;

  Serial.printf("%u written in %lu us\n", num_write_blocks*512, end);

  
}



