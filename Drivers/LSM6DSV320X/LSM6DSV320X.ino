#include <Wire.h>

// Defining Address of Sensor
#define IMU_ADDR                0x6A  //0x6B if SDA tied high

// Defining Address of Registers
#define IMU_WHOAMI_REG          0x0F

#define IMU_CTRL1               0x10     // ACC:  Op mode, ODR
#define IMU_CTRL2               0x11     // GYRO: Op mode, ODR
#define IMU_CTRL3               0x12     //  N/A  for now
#define IMU_CTRL4               0x13     //  N/A  for now
#define IMU_CTRL5               0x14     //  N/A  for now
#define IMU_CTRL6               0x15     // GYRO: LPF1, FS Sel
#define IMU_CTRL7               0x16     // GYRO: Enbl LPF`
#define IMU_CTRL8               0x17     // ACC:  LPF2, FS Sel
#define IMU_CTRL9               0x18     // ACC:  Configuring Filters (Leave for Now)
#define IMU_CTRL10              0x19     //  IMU: Enbl dbg for Embedded functions and Self Testing (Leaving for now)

#define IMU_OUTX_L_G            0x22
#define IMU_OUTX_H_G            0x23
#define IMU_OUTY_L_G            0x24
#define IMU_OUTY_H_G            0x25
#define IMU_OUTZ_L_G            0x26
#define IMU_OUTZ_H_G            0x27
#define IMU_OUTX_L_A            0x28
#define IMU_OUTX_H_A            0x29
#define IMU_OUTY_L_A            0x2A
#define IMU_OUTY_H_A            0x2B
#define IMU_OUTZ_L_A            0x2C
#define IMU_OUTZ_H_A            0x2D

#define IMU_UI_OUTX_L_A_OIS_HG  0x34
#define IMU_UI_OUTX_H_A_OIS_HG  0x35
#define IMU_UI_OUTY_L_A_OIS_HG  0x36
#define IMU_UI_OUTY_H_A_OIS_HG  0x37
#define IMU_UI_OUTZ_L_A_OIS_HG  0x38
#define IMU_UI_OUTZ_H_A_OIS_HG  0x39

#define IMU_CTRL2_XL_HG         0x4D     // HG ACC: HG self-test sel (leave default for now)
#define IMU_CTRL1_XL_HG         0x4E     // HG ACC: Enbl output (reg: 34-39), hg user offset (out reg), ODR, FS Sel


// Defining Values
#define IMU_WHOAMI_VAL          0x72


// To test a single register, ie: WHOAMI
//const byte size = 1;
//byte value[size] = {};


////////
const int axis = 9;     // gyro + acc + hg_acc
//const int hg_axis = 3;  // hg acc


// This will recieve register data (non concatinated)
const byte imu_size = axis*2;
byte imu_reg_data[axis*2] = {};

// This will recieve register data from hg (non concatinated)
//const byte imu_size = hg_axis*2;
//byte hg_reg_data[hg_axis*2] = {};

// This will be the parsed data
int16_t raw_imu[axis] = {};
double imu[axis] = {};
////////







// Function Declaration
byte readRegisters(byte _addr, byte _reg, byte* _value,byte _size = 1);
byte writeRegister(byte _addr, byte _reg, byte _value);
void config_imu();
void read_imu(byte* _value, int16_t* _raw_data, double* _data);


void setup() {
  
  Serial.begin(9600);
  Wire.begin();

  Serial.println("Init Sensors Now...");

  config_imu();

  Serial.println("Configured, Starting Main Loop...");

}

void loop() {
  
    read_imu(imu_reg_data, raw_imu, imu);
  
    // Human Readable
    
    /*
    Serial.print("Gyro Data: ");
    Serial.print(imu[0]);
    Serial.print(", ");
    Serial.print(imu[1]);
    Serial.print(", ");
    Serial.print(imu[2]);
    Serial.print(" | Imu Data: ");
    Serial.print(imu[3]);
    Serial.print(", ");
    Serial.print(imu[4]);
    Serial.print(", ");
    Serial.print(imu[5]);
    Serial.print(" | HG Imu Data: ");
    Serial.print(imu[6]);
    Serial.print(", ");
    Serial.print(imu[7]);
    Serial.print(", ");
    Serial.println(imu[8]);
    */

    // Serial Plotter Friendly 
    Serial.print(imu[0]);
    Serial.print(", ");
    Serial.print(imu[1]);
    Serial.print(", ");
    Serial.print(imu[2]);
    Serial.print(", ");
    Serial.print(imu[3]);
    Serial.print(", ");
    Serial.print(imu[4]);
    Serial.print(", ");
    Serial.print(imu[5]);
    Serial.print(", ");
    Serial.print(imu[6]);
    Serial.print(", ");
    Serial.print(imu[7]);
    Serial.print(", ");
    Serial.println(imu[8]);


    delay(100);

}


byte readRegisters(byte _addr, byte _reg, byte* _value,byte _size){
  // byte _addr: address of the I2C sensor you wish to communicate with
  // byte _reg: address of the register within said I2C sensor you wish to read from
  // byte* _value: pointer to the array you will store the value(s) read
  // byte _size: size of the value array
  // byte returnval: error code, 0 is normal operation

  Wire.beginTransmission(_addr);
  Wire.write(_reg);
  Wire.requestFrom(_addr, _size);
  while(Wire.available()){
    *_value = Wire.read();
    _value++;
  }
  return Wire.endTransmission();
}


byte writeRegister(byte _addr, byte _reg, byte _value){
  // byte _addr: address of the I2C sensor you wish to communicate with
  // byte _reg: address of the register within said I2C sensor you wish to write to
  // byte _value: byte you wish to write to said register

  Wire.beginTransmission(_addr);
  Wire.write(_reg);
  Wire.write(_value);
  return Wire.endTransmission();
}

void config_imu(){
  // This will be default configuration for everyone, future plans to add customization
  byte results = writeRegister(IMU_ADDR, IMU_CTRL1, B00001000);   // High-performance, ODR 480 Hz
  results = writeRegister(IMU_ADDR, IMU_CTRL2, B00001000);        // High-performance, ODR 480 Hz
  results = writeRegister(IMU_ADDR, IMU_CTRL6, B00001101);        // LPF1: 175, FS +/- 4000 dps
  results = writeRegister(IMU_ADDR, IMU_CTRL8, B00000011);        // BW: ODR/4, FS +/- 16 g
  results = writeRegister(IMU_ADDR, IMU_CTRL9, B00001000);        // Selecting output from ACC LPF2. Come back to this one to tune fitlering
  results = writeRegister(IMU_ADDR, IMU_CTRL1_XL_HG, B11011000);  // HG ODR 480 Hz, +/- 32 g, Enble Output, Enble user offset on out reg
}

void read_imu(byte* _value, int16_t* _raw_data, double* _data){
  // This will read the gyro -> low g acc -> high g acc


  // *** HARD CODED LENGTHS ***
  readRegisters(IMU_ADDR, IMU_OUTX_L_G, _value, 12);              // _value == imu_reg_data
  readRegisters(IMU_ADDR, IMU_UI_OUTX_L_A_OIS_HG, _value+12, 6);  // reading in the last 6 bytes into _value
  
  // Byte Concatenation: _raw_data -> [gyroX,gyroY,gyroZ,accX,accY,accZ,hg_accx,hg_accy,hg_accz]
  _raw_data[0] = (_value[1]<<8)|(_value[0]);
  _raw_data[1] = (_value[3]<<8)|(_value[2]);
  _raw_data[2] = (_value[5]<<8)|(_value[4]);

  _raw_data[3] = (_value[7]<<8)|(_value[6]);
  _raw_data[4] = (_value[9]<<8)|(_value[8]);
  _raw_data[5] = (_value[11]<<8)|(_value[10]);

  _raw_data[6] = (_value[13]<<8)|(_value[12]);
  _raw_data[7] = (_value[15]<<8)|(_value[14]);
  _raw_data[8] = (_value[17]<<8)|(_value[16]);

  // Convert Gyro into dps
  double fs_dps = 4000;
  double max_16_signed = 32767;
  double gyro_scale_factor = fs_dps / max_16_signed;

  for (int i=0; i<3; i++){
    _data[i] = _raw_data[i] * gyro_scale_factor;
  }

  // Convert Acc into g
  double fs_g = 16;
  double acc_scale_factor = fs_g / max_16_signed;

  for (int i=3; i<6; i++){
    _data[i] = _raw_data[i] * acc_scale_factor;
  }

  // Convert HG Acc into g
  double fs_hg = 32;
  double hg_scale_factor = fs_hg / max_16_signed;

  for (int i=6; i<9; i++){
    _data[i] = _raw_data[i] * hg_scale_factor;
  }

}

