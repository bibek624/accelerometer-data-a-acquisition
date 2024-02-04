#include <HX711.h>



//#include "BMI088.h"
#include "Wire.h"


#include <ESP32TimerInterrupt.h>




#define DOUT1 22 //pin 3 Arduino and sensor DAT output
#define CLK1 23 //pin 2 Arduino and sensor CLK output




#define BMI088_ACC_ADDRESS          0x19
#define BMI088_ACC_ALT_ADDRESS      0x18

#define BMI088_ACC_CHIP_ID          0x00 // Default value 0x1E
#define BMI088_ACC_ERR_REG          0x02
#define BMI088_ACC_STATUS           0x03

#define BMI088_ACC_X_LSB            0x12
#define BMI088_ACC_X_MSB            0x13
#define BMI088_ACC_Y_LSB            0x14
#define BMI088_ACC_Y_MSB            0x15
#define BMI088_ACC_Z_LSB            0x16
#define BMI088_ACC_Z_MSB            0x17

#define BMI088_ACC_SENSOR_TIME_0    0x18
#define BMI088_ACC_SENSOR_TIME_1    0x19
#define BMI088_ACC_SENSOR_TIME_2    0x1A

#define BMI088_ACC_INT_STAT_1       0x1D

#define BMI088_ACC_TEMP_MSB         0x22
#define BMI088_ACC_TEMP_LSB         0x23

#define BMI088_ACC_CONF             0x40
#define BMI088_ACC_RANGE            0x41

#define BMI088_ACC_INT1_IO_CTRL     0x53
#define BMI088_ACC_INT2_IO_CTRL     0x54
#define BMI088_ACC_INT_MAP_DATA     0x58

#define BMI088_ACC_SELF_TEST        0x6D

#define BMI088_ACC_PWR_CONF         0x7C
#define BMI088_ACC_PWR_CTRl         0x7D

#define BMI088_ACC_SOFT_RESET       0x7E

#define BMI088_GYRO_ADDRESS         0x69
#define BMI088_GYRO_ALT_ADDRESS     0x68


#define BMI088_GYRO_CHIP_ID             0x00 // Default value 0x0F

#define BMI088_GYRO_RATE_X_LSB          0x02
#define BMI088_GYRO_RATE_X_MSB          0x03
#define BMI088_GYRO_RATE_Y_LSB          0x04
#define BMI088_GYRO_RATE_Y_MSB          0x05
#define BMI088_GYRO_RATE_Z_LSB          0x06
#define BMI088_GYRO_RATE_Z_MSB          0x07

#define BMI088_GYRO_INT_STAT_1          0x0A

#define BMI088_GYRO_RANGE               0x0F
#define BMI088_GYRO_BAND_WIDTH          0x10

#define BMI088_GYRO_LPM_1               0x11

#define BMI088_GYRO_SOFT_RESET          0x14

#define BMI088_GYRO_INT_CTRL            0x15
#define BMI088_GYRO_INT3_INT4_IO_CONF   0x16
#define BMI088_GYRO_INT3_INT4_IO_MAP    0x18

#define BMI088_GYRO_SELF_TEST           0x3C

enum device_type_t { // device type
  ACC = 0x00, //
  GYRO = 0x01, //
};

enum acc_scale_type_t { // measurement rage
  RANGE_3G = 0x00, //
  RANGE_6G = 0x01, //
  RANGE_12G = 0x02, //
  RANGE_24G = 0x03, //
};

enum acc_odr_type_t { // output data rate
  ODR_12 = 0x05, //
  ODR_25 = 0x06, //
  ODR_50 = 0x07, //
  ODR_100 = 0x08, //
  ODR_200 = 0x09, //
  ODR_400 = 0x0A, //
  ODR_800 = 0x0B, //
  ODR_1600 = 0x0C, //
};

enum acc_power_type_t { // power mode
  ACC_ACTIVE = 0x00, //
  ACC_SUSPEND = 0x03, //
};

enum gyro_scale_type_t { // measurement rage
  RANGE_2000 = 0x00, //
  RANGE_1000 = 0x01, //
  RANGE_500 = 0x02, //
  RANGE_250 = 0x03, //
  RANGE_125 = 0x04, //
};

enum gyro_odr_type_t { // output data rate
  ODR_2000_BW_532 = 0x00, //
  ODR_2000_BW_230 = 0x01, //
  ODR_1000_BW_116 = 0x02, //
  ODR_400_BW_47 = 0x03, //
  ODR_200_BW_23 = 0x04, //
  ODR_100_BW_12 = 0x05, //
  ODR_200_BW_64 = 0x06, //
  ODR_100_BW_32 = 0x07, //
};

enum gyro_power_type_t { // power mode
  GYRO_NORMAL = 0x00, //
  GYRO_SUSPEND = 0x80, //
  GYRO_DEEP_SUSPEND = 0x20, //
};


float ax = 0, ay = 0, az = 0;
float gx = 0, gy = 0, gz = 0;
int16_t temp = 0;


ESP32Timer Timer1(1);

//TwoWire tw1 = TwoWire(2);
//
//TwoWire tw2 = TwoWire(3);


float accRange;
float gyroRange;
uint8_t devAddrAcc = BMI088_ACC_ADDRESS;
uint8_t devAddrGyro = BMI088_GYRO_ADDRESS;


TwoWire wire = Wire;


HX711 scale1 ;
HX711 scale2 ;


int t = 0;
bool sendData = false;

float a1, a2, disp1,disp2 = 0;

//int disp = 0;
struct Accelerometer {
  float ax;
  float ay;
  float az;
};


const int n_acc = 2;
TwoWire wires[n_acc] = {TwoWire(2), TwoWire(3)};
int SDAs[n_acc] = {26, 14};
int SCKs[n_acc] = {25, 27};
int connections[n_acc] = {0, 0};

Accelerometer accelerometers[n_acc];



char buffer[100];

void setup(void) {

  //  tw1.begin(I2C_SDA1, I2C_SCL1);
  //  tw2.begin(I2C_SDA2, I2C_SCL2);
  //
  Serial.begin(115200);
  initialize_accelerometers();
  //    while (1) {
  //
  //        wire = tw1;
  ////        delay(2);
  //        if (isConnection()) {
  //            initialize();
  //            Serial.println("BMI088 1 is connected");
  //            break;
  //        } else {
  //            Serial.println("BMI088 1 is not connected");
  //        }
  //        delay(1000);
  //    }
  //
  //    while(1){
  //
  //        wire = tw2;
  ////        delay(2);
  //         if (isConnection()) {
  //            initialize();
  //            Serial.println("BMI088 2 is connected");
  //            break;
  //        } else {
  //            Serial.println("BMI088 2 is not connected");
  //        }
  //      delay(1000);
  //    }
  //


  scale1.begin(DOUT1, CLK1);
  scale1.set_scale();
  scale1.tare(); //Resets scale to 0



  Timer1.attachInterruptInterval(12.5 * 1000, updateFlag);

}


void loop(void) {

  //  return;

  if (!Serial) {
    t = 0;
    delay(1000);

  }

  if (sendData) {
    //      wire = tw1;
    //      //  delay(2);
    //      a1 = getAccelerationZ();
    //      wire = tw2;
    //      //  delay(2);
    //      a2 = getAccelerationY();

    read_accelerometers();
//    scale1.set_gain(128);



    disp1 = (50.0 / 11315022.45) * (scale1.read() + 5640201.3) - 25;
//    scale1.set_gain(32);
//    disp2 = (50.0 / 11315022.45) * (scale1.read() + 5640201.3) - 25;
    //  disp = scale.read();

//    sprintf(buffer, "{%d,%f,%f,%f,%f,%f,%f,%f,%f,%d}", t, accelerometers[0].ax, accelerometers[0].ay,accelerometers[0].az,accelerometers[1].ax,accelerometers[1].ay,accelerometers[1].az,disp1,disp2,0);
          sprintf(buffer, "{%d,%f,%f,%f,%d}", t, accelerometers[0].ax, accelerometers[1].az, disp1, 0);
    //    sprintf(buffer, "%d,%f,%f,%f", 0, 0, 0, disp);
    //  sprintf(buffer, "%d,%f}", t, disp);
          Serial.write(buffer);
//    Serial.println(buffer);
    sendData = false;
  }

}

bool isConnection(void) {

  return ((getAccID() == 0x1E) && (getGyroID() == 0x0F));

}



bool updateFlag(void * timerNo) {


  t++;
  sendData = true;
  return true;

}


void read_accelerometers() {


  for (int k = 0; k < n_acc; k++) {
    if (connections[k] == 1) {
      wire = wires[k];
      accelerometers[k] = {getAccelerationX(), getAccelerationY(), getAccelerationZ()};
    }
    else {
      accelerometers[k] = {0, 0, 0};
    }
  }

}







float getAccelerationX(void) {
  uint16_t ax = 0;
  float value = 0;

  ax = read16(ACC, BMI088_ACC_X_LSB);

  value = (int16_t)ax;
  value = accRange * value / 32768;

  return value;
}

float getAccelerationY(void) {
  uint16_t ay = 0;
  float value = 0;

  ay = read16(ACC, BMI088_ACC_Y_LSB);

  value = (int16_t)ay;
  value = accRange * value / 32768;

  return value;
}

float getAccelerationZ(void) {
  uint16_t az = 0;
  float value = 0;

  az = read16(ACC, BMI088_ACC_Z_LSB);

  value = (int16_t)az;
  value = accRange * value / 32768;

  return value;
}




void initialize() {
  setAccScaleRange(RANGE_6G);
  setAccOutputDataRate(ODR_100);
  setAccPoweMode(ACC_ACTIVE);

  setGyroScaleRange(RANGE_2000);
  setGyroOutputDataRate(ODR_2000_BW_532);
  setGyroPoweMode(GYRO_NORMAL);
}

void initialize_accelerometers() {

  for (int k = 0; k < n_acc; k++) {
    wires[k].begin(SDAs[k], SCKs[k]);
    wire = wires[k];

    char string[100];

    if (isConnection()) {
      initialize();

      sprintf(string, "BMI088 %d is connected",k);
      Serial.println(string);
      
      connections[k] = 1;
    } else {
      connections[k] = 0;
      
      sprintf(string, "BMI088 %d is not connected",k);
      Serial.println(string);
  
    }
  }

}



uint8_t getAccID(void) {
  return read8(ACC, BMI088_GYRO_CHIP_ID);
}

uint8_t getGyroID(void) {
  return read8(GYRO, BMI088_GYRO_CHIP_ID);
}

void setAccPoweMode(acc_power_type_t mode) {
  if (mode == ACC_ACTIVE) {
    write8(ACC, BMI088_ACC_PWR_CTRl, 0x04);
    write8(ACC, BMI088_ACC_PWR_CONF, 0x00);
  } else if (mode == ACC_SUSPEND) {
    write8(ACC, BMI088_ACC_PWR_CONF, 0x03);
    write8(ACC, BMI088_ACC_PWR_CTRl, 0x00);
  }
}

void setGyroPoweMode(gyro_power_type_t mode) {
  if (mode == GYRO_NORMAL) {
    write8(GYRO, BMI088_GYRO_LPM_1, (uint8_t)GYRO_NORMAL);
  } else if (mode == GYRO_SUSPEND) {
    write8(GYRO, BMI088_GYRO_LPM_1, (uint8_t)GYRO_SUSPEND);
  } else if (mode == GYRO_DEEP_SUSPEND) {
    write8(GYRO, BMI088_GYRO_LPM_1, (uint8_t)GYRO_DEEP_SUSPEND);
  }
}

void setAccScaleRange(acc_scale_type_t range) {
  if (range == RANGE_3G) {
    accRange = 3000;
  } else if (range == RANGE_6G) {
    accRange = 6000;
  } else if (range == RANGE_12G) {
    accRange = 12000;
  } else if (range == RANGE_24G) {
    accRange = 24000;
  }

  write8(ACC, BMI088_ACC_RANGE, (uint8_t)range);
}

void setAccOutputDataRate(acc_odr_type_t odr) {
  uint8_t data = 0;

  data = read8(ACC, BMI088_ACC_CONF);
  data = data & 0xf0;
  data = data | (uint8_t)odr;

  write8(ACC, BMI088_ACC_CONF, data);
}

void setGyroScaleRange(gyro_scale_type_t range) {
  if (range == RANGE_2000) {
    gyroRange = 2000;
  } else if (range == RANGE_1000) {
    gyroRange = 1000;
  } else if (range == RANGE_500) {
    gyroRange = 500;
  } else if (range == RANGE_250) {
    gyroRange = 250;
  } else if (range == RANGE_125) {
    gyroRange = 125;
  }

  write8(GYRO, BMI088_GYRO_RANGE, (uint8_t)range);
}

void setGyroOutputDataRate(gyro_odr_type_t odr) {
  write8(GYRO, BMI088_GYRO_BAND_WIDTH, (uint8_t)odr);
}


uint8_t read8(device_type_t dev, uint8_t reg) {
  uint8_t addr = 0, data = 0;

  if (dev) {
    addr = devAddrGyro;
  } else {
    addr = devAddrAcc;
  }

  wire.beginTransmission(addr);
  wire.write(reg);
  wire.endTransmission();

  wire.requestFrom(addr, (uint8_t)1);
  while (wire.available()) {
    data = wire.read();
  }

  return data;
}

uint16_t read16(device_type_t dev, uint8_t reg) {
  uint8_t addr = 0;
  uint16_t msb = 0, lsb = 0;

  if (dev) {
    addr = devAddrGyro;
  } else {
    addr = devAddrAcc;
  }

  wire.beginTransmission(addr);
  wire.write(reg);
  wire.endTransmission();

  wire.requestFrom(addr, (uint8_t)2);
  while (wire.available()) {
    lsb = wire.read();
    msb = wire.read();
  }

  return (lsb | (msb << 8));
}

void write8(device_type_t dev, uint8_t reg, uint8_t val) {
  uint8_t addr = 0;

  if (dev) {
    addr = devAddrGyro;
  } else {
    addr = devAddrAcc;
  }

  wire.beginTransmission(addr);
  wire.write(reg);
  wire.write(val);
  wire.endTransmission();
}
