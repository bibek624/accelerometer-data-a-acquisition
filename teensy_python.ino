#include <TeensyTimerTool.h> // for calling a function in regular interval
using namespace TeensyTimerTool;
#include <Encoder.h> // for storing encoder pulses
#include <SoftwareSerial.h> // using any serial pin as tx rx
#include <PID_v1.h> // PID algorithm
#include <SabertoothSimplified.h> //for sabertooth simplified serial mode communication 
#include "SerialTransfer.h" //for packetized serial communication




// defining pins
#define limitForward 1
#define homePin 2
#define limitBackward 3
#define encA 6
#define encB 5
#define potPin 15
#define relayPin 10

int offset = 36; //36

//filter
double a[] = {1.98519079, -0.98529965};
double b[] = {2.72133124e-05, 5.44266249e-05, 2.72133124e-05};
double xn, xn1, xn2, ynn, yn1, yn2;

double prev;
double prev_output = 0;

// defining variables

//struct __attribute__((packed)) STRUCT {
//  float pid[3];
//  char mode;
//} initialData;

const int range_size = 10;
struct __attribute__((packed)) STRUCT {
  float r1[range_size];
  float r2[range_size];
  float p[range_size];
  float i[range_size];
  float d[range_size];
  char mode;
} initialData;


int lenData = 0;
char mode = '0';
int32_t potLimit = 2500;

const int len = 80000;
int32_t positions[len] ;
//double positions[len] ;


double kp = 0.12;
double ki = 0.00001;
double kd = 0;
double Setpoint, Input, Output;


float pid[3]; //for receiving pid parameters through serial
float sineParameters[2]; // for receiving frequency and amplitude through serial
double t0 = 0;

int maxRpm = 127;
int initializeRpm = 30;

// variables for uploading data
const int nData = 10; // number of data in each upload
double times[nData];
double veloc[nData];
double setPoints[nData];
double currentPositions[nData];
double uploadData[nData * 3]; // for combining all three data in single array


volatile double t = 0;
int t_interval = 2; //ms // time interval between data points
int print_interval = 10; //ms // time interval between data points
//int print_interval = 5;
double lastSetPoint = 0;
int pid_interval = 500; //minimum time to calculate new output in pid algorithm in microSeconds

// flags
bool printFlag = true;
bool metaData = false; // for initial parameters => sine / earthquake
bool error = false;
bool initializing = true;
bool simulate = false;
bool plot = false;
bool fLimit = false;
bool bLimit = false;
bool cLimit = false;
bool initialized = false;

double W, A;

double setpt = 0, err, prev_setpt = 0, to_move = 0;

// loop variables
int ndx = 0; // to update setpoint
int u = 0; // to combine data during upload

char r = '0';

//classes
Timer t1;
Timer t2;
Encoder myEnc(encA, encB);
//PID myPID(&Input, &Output, &Setpoint, 0, 0, 0, DIRECT); //the pid parameter will be updated after receiving through serial
PID myPID(&Input, &Output, &Setpoint, kp, ki, kd, DIRECT);
#define HWSERIAL Serial4 // tx pin 17
SabertoothSimplified ST(HWSERIAL);
SerialTransfer myTransfer;

double velocity = 0.0;
double currentPos = 0;

double c_input = 0;

void setup() {

  xn = xn1 = xn2 = ynn = yn1 = yn2 = 0;

  // initializing serial
  Serial.begin(115200);
  HWSERIAL.begin(38400);
  myTransfer.begin(Serial);

  // defining pin mode
  pinMode(relayPin, OUTPUT);
  pinMode(limitBackward, INPUT);
  pinMode(homePin, INPUT);
  pinMode(limitForward, INPUT);
  pinMode(potPin, INPUT);

  digitalWrite(relayPin, HIGH);

  // stopping motor
  stopMotor();

  // setting the analog resolution to 0-4095
  analogReadResolution(13);

  //initializing PID control
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-maxRpm, maxRpm);
  myPID.SetSampleTime(pid_interval);

  //initializing encoder;o
  myEnc.write(0);

  //  interrupts for proximity sensors
  attachInterrupt(limitForward, forwardLimitReached, RISING);
  attachInterrupt(limitBackward, backwardLimitReached, RISING);
  attachInterrupt(homePin, homeReached, RISING);

  //  triggerRelayOn();

  // attaching interrupt triggered every t_interval ms

  //  stopMotor();
}

void(*resetFunc)(void) = 0;

void loop() {


  if(initialized){
    deccelerate();
    }
  stopMotor();
  triggerRelayOff();
  fetchInitialData();
  fetchMetaData();

  triggerRelayOn();

  if (initialized) {
    center();
  } else {
    gotoHome();
    initialized = true;
  }
  myEnc.write(0);


  sendInitializedSignal();
  flushUploadData();
  simulate = waitForSignal();


  t0 = millis();
  t = 0;
  ndx = 0;
  prev_setpt = 0;

  error = false;
  err = 0;
  
  if (simulate)
    beginInterrupt();

  while (simulate) {
    
    if (error) {
      stopInterrupt();
      handleError();
      break;
    }
    if (!Serial.dtr()) {
//      deccelerate();
      simulate = false;
      break;
    }
    if (mode == 's')
      simulateSineWave();
    else if (mode == 'p')
      simulatePotentiometer();
    else if (mode == 'e')
      simulateEarthquake();

    printData();
  }
}


void beginInterrupt() {
  t1.beginPeriodic(updateData, t_interval * 1000);
  t2.beginPeriodic(updatePrintFlag, print_interval* 1000);
}

void updatePrintFlag(){
  printFlag = true;
  }

void stopInterrupt() {
  t1.stop();
  t2.stop();
  //  t2.beginPeriodic(printData, print_interval* 1000);
}
void deccelerate() {

//  return;

  int vel = (int) Output;
  float reduction = vel/5;
  for (int i = 0; i < 5; i++) {
    vel = (int) vel - reduction ;
    setRpm(vel);
    delay(50);
  }

}

//void fetchInitialData() {
//  waitForBuffer();
//  myTransfer.rxObj(initialData);
//  handShake();
//  mode = initialData.mode;
//  myPID.SetTunings(initialData.pid[0], initialData.pid[1], initialData.pid[2]);
//}

void fetchInitialData() {
  waitForBuffer();
  myTransfer.rxObj(initialData);
  handShake();
  mode = initialData.mode;
  if(mode == 's')
    myPID.SetTunings(initialData.p[0], initialData.i[0], initialData.d[0]);

}

bool waitForSignal() {

  while (!myTransfer.available()) {
    if (!Serial.dtr()) {
      break;
    }
    delay(1);
  }
  myTransfer.rxObj(r);
  handShake();
  if (r == 's')
    return true;
  else
    return false;
}

void handleError() {
  stopMotor();
  sendErrorSignal();
  delay(500);
  triggerRelayOn();
  delay(500);
  center();
  simulate = false;
  error = false;
}

void sendErrorSignal() {
  for (int e = 0; e < nData; e++) {
    uploadData[e] = -100;
    uploadData[e + 1 * nData] = -100;
    uploadData[e + 2 * nData] = -100;
  }
  myTransfer.sendDatum(uploadData);
}

void stopMotor() {
  setRpm(0);
}


void waitForBuffer() {
  while (!myTransfer.available()) {
    delay(1);
  }
}

void handShake() {
  myTransfer.sendDatum(1);
}

void sendInitializedSignal() {
  myTransfer.sendDatum('i');
}

void updateData() {
  //this function runs every t_interval ms

  if (!simulate) {
    return;
  }
//  err = Setpoint - count();
//  setpt = Setpoint - prev_setpt;
//  velocity = velocity + setpt;
//  prev_setpt = Setpoint;
  if (mode == 'e') {
    if (ndx > (lenData - (offset-1))) {
      simulate = false;
      ndx = 0;
      return;
    }
    ndx++;
  }
  t++;
//  printFlag = true;
}

void flushUploadData() {
  for (u = 0; u < nData; u++) {
    uploadData[u] = 0;
    uploadData[u + 1 * nData] = 0;
    uploadData[u + 2 * nData] = 0;
//    uploadData[u + 3 * nData] = 0;
  }
  u = 0;
}

void printData() {
  if (!printFlag) {
    return;
  }


  times[u] =  t * t_interval * 0.001;
  setPoints[u] = prev_setpt;
  currentPositions[u] = count();

//  setPoints[u] = prev_setpt ;
//  currentPositions[u] = err;
//  veloc[u] =  velocity;
  u++;


  //  }

  if (u > nData-1) {
    for (u = 0; u < nData; u++) {
      uploadData[u] = times[u];
      uploadData[u + 1 * nData] = setPoints[u];
      uploadData[u + 2 * nData] = currentPositions[u];
//      uploadData[u + 3 * nData] = veloc[u];
    }
    myTransfer.sendDatum(uploadData);
    u = 0;
  }
  printFlag = false;
}

void setRpm(int rpm) {
  ST.motor(1, rpm);
}

void fetchMetaData() {

  if (mode == 's') {
    waitForBuffer();
    myTransfer.rxObj(sineParameters);
    myTransfer.sendDatum(sineParameters);
    W = 2 * PI * sineParameters[0] ; //0 = frequency
    A = sineParameters[1]; //amplitude

  } else if (mode == 'e') {
    fetchEarthquakeData();
  } else if (mode == 'p') {
    waitForBuffer();
    myTransfer.rxObj(potLimit);
    myTransfer.sendDatum(potLimit);
  }
  t = 0;
}




void simulateSineWave() {

  //  Setpoint = a * sin(W * (millis() - t0) * 0.001);          // modify to fit motor and encoder characteristics, potmeter connected to A0
  //  setPoints[u] = Setpoint ;
  c_input = A * sin(W * t * t_interval * 0.001);
  Setpoint = A * sin(W * (t+offset) * t_interval * 0.001);
  runPID();
}

void fetchEarthquakeData() {

  const uint32_t chunkLength = 50;
  int32_t temp[chunkLength];
  //  double temp[chunkLength];
  uint32_t  chunks = 0;

  while (chunks == 0) {
    if (myTransfer.available())
    {
      myTransfer.rxObj(chunks);
      myTransfer.sendDatum(chunks);
    }
  }

  int i = 0;
  int startPos = 0;
  lenData = chunks * chunkLength;

  char flag = '0';
  while (i < chunks) {

    while (flag != 't') {
      waitForBuffer();
      myTransfer.rxObj(temp);
      startPos = i * chunkLength;

      for (int k = 0; k < chunkLength; k++) {
        positions[startPos + k] = temp[k];
    
      }

      myTransfer.sendDatum(temp);

      waitForBuffer();
      myTransfer.rxObj(flag);
      myTransfer.sendDatum(flag);
    }

    i++;
    flag = 'f';
    //      delay(1);
  }

}

void simulateEarthquake() {

  c_input = positions[ndx];
  Setpoint = positions[ndx+offset];
  runPID();


}

void simulatePotentiometer() {
  Setpoint = map(analogRead(potPin), 0, 4095 , -potLimit, potLimit);
  runPID();
}


//void runPID() {
//
//  xn = count();
//  ynn = a[0]*yn1 + a[1]*yn2 + b[0]*xn + b[1]*xn1 + b[2]*xn2 ;
//
//  yn2=yn1;
//  yn1=ynn;
//
//  xn2=xn1;
//  xn1=xn;
//
//
////  Input = count();
//  Input = xn;
//
//  prev_output = Output;
//
////  for(int l = 0;l<range_)
//  myPID.Compute();
////  setRpm(Output);
//  if(prev_output != Output)
//    setRpm(Output);
//
//
//}


void updatePosition(){
  
  }




void runPID() {

  xn = count();
  ynn = a[0] * yn1 + a[1] * yn2 + b[0] * xn + b[1] * xn1 + b[2] * xn2 ;

  yn2 = yn1;
  yn1 = ynn;

  xn2 = xn1;
  xn1 = xn;


//    Input = count();

    

  
    
  Input = xn;


  

  if(mode == 'e'){
  if (Setpoint !=  prev_setpt) {

//    sp = abs(setpt);

//    err = prev_setpt - Input;
    to_move = abs(Setpoint - Input);
    prev_setpt = Setpoint;
//    currentPos = count();
    
    for (int l = 0; l < range_size; l++) {
      
      if (to_move >= initialData.r1[l] && to_move < initialData.r2[l]) {
        myPID.SetTunings(initialData.p[l], initialData.i[l], initialData.d[l]);
        break;
      }
    }
 
  }
  }
  myPID.Compute();
  
  if (prev_output != Output){
    setRpm(Output);
    prev_output = Output;
  }
}

void goTo(int pos) {
  myPID.SetSampleTime(500);
  myPID.SetTunings(0.05, 0.00019, 0.5);
  myPID.SetOutputLimits(-30, 30);
  Setpoint = pos;
  int t_start = millis();
  while (abs(count() - pos) > 2 && (millis() - t_start) < 5000) {
    Input = count();
    myPID.Compute();
    setRpm(Output);

  }
  stopMotor();

  delay(100);
  myPID.SetSampleTime(pid_interval);
  myPID.SetOutputLimits(-maxRpm, maxRpm); 
  myPID.SetTunings(initialData.p[0], initialData.i[0], initialData.d[0]);
  Setpoint = 0;

}

void center() {
  //  return;
  int sign = 0;
//  deccelerate();
//  delay(300);
  //  if(abs(count())>100){
  if (count() < 0) {
    sign = -1;
  } else {
    sign = 1;
  }

  goTo(100 * sign);
  delay(300);
  goTo(0);
  delay(300);
  goTo(0);
  //  myEnc.write(0);
}

void gotoHome() {
  //  return;

  delay(500);

  bLimit = false;
  cLimit = false;
  fLimit = false;

  while (!bLimit) {
    setRpm(-initializeRpm);
  }

  stopMotor();
  delay(500);
  cLimit = false;
  initialized = false;
  while (!cLimit) {
    setRpm(initializeRpm);
  }
  myEnc.write(0);
  cLimit = false;
  initialized = true;
  deccelerate();
  center();
  //  while (!cLimit) {
  //    setRpm(-15);
  //  }




}


void forwardLimitReached() {

  if (!initialized || error) {
    fLimit = true;
    return;
  }

  if (simulate) {
    error = true;
    triggerRelayOff();
  }
}


void backwardLimitReached() {
  if (!initialized || error ) {
    bLimit = true;
    return;
  }

  if (simulate) {
    error = true;
    triggerRelayOff();
  }
}

void homeReached() {

  if (!initialized) {
    myEnc.write(0);
  }
  cLimit = true;

}

void triggerRelayOff() {
  digitalWrite(relayPin, HIGH);
}

void triggerRelayOn() {
  digitalWrite(relayPin, LOW);
}

double count() {
  return myEnc.read();
}
