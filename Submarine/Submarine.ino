
#define  MOTOR_L_PIN    5
#define  MOTOR_R_PIN    6



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////// MPU6050
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high
 
// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL
 

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards  

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

  
// INTERRUPT DETECTION ROUTINE 

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


#define SERIAL_ENABLED


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float pid_p=0.0,pid_i=0.0, pid_d=0.0, PID=0.0;
float error=0.0, previous_error=0.0 ; 
float kp=2.5;//2.5 , 3.5
float ki=0.01;//0.1
float kd=10.0;//550
float PID_Max = 180;
float desired_angle = 0.0;


float elapsedTime = 0.0, time1= 0.0, timePrev= 0.0;
float UpdateTimer = 0.0 ;
float period = 5.0;



void setup() { 
  // put your setup code here, to run once:
  pinMode(MOTOR_L_PIN, OUTPUT);
  pinMode(MOTOR_R_PIN, OUTPUT);
   pinMode(13, OUTPUT);
delay(500);
  digitalWrite(MOTOR_L_PIN, LOW);
  digitalWrite(MOTOR_R_PIN, LOW);
for(int i=0 ;i<5 ; i++)
{
  digitalWrite(13, HIGH);
  delay(500);
  digitalWrite(13, LOW);
}
digitalWrite(13, LOW);
  //----------------------------------------------------------------------------
  #ifdef SERIAL_ENABLED
  Serial.begin(115200); 
  Serial.println(F("Initializing I2C devices..."));
  #endif
  
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  #ifdef SERIAL_ENABLED
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  
  Serial.println(F("Initializing DMP...")); 
  #else
  mpu.testConnection();
  #endif
  
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);

   if (devStatus == 0) {
      // Calibration Time: generate offsets and calibrate our MPU6050
      mpu.CalibrateAccel(6);
      mpu.CalibrateGyro(6);
      mpu.PrintActiveOffsets();
      // turn on the DMP, now that it's ready
      #ifdef SERIAL_ENABLED
      Serial.println(F("Enabling DMP..."));
      #endif
      mpu.setDMPEnabled(true);

      // enable Arduino interrupt detection
      #ifdef SERIAL_ENABLED
      Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
      Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
      Serial.println(F(")...")); 
      #else
      digitalPinToInterrupt(INTERRUPT_PIN);
      #endif
      attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();

      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      #ifdef SERIAL_ENABLED
      Serial.println(F("DMP ready! Waiting for first interrupt..."));
      #endif
      dmpReady = true;

      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      #ifdef SERIAL_ENABLED
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
      #endif
  }

  //-----------------------------------------------------------------------------

  
  delay(3000);

  time1 = micros(); //Start counting time in milliseconds
  UpdateTimer = millis(); 
}
int motor_Power_1 =80;
//int motor_Power_2 =motor_Power_1-50;


//speed = distance / time = 40cm / 2sec = 20 cm/sec
//diving_speed = 15cm/2sec = 7 cm/sec

int travel_speed = 20; // cm/sec
int diving_speed = 7; // cm/sec
float Total_angleX=0.0, Total_angleY=0.0, Total_angleZ=0.0 ;

int motor_Power_L = 200;
int motor_Power_R = 200;
int motor_Power_set = 200;
int motor_max_power = 250;

void loop() {
 UpdateMPUData();
 
 if( Total_angleZ < 0.0 )//&& direction != right)
    {
      Serial.print(Total_angleZ);
      Serial.print("\t");
      Serial.println("Right");
      digitalWrite(13, LOW);
      //digitalWrite(MOTOR_R_PIN, LOW); 
     // digitalWrite(MOTOR_L_PIN, HIGH); 
      //analogWrite(MOTOR_R_PIN, 0);   
      //analogWrite(MOTOR_L_PIN, 200);
      //direction = right;
    }
    else if(Total_angleZ>0.0 )//&& direction != left)
    {
      Serial.print(Total_angleZ);
      Serial.print("\t");
      Serial.println("Left");
      digitalWrite(13, HIGH);
      //digitalWrite(MOTOR_R_PIN, HIGH); 
      //digitalWrite(MOTOR_L_PIN, LOW); 
      //analogWrite(MOTOR_R_PIN, 200);   
      //analogWrite(MOTOR_L_PIN, 0);
      //direction = left;
    }
    else
    {
      Serial.println("str8");
      //digitalWrite(MOTOR_L_PIN, HIGH); 
    }
}


void UpdateMPUData()
{
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();
  if(fifoCount < packetSize)
  {
      //Lets go back and wait for another interrupt. We shouldn't be here, we got an interrupt from another event
      // This is blocking so don't do it   while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
  }
  // check for overflow (this should never happen unless our code is too inefficient)
  else if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) 
  {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      //  fifoCount = mpu.getFIFOCount();  // will be zero after reset no need to ask
      #ifdef SERIAL_ENABLED
      Serial.println(F("FIFO overflow!")); 
      #endif
      // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } 
  else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) 
  { 
    
      // read a packet from FIFO
      while(fifoCount >= packetSize)
      { // Lets catch up to NOW, someone is using the dreaded delay()!
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
      }
       
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      Total_angleZ = ypr[0] * 180/M_PI ;
      Total_angleY = ypr[1] * 180/M_PI ;
      Total_angleX = ypr[2] * 180/M_PI ; 

      //Serial.print(Total_angleX);Serial.print(", ");Serial.print(Total_angleY);Serial.print(", ");Serial.println(Total_angleZ ); 
 
 } 
}



 
void UpdateGyroAnglesPID2()
{ 
  error = desired_angle - Total_angleZ; 
  pid_p = kp*error;

 if(  error > -1.0 && error < 1.0)
 {
   pid_i = 0.0; 
 }
  //if( error >= -3.0 && error <= 3.0)     //( abs(error_roll) >= 1.0)
  else
  {
    pid_i = pid_i+(ki*error);  
  } 
  /*
  else if( abs(error_roll) < 1.0)
  {
    pid_i_roll = 0; 
  }
  */
  
  pid_d = kd*((error - previous_error)/elapsedTime);
  
  PID = pid_p + pid_i + pid_d;

  if(PID < (-1*PID_Max))
  {
    PID=-1*PID_Max;
  }
  if(PID > PID_Max)
  {
    PID= PID_Max;
  }
  previous_error = error;
}
