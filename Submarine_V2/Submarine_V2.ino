#include <EEPROM.h>


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


#define INTERRUPT_PIN 12  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

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



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

#define  MOTOR_L_PIN    5
#define  MOTOR_R_PIN    6

int motor_Power_L = 200;
int motor_Power_R = 200;
int motor_Power_set = 130;
int motor_max_power = 250;

float Total_angleZ = 0.0,Total_angleX = 0.0,Total_angleY = 0.0;
float pid_p=0.0,pid_i=0.0, pid_d=0.0, PID=0.0;
float error=0.0, previous_error=0.0 ; 
float kp=2.5;//2.5 , 3.5
float ki=0.01;//0.1
float kd=50.0;//550
float PID_Max = 80;
float desired_angle = 0.0;


float elapsedTime = 0.0, time1= 0.0, timePrev= 0.0, step_timer = 0;
float UpdateTimer = 0.0 ;
float period = 5.0;


struct str_program_step{
  int duration_ms;
  float heading_deg;
  int power;
};

struct str_main_program{
  int number_of_steps;
  str_program_step steps[10];
};

str_main_program main_program[2] = {
                                      {
                                        8, {{3000, 0.0, 230},
                                            {1000, 0.0, 230},
                                            {500,180.0, 180},
                                            {7000,180.0, 0},
                                            {3000,180.0, 230},
                                            {1000,180.0, 230},
                                            {500,0.0, 180},
                                            {7000,0.0, 0}}
                                      },
                                      {
                                        8, {{2000, 0.0, 200},
                                            {2000, 270.0, 200},
                                            {2000, 180.0, 200},
                                            {2000, 90.0, 200},
                                            {2000, 0.0, 200},
                                            {2000, 270.0, 200},
                                            {2000, 180.0, 200},
                                            {2000, 90.0, 200}}
                                      }
                                    }; 

#define possible_angles_count 7
float possible_angles[possible_angles_count] = {0.0, 45.0, 90.0, 315.0, 270.0, 135, 225.0};

int current_step = 0;
int program_index = 0;

#define SERIAL_ENABLED
#define MOTOR_ENABLED

void ExecuteProgram(void);
void UpdateMPUData(void);
void ExecutePID(void);
void UpdateMotorPower(void);
void MPUInit(void);

void setup() {
      program_index = 0;
//    program_index = EEPROM.read(0);
//    program_index++;
//    if ( program_index > 1 )
//      program_index = 0;
//    EEPROM.write(0, program_index); 

    for(int i=0; i<=program_index ;i++)
    {
      digitalWrite(LED_PIN, HIGH);
      delay(500);
      digitalWrite(LED_PIN, LOW);
      delay(500);
    }
    
    pinMode(LED_PIN, OUTPUT); 
    pinMode(MOTOR_L_PIN, OUTPUT);
    pinMode(MOTOR_R_PIN, OUTPUT);
    digitalWrite(MOTOR_L_PIN, LOW);
    digitalWrite(MOTOR_R_PIN, LOW);
    digitalWrite(LED_PIN, LOW);
    
    MPUInit();
 
    for(int i=0; i<15 ;i++)
    {
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
      delay(100);
    }
//    delay(300000);
    
    time1 = micros(); //Start counting time in milliseconds
    UpdateTimer = millis(); 
    step_timer =UpdateTimer;
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() { 
  //analogWrite(MOTOR_R_PIN, 250);   
  //analogWrite(MOTOR_L_PIN, 250);
  
   if (millis() > UpdateTimer+period)
    {
      UpdateTimer = millis(); 
      timePrev = time1;  // the previous time is stored before the actual time read
      time1 = micros();  // actual time read
      elapsedTime = (time1 - timePrev)/1000.0 ;  


      ExecuteProgram2();
      UpdateMPUData();
      ExecutePID();
      UpdateMotorPower(); 

#ifdef SERIAL_ENABLED
      
 
      Serial.print(current_step );
      Serial.print(", " );
      Serial.print(desired_angle );
      //Serial.print(", " );
      //Serial.print(Total_angleX );
      Serial.print(", " );
      Serial.print(Total_angleY );
      Serial.print(", " );
      Serial.print(Total_angleZ );
      Serial.print(", " );
      Serial.print(PID );
  
      Serial.print(",\t\t" );
      Serial.print(motor_Power_L );
      Serial.print(", " );
      Serial.println(motor_Power_R );
   
#endif      
      delay(5);
    } 
    
}


///////////////////////////////////////////////////////////////////////////
/////////////////////////////  ExecuteProgram  ////////////////////////////
bool is_prog_running = true;
void ExecuteProgram(void)
{ 
  if( is_prog_running == true )
  {
    if( millis() - step_timer >= main_program[program_index].steps[current_step].duration_ms)
    {
      current_step++;
      if( current_step > main_program[program_index].number_of_steps )
      {
        current_step = 0; 
        is_prog_running = false;
        digitalWrite(LED_PIN, HIGH);
      }
      step_timer = millis();
    }
    desired_angle = main_program[program_index].steps[current_step].heading_deg;
    motor_Power_set = main_program[program_index].steps[current_step].power;
  } 
}

void ExecuteProgram2(void)
{ 
  if( is_prog_running == true )
  {
    if( millis() - step_timer >= 2000)
    {
      int rand_angle = random(0, possible_angles_count+1); 
      current_step++;
      if( current_step > 30 )
      {
        current_step = 0; 
        is_prog_running = false;
        digitalWrite(LED_PIN, HIGH);
      }
      step_timer = millis();
      desired_angle = possible_angles[rand_angle];
    } 
  } 
}


///////////////////////////////////////////////////////////////////////////
/////////////////////////////  UpdateMPUData  /////////////////////////////
void UpdateMPUData(void)
{
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
        
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      //Serial.print("ypr\t");
      //Serial.println(ypr[0] * 180/M_PI); 
      Total_angleZ = ypr[0]* 180/M_PI;
      Total_angleY = ypr[1] * 180/M_PI ;
      Total_angleX = ypr[2] * 180/M_PI ;
  }
}

///////////////////////////////////////////////////////////////////////////
/////////////////////////////  ExecutePID  ////////////////////////////////
void ExecutePID(void)
{  
  if( Total_angleZ<0)
  {
    Total_angleZ += 360.0;
  }
  //if( desired_angle == 0 && Total_angleZ >= 350 && Total_angleZ <= 360)
  //{
  //  Total_angleZ -= 360.0;
  //}
  
  error = desired_angle - Total_angleZ; 
  //error -= (360.0) * (error / (360.0));
  error -= 360*floor(0.5+error/360);
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

///////////////////////////////////////////////////////////////////////////
/////////////////////////////  UpdateMotorPower  //////////////////////////
int current_position = 0;
void UpdateMotorPower(void)
{
  motor_Power_R = motor_Power_set - (int) PID;
  motor_Power_L = motor_Power_set + (int) PID; 
  
  if( motor_Power_L > motor_max_power )
  {
    motor_Power_L = motor_max_power;
  }
  else if( motor_Power_L < 0 )
  {
    motor_Power_L = 0;
  }
  if( motor_Power_R > motor_max_power )
  {
    motor_Power_R = motor_max_power;
  }
  else if( motor_Power_R < 0 )
  {
    motor_Power_R = 0;
  }

  if( Total_angleY > 130.0 || Total_angleY < -130.0)
  {
    //Serial.println("E1");
    //Serial.print("OFF, " );
    digitalWrite(MOTOR_L_PIN, LOW);
    digitalWrite(MOTOR_R_PIN, LOW);
    if( is_prog_running == true )
    {
      Serial.println("OFFFFFFFFF");
      is_prog_running = false;
      current_step = 0; 
    }
     digitalWrite(LED_PIN, LOW);
    current_position = 1;
  }
  else if( is_prog_running == true )
  {
    //Serial.print("ON, " );
#ifdef MOTOR_ENABLED
    analogWrite(MOTOR_R_PIN, motor_Power_R);   
    analogWrite(MOTOR_L_PIN, motor_Power_L);
#endif
  } 
  else if( is_prog_running == false && current_position == 1 )
  {
    Serial.println("ONNNNNNNN");
    delay(1000);
    current_position = 0;
    is_prog_running = true;
    program_index++;
    if( program_index > 1)
      program_index = 0;
      
    for(int i=0; i<=program_index ;i++)
    {
      digitalWrite(LED_PIN, HIGH);
      delay(500);
      digitalWrite(LED_PIN, LOW);
      delay(500);
    }
    delay(4000);
    step_timer = millis();
  }
  else
  {
    digitalWrite(MOTOR_L_PIN, LOW);
    digitalWrite(MOTOR_R_PIN, LOW);
  }
}
///////////////////////////////////////////////////////////////////////////
/////////////////////////////  MPUInit  ///////////////////////////////////
void MPUInit(void)
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
  // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  //mpu.setSleepEnabled(false);
  // wait for ready
  //Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  //while (Serial.available() && Serial.read()); // empty buffer
  //while (!Serial.available());                 // wait for data
  //while (Serial.available() && Serial.read()); // empty buffer again

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
  
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
      // Calibration Time: generate offsets and calibrate our MPU6050
      mpu.CalibrateAccel(6);
      mpu.CalibrateGyro(6);
      mpu.PrintActiveOffsets();
      // turn on the DMP, now that it's ready
      Serial.println(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);

      // enable Arduino interrupt detection
      Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
      Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
      Serial.println(F(")..."));
      attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();

      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      Serial.println(F("DMP ready! Waiting for first interrupt..."));
      dmpReady = true;

      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
  }
}
