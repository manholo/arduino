
 
#include <SoftwareSerial.h>
#include <Servo.h>

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


// code originally from https://wiki.wxwidgets.org/Development:_Small_Table_CRC

const uint32_t INITIAL_CRC = 0xFFFFFFFF;

uint32_t crc_table[] =
{
  0x00000000,
  0x1db71064,
  0x3b6e20c8,
  0x26d930ac,
  0x76dc4190,
  0x6b6b51f4,
  0x4db26158,
  0x5005713c,
  0xedb88320,
  0xf00f9344,
  0xd6d6a3e8,
  0xcb61b38c,
  0x9b64c2b0,
  0x86d3d2d4,
  0xa00ae278,
  0xbdbdf21c,
};
 
uint32_t update_crc( uint32_t crc, String s)
{
  for (int i = 0; i < s.length(); i++)
  {
    // XOR in the data. 
    crc ^= (uint8_t)s.charAt(i);
    // Perform the XORing for each nybble. Avoid sign extension using uints 
    crc = (crc >> 4) ^ crc_table[crc & 0x0f];
    crc = (crc >> 4) ^ crc_table[crc & 0x0f];
  }
  return crc;
}

uint32_t update_crc( uint32_t crc, byte* s, int len)
{
  for (int i = 0; i < len; i++)
  {
    // XOR in the data. 
    crc ^= (uint8_t)s[i]; //Serial.print ("<"); Serial.print((uint8_t)s[i], HEX); Serial.println(">");
    // Perform the XORing for each nybble. Avoid sign extension using uints 
    crc = (crc >> 4) ^ crc_table[crc & 0x0f]; //Serial.println (crc_table[crc & 0x0f], HEX);
    crc = (crc >> 4) ^ crc_table[crc & 0x0f]; //Serial.println (crc_table[crc & 0x0f], HEX);
  }
  return crc;
}
uint32_t compute_crc(String s)
{
  // initialize the crc -- start with this value each time you start a session. 
  return update_crc( INITIAL_CRC, s ) ; 
}

uint32_t compute_crc(byte* s, int len)
{
  // initialize the crc -- start with this value each time you start a session. 
  return update_crc( INITIAL_CRC, s, len ) ; 
}

boolean check_crc( byte* data, int len) {
  uint32_t crc = (uint32_t)(byte)data[len-4] << 24 | 
                 (uint32_t)(byte)data[len-3] << 16 | 
                 (uint32_t)(byte)data[len-2] << 8  |
                 (uint32_t)(byte)data[len-1];
  return compute_crc(  data, len - 4 ) == crc ;
  //uint32_t ccrc = compute_crc(  data, len - 4 ) ;
  //Serial.print("CRC Read:     ");
  //Serial.println(crc, HEX);
  //Serial.print("CRC Computed: ");
  //Serial.println(ccrc, HEX);
  //return ccrc == crc ;
}

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

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
VectorInt16 gyro;       // [x, y, z]            gyro angular velocity
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


// INTERRUPT DETECTION ROUTINE
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
volatile bool waiting_for_FIFO = false;
void dmpDataReady() {
    mpuInterrupt = true;
    if (waiting_for_FIFO) { Wire_setup(); mpuInterrupt = false; waiting_for_FIFO = false; }
}

//////////

const int pinint = 2 ; // interrupt pin for MPU 6050

// motors
const int pinENA = 5; // PWM R
const int pinIN1 = 7;
const int pinIN2 = 8;

const int pinIN3 = 3;  
const int pinIN4 = 4 ; 
const int pinENB = 6; // PWM L

const int pinMotorR[3] = { pinENA, pinIN1, pinIN2 };
const int pinMotorL[3] = { pinENB, pinIN3, pinIN4 };


// servos
const int pinSrvL = 9 ;
const int pinSrvR = 11 ;

// HC-06 (bluetooth)
const int rxPin = 12;
const int txPin = 10;
const int BTLEDPin = 13 ; // for BT visual signaling


Servo Lservo;  // Left servo
Servo Rservo;  // Right


SoftwareSerial BTserial(rxPin, txPin); // RX | TX
// Connect the HC-06 TX to the Arduino RX on pin rxPin. 
// Connect the HC-06 RX to the Arduino TX on pin txPin
// use a voltage divider or a level converter


bool LOGGING = false ;   // enable logging to GUI
const bool RUN = true ;  // enable motors

/// timming
unsigned long t_now = 0;
unsigned long t_last = 0;
unsigned long t_last_log = -1 ; 


/////////////////////////////////////////////////////////////////
int pos = 105;    // leg pos

// motor velocities
int max_speed = 250 ; // 120 
int min_speed = 70 ; // 100 90 
float crash_ang = 60.0 ;

// target angle and safe zone. depends on, e.g. kind of batteries used.
// todo: nest another PID to find the target angle (is the one that gets null ang. vel.)
 // for pos = 105:
float target =  6.0 ;  // 4.9 ;  // lipo 400mAh batts
//float target = 7.0 ;  // standard duracell
float hist = 0.1 ; // degrees

// kP = 1.5 kI = 0.01 kD = .04
// pid controler parameters
float pid_P = .32;// .81 ; // 3.0; 5.5
float pid_I = 0.001; // 0.02  .002
float pid_D = .17; // .002 ;
/////////////////////////////////////////////////////////////////

float int_error = 0 ;

//const int motor_delay = 100;
const float dg_rad = 180/M_PI ;

enum CMD : byte { 
  NONE = 0,
  LIGHTS_ON = 5,
  LIGHTS_OFF = 6,
  END = 10, // do not use
  SHOW_PARAMS = 12, 
  SET_1PARAM = 11,
  STATUS = 20,
  START_LOGGING = 30,
  STOP_LOGGING = 35,
  LEAN = 40,
  HALT = 100  
} ;

enum PARAM : byte { 
  KP = 1,
  KI = 2,
  KD = 3,
  TA = 4,
  LS = 5,
  HS = 6,
  POS = 10,
  VEL = 20
};
String param_name[] = {"", "KP", "KI", "KD", "TA", "LS", "HS", "POS", "VEL"} ;

enum COMMS : byte {
  WAITING,
  READING_LEN,
  READING_SET,
  READING_CMD,
  READING_CRC,
  RUN_CMD_BUFFER
};

COMMS COMMST = WAITING ;


const int max_param_buffer_len = 255 ;
byte param_buffer[max_param_buffer_len] ;
int buffer_pos = 0; 
int buffer_expected_len ;
unsigned long timeout ;


const char LF = 10; 
const char CMD = 'C'; 

const int n_params = 5 ;

const int data_len = 5;
float log_point[data_len] ; // t, angle, fbp, fbi, fbd, speed  
// 

//const int LIMIT = 1500;
//int collar( int x) { return x<-LIMIT? -LIMIT : x>LIMIT? LIMIT: x ; }

void Wire_setup() {

    Serial.println(F("Wire setup"));
     
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
}

void IMU_setup() {
      // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // gyro offsets computed with MPU6050_calibration.ino 
    mpu.setXGyroOffset(95); 
    mpu.setYGyroOffset(10); 
    mpu.setZGyroOffset(-10);  
    mpu.setXAccelOffset(-2441);
    mpu.setYAccelOffset(-2070);
    mpu.setZAccelOffset(991); 
    
    // 0 = +/- 250 degrees/sec 
    // 1 = +/- 500 degrees/sec 
    // 2 = +/- 1000 degrees/sec 
    // 3 =  +/- 2000 degrees/sec
    mpu.setFullScaleGyroRange(0); 
    // 0 = +/- 2g 
    // 1 = +/- 4g 
    // 2 = +/- 8g 
    // 3 =  +/- 16g 
    mpu.setFullScaleAccelRange(0); 
     
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(pinint), dmpDataReady, RISING);
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

void IO_setup() {
  // IMU interrupt
  pinMode(pinint, INPUT);
   // motors
  pinMode(pinIN1, OUTPUT);
  pinMode(pinIN2, OUTPUT);
  pinMode(pinENA, OUTPUT);
  pinMode(pinIN3, OUTPUT);
  pinMode(pinIN4, OUTPUT);
  pinMode(pinENB, OUTPUT);
  // BT tx, rx, and led pins
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  pinMode(BTLEDPin, OUTPUT); 
  // servos
  Lservo.attach(pinSrvL);  // attaches the Left servo
  Rservo.attach(pinSrvR);  // Right
}


void comms_setup() {

  Serial.begin(115200); 
  Serial.println(F("Pair with me!"));
  // HC-06 default serial speed is 9600, this unit was setup for 115200
  BTserial.begin(115200);
  BTserial.setTimeout(100);
  
}

void T_setup() {
    
  for(int p = 0; p<=data_len; ++p ) log_point[p] = 0.0; 
  t_now = millis() ;
  t_last = t_now - 500  ;
  timeout = t_now + 1500 ;

}

void IamAlive() {
  
  digitalWrite(BTLEDPin, HIGH);  
  delay(1000);
  digitalWrite(BTLEDPin, LOW);
      
  zero(60) ;
  delay(1000); 
  zero(pos);

  //Lservo.detach();  // detach servos for tests or similar, the valkyrie won't support itself!
  //Rservo.detach();  // (the servos catch lots of EMI from the PCM for the motors: use RF chokes!)

  Serial.print(F(   "kP = ")) ;  Serial.print( pid_P, 3 ) ; 
  Serial.print(F( ", kI = ")) ;  Serial.print( pid_I, 3 ) ; 
  Serial.print(F( ", kD = ")) ;  Serial.print( pid_D, 3 ) ; 
  Serial.print(F( ", TA = ")) ;  Serial.println( target, 3 ) ; 

}


void setup() {

  comms_setup();
  IO_setup();
  Wire_setup();
  IMU_setup();
  T_setup();
  // demonstrate it:
  IamAlive() ;

}
  
boolean lflip = true ;

void loop(){
  t_now = millis() ;
  // heartbeat
  boolean flip = int(t_now/500)%2; 
  if (flip != lflip ) { BTserial.println(F("HB")); lflip = flip; digitalWrite(BTLEDPin, lflip);} 
  // 
  //comms_loop();
  imu_loop() ;
  t_last = t_now ;
}




void flush_out()
{
  Serial.println(F("Flushing!")) ;
  while(BTserial.read()>=0); // && !mpuInterrupt)  ; 
  //Serial.println("Done!") ;
}

// data = { 0x43, 0x0B, 0x0C, <p1>, <v1>, <v2>, <v3>, <v4>, <4 bytes crc>, 0x00 }   
void set_param(byte* data){
  //Serial.println(F("set_param"));
   int param = data[3] ; 
   long vint = data[4] << 24 | data[5] << 16 | data[6] << 8 | data[7] ;
   float value = vint/1000.0;
   set_param( param, value) ;
  }

void set_param( int param, float value) {

    switch (param) {
      case KP: pid_P     = value ; break ;
      case KI: pid_I     = value ; break ;
      case KD: pid_D     = value ; break ;
      case TA: target    = value ; break ;
      case LS: min_speed = value ; break ;
      case HS: max_speed = value ; break ;      
      default:      
          BTserial.println( "WP " + String(param, HEX)) ;
          return ;  
    }    
    String msg = "SP " + param_name[param] + " = " + String(value, 3) ;
    Serial.println( msg ) ;
    BTserial.println( msg ) ;    
}

void reset_buffer(){
  buffer_pos = 0;
  buffer_expected_len = 0 ;
}

// COMMS
int expected_data_len( byte c ) {
  //Serial.print(F("expected len, read: "));
  //Serial.println(c, HEX);
  if (c>max_param_buffer_len) {
    flush_out();
    BTserial.println( F("R:BD0")) ;
    return WAITING ;
  }
  buffer_expected_len = (int)c ;
  param_buffer[buffer_pos++] = c ; // crc covers len too
  return READING_SET;
}


// COMMS
int build_param_data( byte c ) {
   //Serial.print(F("[")) ; 
   //Serial.print( buffer_pos) ; Serial.print( F("/" ));
   //Serial.print( buffer_expected_len) ; Serial.print( F("] ") );
   //Serial.print(c, HEX) ; 
   if (buffer_pos < buffer_expected_len - 1 ) {
      param_buffer[buffer_pos++] = c ; 
      //Serial.println(F(" Added")) ; 
      return COMMST;
   }
   if (buffer_pos > buffer_expected_len - 1 ) {
       flush_out();
       reset_buffer();
       BTserial.println( F("R:??LEN")) ;
       Serial.println(F(" EXTRA!")) ; 
       return WAITING ;       

   }
   // buffer_pos == expected_data_len - 1
   //Serial.println(F(" LAST ONE")) ; 
   param_buffer[buffer_pos++] = c ; 
   param_buffer[buffer_pos] = 0 ; // null terminated, now buffer_pos == expected_data_len 
   
   if (check_crc( param_buffer, buffer_pos )) {
       Serial.println(F("CRC Passed")) ; 
       return RUN_CMD_BUFFER ; 
   }
   else {
       flush_out();
       BTserial.println( F("R:CRC")) ;
       Serial.println(F("Wrong CRC")) ;
       return WAITING ;       
   }
}


int current_cmd ;

void start_buffering(byte c){
   Serial.print(F("Buffering [")) ;Serial.print(c,HEX) ; Serial.println(F("]")) ;
   buffer_pos = 0 ;
   param_buffer[buffer_pos++] = CMD ;
   param_buffer[buffer_pos++] = c ; // crc covers cmd header too
   current_cmd = c ;
}

// COMMS
int read_cmd( byte c) {
      Serial.print(F("cmd: [")); Serial.print(c, HEX); ; Serial.println(F("]")) ;
      switch (c) {
        case STATUS :      
        case START_LOGGING :
        case STOP_LOGGING :      
        case LIGHTS_ON :
        case LIGHTS_OFF : 
        case SHOW_PARAMS :
          start_buffering(c);
          buffer_expected_len = 6 ; // "C" + CMD + 4 CRC bytes 
          COMMST = READING_CRC ;
          break ;
        case SET_1PARAM :
          start_buffering(c);
          COMMST = READING_LEN ;
          break ;
        default: 
          BTserial.println(F( "E: ?!") ) ;
          flush_out();      
          COMMST = WAITING ;   
      }
      //BTserial.flush();
      return COMMST ;
}

void telemetry() {    
    if (LOGGING && t_now - t_last_log > 200) {
        BTserial.print( F("LOG:") );
        BTserial.print( t_now ); BTserial.print( F(":" ));
        for ( int p=0; p <= data_len; ++p ) { 
           BTserial.print( log_point[p], 2);
           BTserial.print( (p==data_len)? "\n" : ":");
        }
        t_last_log = t_now ;
        //Serial.println(log_point[1]);
    }
}


COMMS last_COMMST=(COMMS)-1;

// process incomming bytes, one by one
void comms_loop()
{  
    //digitalWrite(BTLEDPin, HIGH);
    
    //Serial.println("Comms loop t=" + String(t_now) );
    if (COMMST!=last_COMMST) {
      Serial.println("Comms Status: " + String(COMMST));
      last_COMMST = COMMST;
    }
        
    if (COMMST != WAITING && t_now>timeout) {
       Serial.println( "t_now: " + String(t_now));
       Serial.println( "timeout: " + String(timeout));
       flush_out();    
       BTserial.println( F("R:TO") ) ;
       timeout = t_now + 1000 ;
       COMMST = WAITING ;        
    }

    if (BTserial.available())  { 
      delay(1); // wait for buffer fill up 
      char c = BTserial.read() ;
      //Serial.print(F("char read: ")); Serial.println(c, HEX); 
      switch (COMMST) {
        
        case READING_CMD: COMMST = (COMMS)read_cmd( c) ;           break ;
        case READING_CRC: COMMST = (COMMS)build_param_data(c) ;    break ;
        case READING_LEN: COMMST = (COMMS)expected_data_len(c);    break ;
        case READING_SET: COMMST = (COMMS)build_param_data(c);     break ;
        //case RUN_CMD_BUFFER:     COMMST = run_cmd(current_cmd, param_buffer);    break ;
        case WAITING: if (c == CMD) { COMMST = READING_CMD ;  timeout = t_now + 1000 ; break ;}
        default: Serial.print("{"); Serial.print(c, HEX); Serial.println("}");
      }
    }
    
    if (COMMST == RUN_CMD_BUFFER ) {
        COMMST = (COMMS)run_cmd(current_cmd, param_buffer); 
    }
    //digitalWrite(BTLEDPin, LOW);
}

// COMMS
int run_cmd( byte cmd, byte* data) {
      Serial.print(F("running cmd [")); Serial.print(cmd, HEX); ; Serial.println(F("]")) ;
      switch (cmd) {
        case STATUS :
          BTserial.println( F("OK: Ready!")) ; 
          COMMST = WAITING;
          break ;        
        case START_LOGGING :
          BTserial.println( F("OK: TL on")) ;
          LOGGING = true; 
          COMMST = WAITING;   
          break ;
        case STOP_LOGGING :
          BTserial.println( F("OK: TL off")) ; 
          LOGGING = false;
          COMMST = WAITING;    
          break ;        
        case LIGHTS_ON :
          BTserial.println( F("OK: L on")) ;
          digitalWrite(BTLEDPin, HIGH);
          COMMST = WAITING;
          break ;
        case LIGHTS_OFF : 
          BTserial.println( F("OK: L off")) ;
          digitalWrite(BTLEDPin, LOW);
          COMMST = WAITING;
          break ;
        case SHOW_PARAMS : 
          {
          // sprintf in the libstc version lite, used for linking in Arduino IDE, do not support %f :( 
          //char msg[64];
          //sprintf(msg, "OK: kP = %6.4f, kI = %6.4f, kD = %6.4f, target = %6.4f\n", pid_P, pid_I, pid_D, target) ; 
          //BTserial.print(msg) ;
          BTserial.print( F("OK: ")) ; 
          BTserial.print( F(  "kP = ")) ;  BTserial.print( pid_P, 4) ; 
          BTserial.print( F(", kI = ")) ;  BTserial.print( pid_I, 4 ) ; 
          BTserial.print( F(", kD = ")) ;  BTserial.print( pid_D, 4 ) ;
          BTserial.print( F(", target = ")) ;  BTserial.println( target, 4 ) ;
          } 
          COMMST = WAITING;
          break ;
        case SET_1PARAM :
          BTserial.println( F("OK: set")) ;
          set_param( data );
          COMMST = WAITING;
          break ;
        default: 
          BTserial.print( F("E: ")) ; BTserial.print( cmd, HEX) ; BTserial.println(F("?!")) ; 
          COMMST = WAITING;
          break ; 
      }
      BTserial.println(F(".")) ; // read buffer
      return COMMST ;
}


void imu_loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // do other stuff meanwhile
        comms_loop() ;
        telemetry() ;
    }
    
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        BTserial.println(F("FIFO overflow!"));
        Serial.println(F("FIFO overflow!"));
        return ;
    }
    // otherwise, check for DMP data ready interrupt (this should happen frequently) 
    if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        waiting_for_FIFO = true;
        while (fifoCount < packetSize ) {
            fifoCount = mpu.getFIFOCount(); 
            //Serial.print(".");
        }
        waiting_for_FIFO = false;
        //Serial.println("*");
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        mpu.dmpGetGyro( &gyro, fifoBuffer) ;
   
        //// 
        long inc_t = t_now - t_last ;
        const float alpha = ypr[1] * dg_rad ; // pitch in dg, pve means head's down
        const float w = - gyro.y / 131.0 ; // 1 dg/s = 131 in register. d_e/d_t = - w
        const float error = target - alpha ; 
        int_error += error * (float)inc_t / 1000.0 ; // per s
        
        //const float fb = pid_P * error  + pid_D * gyro.y * 180/M_PI + pid_I * int_error ;
        const float fbp = pid_P * error ;
        const float fbi = pid_I * int_error ;
        const float fbd = pid_D * w; 
        const float fb = fbp + fbi + fbd ;
        // positive error means go backwards
        if (fabs(error) > hist ) {
          const float speed = -constrain( 
                              fb > 0 ? 
                              map( fb, 0, crash_ang, min_speed, max_speed  ) :
                              map( fb, -crash_ang, 0, -max_speed, -min_speed  ),
                              -max_speed, max_speed );

          log_point[0] = alpha; 
          log_point[1] = error; 
          log_point[2] = fbp ;
          log_point[3] = fbi;
          log_point[4] = fbd ;
          log_point[5] = speed ;

//          log_point[0] = alpha; 
//          log_point[1] = error; 
//          log_point[2] = int_error ;
//          log_point[3] = w ;
//          log_point[4] = fb ;
//          log_point[5] = speed ;

            
          moveForward( speed ) ;
          
        }
        else { 
          int_error = 0 ; 
          
          log_point[0] = alpha; 
          log_point[1] = error; 
          log_point[2] = 0 ;
          log_point[3] = 0;
          log_point[4] = 0 ;
          log_point[5] = 0 ;
          
          fullStop();
        }
    }
}


void test(int pos, int speed) { 
    zero(pos);
    moveForward( speed ) ;  
}

void zero(int pos) {
    Serial.println("pos: " + String(pos));
    Lservo.write(180-pos); // goto pos mark
    Rservo.write(pos-11);  // both legs, 180 and 11 are the limits of each side
    delay(15);             // wait 15ms for the servos to reach the position
  }

void sweep() {
  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    Lservo.write(180-pos);            
    Rservo.write(pos);              
    delay(15);                      
  }
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    Lservo.write(180-pos);  
    Rservo.write(pos);      
    delay(15);              
  }
}



void debug_motors() { 
       const int step = 20 ;
       Serial.println (F("forward"));
       for (int speed = 200; speed; speed-=step ) {    
        Serial.println (speed );
        moveForward(speed);
        delay(500);
       }
       Serial.println (F("backward" ));
       for (int speed = 200; speed; speed-=step ) {    
        Serial.println (speed );
        moveBackward(speed);
        delay(500);
       }
       Serial.println (F("right") );
       for (int speed = 200; speed; speed-=step ) {    
        Serial.println (speed );
        rotateRight(speed);
        delay(500);
       }
       Serial.println (F("left" ));
       for (int speed = 200; speed; speed-=step ) {    
        Serial.println (speed );
        rotateLeft(speed);
        delay(500);
       }

       fullStop();
       
}




///// motors 

// 
void motorBackward(const int pinMotor[3], int speed)
{
  if (!RUN) return ;
  if (speed < 0) return motorForward(pinMotor, -speed);
  digitalWrite(pinMotor[1], HIGH);
  digitalWrite(pinMotor[2], LOW);
 
  analogWrite(pinMotor[0], speed);
}

//  
void motorForward(const int pinMotor[3], int speed)
{
  if (!RUN) return ;
  if (speed < 0) return motorBackward(pinMotor, -speed);
  digitalWrite(pinMotor[1], LOW);
  digitalWrite(pinMotor[2], HIGH);
 
  analogWrite(pinMotor[0], speed);
}

void moveForward( int speed)
{
      if (speed <0) { moveBackward(-speed); return ; }
      motorForward(pinMotorR, speed);
      motorForward(pinMotorL, speed) ;
}


void moveBackward( int speed)
{
      if (speed <0) { moveForward(-speed); return ; }
      motorBackward(pinMotorR, speed);
      motorBackward(pinMotorL, speed) ;
}

void rotateLeft( int speed)
{
      motorBackward(pinMotorR, speed);
      motorForward(pinMotorL, speed) ;
}


void rotateRight(int speed)
{
      motorBackward(pinMotorL, speed);
      motorForward(pinMotorR, speed) ;
}

void turn( int speedL, int speedR)
{
      motorForward(pinMotorR, speedR) ;
      motorForward(pinMotorL, speedL) ;
}

void fullStop() 
{
  motorFullStop( pinMotorR); 
  motorFullStop( pinMotorL); 
  
}


void motorFullStop(const int pinMotor[3])
{
  digitalWrite(pinMotor[1], LOW);
  digitalWrite(pinMotor[2], LOW);
 
  analogWrite(pinMotor[0], 0);
}



float fscale( float originalMin, float originalMax, float newBegin, float newEnd, float inputValue, float curve){

  float OriginalRange = 0;
  float NewRange = 0;
  float zeroRefCurVal = 0;
  float normalizedCurVal = 0;
  float rangedValue = 0;
  boolean invFlag = 0;


  // condition curve parameter
  // limit range

  if (curve > 10) curve = 10;
  if (curve < -10) curve = -10;

  curve = (curve * -.1) ; // - invert and scale - this seems more intuitive - postive numbers give more weight to high end on output
  curve = pow(10, curve); // convert linear scale into logarithmic exponent for other pow function

  /*
   Serial.println(curve * 100, DEC);   // multply by 100 to preserve resolution  
   Serial.println();
   */

  // Check for out of range inputValues
  if (inputValue < originalMin) {
    inputValue = originalMin;
  }
  if (inputValue > originalMax) {
    inputValue = originalMax;
  }

  // Zero Refference the values
  OriginalRange = originalMax - originalMin;

  if (newEnd > newBegin){
    NewRange = newEnd - newBegin;
  }
  else
  {
    NewRange = newBegin - newEnd;
    invFlag = 1;
  }

  zeroRefCurVal = inputValue - originalMin;
  normalizedCurVal  =  zeroRefCurVal / OriginalRange;   // normalize to 0 - 1 float

  /*
  Serial.print(OriginalRange, DEC);  
   Serial.print("   ");  
   Serial.print(NewRange, DEC);  
   Serial.print("   ");  
   Serial.println(zeroRefCurVal, DEC);  
   Serial.println();  
   */

  // Check for originalMin > originalMax  - the math for all other cases i.e. negative numbers seems to work out fine
  if (originalMin > originalMax ) {
    return 0;
  }

  if (invFlag == 0){
    rangedValue =  (pow(normalizedCurVal, curve) * NewRange) + newBegin;

  }
  else     // invert the ranges
  {  
    rangedValue =  newBegin - (pow(normalizedCurVal, curve) * NewRange);
  }

  return rangedValue;
}

