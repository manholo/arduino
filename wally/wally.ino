#include <NewPing.h> // uses timer 2
#include <NewTone.h> // standard tone lib (timer 2, PWM pins 3, 11) conflicts with NewPing lib, so use this (it uses timer 1)
// timer 0 controls pins 5, 6 (millis() func uses this, but it is safe to use for PWM at the same time)
// timer 1 controls pins 9, 10
// timer 2 controls pins 3, 11


//#define DEBUG 
//#define SONG_DEBUG  
//#define DANCE_DEBUG  
const int RUN = 1 ;

// hc sr04 sonar
const int pinTrg  = 12 ;
const int pinEcho = 12 ; // 13
const int MAX_DISTANCE = 200 ;
const int CLOSE = 15 ;

// leds
const int pinGreen = 3; // PWM was 9, better 3
const int pinYellow = 9;  // PWM was 3, better 9
const int pinRed =  11 ;    // PWM was 6, better 11
const int pinBlue = 13;    // no PWM

int led[] = { pinGreen, pinYellow, pinRed } ;
 
// buzzer
const int pinBuzz = 10; // was 5 better 10 

// motors
const int pinENA = 5; // PWM was 10, better 5
const int pinIN1 = 7;
const int pinIN2 = 8;

const int pinIN3 = 2;  
const int pinIN4 = 4 ; //
const int pinENB = 6; // PWM was 11 better 6

const int MBIAS = 30 ; // motor A over B


// LDR
const int pinLDR = 0 ;


const int waitTime = 2000;  // espera entre fases
const int bwdTime = 500;    // backward walk time

const int speed = 200;      //velocidad de giro
 
const int pinMotorA[3] = { pinENA, pinIN1, pinIN2 };
const int pinMotorB[3] = { pinENB, pinIN3, pinIN4 };

// millis per beat 
const int mpsb = 200 ;
// melody
const int song_len = 14 ;
int current_note = -1 ;
int next_beat = 0 ;
int beat_len ;
int note[] = {
  147, 131, 123, 110, 0, 0, 82,
  98, 0, 0, 123, 110, 0, 0 
};

int dur[] = {
  2, 1, 1, 1, 1, 1, 1,
  1, 1, 1, 1, 1, 1, 2 
};

// dance motor speed, negative means backwards

int danceA[]   = { 0, 150, -105, 100, 0, -150, 180,   90 } ;
int danceB[]   = { 0, 100, -150, 90,  0, -150,  90,  180 } ;
int dance_dur[] = { 5,  50,  50, 25, 1,   25,   25,   25 } ;
const int dance_len = 8;
int current_step = -1 ;  
int next_step = 0 ;

// mood
const int HAPPY = 0 ;
const int ANGRY = 1 ;
const int COLLISION = 3 ;
const int ESCAPE = 4 ;
const int LIGHTSON = 5 ;
const int LIGHTSOFF = 6 ;
const int TIRED = 7 ;
const int RESTING = 8 ;
const int SLEEPT = 9 ;
const int DANZING = 10 ;
const int AVOID = 11 ;
const int DEBUGING = 255 ;

int collisions = 0 ;
int tireness = 0 ;
unsigned long nap = 0 ;


//int state = DEBUGING ;
int state =  RESTING ; // be happy !

const int OFF = 0 ;
const int ON = 1 ;
int lights = OFF ;

NewPing sonar(pinTrg, pinEcho, MAX_DISTANCE);

void setup()
{
  Serial.begin(9600);      // open the serial port
  // leds
  pinMode(pinGreen, OUTPUT);
  pinMode(pinYellow, OUTPUT);
  pinMode(pinRed, OUTPUT);
  pinMode(pinBlue, OUTPUT);
  // sonar
  pinMode(pinTrg, OUTPUT);
  pinMode(pinEcho, INPUT);
  // buzzer
  pinMode(pinBuzz, OUTPUT);
  
  // motors
  pinMode(pinIN1, OUTPUT);
  pinMode(pinIN2, OUTPUT);
  pinMode(pinENA, OUTPUT);
  pinMode(pinIN3, OUTPUT);
  pinMode(pinIN4, OUTPUT);
  pinMode(pinENB, OUTPUT);
  // for random
  randomSeed(analogRead(0));
  // flash some lights
  flash_with_sound(440, 50); flash_with_sound(440*1.5, 50); flash_with_sound(440*3, 50);
  int b=song_len  ; while(b) beat_len += dur[--b] ;
  Serial.println( String("beat len: ") + beat_len);
}


void flash_with_sound(int note, int dur) {
    for ( int i = 1; i > -1 ; --i ) 
      for (int l = 0; l < 3 ; ++l) { 
        digitalWrite(led[l], i); 
        digitalWrite(pinBlue, i); 
        NewTone( pinBuzz, note*(l?l*1.5:1), dur * .6) ; delay(dur) ;
     }
}


void angry()
{
    if (random(0,1) ) rotateLeft(200) ; else rotateRight(200) ;
    for (int i = 1; i<7 ;++i) flash_with_sound(440 * i, 100);  
    collisions = 0 ;  
    tireness += 100 ;
}

void sing()
{
    int beat = int(millis()/mpsb)%beat_len ;
    if (beat == next_beat) {
        current_note = (current_note + 1)%song_len;
        if (note[current_note]) {
          NewTone( pinBuzz, note[current_note], mpsb * dur[current_note] * .6 ) ;
          digitalWrite(pinYellow, HIGH);
        }
        else  noNewTone(pinBuzz);
    next_beat = (beat + dur[current_note])%beat_len ;
    digitalWrite(pinYellow, LOW);
        
    #ifdef SONG_DEBUG
        Serial.print( String("beat: ") + beat); Serial.print( String(", next beat: ") + next_beat);
        Serial.print( String(", note: ") + current_note);
        Serial.print( String(": ") + note[beat] );
        Serial.println( String("/") + mpsb * dur[beat]);
   #endif
   }
 }

void dance()
{
    Serial.println ("Lets dance!" );
    int beat = int(millis()/mpsb)%beat_len ;
    if (beat == next_step) {
        current_step = (current_step + 1)%dance_len;
        motorForward(pinMotorA, danceA[current_step]);
        motorForward(pinMotorB, danceB[current_step]);  
        next_step = (beat + dance_dur[current_step])%beat_len ;    
    #ifdef DANCE_DEBUG
        Serial.print( String("beat: ") + beat); Serial.print( String(", next step: ") + next_step);
        Serial.print( String(", step: ") + current_step);
        Serial.print( String(" A : ") + danceA[current_step] );
        Serial.print( String(" B : ") + danceB[current_step] );
        Serial.println( String(" d : ") + dance_dur[current_step] );
   #endif
   
   }
    
   digitalWrite(pinRed, beat%2);  
   if (random(0,1000)<1) { angry(); tireness = 0; state = HAPPY ; }     
}



void happy()
{
      Serial.println("forward!");
      digitalWrite(pinRed, LOW);
      digitalWrite(pinGreen, HIGH); 
      moveForward(max(200 - tireness/10, 100));
      sing() ;
      if (millis()%2000 < 200) {
        if (collisions>0) collisions-- ;
        tireness ++ ;
      }
      if (random(0,1000)<5) state = DANZING ;
      //digitalWrite(pinYellow, LOW);      
}


void rest() {
      Serial.println("having a rest");  
      digitalWrite(pinGreen, LOW);
      digitalWrite(pinRed, LOW);
      digitalWrite(pinYellow, LOW);
      moveBackward(120);
      delay(bwdTime);
      if (random(0,1) ) rotateLeft(120) ; else rotateRight(120) ;
      delay(bwdTime*10);
      fullStop(pinMotorA);
      fullStop(pinMotorB);
      switch_lights(OFF) ;
      state = RESTING ;
      nap = millis () ;
}

void resting () {
    unsigned int passed = millis() - nap ;
    Serial.print(passed) ; Serial.println (" Z Z z ." );
    int level = int(passed / 5 + 256) % 512 - 256 ;
    #ifdef DEBUG
      Serial.println(String("Level: ") + abs(level));
    #endif
    analogWrite( pinGreen, abs(level) ); 
    tireness = tireness <= 0 ? 0  : tireness - 25 ;
    if (passed > 60000) {
       Serial.println (" OOAAAAA!!!" );
       flash_with_sound(440, 100);
       if (random(0,1) ) rotateLeft(120) ; else rotateRight(120) ;
       delay(bwdTime*10);
       fullStop(pinMotorA);
       fullStop(pinMotorB);
       
       tireness = 0 ; 
       nap = 0 ;
       state = HAPPY ; 
    }
}

void avoid_collision () {
      Serial.println("avoiding obstacle");
      collisions++;
      digitalWrite(pinGreen, LOW);
      digitalWrite(pinRed, HIGH);
      moveBackward(180);
      delay(bwdTime);
      fullStop(pinMotorA);
      fullStop(pinMotorB);
      state = AVOID ;
}

void avoid() {        
      escape();
      state = ESCAPE ;
}

void escape() {        
        if (random(0,1) ) rotateLeft(180) ; else rotateRight(180) ; 
        int wait = random(bwdTime) ;
        NewTone( pinBuzz, 1320 + random(-10,10)*55, wait) ;
        delay(wait);
        tireness += 5 ;
}


void switch_lights(int onoff)
{
  if (onoff) {
     Serial.println ("LIGHTS ON" );
     lights = ON ;
     digitalWrite(pinBlue, HIGH);
  }
  else {
     Serial.println ("LIGHTS OFF" );
     lights = OFF ;
     digitalWrite(pinBlue, LOW);
  }
}


void debugme() {
  sing(); 
  dance();
}

void debug_motors() { 
       const int step = 20 ;
       Serial.println ("forward" );
       for (int speed = 200; speed; speed-=step ) {    
        Serial.println (speed );
        moveForward(speed);
        delay(500);
       }
       Serial.println ("backward" );
       for (int speed = 200; speed; speed-=step ) {    
        Serial.println (speed );
        moveBackward(speed);
        delay(500);
       }
       Serial.println ("right" );
       for (int speed = 200; speed; speed-=step ) {    
        Serial.println (speed );
        rotateRight(speed);
        delay(500);
       }
       Serial.println ("left" );
       for (int speed = 200; speed; speed-=step ) {    
        Serial.println (speed );
        rotateLeft(speed);
        delay(500);
       }

       fullStop(pinMotorA);
       fullStop(pinMotorB);
       
}

 
void loop()
{
  int d,ldr ;
  int beat  ;
  d = ping() ; 
  ldr = analogRead(pinLDR) ;


  state = state == DEBUGING                                 ? DEBUGING  :
          state == RESTING && (d >  CLOSE || d == NO_ECHO)  ? RESTING   :
          state == DANZING && (d >  CLOSE || d == NO_ECHO)  ? DANZING   :
          state == AVOID                                    ? AVOID     :
          state == ESCAPE  &&  d <= CLOSE                   ? ESCAPE    :
          d <= CLOSE                                        ? COLLISION :
          collisions > 5                                    ? ANGRY     :
          ldr < 500 && lights == OFF                        ? LIGHTSON  : 
          ldr > 600 && lights == ON                         ? LIGHTSOFF :
          tireness > 200                                    ? TIRED     : 
      /*  d > CLOSE || d == NO_ECHO */                        HAPPY     ;  

  #ifdef DEBUG
  Serial.println (String("echo: ") + d );
  Serial.println (String("LDR: ") + ldr );
  Serial.println (String("tireness: ") + tireness );
  Serial.println (String("nap: ") + nap );
  Serial.print("# of collisions: ");  Serial.println( collisions ); 
  Serial.println (String("state: ") + state );
  #endif            
  
  switch (state) {
  case DEBUGING  : debugme() ; break ;
  case ANGRY     : angry() ; break ;
  case HAPPY     : happy() ; break ;
  case TIRED     : rest() ;  break ;
  case RESTING   : resting() ;  break ;
  case COLLISION : avoid_collision() ; break ; 
  case ESCAPE    : escape() ;  break ;
  case AVOID     : avoid() ;  break ;
  case LIGHTSON  : switch_lights(ON) ; break ;
  case LIGHTSOFF : switch_lights(OFF) ; break ;
  case DANZING   : sing() ; dance() ;  break ;
  default        : happy() ;     
  }
}


int ping()
{
      if (state == ESCAPE) digitalWrite(pinYellow, HIGH);
      const int uS = sonar.ping_median();
      const int d = uS / US_ROUNDTRIP_CM ;
      if (state == ESCAPE) digitalWrite(pinYellow, LOW);
      return d ;
}

// the head is mounted backwards, so flip fordwards and backwards
void motorBackward(const int pinMotor[3], int speed)
{
  if (!RUN) return ;
  if (speed < 0) return motorForward(pinMotor, -speed);
  digitalWrite(pinMotor[1], HIGH);
  digitalWrite(pinMotor[2], LOW);
 
  analogWrite(pinMotor[0], speed);
}
//  flipped
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
      if (speed <0) return moveBackward(-speed);
      motorForward(pinMotorA, speed);
      motorForward(pinMotorB, speed) ;
}


void moveBackward( int speed)
{
      if (speed <0) return moveForward(-speed);
      motorBackward(pinMotorA, speed);
      motorBackward(pinMotorB, speed) ;
}

void rotateLeft( int speed)
{
      motorBackward(pinMotorA, speed);
      motorForward(pinMotorB, speed) ;
}


void rotateRight(int speed)
{
      motorBackward(pinMotorA, speed);
      motorForward(pinMotorB, speed) ;
}


//// correct way
//void moveForward(const int pinMotor[3], int speed)
//{
//  digitalWrite(pinMotor[1], HIGH);
//  digitalWrite(pinMotor[2], LOW);
// 
//  analogWrite(pinMotor[0], speed);
//}
//
// // correct way 
//void moveBackward(const int pinMotor[3], int speed)
//{
//  digitalWrite(pinMotor[1], LOW);
//  digitalWrite(pinMotor[2], HIGH);
// 
//  analogWrite(pinMotor[0], speed);
//}
 
void fullStop(const int pinMotor[3])
{
  digitalWrite(pinMotor[1], LOW);
  digitalWrite(pinMotor[2], LOW);
 
  analogWrite(pinMotor[0], 0);
}
