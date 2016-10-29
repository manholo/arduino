
import controlP5.*;
import processing.serial.*;

// code originally from https://wiki.wxwidgets.org/Development:_Small_Table_CRC

final int INITIAL_CRC = 0xFFFFFFFF;

// implements  
 int crc_table[] =
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
 
int update_crc( int crc, byte[] s)
{
  for (int i = 0; i < s.length; i++)
  {
    /* XOR in the data. */
    crc ^= s[i] & 0x0000ff; //println(String.format("<%H>", s[i]& 0x0000ff));
    /* Perform the XORing for each nybble. avoid sign extension */
    crc = (crc >>> 4) ^ crc_table[crc & 0x0f]; //println(String.format("%H", crc_table[crc & 0x0f] ));
    crc = (crc >>> 4) ^ crc_table[crc & 0x0f]; //println(String.format("%H", crc_table[crc & 0x0f] ));
  }
  return crc;
}
 
int compute_crc( byte[] s)
{
  /* initialize the crc -- start with this value each time you start a session. */
  return update_crc( INITIAL_CRC, s) ;
}

PrintWriter log;
ControlP5 cp5;

int background = color(0,0,0);
int bgcolor;           // Background color
int fgcolor;           // Fill color

float kP = .98;
float kI = .01; 
float kD = .5; 
 
float minkP = 0 ;
float maxkP = 5 ;

float minkI = 0 ;
float maxkI = 1 ;

float minkD = 0 ;
float maxkD = 2 ;

boolean heart_beat = false ;

Textlabel alphaText;
Textlabel fpdText;
Textlabel[] text;
Knob kPKnob;
Knob kDKnob;
Knob kIKnob;
Knob posKnob;
Knob targetKnob;
CheckBox lightscb, logcb;
Textarea textarea;
Chart plot;
Println console;

Serial valkyrie; 
boolean valkyrie_connected = false ;

int LF = 10;   

interface CMD { 
  char
  NONE = 0,
  LIGHTS_ON = 5,
  LIGHTS_OFF = 6,
  SHOW_PARAMS = 12, 
  SET_PARAM = 11,
  STATUS = 20,
  START_LOGGING = 30,
  STOP_LOGGING = 35,
  LEAN = 40,
  HALT = 100  ;
} ;

interface PARAM {
  char
  KP = 1,
  KI = 2,
  KD = 3,
  POS = 10,
  VEL = 20 ;
}
final int param_len = 5 ;
String param_names[] = {"", "KP", "KI", "KD", "POS", "VEL" } ;


final int data_len = 5 ;
String log_point_names[] = {"alpha", "error", "fbp", "fbi", "fbd", "speed" } ;
float log_point_scale[] = { 1, 1, 1, 10, 100, 0.1 } ;
color log_point_color[] = { color(200,200,200),
                            color(250,20,20),
                            color(20,20,250), color(200,0,250), color(60,200,250),
                            color(20,250,50) } ;

char last_cmd = CMD.NONE ;
byte[] last_payload ;
boolean waiting_line = false ;
long wait_timeout ;


byte[] append_byte( byte[] payload, int p ) {
  return append ( payload, (byte)  ( p & 0xFF) );
 
}

byte[] append_int( byte[] payload, int p ) {
  payload = append( payload , (byte)(( p >>> 24 ) & 0xFF));
  payload = append( payload , (byte)(( p >>> 16 ) & 0xFF));
  payload = append( payload , (byte)(( p >>>  8 ) & 0xFF));
  payload = append( payload , (byte) ( p & 0xFF));
  //println(payload);
  return payload;
}

byte[] append_crc( byte[] payload ) {
  return append_int( payload, compute_crc(payload));
}

void command( char cmd ) {
  last_payload = append_crc( String.format("C%c", cmd ).getBytes() );
  last_cmd = cmd ;
  resend();
}

void set_param( int p, float value ) {
  last_payload = append_crc( append_int( String.format("C%c%c%c", CMD.SET_PARAM, 12, (byte)p ).getBytes(), int(value * 1000))) ;
  last_cmd = CMD.SET_PARAM ;
  resend();
}

void printhex( byte[] s){
 for (int i = 0; i < s.length; ++i ) {
   print( String.format(" %02X", (byte)s[i]) );
 }
 println("");
}
void resend( ) {
  printhex(last_payload);
  valkyrie.write( last_payload );         
  waiting_line = true ;
  wait_timeout = millis() + 500 ;
}

void setup() {
  size(550, 650);
  smooth();
  noStroke();

  log = createWriter("telemetry.txt");
  // Print a list of the serial ports, for debugging purposes:
  printArray(Serial.list());
  //
  String portName = "/dev/tty.VALKYRIE-DevB"  ; // Serial.list()[5]; 
  valkyrie = new Serial(this, portName, 115200);
  valkyrie.bufferUntil(LF); 
  
  cp5 = new ControlP5(this);
  text = new Textlabel[data_len+1];
  
  for (int l =0 ; l<=data_len; ++l) { //<>//
        println(log_point_names[l]);
        text[l] = cp5.addTextlabel(log_point_names[l])
                 .setText("--")
                 .setPosition(70 + 60 * l,50)
                 .setColorValue(log_point_color[l])
                 .setFont(createFont("Georgia",20))
                  ;
  };
       
  // add a vertical slider for target angle
  cp5.addSlider("target")
     .setPosition(50,300)
     .setSize(20,300)
     .setRange(-10,10)
     .setNumberOfTickMarks(21)
     .setValue(0)
     ;                              
                                
  kPKnob = cp5.addKnob("kP")
               .setRange(minkP, maxkP)
               .setValue(kP)
               .setPosition(100, 300)
               .setRadius(50)
               .setNumberOfTickMarks(20)
               .setTickMarkLength(4)   
               .setDragDirection(Knob.VERTICAL)
               .setResolution(1000)
               //.setViewStyle(Knob.ARC)
               ;
  kIKnob = cp5.addKnob("kI")
               .setRange(minkI, maxkI)
               .setValue(kI)
               .setPosition(220, 300)
               .setRadius(50)
               .setNumberOfTickMarks(20)
               .setTickMarkLength(4)               
               .setDragDirection(Knob.VERTICAL)
               .setResolution(1000)
               ;
  kDKnob = cp5.addKnob("kD")
               .setRange(minkD, maxkD)
               .setValue(kD)
               .setPosition(340, 300)
               .setRadius(50)
               .setNumberOfTickMarks(20)
               .setTickMarkLength(4)                              
               .setDragDirection(Knob.VERTICAL)
               .setResolution(1000)
               ;  

 //checkbox = cp5.addCheckBox("checkBox")
 //               .setPosition(460, 300)
 //               .setSize(40, 40)
 //               .setItemsPerRow(1)
 //               .setSpacingColumn(30)
 //               .setSpacingRow(20)
 //               .addItem("LIGHTS", 0)
 //               .addItem("LOG", 0)
 //               ;  
 
 
 lightscb = cp5.addCheckBox("lightscb")
                .setPosition(460, 300)
                .setSize(40, 40)
                .addItem("LIGHTS", 0)
                ;  
 
 logcb = cp5.addCheckBox("logcb")
                .setPosition(460, 370)
                .setSize(40, 40)
                .addItem("LOG", 0)
                ;  
 
  textarea = cp5.addTextarea("txt")
                  .setPosition(100, 430)
                  .setSize(340, 210)
                  .setFont(createFont("", 10))
                  .setLineHeight(14)
                  .setColor(color(200))
                  .setColorBackground(color(60, 100))
                  .setColorForeground(color(255, 100));
  ;
  console = cp5.addConsole(textarea);//
  plot = cp5.addChart("error plot")
               .setPosition(50, 50)
               .setSize(450, 200)
               .setRange(-30, 30)
               .setColorCaptionLabel(color(40))
               .setView(Chart.LINE) 
               ;          

  plot.getColor().setBackground(color(60, 100));
  plot.setStrokeWeight(1.5);
  for (int p=0; p<= data_len; ++p) {
    plot.addDataSet(log_point_names[p]);
    plot.setColors(log_point_names[p], log_point_color[p]);
    plot.setData(log_point_names[p], new float[100]);
  }
  
  
  
  if (!valkyrie_connected) command( CMD.STATUS ) ;
 

}

void draw() {
  background(background);

  if (heart_beat) fill(255,0,0); 
  else fill(0,0,0);
  ellipse(50,50,5,5);
  heart_beat = false ;
  //if (valkyrie_connected && waiting_line && millis() > wait_timeout) resend() ;
}


void serialEvent(Serial valkyrie) {
  // read response from serial port (valkyrie)
  String response = valkyrie.readStringUntil(LF) ;
  //if (response.length()==0) return ;
  if (response.startsWith("HB")) {    
    heart_beat = true;
    return;
  }
  if (response.startsWith("LOG")) {
    log.print(response); log.flush();
    String points[] = response.split(":") ; 
    for(int p=0; p<= data_len; ++p) {
      float v = float(points[p+2]);
      if (Float.isNaN(v)) v=0;
      plot.push(log_point_names[p], v * log_point_scale[p] );
      text[p].setText(points[p+2]);  
    }

    return; 
  }
  
  
  switch (last_cmd) {
    case CMD.STATUS:
      if (response.startsWith("OK")) valkyrie_connected = true ;
    case CMD.LIGHTS_ON:
    case CMD.LIGHTS_OFF:
    case CMD.START_LOGGING:
    case CMD.STOP_LOGGING:
    case CMD.SHOW_PARAMS:
      print(response);
      
      if (response.startsWith("R")) { resend(); break; }
      if (response.startsWith("E")) { resend(); break; } //valkyrie.clear() ;
      
      last_cmd = CMD.NONE;
      waiting_line = false;
      break ;
    case CMD.SET_PARAM:
      print(response);
      if (response.startsWith("OK") ){
         last_cmd = CMD.NONE;
         waiting_line = false;
         break ;
      } 
      resend();
      break ;
    case CMD.NONE: 
      print(response);
      waiting_line = false;
      break ;
    default : println ("what?");
  }
}

void kP(float theValue) {
  if (valkyrie_connected) {
    println("req KP = " + theValue);
    set_param( PARAM.KP, theValue );
  } 
}

void kI(float theValue) {
  if (valkyrie_connected) {
    println("req KI = " + theValue);
    set_param( PARAM.KI, theValue );
  }
}

void kD(float theValue) {
  if (valkyrie_connected) {
    println("req KD = " + theValue);
    set_param( PARAM.KD, theValue );
  }
}

void lightscb(float[] a) {
  if (valkyrie_connected) {
    if (a[0]>0) command(CMD.LIGHTS_ON); else command(CMD.LIGHTS_OFF);
  }
}

void logcb(float[] a) {
  if (valkyrie_connected) {
    if (a[0]>0) command(CMD.START_LOGGING); else command(CMD.STOP_LOGGING);
  }
}

void keyPressed() {
  switch(key) {
    case('1'): command( CMD.STATUS ) ;break;
    case('2'): command( CMD.SHOW_PARAMS ) ;break;
  //  case('2'): break;
  //  case('3'): break;
  }
  
}