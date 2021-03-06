
import java.util.*;
import java.io.*; 
import controlP5.*;
import processing.serial.*;
import themidibus.*; 
import javax.sound.midi.MidiMessage;
import static javax.swing.JOptionPane.*; // for the popups

MidiBus midibus; // The MidiBus

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

float kP = .0;
float kI = .00; 
float kD = .0; 
 
float minkP = 0 ;
float maxkP = 1000 ;

float minkI = 0 ;
float maxkI = 5000 ;

float minkD = 0 ;
float maxkD = 5000 ;

float minTA = -15 ;
float maxTA = 15 ;

float minSP = 0 ;
float maxSP = 250 ;

Textlabel alphaText;
Textlabel fpdText;
Textlabel[] text;
Slider taSlide;
Knob kPKnob;
Knob kDKnob;
Knob kIKnob;
Knob posKnob;
Knob targetKnob;
CheckBox lightscb, logcb, anlzcb, paniccb, resetcb;
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
  PANIC = 100,
  RESET = 255  ;
} ;

interface PARAM {
  char
  KP = 1,
  KI = 2,
  KD = 3,
  TA = 4,
  LS = 5,
  HS = 6,
  POS = 10,
  VEL = 20 ;
}
final int param_len = 5 ;
String param_names[] = {"", "KP", "KI", "KD", "TA", "LS", "HS", "POS", "VEL" } ;


final int data_len = 5 ;
String log_point_names[] = {"alpha", "error", "fbp", "fbi", "fbd", "speed" } ;
float log_point_scale[] = { .3, .3, 1, 10, 10, 0.15 } ;
color log_point_color[] = { color(200,200,200),
                            color(250,20,20),
                            color(20,20,250), color(200,0,250), color(60,200,250),
                            color(20,250,50) } ;

char last_cmd = CMD.NONE ;
byte[] last_payload ;
boolean waiting_line = false ;
long wait_timeout ;
long midi_debounce ; // do not send any param update from midi until some time has passed since last midi message

HashMap<String,String> profile ; // read all at ince

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
  println(String.format("req %s = %5.2f", param_names[p], value));
  last_payload = append_crc( append_int( String.format("C%c%c%c", CMD.SET_PARAM, 12, (byte)p ).getBytes(), int(value*1000))) ; // va;ue in bp
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
  println("!");
  valkyrie.write( last_payload );         
  waiting_line = true ;
  wait_timeout = millis() + 500 ;
}

void read_profiles()
{
  profile = new HashMap<String, String>();
  String lines[] = loadStrings("profiles.ini"); //<>//
  for (int i = 0 ; i < lines.length; i++) {
    String[] row = lines[i].split(":",0); 
    profile.put( row[0].trim(), row[1]);
  }
}

void write_profiles()
{
  List<String> lines = new ArrayList<String>();
  
  for ( String k : profile.keySet()) 
      lines.add( String.format("%s: %s", k, profile.get(k)))  ;
  saveStrings("profiles.ini", lines.toArray("".split("",0)));
}

void setup() {
  size(550, 650);
  smooth();
  noStroke();
  

  log = createWriter("telemetry.txt");
  
  midibus = new MidiBus(this, "nanoKONTROL", "");
  //midibus = new MidiBus(this,"SLIDER/KNOB", "");
  
   
  read_profiles() ; //<>//
  
  cp5 = new ControlP5(this);
       
  text = new Textlabel[data_len+1];
  
  for (int l =0 ; l<=data_len; ++l) {
        println(log_point_names[l]);
        text[l] = cp5.addTextlabel(log_point_names[l])
                 .setText("--")
                 .setPosition(70 + 60 * l,50)
                 .setColorValue(log_point_color[l])
                 .setFont(createFont("Georgia",20))
                  ;
  };
       

  // add a vertical slider for target angle
  cp5.addSlider("target_shade")
     .setPosition(50,300)
     .setSize(2,300)
     .setRange(maxTA, minTA)
     .setNumberOfTickMarks(31)
     //.setScrollSensitivity(.01)
     .setValue(0)
     ;  
  cp5.getController("target_shade")
     .getValueLabel()
     .alignX(ControlP5.LEFT_OUTSIDE)
     .setPaddingX(15)
     ;
  cp5.getController("target_shade")
     .getCaptionLabel()
     .hide()
     ;

  taSlide = cp5.addSlider("target")
               .setPosition(50,300)
               .setSize(20,300)
               .setRange(maxTA, minTA)
             //.setNumberOfTickMarks(201)
               .setScrollSensitivity(.001)
               .setValue(0)
               ;                              
  
  kPKnob = cp5.addKnob("kP")
               .setRange(minkP, maxkP)
               .setValue(kP)
               .setPosition(100, 280)
               .setRadius(40)
               .setNumberOfTickMarks(20)
               .setTickMarkLength(4)   
               .setDragDirection(Knob.VERTICAL)
               .setResolution(1000)
               //.setViewStyle(Knob.ARC)
               ;
  kPKnob.getValueLabel().setFont(createFont("Courier",18));
  
  kIKnob = cp5.addKnob("kI")
               .setRange(minkI, maxkI)
               .setValue(kI)
               .setPosition(220, 280)
               .setRadius(40)
               .setNumberOfTickMarks(20)
               .setTickMarkLength(4)               
               .setDragDirection(Knob.VERTICAL)
               .setResolution(1000)
               //.setColorValueLabel(color(255,255,20))
               ;
  kIKnob.getValueLabel().setFont(createFont("Georgia",18));           

  kDKnob = cp5.addKnob("kD")
               .setRange(minkD, maxkD)
               .setValue(kD)
               .setPosition(340, 280)
               .setRadius(40)
               .setNumberOfTickMarks(20)
               .setTickMarkLength(4)                              
               .setDragDirection(Knob.VERTICAL)
               .setResolution(1000)
               ;  
 kDKnob.getValueLabel().setFont(createFont("Georgia",18));
 
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
                
 anlzcb = cp5.addCheckBox("anlz")
                .setPosition(460, 440)
                .setSize(40, 40)
                .addItem("ANLZ", 0)
                ;  
 paniccb = cp5.addCheckBox("paniccb")
                .setPosition(460, 510)
                .setSize(40, 40)
                .addItem("PANIC", 0)
                ;   
 resetcb = cp5.addCheckBox("resetcb")
                .setPosition(460, 580)
                .setSize(40, 40)
                .addItem("RESET", 0)
                ;                  
 textarea = cp5.addTextarea("txt")
                  .setPosition(100, 440)
                  .setSize(340, 200)
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

  cp5.addScrollableList("connection")
     .setPosition(50, 20)
     .setSize(200, 100)
     .setBarHeight(20)
     .setItemHeight(20)
     .addItems(Serial.list())
     // .setType(ScrollableList.LIST) // currently supported DROPDOWN and LIST
     ;
                 
  cp5.addScrollableList("profile")
     .setPosition(300, 20)
     .setSize(100, 100)
     .setBarHeight(20)
     .setItemHeight(20)
     .addItems(profile.keySet().toArray("".split("",0))) // that's a String[]
     .setType(ScrollableList.DROPDOWN) // currently supported DROPDOWN and LIST
     ;
     
  cp5.addButton("Save")
     .setPosition(410,20)
     .setSize(30,20)
     ;

    cp5.addButton("Send")
     .setPosition(450,20)
     .setSize(30,20)
     ;
     
  MidiBus.list();

  
  //if (!valkyrie_connected) command( CMD.STATUS ) ;
 

}

boolean heart_beat = false ;
void draw() {
  background(background);

  if (heart_beat) fill(255,0,0); 
  else fill(0,0,0);
  ellipse(50,50,5,5);
  heart_beat = false ;
  if (valkyrie_connected && waiting_line && millis() > wait_timeout) resend() ;
}

 //<>//

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
      if (Float.isNaN(v)) v=0; else v/=100.0 ;
      plot.push(log_point_names[p], v * log_point_scale[p] );
      text[p].setText(str(v));  
    }

    return; 
  }
  
  print(response);
  
  if (response.startsWith("R")) { resend(); return; }
  if (response.startsWith("E")) { resend(); return; } //valkyrie.clear() ;
  if (response.startsWith(".")) { return ; } // for now, ignore
      
  switch (last_cmd) {
    case CMD.STATUS:
      if (response.startsWith("OK") && !valkyrie_connected ) { 
        valkyrie_connected = true ;
        command(CMD.SHOW_PARAMS); // get the params from valyrie at connection
        return ;
      }
    case CMD.PANIC:
    case CMD.RESET:
    case CMD.LIGHTS_ON:
    case CMD.LIGHTS_OFF:
    case CMD.START_LOGGING:
    case CMD.STOP_LOGGING:
    case CMD.SET_PARAM:
      //print(response);      
      break ;
    case CMD.SHOW_PARAMS:
      //print(response);
      if (response.startsWith("OK")) update_params(response.substring(4).split(",", 0)); //<>//
      break ;
    case CMD.NONE:  // nothing expected, may be a SET update, or a "." finalizer

      if (response.startsWith("SP")) { update_params(response.substring(3).split(",", 0)) ; break; }
    default : println ("what?");
   }
   last_cmd = CMD.NONE;
   waiting_line = false;
}


void connection(int n) {

  String portName = String.class.cast((cp5.get(ScrollableList.class, "connection").getItem(n).get("text")));
  println("connecting to ", portName);
  valkyrie = new Serial(this, portName, 115200);
  valkyrie.bufferUntil(LF); 
  valkyrie_connected = false ;
  command( CMD.STATUS ) ;
  
}


void kP(float theValue) {
  if (valkyrie_connected) {
    set_param( PARAM.KP, theValue/100.0 );
  } 
}

void kI(float theValue) {
  if (valkyrie_connected) {
    set_param( PARAM.KI, theValue/100.0 );
  }
}

void kD(float theValue) {
  if (valkyrie_connected) {
    set_param( PARAM.KD, theValue/100.0 );
  }
}


void target(float theValue) {
  if (valkyrie_connected) {
    set_param( PARAM.TA, theValue );
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

void paniccb(float[] a) {
  if (valkyrie_connected) {
    if (a[0]>0) command(CMD.PANIC); //else command(CMD.STATUS);
  }
}

void resetcb(float[] a) {
  if (valkyrie_connected) {
    command(CMD.RESET);
    resetcb.toggle(0);
  }
}
void anlz(float[] a) {
  //if (valkyrie_connected) {
    String[] cmd = { "python",  sketchPath("plot.py"),  sketchPath("telemetry.txt") } ;
    //exec(cmd);

    try {
      Runtime rt = Runtime.getRuntime(); 
      Process p = rt.exec(cmd);
      //int st = p.waitFor();
      BufferedReader input = new BufferedReader(new InputStreamReader(p.getErrorStream()));
      //println(st);
      String line = null;
      while ((line = input.readLine()) != null) {
         System.out.println(line);
      }
    }
    catch(Exception e) {
      e.printStackTrace();
    }

    //if (a[0]>0) launch("python plot.py");
  //}
}

void profile(int n) {

  String name = String.class.cast((cp5.get(ScrollableList.class, "profile").getItem(n).get("text")));
  println("loading profile: ", name, "\n", profile.get(name));
  update_params(profile.get(name).split(",", 0));
  kPKnob.setColorValueLabel(color(255,255,20));
  kIKnob.setColorValueLabel(color(255,255,20));
  kDKnob.setColorValueLabel(color(255,255,20));
               

}

void Save() {
   final String id = showInputDialog("Profile name:");  
   if (id == null || id.length() == 0) return;
   profile.put( id.trim(), 
                String.format("KP = %f, KI = %f, KD = %f, TA = %f, HS = %d, LS = %d",
                kPKnob.getValue()/100.0, kIKnob.getValue()/100.0, kDKnob.getValue()/100.0, 
                taSlide.getValue(), 150, 70));
   cp5.get(ScrollableList.class, "profile").setItems(profile.keySet().toArray("".split("",0)));
   println("New profile: ", id);
   write_profiles();
   cp5.get(ScrollableList.class, "profile").setLabel(id);
}

void update_params( String[] response ) {
 for (int p=0; p<response.length; ++p) {
   String[] p_v = response[p].split("=", 0);
   switch(p_v[0].trim().toUpperCase()) {
     case "KP": kPKnob.changeValue(100.0 * float(p_v[1])); break ;
     case "KI": kIKnob.changeValue(100.0 * float(p_v[1])); break ;
     case "KD": kDKnob.changeValue(100.0 * float(p_v[1])); break ;
     case "TA": taSlide.changeValue(float(p_v[1])); break ;
   }
 }
}

void controllerChange(int channel, int number, int value) {
  //println(String.format("CH%d, CTRL %d = %d", channel, number, value)) ;
  // Receive a controllerChange. No println() to cp5 console here! cp5 event dispatcher gets confused sometimes 
  // nanoKontrol on channel 15
  if (channel!=15) return ;
  switch (number) {
    case 1:  { float v = map(value, 0, 127, minkP, maxkP); kPKnob.setValue(v);  break; } // the knobs and slider will do the rest
    case 2:  { float v = map(value, 0, 127, minkI, maxkI); kIKnob.setValue(v);  break; }
    case 3:  { float v = map(value, 0, 127, minkD, maxkD); kDKnob.setValue(v);  break; }
    case 10: { float v = map(value, 0, 127, maxTA, minTA); taSlide.setValue(v); break; }
    case 11: { float v = map(value, 0, 127, minSP, maxSP);                    ; set_param(PARAM.HS, v); break; }
    case 12: { float v = map(value, 0, 127, minSP, maxSP);                    ; set_param(PARAM.LS, v); break; }
    case 19: logcb.toggle(0); logcb(logcb.getArrayValue()); break;
    case 20: lightscb.toggle(0); lightscb(lightscb.getArrayValue()); break;
    //case 28: command( CMD.STATUS ) ; break;
    case 29: command( CMD.SHOW_PARAMS ) ; break;
    case 38: if (value>0) command( CMD.STATUS ) ; break;
    case 41: paniccb.toggle(0); paniccb(paniccb.getArrayValue()); break;
  }
}

void keyPressed() {
  switch(key) {
    case('1'): command( CMD.STATUS ) ;break;
    case('2'): command( CMD.SHOW_PARAMS ) ;break;
    case('3'): /*change content of the Conection List */
      cp5.get(ScrollableList.class, "connection").setItems(Serial.list());
      break;
  //  case('3'): break;
  }
  
}