  
// Example by Tom Igoe 
 
import processing.serial.*; 
 
Serial myPort;    // The serial port
PFont myFont;     // The display font
String inString;
String inStringA;  // Input string from serial port
String inStringB;
int lf = 10;      // ASCII linefeed 
 
void setup() { 
  size(1400,200); 
  // You'll need to make this font with the Create Font Tool 
  myFont = loadFont("Helvetica-Bold-48.vlw"); 
  textFont(myFont, 48); 
  // List all the available serial ports: 
  printArray(Serial.list()); 
  // I know that the first port in the serial list on my mac 
  // is always my  Keyspan adaptor, so I open Serial.list()[0]. 
  // Open whatever port is the one you're using. 
  myPort = new Serial(this, Serial.list()[3], 115200); 
  myPort.bufferUntil(lf); 
  inStringA ="";
  inStringB="";
  pixelDensity(displayDensity());
} 
 
void draw() { 
  background(0); 
  text("Golf Drone Machine", 10,50);
  text(inStringA, 10,100); 
  text(inStringB, 10,150); 
} 
 
void serialEvent(Serial p) { 
  
  inString = p.readString(); 
  if (inString.charAt(0) == 'M')
  {
    inStringA = inString;
  }
  
    if (inString.charAt(0) == 'R')
  {
    inStringB = inString;
  }
  
  //println(inString);
} 