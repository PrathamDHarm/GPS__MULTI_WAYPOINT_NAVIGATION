#include <SoftwareSerial.h>
#include <MechaQMC5883.h>
#include <Wire.h>
#include<Math.h>
#define PI 3.1415
#define Switch 7
MechaQMC5883 qmc;
            //for GPS
SoftwareSerial gps(4, 3);  //---RX=D4 and TX=D5
String latd[]={"19.199712","19.199711","19.199694","19.199568","19.199484","19.199416","19.199447","19.199381","19.199428"};
String logd[]={"84.745458","84.745678","84.745784","84.745802","84.745803","84.745701","84.745525","84.745339","84.745338"};
String latc, logc;
           //for ULTRASONIC
int Echo = A1;                      // Echo(P2.0)
int Trig =A0;                      //  Trig (P2.1)
int distance1;
           //for MOTOR
const int rp=10;
const int rn=11;
const int lp=6 ;
const int ln=9;
          //for CALCULATION
int inital_timestamp = millis();
String data[5];
int i = 0;
char c;
float bearing;
float heading;
float d;
float finalv;
int w=0;
int k=0;
int distance;
float deltalog,deltalat,latc1,logc1,latd1,logd1,deltalog1,deltalat1;
void setup()
{
  Serial.begin(9600);
  gps.begin(9600);
  gps.println("AT+CGNSPWR=1");
  delay(1000);
  gps.println("AT+CGNSSEQ=\"RMC\"");
  delay(1000);
  pinMode(rp, OUTPUT);
  pinMode(rn, OUTPUT);
  pinMode(lp, OUTPUT);

  pinMode(ln, OUTPUT);
  pinMode(Echo, INPUT);    // 
  pinMode(Trig, OUTPUT);   // 
  pinMode(Switch, INPUT_PULLUP);
  qmc.init();
  Wire.begin();
}

void loop()
{
  float turn_to_move_left_or_right = 0;
  float time_required_for_move_left_or_right = 0;
  Ultrasonic();
  delay(100);
  if(distance1>20){
  k=k+1;
  Serial.println("k=");
   Serial.println(k);
  if(k==140)
  {
    message();
    delay(200);
    k=0;
  }
  Serial.println(w);
  headingcal();
  gpsdata();
  steering();
  turn_to_move_left_or_right = Steering_length();
  time_required_for_move_left_or_right = time_required(turn_to_move_left_or_right);
  int dist_btn_2cord= coord_dist();                               
  Serial.println("distance between two coordinates");
  Serial.print(dist_btn_2cord);
   Serial.println("metre");
   c = gps.read();
          if (c != ',') 
                     {
                     if(turn_to_move_left_or_right < 0){
                                left();
                                delay(time_required_for_move_left_or_right);
                                stop1();
                              }else{
                                right();
                                delay(time_required_for_move_left_or_right);
                                stop1();
                              }
                      if(finalv>=0.88&&finalv<=1.12)
                                   {
                                    forward();
                                   }
                    }
  gpsdata();
  if(dist_btn_2cord<=1)
  {
    // Added to check wheter the bot has reached to checkpoint, then set the destination to the next check point
    w=w+1;
    if(w>=9){ stop1();} //Added by asutosh; if fsild to run remove this
  }
  }else{
    stop1();
    delay(100);
    left();
    delay(1000);
    forward();
  }
}

float time_required(float turn_to_move_left_or_right){
  float speed_of_vehicle = (0.071*PI*150)/216000;
  if(turn_to_move_left_or_right < 0){
    turn_to_move_left_or_right = turn_to_move_left_or_right*(-1);
  }
  float time_ = turn_to_move_left_or_right/speed_of_vehicle;
  Serial.print("Speed_of_vehicle: ");
  Serial.println(speed_of_vehicle, 10);
  Serial.print("timr_required_for_move_left_or_right: ");
  Serial.println(time_);
  return time_;
}

float Steering_length(){
  int angle = (bearing - heading);
  float G = 0.0027777778*0.75396*angle;
  Serial.print("Deference btw bearing and heading: ");
  Serial.println(angle);
  Serial.print("Arc length btn 2 angles: ");
  Serial.println(G,6);
  return G;
}

void gpsdata()
{
    int i=0;
    float x,y;
    logc = "";
    latc= "";
      for(i=0; i<=5; i++)
        data[i]="";
       i=0;
    Serial.println("----Command given----");
    gps.flush();        
    long int time = millis();
    gps.println("AT+CGNSINF");
    
    while ((time+1000 ) > millis())
    {
          while(gps.available())
           {
             c = gps.read();
             if (c != ',') 
           {
              data[i] +=c;
              delay(20);
           } 
            else 
            i++;
         }
        if(i==5)
        break;
    }
    latc = data[3];
    logc = data[4];
    Serial.println(latc);
    Serial.println(logc);
    Serial.println("----Response Print-1--");
    latc1=latc.toFloat()* 0.01745; 
    logc1=logc.toFloat()* 0.01745;
    latd1=latd[w].toFloat()* 0.01745;
    logd1=logd[w].toFloat()* 0.01745;
    deltalog= logd[w].toFloat()-logc.toFloat();
    deltalat=latd[w].toFloat() -latc.toFloat();
    deltalog1=deltalog* 0.01745;
    deltalat1=deltalog* 0.01745;
    x=cos(latd[w].toFloat())*sin(deltalog);
    y=(cos(latc.toFloat())*sin(latd[w].toFloat()))-(sin(latc.toFloat())*cos(latd[w].toFloat())*cos(deltalog));
    bearing=(atan2(x,y))*(180/3.14);
    bearing= ( ((int)bearing + 360) % 360 ); 
    Serial.print("bearing:");
    Serial.print( "\n");
    Serial.println(bearing);
} 
   
void headingcal()
{
  int x,y,z;
  qmc.read(&x,&y,&z);
  delay(20);
  heading=atan2(y,x)*180/3.14;//atan2(x,y)
  heading= ( ((int)heading + 360) % 360 ); 
  Serial.print("heading: ");
  Serial.println(heading);
   if (heading == 0 || heading == 360 ) 
            {
                 Serial.println("N");
            }
    if (heading > 0 && heading < 90 ) 
            {
                 Serial.print("NE");
                 Serial.print(heading);
            }
    if (heading == 90) 
            {
                  Serial.println("E");
            }
    if (heading > 90 && heading < 180 )
            {
                  Serial.print("SE");
                  Serial.print(heading);
            }
    if (heading == 180) 
            {
                  Serial.println("S");
            }
    if (heading > 180 && heading < 270 ) {
                  Serial.print("SW");
                  Serial.print(heading);
           }
    if (heading == 270) 
           {
                  Serial.println("W");
           }
    if (heading > 270 && heading < 360 )
           {
                  Serial.print("NW");
                  Serial.print(heading);
           }  
}
void message()
{
    gps.print("AT+CMGF=1\r");
    delay(2000);
    gps.print("AT+CMGS=\"9348707870\"\r");
    delay(2000);
    gps.print("\r");
    delay(2000);
    gps.println("Dear Bora and Gupta your vehicle is here");
    gps.print("http://maps.google.com/maps?q=loc:");
    gps.print(latc);
    gps.print(",");
    gps.print(logc);
    delay(2000);
    gps.println((char)26);
    delay(2000);
}

void steering()
{
  finalv=heading/bearing;
  Serial.print("finalv: ");
  Serial.println(finalv);  
}

void Ultrasonic()
{
  digitalWrite(Trig, LOW);   // 2s
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);  // 10s10s
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);    // 
  float Fdistance = pulseIn(Echo, HIGH);  
  distance1 = round(Fdistance*0.034/2);
  Serial.println("ud=");
  Serial.println(distance1);
  return Fdistance;
}

int coord_dist(){
   float a,c;
  a=(sin(deltalat1/2))*(sin(deltalat1/2)) + ((cos(latc1))*(cos(latd1))*((sin(deltalog1/2))*(sin(deltalog1/2))));
  c=2*(atan2(sqrt(a),sqrt(1-a)));
  d=6371*c*1000; 
  return d;
}

void forward()
{
  analogWrite(rp,235);
  analogWrite(rn,0);
  analogWrite(lp,255);
  analogWrite(ln,0);
  Serial.print("F");
}

void right()
{
  analogWrite(rp,0);
  analogWrite(rn,170);
  analogWrite(lp,190);
  analogWrite(ln,0);
  Serial.print("R");
}

void left()
{
  analogWrite(rp,170);
  analogWrite(rn,0);
  analogWrite(lp,0);
  analogWrite(ln,190);
  Serial.print("L");
}

void back()
{
  analogWrite(rp,0);
  analogWrite(rn,200);
  analogWrite(lp,0);
  analogWrite(ln,200);
  Serial.print("B");
}

void stop1()
{
  analogWrite(rp,0);
  analogWrite(rn,0);
  analogWrite(lp,0);
  analogWrite(ln,0);
  Serial.print("stop");
   
}
