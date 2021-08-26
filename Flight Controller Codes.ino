#include<Wire.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
WiFiUDP UDP;
char packet[7];
boolean recvState;
//-----------------------------------------------------------------------//   
int ESCout_1 ,ESCout_2 ,ESCout_3 ,ESCout_4;
int input_PITCH = 50;
int input_ROLL = 50;
int input_YAW;
int input_THROTTLE=1100;
int state1,state2,state3,state4;
//-----------------------------------------------------------------------// 
int16_t gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, temperature, acc_total_vector;
float angle_pitch, angle_roll, angle_yaw;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output;
float elapsedTime;
long Time, timePrev, time2;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
//-----------------------------------------------------------------------// 
float pitch_PID,roll_PID,yaw_PID;
float roll_error, roll_previous_error, pitch_error, pitch_previous_error, yaw_error;
float roll_pid_p, roll_pid_d, roll_pid_i, pitch_pid_p, pitch_pid_i, pitch_pid_d, yaw_pid_p, yaw_pid_i;
float roll_desired_angle, pitch_desired_angle, yaw_desired_angle; 
double twoX_kp=5;      
double twoX_ki=0.003;
double twoX_kd=2;     
double yaw_kp=3;    
double yaw_ki=0.002;
//-----------------------------------------------------------------------// 
void setup() {
pinMode(D5,OUTPUT);pinMode(D6,OUTPUT);pinMode(D7,OUTPUT);pinMode(D8,OUTPUT);
delay(1300);
Serial.begin(115200);
WiFi.mode(WIFI_STA);
WiFi.begin("Laptop", "lppasspi");
while (WiFi.status() != WL_CONNECTED){
  GPOS = (1 << 14);GPOS = (1 << 12);GPOS = (1 << 13);GPOS = (1 << 15); 
  delayMicroseconds(1000);
  GPOC = (1 << 14);GPOC = (1 << 12);GPOC = (1 << 13);GPOC = (1 << 15);   
  delayMicroseconds(1000);
  yield(); 
}
Serial.println(WiFi.localIP()); 
UDP.begin(9999);
//-----------------------------------------------------------------------//    
  Wire.begin();    
   Wire.setClock(400000);
  Wire.beginTransmission(0x68);                                        
  Wire.write(0x6B);                                                    
  Wire.write(0x00);                                                  
  Wire.endTransmission();         
  Wire.beginTransmission(0x68);                                      
  Wire.write(0x1C);                                                   
  Wire.write(0x10);                                                   
  Wire.endTransmission();             
  Wire.beginTransmission(0x68);                                        
  Wire.write(0x1B);                                                 
  Wire.write(0x08);                                                   
  Wire.endTransmission();  
  delay(1000);
for (int cal_int = 0; cal_int < 2000 ; cal_int ++){  
  if(cal_int % 125 == 0)Serial.print(".");                                           
  Wire.beginTransmission(0x68);                                       
  Wire.write(0x3B);                                                  
  Wire.endTransmission();                                             
  Wire.requestFrom(0x68,14);                                        
  while(Wire.available() < 14);                                        
  acc_x = Wire.read()<<8|Wire.read();                               
  acc_y = Wire.read()<<8|Wire.read();                               
  acc_z = Wire.read()<<8|Wire.read();                                 
  temperature = Wire.read()<<8|Wire.read();                           
  gyro_x = Wire.read()<<8|Wire.read();                                
  gyro_y = Wire.read()<<8|Wire.read();                                 
  gyro_z = Wire.read()<<8|Wire.read();                                              
  gyro_x_cal += gyro_x;                                              
  gyro_y_cal += gyro_y;                                             
  gyro_z_cal += gyro_z;                                             
  GPOS = (1 << 14);GPOS = (1 << 12);GPOS = (1 << 13);GPOS = (1 << 15); 
  delayMicroseconds(1000);
  GPOC = (1 << 14);GPOC = (1 << 12);GPOC = (1 << 13);GPOC = (1 << 15);   
  delayMicroseconds(1000);   
  yield();                                                    
}
gyro_x_cal /= 2000;                                                 
gyro_y_cal /= 2000;                                                 
gyro_z_cal /= 2000;
//-----------------------------------------------------------------------//    
Time = micros();                                                                            
}

void loop(){ 
GPOS = (1 << 14);GPOS = (1 << 12);GPOS = (1 << 13);GPOS = (1 << 15); 
  timePrev = Time;                   
  Time = micros();  
  elapsedTime = (float)(Time - timePrev) / (float)1000000;
  Wire.beginTransmission(0x68);                                       
  Wire.write(0x3B);                                                  
  Wire.endTransmission();                                             
  Wire.requestFrom(0x68,14);                                        
  while(Wire.available() < 14);                                        
  acc_x = Wire.read()<<8|Wire.read();                               
  acc_y = Wire.read()<<8|Wire.read();                               
  acc_z = Wire.read()<<8|Wire.read();                                 
  temperature = Wire.read()<<8|Wire.read();                           
  gyro_x = Wire.read()<<8|Wire.read();                                
  gyro_y = Wire.read()<<8|Wire.read();                                 
  gyro_z = Wire.read()<<8|Wire.read(); 
gyro_x -= gyro_x_cal;                                              
gyro_y -= gyro_y_cal;                                               
gyro_z -= gyro_z_cal;                                                       
  angle_pitch += gyro_x * elapsedTime * 0.01526717557;                                 
  angle_roll += gyro_y * elapsedTime * 0.01526717557;
  angle_yaw += gyro_z * elapsedTime * 0.01526717557;
  angle_pitch += angle_roll * sin(gyro_z * 0.000001066);               
  angle_roll -= angle_pitch * sin(gyro_z * 0.000001066); 
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));  
  angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;       
  angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;       
  angle_pitch_acc += 0;                                              
  angle_roll_acc += 0;                                             
  if(set_gyro_angles){                                                
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;     
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;        
  }
  else{                                                               
    angle_pitch = angle_pitch_acc;                                     
    angle_roll = angle_roll_acc;                                       
    set_gyro_angles = true;                                            
  }
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1; 
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;  
//-----------------------------------------------------------------------//
roll_desired_angle = 3*((float)input_ROLL/(float)10 - (float)5);
pitch_desired_angle =3*((float)input_PITCH/(float)10 - (float)5);
//yaw_desired_angle =0;

roll_error =  angle_roll_output - roll_desired_angle;
pitch_error = angle_pitch_output - pitch_desired_angle;  
yaw_error = angle_yaw - yaw_desired_angle;  
  
roll_pid_p = twoX_kp*roll_error;
pitch_pid_p = twoX_kp*pitch_error;
yaw_pid_p = yaw_kp*yaw_error;

if(-3 < roll_error <3){roll_pid_i = roll_pid_i+(twoX_ki*roll_error);}
if(-3 < pitch_error <3){pitch_pid_i = pitch_pid_i+(twoX_ki*pitch_error);}
if(-3 < yaw_error <3){yaw_pid_i = yaw_pid_i+(yaw_ki*yaw_error);}

roll_pid_d = twoX_kd*((roll_error - roll_previous_error)/elapsedTime);
pitch_pid_d = twoX_kd*((pitch_error - pitch_previous_error)/elapsedTime);
roll_PID = roll_pid_p + roll_pid_i + roll_pid_d;
pitch_PID = pitch_pid_p + pitch_pid_i + pitch_pid_d;
yaw_PID = yaw_pid_p + yaw_pid_i;

if(roll_PID < -400){roll_PID=-400;}
else if(roll_PID > 400) {roll_PID=400;}
if(pitch_PID < -400){pitch_PID=-400;}
else if(pitch_PID > 400) {pitch_PID=400;}
if(yaw_PID < -400){yaw_PID=-400;}
else if(yaw_PID > 400) {yaw_PID=400;}

ESCout_1 = input_THROTTLE - roll_PID - pitch_PID - yaw_PID;
ESCout_2 = input_THROTTLE + roll_PID - pitch_PID + yaw_PID;
ESCout_3 = input_THROTTLE + roll_PID + pitch_PID - yaw_PID;
ESCout_4 = input_THROTTLE - roll_PID + pitch_PID + yaw_PID;

if(ESCout_1>2000) ESCout_1=2000;
else if(ESCout_1<1100) ESCout_1=1100;
if(ESCout_2>2000) ESCout_2=2000;
else if(ESCout_2<1100) ESCout_2=1100;
if(ESCout_3>2000) ESCout_3=2000;
else if(ESCout_3<1100) ESCout_3=1100;
if(ESCout_4>2000) ESCout_4=2000;
else if(ESCout_4<1100) ESCout_4=1100;

roll_previous_error = roll_error;
pitch_previous_error = pitch_error;
//-----------------------------------------------------------------------//
while((micros() - Time) < 1000);
state1 = 1; state2 = 1; state3 = 1; state4 = 1;
while(state1 == 1 || state2 == 1 || state3 == 1 || state4 == 1){
  time2 = micros();
  if((time2 - Time) >= ESCout_1 && state1 == 1){ GPOC = (1 << 14); state1=0;}
  if((time2 - Time) >= ESCout_2 && state2 == 1){ GPOC = (1 << 12);state2=0;}
  if((time2 - Time) >= ESCout_3 && state3 == 1){ GPOC = (1 << 13);state3=0;}
  if((time2 - Time) >= ESCout_4 && state4 == 1){ GPOC = (1 << 15);state4=0;}
}
//-----------------------------------------------------------------------//
if(!recvState){
  int packetSize = UDP.parsePacket();
  if (packetSize) {
    int len = UDP.read(packet, 6);
    packet[len] = '\0';    
if(String(packet[0]) == "a"){
input_ROLL = int(packet[1]);
input_PITCH = int(packet[2]); 
input_THROTTLE = 1000 + int(packet[3]);
input_YAW = int(packet[4]);
}
else if(String(packet[0]) == "b"){
input_ROLL = int(packet[1]);
input_PITCH = int(packet[2]); 
input_THROTTLE = 1000 + int(packet[3])*100 + int(packet[4]);
input_YAW = int(packet[5]);
}
if(String(packet[0]) == "1"){
twoX_kp = (float)int(packet[1])/(float)100;
twoX_ki = (float)int(packet[2])/(float)1000; 
twoX_kd = (float)int(packet[3])/(float)100;
}  
else if(String(packet[0]) == "2"){
twoX_kp = (float)(float)(int(packet[1])*100 + int(packet[2]))/(float)100;
twoX_ki = (float)(float)(int(packet[3])*100 + int(packet[4]))/(float)1000; 
twoX_kd = (float)(float)(int(packet[5])*100 + int(packet[6]))/(float)100;
}    
else if(String(packet[0]) == "3"){
twoX_kp = (float)(int(packet[1])*100 + int(packet[2]))/(float)100;
twoX_ki = (float)int(packet[3])/(float)1000; 
twoX_kd = (float)int(packet[4])/(float)100;
}  
else if(String(packet[0]) == "4"){
twoX_kp = (float)int(packet[1])/(float)100;
twoX_ki = (float)(int(packet[2])*100 + int(packet[3]))/(float)1000; 
twoX_kd = (float)int(packet[4])/(float)100;
}  
else if(String(packet[0]) == "5"){
twoX_kp = (float)int(packet[1])/(float)100;
twoX_ki = (float)int(packet[2])/(float)1000; 
twoX_kd = (float)(int(packet[3])*100 + int(packet[4]))/(float)100;
}    
else if(String(packet[0]) == "6"){
twoX_kp = (float)(int(packet[1])*100 + int(packet[2]))/(float)100;
twoX_ki = (float)(int(packet[3])*100 + int(packet[4]))/(float)1000; 
twoX_kd = (float)int(packet[5])/(float)100;
}  
else if(String(packet[0]) == "7"){
twoX_kp = (float)int(packet[1])/(float)100;
twoX_ki = (float)(int(packet[2])*100 + int(packet[3]))/(float)1000; 
twoX_kd = (float)(int(packet[4])*100 + int(packet[5]))/(float)100;
} 
else if(String(packet[0]) == "8"){
twoX_kp = (float)(int(packet[1])*100 + int(packet[2]))/(float)100;
twoX_ki = (float)int(packet[3])/(float)1000; 
twoX_kd = (float)(int(packet[4])*100 + int(packet[5]))/(float)100;
}
Serial.print(input_ROLL);Serial.print(" ");
Serial.print(input_THROTTLE);Serial.print(" ");
Serial.print(twoX_kp);Serial.print(" ");
Serial.print(twoX_ki,3);Serial.print(" ");
Serial.print(twoX_kd);Serial.println();
}
}
else if(recvState){
UDP.beginPacket(UDP.remoteIP(), UDP.remotePort());
UDP.print(Time - timePrev);
UDP.endPacket();
}
recvState = !recvState;
//-----------------------------------------------------------------------//
Serial.print(angle_roll_output);Serial.print("  ");
Serial.print(angle_pitch_output);Serial.print(" | ");
Serial.print(roll_desired_angle);Serial.print("  ");
Serial.print(pitch_desired_angle);
Serial.println();
} 
