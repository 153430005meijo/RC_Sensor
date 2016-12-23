#include "mcp_can.h"
#include "NAxisMotion.h"
#include <Wire.h>
#include <SPI.h>
#define INT8U unsigned char

NAxisMotion mySensor;                 //Object that for the sensor
//unsigned long lastStreamTime = 0;     //To store the last streamed time stamp
//const int streamPeriod = 40;          //To stream at 25Hz without using additional timers (time period(ms) =1000/frequency(Hz))
//bool updateSensorData = true;         //Flag to update the sensor data. Default is true to perform the first read before the first stream

const int ON = 1;
const int OFF = 0;
int STATE = OFF;
int RSTATE = OFF;
int SAFE = OFF;
int DIVER = OFF;
int AUTOLIGHT = OFF;

INT8U Flag_Recv = 0;
INT8U len = 0;
INT8U buf[8];
char str[20];

float accelx;
float accelz;
float DIVn;

//First Value
String Light0;
String accelX0;
String accelZ0;
int light0;
int accelx0;
int accelz0;
float DIV;

//String Separate
String accelX0_1;
String accelX0_2;
String accelZ0_1;
String accelZ0_2;

void setup()
{
  Serial.begin(115200);

  if (CAN.begin(CAN_500KBPS) == CAN_OK) Serial.print("can init ok!!\r\n");
  else Serial.print("Can init fail!!\r\n"); // init can bus : baudrate = 500k
  attachInterrupt(0, MCP2515_ISR, FALLING); // start interrupt
  
  //9AxisMotionSensor
  I2C.begin();                    //Initialize I2C communication to the let the library communicate with the sensor.
  //Sensor Initialization
  mySensor.initSensor();          //The I2C Address can be changed here inside this function in the library
  mySensor.setOperationMode(OPERATION_MODE_NDOF);   //Can be configured to other operation modes as desired
  mySensor.setUpdateMode(AUTO);  //The default is AUTO. Changing to manual requires calling the relevant update functions prior to calling the read functions
  //Setting to MANUAL requires lesser reads to the sensor
  //mySensor.updateAccelConfig();
  //updateSensorData = true;
  Serial.println();             Serial.println("Default accelerometer configuration settings...");
  Serial.print("Range: ");      Serial.println(mySensor.readAccelRange());
  Serial.print("Bandwidth: ");  Serial.println(mySensor.readAccelBandwidth());
  Serial.print("Power Mode: "); Serial.println(mySensor.readAccelPowerMode());

  Serial.println("Setup Complete");
}

//CAN Example
// send data:  id = 0x00, standrad flame,data len = 8, stmp: data buf

//For AUTO LIGHT，Immobilizer (CAN)
byte stmp1[8] = {0x80, 0x00, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40};
byte stmp2[8] = {0x80, 0x00, 0x40, 0x40, 0x40, 0x40, 0x40, 0x00};

byte cut1[2] = {0x80, 0x00};//LIGHT ON
byte cut2[2] = {0x00, 0x00};//LIGHT OFF

//For Accel (CAN)
byte accl[8] =  {0x80, 0x00, 0x00, 0x40, 0x40, 0x40, 0x35, 0x75};

void MCP2515_ISR()
{
  Flag_Recv = 1;
}

int flag = 0;//For FLAG（Only 1 STEP）
//Sensor Matrix   0:1st Light Sensor Value 1:DiFF Light Sensor Value
//           2,3:1st X AXIS Accel  4:DiFF X AXIS Accel 
//           5,6:1st Z AXIS Accel  7:DiFF Z AXIS Accel
//You can CHANGE X to Y AXIS
byte sensor[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

void canSend()
{
  Serial.print(", Send CAN>>>");
  for (int i = 0; i < 8; i++)
  {
    Serial.print(sensor[i]);
    if (i == 7)
    {
      Serial.println();
      break;
    }
    Serial.print(",");
  }

  CAN.sendMsgBuf(0x777, 0, 8, sensor);
  delay(100);
}

void beeper()
{
  Serial.print(" !! WARNING WARNING !! ");
  STATE = ON;
  SAFE = ON;
  for (int m = 0; m < 15 ; m++)
  {
    tone(8, 3729, 30);
    delay(15);
    tone(8, 3529, 30);
    delay(15);
  }
}

void loop()
{
  //Flag_Recv = 0;                // clear flag
  //CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf

  //Sensoer Activation_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
  if (flag == 0)
  {
    //LIGHT SENSOR--------------------
    if (analogRead(0) > 0)
    {
      Serial.print("Light Sensor Activate.(initValue:");
      light0 = analogRead(0) / 4;
      Light0 = String(light0, HEX);
      sensor[0] = light0;
      Serial.println(Light0 + "(DEC:" + light0 + "))");
    }//------------------------------

    //9AXIS SENSOR--------------------------------
    if (abs(mySensor.readAccelRange()) > 0)
    {
      Serial.print("9AXIS Sensor Activate.(init Value:");
      //X AXIS------------------------------------------
      Serial.print("aX: ");
      //Serial.print(mySensor.readAccelX()); //Accelerometer X-Axis data
      //Serial.print("m/s2 ");
      accelx = mySensor.readAccelX();
      accelX0 = String(int(accelx * 100), HEX);
      Serial.print(accelX0 + "(DEC/100:" + accelx + ")");

      if (int(accelx * 100) < 0)
      {
        accelx0 = 65536 + int(accelx * 100);
        sensor[2] = 255;
        sensor[3] = accelx0 % 256;
      }
      else
      {
        sensor[3] = int(accelx * 100) % 255;
      }//-----------------------------------------------

      //Z AXIS-------------------------------------------
      Serial.print(" aZ: ");
      //Serial.print(mySensor.readAccelZ());  //Accelerometer Z-Axis data
      //Serial.println("m/s2 ");
      //Serial.println();
      accelz = mySensor.readAccelZ();
      accelZ0 = String("0" + String(int(accelz * 100), HEX));
      Serial.println(accelZ0 + "(DEC/100:" + accelz + "))");

      //Only +Value
      accelz0 = int(accelz * 100);
      sensor[6] = accelz0 % 256;
      sensor[5] = (accelz0 - sensor[6]) / 256;
      //-------------------------------------------------------
    }//--------------------------------------------------------------------------------

    //DIVER
    DIV = (accelx / accelz) * 100;

    //ACCEL DIVISION---------------------------------------
    accelX0.toUpperCase();

    accelX0_1 = accelX0;
    accelX0_2 = accelX0;
    accelX0_1.remove(0, 2);
    accelX0_2.remove(2, 2);

    accelZ0.toUpperCase();
    accelZ0_1 = accelZ0;
    accelZ0_2 = accelZ0;
    accelZ0_1.remove(0, 2);
    accelZ0_2.remove(2, 2);
    //------------------------------------------------------
    //Data Send (For Validation)
    Serial.println("accelX0:" + accelX0_2 + accelX0_1 + " accelZ0:" + accelZ0_2 + accelZ0_1 + " DIV:" + DIV);

    //Data Communication (For Validation)---------------------------------------------------------
    Serial.println("---Send Following CAN Message---");
    for (int i = 0; i < 8; i++)
    {
      Serial.print(sensor[i]);
      if (i == 7)
      {
        Serial.println();
        break;
      }
      Serial.print(",");
    }
    CAN.sendMsgBuf(0x777, 0, 8, sensor);
    delay(100);
    //--------------------------------------------------------------------------------------------

    flag = 1;
  }//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/

  //AUTO LIGHT ON/OFF
  if((digitalRead(7)==HIGH) && AUTOLIGHT == OFF) {
    AUTOLIGHT = ON;
    Serial.println("AUTOLIGHT SYSTEM ACTIVATED");
    delay(500);
  }
  if((digitalRead(7)==HIGH) && AUTOLIGHT == ON) {
    AUTOLIGHT = OFF;
    Serial.println("AUTOLIGHT SYSTEM SHUTDOWN");
    delay(500);        
  }

  //Read CAN
/*  
    if((CAN.readMsgBuf(&len, buf) != -1) && (len > 0))
    {
    Serial.println("CAN_BUS GET DATA!");
    Serial.print("data len = ");Serial.println(len);
    for(int i = 0; i<len; i++)    // print the data
    {
      Serial.print(buf[i],HEX);Serial.print(" ");
      buf[i] = 0;
    }
    len = 0;
    }
*/
  Serial.print("aX: ");
  accelx = mySensor.readAccelX();
  Serial.print(accelx); //Accelerometer X-Axis data
  Serial.print("m/s2 ");

  Serial.print(" aZ: ");
  accelz = mySensor.readAccelZ();
  Serial.print(accelz);  //Accelerometer Z-Axis data
  Serial.print("m/s2 ");
  /*
    Serial.print("lX: ");
    Serial.print(mySensor.readLinearAccelX()); //Linear Acceleration X-Axis data
    Serial.print("m/s2 ");

    Serial.print(" lY: ");
    Serial.print(mySensor.readLinearAccelY());  //Linear Acceleration Y-Axis data
    Serial.print("m/s2 ");

    Serial.print(" lZ: ");
    Serial.print(mySensor.readLinearAccelZ());  //Linear Acceleration Z-Axis data
    Serial.print("m/s2 ");
  */
  //LIGHT SENSOR
  Serial.print("Light Sensor Value:");
  Serial.print(analogRead(0) / 4);
  Serial.print(" ");

  //DIVISION
  DIVn = 100 * accelx / accelz;
  Serial.print("DIVn:");
  Serial.println(DIVn);

  delay(5);

  //LIGHT ON
  if ((RSTATE == OFF) && (analogRead(0) / 4 - sensor[0] >= 40) && (AUTOLIGHT == ON) )
  {
    Serial.print(" Light ON ");
    RSTATE = ON;
    tone(8, 300, 100);
    delay(50);
    CAN.sendMsgBuf(0x003, 0, 8, stmp1);
    delay(50);
    CAN.sendMsgBuf(0x003, 0, 8, stmp2);
    delay(50);
    CAN.sendMsgBuf(0x010, 0, 2, cut1);
    delay(50);
    canSend();
  }

  //LIGHT OFF
  if ((RSTATE == ON) && (analogRead(0) / 4 - sensor[0] < 40) && (AUTOLIGHT == ON) )
  {
    Serial.print(" Light OFF ");
    RSTATE = OFF;
    tone(8, 300, 100);
    delay(50);
    CAN.sendMsgBuf(0x003, 0, 8, stmp1);
    delay(50);
    CAN.sendMsgBuf(0x003, 0, 8, stmp2);
    delay(50);
    CAN.sendMsgBuf(0x010, 0, 2, cut2);
    delay(50);
    canSend();
  }
  delay(5);

  //IMMOBILIZER
  if ( (SAFE == OFF) && (abs(mySensor.readLinearAccelX()) > 5.0 | abs(mySensor.readLinearAccelZ()) > 5.0 | abs(mySensor.readLinearAccelY()) > 5.0) && (DIVER == OFF))
  {
    beeper();
    delay(30);
    STATE = OFF;
    SAFE = OFF;
    canSend();
  }

  //DIV ALERT---------------------------------
  if ( (DIVn - DIV) < -15 && SAFE != ON)
  {
    Serial.print("UP HILL");
    DIVER = ON;
    tone(8, 3000, 30);
    delay(30);
    tone(8, 3300, 30);
    delay(30);

    canSend();
    delay(200);
  }
  if ( (DIVn  - DIV ) >= 15 && SAFE != ON)
  {
    Serial.print("DOWN HILL");
    DIVER = ON;
    tone(8, 2000, 30);
    delay(30);
    tone(8, 1700, 30);
    delay(30);
    canSend();
    delay(200);
  }
  //------------------------------------------------------------------------------------

  //mySensor.updateAccel();        //Update the Accelerometer data
  //mySensor.updateLinearAccel();  //Update the Linear Acceleration data

  //UPDATE CAN MESSAGE-------------------------------------
    //UPDATE LIGHT SENSOR
  sensor[1] = abs(analogRead(0) / 4 - sensor[0]);

    //UPDATE ACCEL---------------------------------
  if (abs(DIVn - DIV) < 15)   DIVER = OFF;

      //X AXIS
  sensor[4] = abs(accelx0 - int((mySensor.readAccelX() * 100)));

      //Z AXIS
  sensor[7] = abs(accelz0 - int((mySensor.readAccelZ() * 100)));
    //-----------------------------------------------------------
  Serial.println();
  delay(450);
}
