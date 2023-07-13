#include "Arduino.h"
#include "BluetoothSerial.h"     //手機藍芽聲控=>顯示指令
#include<SoftwareSerial.h>
#include <stdlib.h>
#include <Wire.h>
hw_timer_t *My_timer=NULL;
// 定义Arduino的I2C地址
#define ARDUINO_I2C_ADDRESS 0x08
BluetoothSerial BT;
#define RX1 26
#define TX1 27
HardwareSerial abc0(1);     //1 3腳位
HardwareSerial abc2(2);
//SoftwareSerial abc2(16,17);
float data1, data2;

void setup() {
  //bluetooth
  Serial.begin(115200);
  abc0.begin(9600, SERIAL_8N1, RX1, TX1);
  //abc1.begin(9600, SERIAL_8N1, RX1, TX1);
  abc2.begin(9600);
  BT.begin("zxc"); //Bluetooth device name
  Wire.begin(21, 22);  // 初始化I2C通信，使用引脚21和22 A4->21 A5->22
  Wire.setClock(1000000);  // 设置I2C通信速率为1000kHz
}


float gps_target(float target_x ,float target_y)
{
  return  sq(data1-target_x)+sq(data2-target_y);
}

void loop() {
   Wire.requestFrom(ARDUINO_I2C_ADDRESS, 8);
    
    if (Wire.available() >= 8) {
      Wire.readBytes((uint8_t*)&data1, sizeof(data1));  // 读取第一个浮点数数据
      Wire.readBytes((uint8_t*)&data2, sizeof(data2));  // 读取第二个浮点数数据
    }
    Serial.print("hedgehog_x: ");
    Serial.print(data1);
    Serial.print(" hedgehog_y: ");
    Serial.println(data2);
//    float  b = gps_target(1,0);
//    if(b < 1){
//      Serial.println("ok");
//      abc0.write("-1");
//      abc2.write("-1");
//      delay(100); 
//    
if (BT.available()) {
    String value = BT.readString();
    if (value == "start"){
      abc0.write("1");
      abc2.write("1");
      delay(5000);
      
    } 
    
    if (value == "stop") {
      abc0.write("-1");
      abc2.write("-1");  
      delay(100);         
    }
    if (value == "right") {
      abc0.write("-1");
      abc2.write("-1");  
      delay(1500);
      abc0.write("2");
      abc2.write("2");
      delay(100);         
    }
    if (value == "left") {
      abc0.write("-1");
      abc2.write("-1");  
      delay(1500);
      abc0.write("-2");
      abc2.write("-2"); 
      delay(100);         
    }
  }
  
}
