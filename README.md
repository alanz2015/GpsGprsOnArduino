# Support SIM868 (GPS + GPRS) on Arduino MEGA2560
This project implements the GPS + GPRS functions on MEGA2560 + SIM868.

# Water Level Sensor Connection:
DC/Battery --> DC/DC Convert --> +24V
Voltage Analog Signal --> A12

Programming Interface: 
#define waterLevelPin 12
..................
analogRead(waterLevelPin)

# 3-Color LED Connection:
DC/Battery --> DC/DC Convert --> +12V
redLED <--> 31
yellowLED <--> 33
greenLED <--> 35

Programming Interface:
int yellowLED = 31;
int greenLED = 33;
int redLED = 35;

..................
  pinMode(redLED, OUTPUT);
  pinMode(yellowLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  ......
  digitalWrite(redLED, HIGH);
  digitalWrite(yellowLED, HIGH);
  digitalWrite(greenLED, HIGH);
  ......
  delay(3000);
  digitalWrite(redLED, LOW);
  digitalWrite(yellowLED, LOW);
  digitalWrite(greenLED, LOW);
  
  # CMCC IoT Cloud Platform Utilization
  https://i.xue.taobao.com/detail.htm?spm=a2174.7765247.0.0.yFdV0L&courseId=78915

