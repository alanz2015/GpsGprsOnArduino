/*
 * Test SIM868 GPRS functionalities.
 * Reference: https://blog.csdn.net/ivy_reny/article/details/47422943
 * An important reference project associated with SIM80X series is TinyGSM
 * https://github.com/vshymanskyy/TinyGSM
 * 访问HTTP

//测试指令

//检测信号质量，确定是否可以登陆上网络；若返回10~31, 0之间的信号数字则继续，如果信号是99, 99，则应该考虑不停地让模块去搜索网络。

AT+CSQ                                                                                      

    +CSQ: 24,0

    OK

//查询GPRS网络注册状态

AT+CREG?                                                            

    +CGREG: 0,1

    OK

 //是否附着GPRS网络
 */
String Arsp, Grsp;
#define DebugSerial Serial
#define gsm Serial3


void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
  Serial.println("Testing GSM SIM800L");
  gsm.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:

  if(gsm.available())
  {
    Grsp = gsm.readString();
    Serial.println(Grsp);
  }

  if(Serial.available())
  {
    Arsp = Serial.readString();
    gsm.println(Arsp);
  }

}
