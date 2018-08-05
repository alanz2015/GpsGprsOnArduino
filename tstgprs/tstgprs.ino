/*
 * Test SIM868 GPRS functionalities.
 * Reference: https://blog.csdn.net/ivy_reny/article/details/47422943
 * An important reference project associated with SIM80X series is TinyGSM
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
