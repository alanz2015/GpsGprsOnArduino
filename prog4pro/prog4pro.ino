/*
 * Recommended Minimum Specific GPS/TRANSIT Data（RMC）: $GNRMC,
 * UTC時間，hhmmss（時分秒）格式 : 081712.000,
 * 定位狀態，A=有效定位，V=無效定位 : A,
 * 緯度ddmm.mmmm（度分）格式（前面的0也將被傳輸）: 2233.824332,
 * 緯度半球N（北半球）或S（南半球）: N,
 * 經度dddmm.mmmm（度分）格式（前面的0也將被傳輸）: 11407.169070,
 * 經度半球E（東經）或W（西經）: E,
 * 地面速率（000.0~999.9節，前面的0也將被傳輸）: 0.00,
 * 地面航向（000.0~359.9度，以真北為參考基準，前面的0也將被傳輸）: 301.04,20$GNGGA,081713.000
 * GRPS Location:
 * AT+SAPBR=1,1  //激活网络场景
 * AT+SAPBR=2,1  //获取分配IP地址
 * AT+CIPGSMLOC=1,1 //获得定位信息
 * AT+CIPGSMLOC=2,1 //获得时间信息
 * AT+SAPBR=0,1 //关闭网络场景
 *
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

#include "TimerOne.h"

#define DebugSerial Serial
#define GprsSerail Serial3
#define GpsSerial  Serial2

struct
{
	char GPS_Buffer[80];
	bool isGetData;   //是否获取到GPS数据
	bool isParseData; //是否解析完成
	char UTCTime[10];   //UTC时间
	char latitude[20];    //纬度
	char N_S;    //N/S
	char longitude[20];   //经度
	char E_W;    //E/W
	bool isUsefull;   //定位信息是否有效
  char UTCDate[12]; // Date information
} Save_Data;

/* Assign a unique ID to this sensor at the same time */
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

const unsigned int gpsRxBufferLength = 600;
char gpsRxBuffer[gpsRxBufferLength];
unsigned int gpsRxCount = 0;

#define Success 1U
#define Failure 0U

int L = 13; // LED指示灯引脚
int curLightLED = 0;
int yellowLED = 31;
int greenLED = 33;
int redLED = 35;
int waterLevelPin = 12;  // Water level sensor data

volatile unsigned long  Time_Cont = 0;       // for GPRS Timeout Counter
volatile unsigned long  Time_Cont2 = 0;       // for GPS Timeout Counter

const unsigned int gprsRxBufferLength = 600;
char gprsRxBuffer[gprsRxBufferLength];
unsigned int gprsBufferCount = 0;
char OneNetServer[] = "api.heclouds.com";       //不需要修改


char device_id[] = "31027885";    //修改为自己的设备ID
char API_KEY[] = "rLhsBYEcRfk6BBAwcpNHKIZ199Y=";    //修改为自己的API_KEY
char sensor_gps[] = "location";
char sensor_level[] = "waterlevel";  // Water level
/* Analog input from water level sensor */
float waterLevel = 0.0;

void displaySensorDetails(void)
{
  sensor_t sensor;
  accel.getSensor(&sensor);
  DebugSerial.println("------------------------------------");
  DebugSerial.print  ("Sensor:       "); DebugSerial.println(sensor.name);
  DebugSerial.print  ("Driver Ver:   "); DebugSerial.println(sensor.version);
  DebugSerial.print  ("Unique ID:    "); DebugSerial.println(sensor.sensor_id);
  DebugSerial.print  ("Max Value:    "); DebugSerial.print(sensor.max_value);  DebugSerial.println(" m/s^2");
  DebugSerial.print  ("Min Value:    "); DebugSerial.print(sensor.min_value);  DebugSerial.println(" m/s^2");
  DebugSerial.print  ("Resolution:   "); DebugSerial.print(sensor.resolution); DebugSerial.println(" m/s^2");  
  DebugSerial.println("------------------------------------");
  DebugSerial.println("");
  delay(500);
}


void displayDataRate(void)
{
  DebugSerial.print  ("Data Rate:    "); 
  
  switch(accel.getDataRate())
  {
    case ADXL345_DATARATE_3200_HZ:
      DebugSerial.print  ("3200 "); 
      break;
    case ADXL345_DATARATE_1600_HZ:
      DebugSerial.print  ("1600 "); 
      break;
    case ADXL345_DATARATE_800_HZ:
      DebugSerial.print  ("800 "); 
      break;
    case ADXL345_DATARATE_400_HZ:
      DebugSerial.print  ("400 "); 
      break;
    case ADXL345_DATARATE_200_HZ:
      DebugSerial.print  ("200 "); 
      break;
    case ADXL345_DATARATE_100_HZ:
      DebugSerial.print  ("100 "); 
      break;
    case ADXL345_DATARATE_50_HZ:
      DebugSerial.print  ("50 "); 
      break;
    case ADXL345_DATARATE_25_HZ:
      DebugSerial.print  ("25 "); 
      break;
    case ADXL345_DATARATE_12_5_HZ:
      DebugSerial.print  ("12.5 "); 
      break;
    case ADXL345_DATARATE_6_25HZ:
      DebugSerial.print  ("6.25 "); 
      break;
    case ADXL345_DATARATE_3_13_HZ:
      DebugSerial.print  ("3.13 "); 
      break;
    case ADXL345_DATARATE_1_56_HZ:
      DebugSerial.print  ("1.56 "); 
      break;
    case ADXL345_DATARATE_0_78_HZ:
      DebugSerial.print  ("0.78 "); 
      break;
    case ADXL345_DATARATE_0_39_HZ:
      DebugSerial.print  ("0.39 "); 
      break;
    case ADXL345_DATARATE_0_20_HZ:
      DebugSerial.print  ("0.20 "); 
      break;
    case ADXL345_DATARATE_0_10_HZ:
      DebugSerial.print  ("0.10 "); 
      break;
    default:
      DebugSerial.print  ("Invalid datarate  "); 
      break;
  }  
  DebugSerial.println(" Hz");  
}

void displayRange(void)
{
  DebugSerial.print  ("Range:         +/- "); 
  
  switch(accel.getRange())
  {
    case ADXL345_RANGE_16_G:
      DebugSerial.print  ("16 "); 
      break;
    case ADXL345_RANGE_8_G:
      DebugSerial.print  ("8 "); 
      break;
    case ADXL345_RANGE_4_G:
      DebugSerial.print  ("4 "); 
      break;
    case ADXL345_RANGE_2_G:
      DebugSerial.print  ("2 "); 
      break;
    default:
      DebugSerial.print  ("?? "); 
      break;
  }  
  DebugSerial.println(" g");  
}

void setup() {
	pinMode(L, OUTPUT);
	digitalWrite(L, LOW);

  /*
   * Setup 3-Color LED display control pins
   */
  pinMode(redLED, OUTPUT);
  pinMode(yellowLED, OUTPUT);
  pinMode(greenLED, OUTPUT);

  /*
   * Set LED control pins to HIGH will make LEDs to flickering automatically by itself.
   */
  digitalWrite(redLED, HIGH);
  digitalWrite(yellowLED, HIGH);
  digitalWrite(greenLED, HIGH);

  delay(1000);
  /*
   * Set LED control pins to LOW will make LEDs light off.
   */
  digitalWrite(redLED, LOW);
  digitalWrite(yellowLED, LOW);
  digitalWrite(greenLED, LOW);
  /*
   * Done 3-Color LED self-checking to indicate system power-on
   */

	Save_Data.isGetData = false;
	Save_Data.isParseData = false;
	Save_Data.isUsefull = false;
	clrGpsRxBuffer();

	DebugSerial.begin(115200);
	GprsSerail.begin(9600);
	GpsSerial.begin(115200);      //115200，和我们店铺的GPS输出的波特率一致

  /*
   * Initialize ADXL345
   */
  DebugSerial.println("Accelerometer Test"); 
  DebugSerial.println("");
  
  /* Initialise the sensor */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    DebugSerial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while(1);
  }

  /* Set the range to whatever is appropriate for your project */
  // accel.setRange(ADXL345_RANGE_16_G);
  // displaySetRange(ADXL345_RANGE_8_G);
  // displaySetRange(ADXL345_RANGE_4_G);
  accel.setRange(ADXL345_RANGE_2_G);
  
  /* Display some basic information on this sensor */
  displaySensorDetails();
  
  /* Display additional settings (outside the scope of sensor_t) */
  displayDataRate();
  displayRange();
  Serial.println("");


	Timer1.initialize(1000);
	Timer1.attachInterrupt(Timer1_handler);
	initGprs();

  digitalWrite(greenLED, HIGH);
  delay(2000);
  digitalWrite(greenLED, LOW);
  
	DebugSerial.println("\r\nSetup done!");
}

void loop() {
  int retVal = 0;
  /* Get a new acclerator sensor event */ 
  sensors_event_t event;
  
	Time_Cont2 = 0;

  // Check GPS signal quality
  while ((retVal = gpsRead()) < 0) {
    //获取GPS数据
    digitalWrite(redLED, HIGH);
    digitalWrite(yellowLED, HIGH);
    digitalWrite(greenLED, LOW);
    delay(5000);
  }
  
  DebugSerial.println("GPS signal quality is OK!!!");
  digitalWrite(redLED, LOW);
  digitalWrite(yellowLED, LOW);
	
	while (Time_Cont2 < 5)	//5s内不停读取GPS
	{
		retVal = gpsRead();  //获取GPS数据
		retVal = parseGpsBuffer();//解析GPS数据		
    if (retVal == 0)
      Time_Cont2 = 10;  // Exit from this read GPS loop
	}

  waterLevel = analogRead(waterLevelPin);
  if ((1000.00 < waterLevel) && (waterLevel <= 2023.00)) {
       if (curLightLED != 1) {
         digitalWrite(greenLED, HIGH);
         digitalWrite(yellowLED, LOW);
         digitalWrite(redLED, LOW);
         curLightLED = 1; // 1 stands for GREEN LED
       }
  }
  else {
    if ((2023.00 < waterLevel) && (waterLevel <= 4023.00)) {
      if (curLightLED != 2) {
         digitalWrite(greenLED, LOW);
         digitalWrite(yellowLED, HIGH);
         digitalWrite(redLED, LOW);
         curLightLED = 2; // 2 stands for YELLOW LED
      }
    }
    else {
      if (waterLevel > 4023.00) {
        if (curLightLED != 3) {
           digitalWrite(yellowLED, LOW);
           digitalWrite(redLED, HIGH);
           digitalWrite(greenLED, LOW);
           curLightLED = 3; // 3 stands for RED LED
        }
      }
    }
  }
  
  #if 1
  DebugSerial.print("Measured Water Level: ");
  DebugSerial.print((waterLevel - 1023.00));
  DebugSerial.println(" cm");
  #endif

  printGpsBuffer();//输出解析后的数据  ,包括发送到OneNet服务器
  
  #if 1
  accel.getEvent(&event);
  /* Display the results (acceleration is measured in m/s^2) */
  DebugSerial.print("X: "); DebugSerial.print(event.acceleration.x); DebugSerial.print("  ");
  DebugSerial.print("Y: "); DebugSerial.print(event.acceleration.y); DebugSerial.print("  ");
  DebugSerial.print("Z: "); DebugSerial.print(event.acceleration.z); DebugSerial.print("  ");
  DebugSerial.println("m/s^2 ");
  #endif
}

void printGpsBuffer()
{
	if (Save_Data.isParseData)
	{
		Save_Data.isParseData = false;
    DebugSerial.print("Save_Data.UTCDate = ");
    DebugSerial.println(Save_Data.UTCDate);
		DebugSerial.print("Save_Data.UTCTime = ");
		DebugSerial.println(Save_Data.UTCTime);

		if (Save_Data.isUsefull)
		{
			Save_Data.isUsefull = false;
			DebugSerial.print("Save_Data.latitude = ");
			DebugSerial.println(Save_Data.latitude);
			DebugSerial.print("Save_Data.N_S = ");
			DebugSerial.println(Save_Data.N_S);
			DebugSerial.print("Save_Data.longitude = ");
			DebugSerial.println(Save_Data.longitude);
			DebugSerial.print("Save_Data.E_W = ");
			DebugSerial.println(Save_Data.E_W);
      #if 1
      /*
       * Upload the field sampling water level data to CMCC OneNet IoT platform
       */
			postGpsDataToOneNet(API_KEY, device_id, sensor_gps, Save_Data.longitude, Save_Data.latitude, waterLevel);
      postGpsDataToOneNet(API_KEY, device_id, sensor_level, Save_Data.longitude, Save_Data.latitude, waterLevel);
      #endif
      DebugSerial.println("GPS DATA is usefull!");
		}
		else
		{
			DebugSerial.println("GPS DATA is NOT usefull!");
      
		}

	}
}

int parseGpsBuffer()
{
	char *subString;
	char *subStringNext;
  char localString[20];
  
	if (Save_Data.isGetData)
	{
		Save_Data.isGetData = false;
    Save_Data.isUsefull = false;

    #if 1
    subString = &Save_Data.GPS_Buffer[7]; // Override "$GPRMC,"
    subStringNext = strstr(subString, ",");
    if (subStringNext != NULL) {
      // Extract UTC Time hhmmss:xxx
      memset(localString, 0, sizeof(localString));
      memset(Save_Data.UTCTime, 0, sizeof(Save_Data.UTCTime));
      memcpy(localString, subString, (subStringNext - subString));
      memcpy(Save_Data.UTCTime, localString, 6);
      
      #if 0
      DebugSerial.print("Index Nb: ");
      DebugSerial.println(subStringNext - subString);
      DebugSerial.print("Extracted UTCTime: ");
      DebugSerial.println(Save_Data.UTCTime);
      DebugSerial.println(subString);
      DebugSerial.println(subStringNext);
      DebugSerial.println("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
      #endif
      
      subString = subStringNext + 1;  // Go ahead to the left string clip with overriding ','
      subStringNext = strstr(subString, ",");
      if (subStringNext != NULL) {
        // Extract message validation symbol
        memset(localString, 0, sizeof(localString));
        memcpy(localString, subString, (subStringNext - subString));
        if (localString[0] == 'A')
          Save_Data.isUsefull = true;
        else if (localString[0] == 'V')
          Save_Data.isUsefull = false;
        else {
          Save_Data.isUsefull = false;
          DebugSerial.println("Error Active/Valid indication.");
          return -3;
        }

        subString = subStringNext + 1;
        subStringNext = strstr(subString, ",");
        if (subStringNext != NULL) {
          // Extract Latitude
          memset(localString, 0, sizeof(localString));
          memcpy(localString, subString, (subStringNext - subString));
          memcpy(Save_Data.latitude, localString, sizeof(localString));

          subString = subStringNext + 1;
          Save_Data.N_S = subString[0];

          subStringNext = strstr(subString, ",");
          if (subStringNext != NULL) {
            subString = subStringNext + 1;
            subStringNext = strstr(subString, ",");
            if (subStringNext != NULL) {
              // Extract longitude
              memset(localString, 0, sizeof(localString));
              memcpy(localString, subString, (subStringNext - subString));
              memcpy(Save_Data.longitude, localString, sizeof(localString));

              subString = subStringNext + 1;
              Save_Data.E_W = subString[0];
              
              #if 0
              DebugSerial.println(Save_Data.N_S);
              DebugSerial.println(Save_Data.E_W);
              DebugSerial.println(Save_Data.longitude);
              DebugSerial.println(Save_Data.latitude);
              DebugSerial.println("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
              #endif
              
              subStringNext = strstr(subString, ",");
              if (subStringNext != NULL) {
                // Override ground speed field
                subString = subStringNext + 1;
                subStringNext = strstr(subString, ",");
                if (subStringNext != NULL) {
                  subString = subStringNext + 1;
                  subStringNext = strstr(subString, ",");
                  if (subStringNext != NULL) {
                    subString = subStringNext + 1;
                    subStringNext = strstr(subString, ",");
                    memset(localString, 0, sizeof(localString));
                    memcpy(localString, subString, (subStringNext - subString));
                    memcpy(Save_Data.UTCDate, localString, sizeof(localString));

                    #if 0
                    DebugSerial.println("++++++++++ Extract Date Information ++++++++++++");
                    DebugSerial.println(subString);
                    DebugSerial.println(Save_Data.UTCDate);
                    #endif
                    
                    Save_Data.isParseData = true;
                    return 0;
                  }
                }
              }
            }
          }
        }
      }
    }
    else {
      errorLog(12);  //解析错误
      return -1;
    }
    #endif
    return -2;
	}
}

//
// Format Description:
// http://b8807053.pixnet.net/blog/post/3610870-gps%E8%B3%87%E6%96%99%E6%A0%BC%E5%BC%8F
// RMC : UTC時間、定位狀態（A－可用，V－可能有錯誤）、緯度值、經度值、對地速度、日期等
// $GPGSV,4,1,13,02,58,044,,05,56,346,,13,54,173,19,06,28,093,*7E
// 定位模式（M－手動，強制二維或三維定位；A－自動，自動二維或三維定位）、定位中使用的衛星ID號、PDOP值、HDOP值、VDOP值
//

int gpsRead() {
  char *subString;
  char *subStringNext;
  char checkSatNb[4];
  int nbSat;
    
	while (GpsSerial.available())
	{
		gpsRxBuffer[gpsRxCount] = GpsSerial.read();
		if (gpsRxBuffer[gpsRxCount++] == '\n')
		{
			char* GPS_BufferHead;
			char* GPS_BufferTail;

      /*
       * First parse xxGSV to understand how many satellites can be watched.
       */
      if (((GPS_BufferHead = strstr(gpsRxBuffer, "$GPGSV,")) != NULL) || ((GPS_BufferHead = strstr(gpsRxBuffer, "$GLGSV,")) != NULL)) {
        subString = GPS_BufferHead + 7;
        subStringNext = strstr(subString, ","); // Process from the start char just after "$GPGSV," to locate ","
        if (subStringNext != NULL) {
          subString = subStringNext + 1;  // Skip Nb of GSV msg
          subStringNext = strstr(subString, ",");
          if (subStringNext != NULL) {
            subString = subStringNext + 1;  // Skip Nb of self nb
            memset(checkSatNb, 0, sizeof(checkSatNb));
            memcpy(checkSatNb, subString, 2);
            nbSat = atoi(checkSatNb);
            if (nbSat < 5) {
              DebugSerial.print("Fail to locate satellite, ");
              DebugSerial.println(nbSat);
              return -1;
            }
            else {
              DebugSerial.print("Success to lock GPS satellite, ");
              DebugSerial.println(nbSat);
            }
          }
        }
      }
      
			if ((GPS_BufferHead = strstr(gpsRxBuffer, "$GPRMC,")) != NULL || (GPS_BufferHead = strstr(gpsRxBuffer, "$GNRMC,")) != NULL )
			{

				if (((GPS_BufferTail = strstr(GPS_BufferHead, "\r\n")) != NULL) && (GPS_BufferTail > GPS_BufferHead))
				{
					memcpy(Save_Data.GPS_Buffer, GPS_BufferHead, GPS_BufferTail - GPS_BufferHead);
					Save_Data.isGetData = true;
          #if 1
          DebugSerial.println("----------------- Received GPS RMC Raw Data ----------------------");
          DebugSerial.println(Save_Data.GPS_Buffer);
          DebugSerial.println("----------------- End of GPS RMC Raw Data ------------------------");
          #endif
				}

			}
      clrGpsRxBuffer();
		}
		if (gpsRxCount == gpsRxBufferLength)
		  clrGpsRxBuffer();
	}
  return 0;
}

void clrGpsRxBuffer(void)
{
	memset(gpsRxBuffer, 0, gpsRxBufferLength);      //清空
	gpsRxCount = 0;
}

double longitudeToOnenetFormat(char *lon_str_temp)
{
	double lon_temp = 0;
	long lon_Onenet = 0;
	int dd_int = 0;
	long mm_int = 0;
	double lon_Onenet_double = 0;

	lon_temp = atof(lon_str_temp);
	lon_Onenet = lon_temp * 100000; //转换为整数

	dd_int = lon_Onenet / 10000000; //取出dd

	mm_int = lon_Onenet % 10000000; //取出MM部分


	lon_Onenet_double = dd_int + (double)mm_int / 60 / 100000; //换算为Onenet格式


	return lon_Onenet_double;
}

double latitudeToOnenetFormat(char *lat_str_temp)
{
	double lat_temp = 0;
	long lat_Onenet = 0;
	int dd_int = 0;
	long mm_int = 0;

	double lat_Onenet_double = 0;

	lat_temp = atof(lat_str_temp);
	lat_Onenet = lat_temp * 100000; //转换为整数

	dd_int = lat_Onenet / 10000000; //取出dd

	mm_int = lat_Onenet % 10000000; //取出MM部分

	lat_Onenet_double = dd_int + (double)mm_int / 60 / 100000; //换算为Onenet格式


	return lat_Onenet_double;
}

void postGpsDataToOneNet(char* API_VALUE_temp, char* device_id_temp, char* sensor_id_temp, char* lon_temp, char* lat_temp, float paraVar)
{
	char send_buf[400] = {0};
	char text[100] = {0};
	char tmp[25] = {0};

	char lon_str_end[15] = {0};
	char lat_str_end[15] = {0};

	dtostrf(longitudeToOnenetFormat(lon_temp), 3, 6, lon_str_end); //转换成字符串输出
	dtostrf(latitudeToOnenetFormat(lat_temp), 2, 6, lat_str_end); //转换成字符串输出

	//连接服务器
	memset(send_buf, 0, 400);    //清空
	strcpy(send_buf, "AT+CIPSTART=\"TCP\",\"");
	strcat(send_buf, OneNetServer);
	strcat(send_buf, "\",80\r\n");
	if (sendCommand(send_buf, "CONNECT", 10000, 5) == Success);
	else errorLog(7);

	//发送数据
	if (sendCommand("AT+CIPSEND\r\n", ">", 3000, 1) == Success);
	else errorLog(8);

	memset(send_buf, 0, 400);    //清空

	/*准备JSON串*/
	//ARDUINO平台不支持sprintf的double的打印，只能转换到字符串然后打印
  if (! strcmp(sensor_id_temp, sensor_gps)) {
      sprintf(text, "{\"datastreams\":[{\"id\":\"%s\",\"datapoints\":[{\"value\":{\"lon\":%s,\"lat\":%s}}]}]}", sensor_id_temp, lon_str_end, lat_str_end);
  }
  if (! strcmp(sensor_id_temp, sensor_level)) {
      sprintf(text, "{\"datastreams\":[{\"id\":\"%s\",\"datapoints\":[{\"value\":{\"Water Level\":%f}}]}]}", sensor_id_temp, paraVar);
  }

  /*
   {
       "datastreams":[
           {
               "id":"Location",
               "datapoints":[
                   {
                       "value":{
                           "lon":Longitude,
                           "lat":Latitude
                       }
                   }
               ]
           }
       ]
   }
   */

	/*准备HTTP报头*/
	send_buf[0] = 0;
	strcat(send_buf, "POST /devices/");
	strcat(send_buf, device_id_temp);
	strcat(send_buf, "/datapoints HTTP/1.1\r\n"); //注意后面必须加上\r\n
	strcat(send_buf, "api-key:");
	strcat(send_buf, API_VALUE_temp);
	strcat(send_buf, "\r\n");
	strcat(send_buf, "Host:");
	strcat(send_buf, OneNetServer);
	strcat(send_buf, "\r\n");
	sprintf(tmp, "Content-Length:%d\r\n\r\n", strlen(text)); //计算JSON串长度
	strcat(send_buf, tmp);
	strcat(send_buf, text);

	if (sendCommand(send_buf, send_buf, 3000, 1) == Success);
	else errorLog(9);

	char sendCom[2] = {0x1A};
	if (sendCommand(sendCom, "\"succ\"}", 3000, 1) == Success);
	else errorLog(10);

	if (sendCommand("AT+CIPCLOSE\r\n", "CLOSE OK\r\n", 3000, 1) == Success);
	else errorLog(11);

	if (sendCommand("AT+CIPSHUT\r\n", "SHUT OK\r\n", 3000, 1) == Success);
	else errorLog(11);
}

void initGprs()
{
	if (sendCommand("AT\r\n", "OK\r\n", 3000, 10) == Success);
	else errorLog(1);

	if (sendCommand("AT+CREG?\r\n", "0,", 3000, 10) == Success);	//本地SIM卡
	else if(sendCommand("AT+CREG?\r\n", ",5", 3000, 10) == Success	);//漫游SIM卡
	else	errorLog(3);
	delay(10);

	if (sendCommand("AT+CGCLASS=\"B\"\r\n", "OK\r\n", 3000, 2) == Success);
	else errorLog(3);
  // 3gnet for UNICOM, cmnet for CMMC
	if (sendCommand("AT+CGDCONT=1,\"IP\",\"cmnet\"\r\n", "OK", 3000, 2) == Success);
	else errorLog(4);

	if (sendCommand("AT+CGATT=1\r\n", "OK\r\n", 3000, 2) == Success);
	else errorLog(5);

	if (sendCommand("AT+CLPORT=\"TCP\",\"2000\"\r\n", "OK\r\n", 3000, 2) == Success);
	else errorLog(6);
  #if 1
  DebugSerial.print("GPRS init ok.");
  #endif
}

void(* resetFunc) (void) = 0; //制造重启命令

void errorLog(int num)
{
	DebugSerial.print("ERROR");
	DebugSerial.println(num);
	while (1)
	{
		digitalWrite(L, HIGH);
		delay(300);
		digitalWrite(L, LOW);
		delay(300);

		if (sendCommand("AT\r\n", "OK\r\n", 100, 10) == Success)
		{
			DebugSerial.print("\r\nRESET!!!!!!\r\n");
			resetFunc();
		}
	}
}



unsigned int sendCommand(char *Command, char *Response, unsigned long Timeout, unsigned char Retry)
{
	clrGprsRxBuffer();
	for (unsigned char n = 0; n < Retry; n++)
	{
		DebugSerial.print("\r\n---------send AT Command:---------\r\n");
		DebugSerial.write(Command);

		GprsSerail.write(Command);

		Time_Cont = 0;
		while (Time_Cont < Timeout)
		{
			gprsReadBuffer();
			if (strstr(gprsRxBuffer, Response) != NULL)
			{
				DebugSerial.print("\r\n==========receive AT Command:==========\r\n");
				DebugSerial.print(gprsRxBuffer); //输出接收到的信息
				clrGprsRxBuffer();
				return Success;
			}
		}
		Time_Cont = 0;
	}
	DebugSerial.print("\r\n==========receive AT Command:==========\r\n");
	DebugSerial.print(gprsRxBuffer);//输出接收到的信息
	clrGprsRxBuffer();
	return Failure;
}

void Timer1_handler(void)
{
	Time_Cont++;
	Time_Cont2++;
}

void gprsReadBuffer() {
	while (GprsSerail.available())
	{
		gprsRxBuffer[gprsBufferCount++] = GprsSerail.read();
		if (gprsBufferCount == gprsRxBufferLength)clrGprsRxBuffer();
	}
}

void clrGprsRxBuffer(void)
{
	memset(gprsRxBuffer, 0, gprsRxBufferLength);      //清空
	gprsBufferCount = 0;
}
