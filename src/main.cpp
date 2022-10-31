#include <DFRobot_BMI160.h>
#include <Arduino.h>
#include <WiFi.h>
#include "WiFiUdp.h"
#include "DFRobot_AS7341.h"
#define serialPlotterDebug 0 // 0:serialPlotter; 1:normal run
#define AS7341ScanDebug 0    // 0:六波段光谱检测 1:八波段光谱检测

RTC_DATA_ATTR float ACCEL[10];
RTC_DATA_ATTR float UV[10];
RTC_DATA_ATTR int cyclesCount = 0;
RTC_DATA_ATTR int bootCount = 0;

/*************************** AS7341 ******************************/

DFRobot_AS7341 as7341;
DFRobot_AS7341::sModeOneData_t data1;
DFRobot_AS7341::sModeTwoData_t data2;

void AS7341Scan(void)
{
  while (as7341.begin() != 0)
  {
#if serialPlotterDebug
    Serial.print("IIC初始化失败，请检测连线是否正确");
#endif
    delay(100);
  }
  /*************************** freq ******************************/
  // uint8_t freq = 0;
  // //Read the value of register flicker, through which the flicker frequency of the light source can be predicted
  // freq = as7341.readFlickerData();
  // if (freq == 1) {
  //   Serial.println("Unknown frequency");
  // } else if (freq == 0) {
  //   Serial.println("No flicker");
  // } else {
  //   Serial.print(freq);
  //   Serial.println("Hz");
  // }
  /***************************************************************/
  //  //Integration time = (ATIME + 1) x (ASTEP + 1) x 2.78µs
  //  //设置寄存器ATIME的值(1-255)，通过该值可计算Integration time的值，该值表示读取数据过程中必须要消耗的时间
  //  as7341.setAtime(29);
  //  //设置ASTEP寄存器的值(0-65534)，通过该值可计算Integration time的值，该值表示读取数据过程中必须要消耗的时间
  //  as7341.setAstep(599);
  // //设置增益(0~10对应 X0.5,X1,X2,X4,X8,X16,X32,X64,X128,X256,X512)
  // as7341.setAGAIN(9);
  // //  使能LED
  // as7341.enableLed(true);
  // //  设置引脚电流控制亮度(1~20对应电流 4mA,6mA,8mA,10mA,12mA,......,42mA)
  // as7341.controlLed(4);

  //开始光谱的测量.
  //通道映射的模式 :1.eF1F4ClearNIR,2.eF5F8ClearNIR
  as7341.startMeasure(as7341.eF1F4ClearNIR);
  //读取传感器数据通道0~5的值，eF1F4ClearNIR模式下.
  data1 = as7341.readSpectralDataOne();

#if AS7341ScanDebug
  Serial.print("F1(405-425nm):");
  Serial.println(data1.ADF1);
  Serial.print(",");
#endif

  Serial.print("F2(435-455nm):");
  Serial.println(data1.ADF2);
  Serial.print(",");

  Serial.print("F3(470-490nm):");
  Serial.println(data1.ADF3);
  Serial.print(",");

  Serial.print("F4(505-525nm):");
  Serial.println(data1.ADF4);
  Serial.print(",");

  as7341.startMeasure(as7341.eF5F8ClearNIR); //读取传感器数据通道0~5的值,eF5F8ClearNIR模式下.
  data2 = as7341.readSpectralDataTwo();

  Serial.print("F5(545-565nm):");
  Serial.println(data2.ADF5);
  Serial.print(",");

  Serial.print("F6(580-600nm):");
  Serial.println(data2.ADF6);
  Serial.print(",");

  Serial.print("F7(620-640nm):");
  Serial.println(data2.ADF7);
  Serial.print(",");

#if AS7341ScanDebug
  Serial.print("F8(670-690nm):");
  Serial.println(data2.ADF8);
  Serial.print(",");
#endif

  // Serial.print("Clear:");  // 可见光
  // Serial.println(data2.ADCLEAR);
  // Serial.print("NIR:");  // 近红外光
  // Serial.println(data2.ADNIR);
}

/*************************** WIFI ******************************/

#define ssid "liu"          //这里改成你的设备当前环境下WIFI名字
#define password "33333333" //这里改成你的设备当前环境下WIFI密码

WiFiUDP Udp;                       //实例化WiFiUDP对象
unsigned int localUdpPort = 1234;  // 自定义本地监听端口
unsigned int remoteUdpPort = 4321; // 自定义远程监听端口
char incomingPacket[255];          // 保存Udp工具发过来的消息

//向udp工具发送消息
void sendCallBack(const char *buffer, float fVal)
{
  Udp.beginPacket(Udp.remoteIP(), remoteUdpPort); //配置远端ip地址和端口
  Udp.print(buffer);                              //把数据写入发送缓冲区  Udp.beginPacket(Udp.remoteIP(), remoteUdpPort);//配置远端ip地址和端口
  Udp.println(fVal);                              //把数据写入发送缓冲区
  Udp.endPacket();                                //发送数据
}

void Esp32WifiInit()
{
#if serialPlotterDebug
  Serial.printf("正在连接 %s ", ssid);
#endif

  WiFi.begin(ssid, password);           //连接到wifi
  while (WiFi.status() != WL_CONNECTED) //等待连接
  {
#if serialPlotterDebug
    Serial.print(".");
#endif
    delay(500);
  }
#if serialPlotterDebug
  Serial.println("连接成功");
#endif

  if (Udp.begin(localUdpPort))
  { //启动Udp监听服务
#if serialPlotterDebug
    Serial.println("监听成功");
    //打印本地的ip地址，在UDP工具中会使用到
    // WiFi.localIP().toString().c_str()用于将获取的本地IP地址转化为字符串
    Serial.printf("现在收听IP：%s, UDP端口：%d\n", WiFi.localIP().toString().c_str(), localUdpPort);
#endif
  }
  else
  {
#if serialPlotterDebug
    Serial.println("监听失败");
#endif
  }
}

void Wifiloop()
{
  //解析Udp数据包
  int packetSize = Udp.parsePacket(); //获得解析包
  if (packetSize)                     //解析包不为空
  {
//收到Udp数据包
// Udp.remoteIP().toString().c_str()用于将获取的远端IP地址转化为字符串
#if serialPlotterDebug
    Serial.printf("收到来自远程IP：%s（远程端口：%d）的数据包字节数：%d\n", Udp.remoteIP().toString().c_str(), Udp.remotePort(), packetSize);
#endif
    // 读取Udp数据包并存放在incomingPacket
    int len = Udp.read(incomingPacket, 255); //返回数据包字节数
    if (len > 0)
    {
      incomingPacket[len] = 0; //清空缓存
#if serialPlotterDebug
      Serial.printf("UDP数据包内容为: %s\n", incomingPacket); //向串口打印信息
#endif
      // strcmp函数是string compare(字符串比较)的缩写，用于比较两个字符串并根据比较结果返回整数。
      //基本形式为strcmp(str1,str2)，若str1=str2，则返回零；若str1<str2，则返回负数；若str1>str2，则返回正数。

      // sendCallBack("YES!", 1);
    }
    sendCallBack("F1(405-425nm):", data1.ADF1);
    delay(10);
    sendCallBack("F2(435-455nm):", data1.ADF2);
    delay(10);
    sendCallBack("F3(470-490nm):", data1.ADF3);
    delay(10);
    sendCallBack("F4(505-525nm):", data1.ADF4);
    delay(10);
    sendCallBack("F5(545-565nm):", data2.ADF5);
    delay(10);
    for (int i = 0; i < 10; i++)
    {
      float TrueVal = ACCEL[i];
      float TrueUv = UV[i];
      sendCallBack("Angle =  ", TrueVal);
      sendCallBack("UV Intensity: ", TrueUv);
      delay(10);
    }
  }
}

/*************************** POWER ******************************/

#define uS_TO_S_FACTOR 1000000ULL /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 7           /* Time ESP32 will go to sleep (in seconds) */

/*
Method to print the reason by which ESP32
has been awaken from sleep
*/
void print_wakeup_reason()
{
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason)
  {
#if serialPlotterDebug
  case ESP_SLEEP_WAKEUP_EXT0:
    Serial.println("Wakeup caused by external signal using RTC_IO");
    break;
  case ESP_SLEEP_WAKEUP_EXT1:
    Serial.println("Wakeup caused by external signal using RTC_CNTL");
    break;
  case ESP_SLEEP_WAKEUP_TIMER:
    Serial.println("Wakeup caused by timer");
    break;
  case ESP_SLEEP_WAKEUP_TOUCHPAD:
    Serial.println("Wakeup caused by touchpad");
    break;
  case ESP_SLEEP_WAKEUP_ULP:
    Serial.println("Wakeup caused by ULP program");
    break;
  default:
    Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason);
#endif
    break;
  }
}
/*************************** UV ******************************/
int ReadUVintensityPin = 1; // Output from the sensor

float mapfloat(float x, float in_min/*噪声*/, float in_max, float out_min/*偏移*/, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Takes an average of readings on a given pin
// Returns the average
int averageAnalogRead(int pinToRead)
{
  byte numberOfReadings = 8;//Averg_times
  unsigned int runningValue = 0;

  for (int x = 0; x < numberOfReadings; x++)
    runningValue += analogRead(pinToRead);
  runningValue /= numberOfReadings;

  return (runningValue);
}

void UVInit()
{
  pinMode(ReadUVintensityPin, INPUT);
}

void UVDisplay()
{
  int uvLevel = averageAnalogRead(ReadUVintensityPin);

  float outputVoltage = 5.0 * uvLevel / 1024;
  float uvIntensity = mapfloat(outputVoltage, 0.99, 2.9, 0.0, 15.0);

  // Serial.print("UVAnalogOutput: ");
  // Serial.println(uvLevel);

  // Serial.print(" OutputVoltage: ");
  // Serial.print(outputVoltage);

#if !AS7341ScanDebug
  Serial.print("UV Intensity: ");
  Serial.println(uvIntensity);
  // Serial.print(" mW/cm^2");
  Serial.print(",");
#endif

  // if (cyclesCount == 10)
  //   UV[bootCount - 1] = uvIntensity;
  // sendCallBack(" UV Intensity: ", uvIntensity);
}

/*************************** MPU6050 ******************************/
DFRobot_BMI160 bmi160;
const int8_t i2c_addr = 0x69;

int16_t ax, ay, az;
int16_t gx, gy, gz;
double total_angle = 0;
// #define LED_PIN 13

/* 把mpu6050放在水平桌面上，分别读取读取2000次，然后求平均值 */
#define AZ_ZERO (747.25) /* 加速度计的0偏修正值 */
#define GY_ZERO (11.52)  /* 陀螺仪的0偏修正值 */

/***************************************************************/
#define AY_ZERO (0) /* 加速度计的0偏修正值 */
#define GX_ZERO (0) /* 陀螺仪的0偏修正值 */
/***************************************************************/

/* 通过卡尔曼滤波得到的最终角度 */
float Angle = 0.0;

/*由角速度计算的倾斜角度 */
// float Angle_gy = 0.0;

float Q_angle = 0.001;
float Q_gyro = 0.003;
float R_angle = 0.03;
float dt = 0.005; /* dt为kalman滤波器采样时间; */
char C_0 = 1;
float Q_bias, Angle_err;
float PCt_0 = 0.0, PCt_1 = 0.0, E = 0.0;
float K_0 = 0.0, K_1 = 0.0, t_0 = 0.0, t_1 = 0.0;
float Pdot[4] = {0, 0, 0, 0};
float PP[2][2] = {{1, 0}, {0, 1}};

/* 卡尔曼滤波函数 */
void Kalman_Filter(float Accel, float Gyro)
{
  Angle += (Gyro - Q_bias) * dt;

  Pdot[0] = Q_angle - PP[0][1] - PP[1][0];

  Pdot[1] = -PP[1][1];
  Pdot[2] = -PP[1][1];
  Pdot[3] = Q_gyro;

  PP[0][0] += Pdot[0] * dt;
  PP[0][1] += Pdot[1] * dt;
  PP[1][0] += Pdot[2] * dt;
  PP[1][1] += Pdot[3] * dt;

  Angle_err = Accel - Angle;

  PCt_0 = C_0 * PP[0][0];
  PCt_1 = C_0 * PP[1][0];

  E = R_angle + C_0 * PCt_0;

  if (E != 0)
  {
    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;
  }

  t_0 = PCt_0;
  t_1 = C_0 * PP[0][1];

  PP[0][0] -= K_0 * t_0;
  PP[0][1] -= K_0 * t_1;
  PP[1][0] -= K_1 * t_0;
  PP[1][1] -= K_1 * t_1;

  Angle += K_0 * Angle_err;
  Q_bias += K_1 * Angle_err;
}

void MPU6050Init()
{
  if (bmi160.softReset() != BMI160_OK)
  {
#if serialPlotterDebug
    Serial.print("reset false");
#endif
    while (1)
      ;
  }

  // set and init the bmi160 i2c address
  if (bmi160.I2cInit(i2c_addr) != BMI160_OK)
  {
#if serialPlotterDebug
    Serial.print("init false");
#endif
    while (1)
      ;
  }
}

/*********************************************************/

void setup()
{
  Serial.begin(115200);
  delay(100);

  // Increment boot number and print it every reboot
  ++bootCount;
  if (bootCount == 10)
    bootCount = 0;
  cyclesCount = 0;
  // Serial.println("Boot number: " + String(bootCount));

  /*************************** POWER ******************************/
  // // Print the wakeup reason for ESP32
  // print_wakeup_reason();

  // /*
  // First we configure the wake up source
  // We set our ESP32 to wake up every 5 seconds
  // */
  // esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  // Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) +
  //                " Seconds");
  // Serial.println();
  /****************************************************************/

  MPU6050Init();

  UVInit();

  /*************************** WIFI ******************************/
  // Esp32WifiInit();
  // Serial.println();
  /****************************************************************/
}
unsigned long mictime = 0;
static unsigned long pretime = 0;
double az_angle = 0.0;
double gy_angle = 0.0;
float gyro = 0.0;

/***************************************************************/
double ay_angle = 0.0;
double gx_angle = 0.0;
float gxro = 0.0;
/***************************************************************/

void loop()
{
  // Wifiloop();
  ++cyclesCount;

  unsigned long time = 0;
  int i = 0;
  int rslt;
  int16_t accelGyro[6] = {0};

  // get both accel and gyro data from bmi160
  // parameter accelGyro is the pointer to store the data
  rslt = bmi160.getAccelGyroData(accelGyro);
  if (rslt == 0)
  {
    for (i = 0; i < 6; i++)
    {
      if (i < 3)
      {
        // the first three are gyro data
        // Serial.print(accelGyro[i] * 3.14 / 180.0);
        // Serial.print("\t");
        if (i == 0)
          gx = accelGyro[i];
        if (i == 1)
          gy = accelGyro[i];
        if (i == 2)
          gz = accelGyro[i];
      }
      else
      {
        // the following three data are accel data
        // Serial.print(accelGyro[i] / 16384.0);
        // Serial.print("\t");
        if (i == 3)
          ax = accelGyro[i];
        if (i == 4)
          ay = accelGyro[i];
        if (i == 5)
          az = accelGyro[i];
      }
    }
  }
  else
  {
#if serialPlotterDebug
    Serial.print("err");
#endif
  }

  if (pretime == 0)
  {
    pretime = millis();
    return;
  }
  mictime = millis();
  time = mictime - pretime;
  /* 加速度量程范围设置2g 16384 LSB/g
   * 计算公式：
   * 前边已经推导过这里再列出来一次
   * x是小车倾斜的角度,y是加速度计读出的值
   * sinx = 0.92*3.14*x/180 = y/16384
   * x=180*y/(0.92*3.14*16384)=
   */
  az -= AZ_ZERO;
  az_angle = az / 262;

  /***************************************************************/
  ay -= AY_ZERO;
  ay_angle = ay / 262;
  /***************************************************************/

  /* 陀螺仪量程范围设置250 131 LSB//s
   * 陀螺仪角度计算公式:
   * 小车倾斜角度是gx_angle,陀螺仪读数是y,时间是dt
   * gx_angle +=(y/(131*1000))*dt
   */

  gy -= GY_ZERO;
  gyro = gy / 131.0;

  gx -= GX_ZERO;
  gxro = gx / 131.0;

  pretime = mictime;

  gy_angle = gyro * time;
  gy_angle = gy_angle / 1000.0;

  gx_angle = gxro * time;
  gx_angle = gx_angle / 1000.0;
  /***************************************************************/

  total_angle -= gy_angle;

  //  获取运行时间必须放kalman最后
  dt = time / 1000.0;
  Kalman_Filter(az_angle, gyro);

// Serial.print(az_angle);
// Serial.print("   ,   ");
// Serial.print(total_angle);
// Serial.print("   ,   ");
#if !AS7341ScanDebug
  Serial.print("Angle1:");
  Serial.println(Angle);
  Serial.print(",");
#endif
  /***************************************************************/
//   Kalman_Filter(ay_angle, gxro);

// // Serial.print(az_angle);
// // Serial.print("   ,   ");
// // Serial.print(total_angle);
// // Serial.print("   ,   ");
// #if !AS7341ScanDebug
//   Serial.print("Angle2:");
//   Serial.println(Angle);
//   Serial.print(",");
// #endif
  /***************************************************************/

  // if (cyclesCount == 10)
  //   ACCEL[bootCount - 1] = Angle;

  UVDisplay();

  // if (cyclesCount == 10) //进入休眠
  {
    AS7341Scan();
    // delay(100);
    // Serial.println("Going to sleep now");
    // esp_deep_sleep_start();
    cyclesCount = 0;
    MPU6050Init();
    mictime = 0;pretime = 0;
  }
  // delay(50);
}
