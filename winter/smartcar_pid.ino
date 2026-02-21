#include <Servo.h>
//#define TEST
Servo myservo;  //声明myservo为 Servo类的一个实例，在这之后就可以通过myservo调用Servo类的函数

// 引脚定义
int MOTOR_LEFT_BEHIND = 3;    //左后电机PWM引脚定义
int MOTOR_RIGHT_BEHIND = 11;  //右后电机PWM引脚定义
int SERVO = 9;  //舵机PWM引脚定义
//传感器输入端口定义
int D1 = A0;
int D2 = A1;
int D3 = A2;
int D4 = A3;
int D5 = A4;
int D6 = 2;
int D8 = 7;
int D9 = 8;
int D10 = 12;
//int D11 = 13;
int D11 = A5;
int D12 = 4;

//全局变量初始化
static int MotorPWM = 0;          //电机PWM值(0-100)
static int ServoAngle = 0;        //舵机角度值(-90-90度)
int digitalSensor_Binary = 0;     //当前传感器状态对应数值(二进制表示) 
int Last_digitalSensor_Binary = 0;  //上一次传感器状态对应数值
int nowErr = 0;   //当前误差
int lastErr = 0;  //上一次误差
int carMode = 1;  //运行模式

// 电机差速表
#define TABLE_DIMENSION    70
const int speedDifferenceTable[TABLE_DIMENSION]  ={100,98,98,97,97,95,95,93,93,92,
                                                    92,90,90,88,88,87,87,85,85,84,
                                                    84,82,82,81,81,79,79,78,78,76,
                                                    76,75,75,73,73,72,72,71,71,69,
                                                    69,68,68,66,66,65,65,64,64,62,
                                                    62,61,61,59,59,58,58,57,57,55,
                                                    55,54,52,51,50, 48,47,45,44,42};
                                                    //41,39,38,36,35, 33,32,30,29,27};
                                                    //25,24,22,20,19, 17,15,13,11,9};

#define SERVO_CENTER 90   //舵机中值
#define MAX_ANGLE 35      //舵机角度上限
//设置不同偏差对应的速度(直道加速 弯道减速)
int straightSpeed = 45;
int R600Speed     = 35;
int R450Speed     = 38;
//定义不同位置传感器对应的偏差量，用于计算当前偏差，具体使用见GetErr()函数
int sensorDistance[11] = {45, 28, 18, 8, 2, 0, -2, -8, -18, -28, -45};  
float Kp = 1.05; //舵机打角的比例系数
float Kd = 18;
// #define TEST


/************************************************************************
 * 
 * 函数功能         初始化                                                                                     
 * 输入参数         NONE                                                
 * 返回值           NONE                                                
 *
 ************************************************************************/
void setup()
{
  myservo.attach(SERVO);  //将SERVO引脚连接到myservo
  Serial.begin(115200);
  Serial.println();
}


/************************************************************************
 * 
 * 函数功能         计算角度对应占空比(16bit)并设限制舵机最大打角，保护舵机，防止舵机堵转                                                                                     
 * 输入参数         angle                                                
 * 返回值           NONE                                                
 *
 ************************************************************************/
void SetServoAngle(int angle)
{
  if(angle > MAX_ANGLE)                      
  {
      angle = MAX_ANGLE;
  }
  else if(angle < -MAX_ANGLE)
  {
      angle = -MAX_ANGLE;
  }
  if(angle > 0)
  {
    angle=angle * 3/2;
  }
  else
  {
    angle=angle*1.1;
  }
  myservo.write(SERVO_CENTER + angle);  //输出角度SERVO_CENTER + angle
}

/************************************************************************
 * 
 * 函数功能         差速计算并通过PWMOut2输出电机对应pwm(两驱方案以外侧后轮为基准，内轮乘以<1的系数实现减速)                                                                               
 * 输入参数         pwmduty,angle                                                
 * 返回值           NONE                                                
 *
 ************************************************************************/
void PWMOut(int pwmduty, int angle)
{
  char InsideFDiff = 0;     // 内侧电机的差速系数

  if (angle > 0)
  {
    if (angle > (TABLE_DIMENSION - 1))  //防止寻址溢出
    {
        angle = TABLE_DIMENSION - 1;
    }
    InsideFDiff  = speedDifferenceTable[angle];
    analogWrite(MOTOR_LEFT_BEHIND , pwmduty * 255 / 100);
    analogWrite(MOTOR_RIGHT_BEHIND , (pwmduty * InsideFDiff / 100) * 255 / 100);
  }
  else
  {
    angle = 0 - angle;
    if (angle > (TABLE_DIMENSION - 1))
    {
        angle = TABLE_DIMENSION - 1;
    }
    InsideFDiff  = speedDifferenceTable[angle];
    analogWrite(MOTOR_LEFT_BEHIND , (pwmduty * InsideFDiff / 100) * 255 / 100);
    analogWrite(MOTOR_RIGHT_BEHIND , pwmduty * 255 / 100);
  }
  // char strOut[100];
  // sprintf(strOut, "MOTOR_LEFT_BEHIND:%u\r  MOTOR_RIGHT_BEHIND:%d\r\n", (pwmdutyTemp * InsideFDiff  / 100),pwmdutyTemp);
  // Serial.println(strOut);
}


/************************************************************************
 * 
 * 函数功能         获得传感器状态                                                                               
 * 输入参数         NONE                                              
 * 返回值           DSensorValue                                                
 *
 ************************************************************************/
int GetDigitalSensorValue(bool print)
{
  int DSensorValue = 0;
  int Value[11] = {0};
  int Threshold = 700;

  Value[0] = analogRead(D1)>Threshold;  //D1对应最左侧传感器
  Value[1] = analogRead(D2)>Threshold;
  Value[2] = analogRead(D3)>Threshold;
  Value[3] = analogRead(D4)>Threshold;
  Value[4] = analogRead(D5)>Threshold;
  Value[5] = digitalRead(D6);
  Value[6] = digitalRead(D8);
  Value[7] = digitalRead(D9);
  Value[8] = digitalRead(D10);
  Value[9] = digitalRead(D11);
  Value[10] = digitalRead(D12);

  if(print==1)
  {
    for(int i = 0; i < 11; ++i)
    {
      Serial.print(1 - Value[i]);
    }   
    Serial.println();
  }
  //将11个传感器状态转成11位2进制数,从00000000000(十进制0)到11111111111(十进制2047)
  DSensorValue |= (!(Value[10] & 0x01));
  DSensorValue |= (!(Value[9] & 0x01))  << 1;
  DSensorValue |= (!(Value[8] & 0x01))  << 2;
  DSensorValue |= (!(Value[7] & 0x01))  << 3;
  DSensorValue |= (!(Value[6] & 0x01))  << 4; 
  DSensorValue |= (!(Value[5] & 0x01))  << 5;
  DSensorValue |= (!(Value[4] & 0x01))  << 6;
  DSensorValue |= (!(Value[3] & 0x01))  << 7;
  DSensorValue |= (!(Value[2] & 0x01))  << 8;
  DSensorValue |= (!(Value[1] & 0x01))  << 9;
  DSensorValue |= (!(Value[0] & 0x01))  << 10;
  return DSensorValue;
}

/************************************************************************
 * 
 * 函数功能         GetErr                                                                               
 * 输入参数         NONE                                              
 * 返回值           Err                                                
 * 修改时间:                                                
 * 01a 2017-10-21 14:41:48   
 *
 ************************************************************************/
int GetErr(void)
{
  int i                    = 0;
  int iPositionErr         = 0;
  static int s_iLastErr    = 0;
  unsigned int sensor      = 0;
  unsigned char lightCount = 0; //检测到黑线的传感器数量
  
  sensor = digitalSensor_Binary; //将传感器状态赋给临时变量，防止操作过程被误修改
  for(i = 0; i < 11; i ++)//12个灯的误差进行累加
  { 
    if (sensor & 0x01)
    {
      /* if the light is too far from last m ,do not count*/
      if (((sensorDistance[i] - nowErr) < 20) && ((sensorDistance[i] - nowErr) > -20))
      {
        iPositionErr += sensorDistance[i];
        lightCount++;
      }
    }
    sensor >>= 1;
  } 

  if (lightCount > 0)//平均误差为总误差除以感应到的灯数
  {
    s_iLastErr = iPositionErr / lightCount;
    return (iPositionErr / lightCount);
  }
  else
  {
     return s_iLastErr;
  } 
}

/************************************************************************
 * 
 * 函数功能         Control                                                                               
 * 输入参数         NONE                                              
 * 返回值           NONE                                                 
 *
 ************************************************************************/
void Control(void)
{
  int servoAngleTemp = 0;

  //计算当前偏差
  nowErr = GetErr(); 
 // servoAngleTemp = Kp*nowErr;
  servoAngleTemp = Kp*nowErr+Kd*(nowErr-lastErr);
  // char strOut[100];
  // sprintf(strOut, "angle:%d  pout:%d  dout:%d", servoAngleTemp,(int)Kp*nowErr,(int)Kd*(nowErr-lastErr));
  // Serial.println(strOut);
  lastErr=nowErr;

  // Serial.println(angle);
  ServoAngle = servoAngleTemp;
  
  //设置舵机角度
  SetServoAngle(ServoAngle);

  //设置电机速度
  //由打角决定差速，把打角都变为绝对值
  if(servoAngleTemp < 0)
  {
      servoAngleTemp = - servoAngleTemp;
  }
  
  if((servoAngleTemp >= 0) && (servoAngleTemp < 5))   //打角0-5度
  {
      MotorPWM = straightSpeed;
      PWMOut(MotorPWM, 0);
  }
  else if(servoAngleTemp < 13)    //打角5-20度
  {
      MotorPWM = R600Speed;
      PWMOut(MotorPWM, ServoAngle);
  }
  else                            //打角20度以上
  {
      MotorPWM = R450Speed;
      PWMOut(MotorPWM, ServoAngle);
  }
}

void loop()
{
  //如果是在测试模式下(#define TEST)，则执行以下代码(#ifdef至#else中的代码)
  #ifdef TEST 
  SetServoAngle(0);
   if(Serial.available())
  {
    String input = Serial.readString();
    input.trim();
    
    if(input == "c") {
      myservo.write(90);  
      Serial.println("标准中心: 90°");
    }
    else if(input == "l") {
      myservo.write(60);  
      Serial.println("左转: 60°");
    }
    else if(input == "r") {
      myservo.write(120); 
      Serial.println("右转: 120°");
    }
    else {
      int out = input.toInt();
      int angle = 90 + out;
      myservo.write(angle);
      Serial.print("设置偏移: ");
      Serial.print(out);
      Serial.print(", 实际角度: ");
      Serial.println(angle);
    }
  }

  #else
  //正常运行模式(没有#define TEST)
  digitalSensor_Binary = GetDigitalSensorValue(1);//获取传感器状态，用二进制数表示从11-0

  switch(carMode)
  {
    case 1://正常运行模式
        Control();
        if((Last_digitalSensor_Binary== 0x0001) && (digitalSensor_Binary == 0x0000))
        {
          carMode = 21;
        }
        else if((Last_digitalSensor_Binary == 0x0400) && (digitalSensor_Binary == 0x0000))
        {
          carMode = 22;
        }
        break;
        
    case 21://右侧传感器丢线
        SetServoAngle(MAX_ANGLE);
        analogWrite(MOTOR_LEFT_BEHIND , 0.3*255);
        analogWrite(MOTOR_RIGHT_BEHIND , 0);
        Serial.println("right_lose");
        if(digitalSensor_Binary & 0x000F) //0x0007=00000000111,最右侧三个传感器之一重新检测到黑线
        {
          carMode = 1; //回到正常运行状态
        }
        break;

    case 22://左侧传感器丢线
        SetServoAngle(-MAX_ANGLE);
        analogWrite(MOTOR_LEFT_BEHIND , 0);
        analogWrite(MOTOR_RIGHT_BEHIND , 0.4*255);
        Serial.println("left_lose");
        if(digitalSensor_Binary & 0x0F00) //0x0700=11100000000,最左侧三个传感器之一重新检测到黑线
        {
          carMode = 1;
        }
        break;

    default:
        carMode = 1;
        break;
    
  }
  Last_digitalSensor_Binary = digitalSensor_Binary;
  #endif
}
