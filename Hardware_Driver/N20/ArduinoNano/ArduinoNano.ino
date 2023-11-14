#include <SCoop.h>
#include "AngleSensor.h"
#include "N20.h"

#define MAX_POSITION 2600
const int LED_pin=13;
const int motor_A1[3] = {2,6,10},
          motor_A2[3] = {3,7,11},
          motor_B1[3] = {4,8,12},
          motor_B2[3] = {5,9,13};
const int ADC_pin[3] = {A0,A1,A2};

class Motor{
  private:
    int A1_pin, A2_pin, B1_pin, B2_pin;
    bool dir=0, ctrl_mode=1;                          //0:position 1:speed
    int tem_step = 1;
    uint16_t position=0, target_pos=0;
    int freq=10;
    const short next[5] = {0,4,3,1,2},
                last[5] = {0,3,4,2,1};
  public:
    Motor() {}
    Motor(int A1_pin, int A2_pin, int B1_pin, int B2_pin){
      this->A1_pin = A1_pin;
      this->A2_pin = A2_pin;
      this->B1_pin = B1_pin;
      this->B2_pin = B2_pin;
    }
    void motor_run(void);
    void set_step(int ID);
    void set_speed(void);
    void set_position(void);
    void reset_position(void);
    void set_ctrl_mode(bool mode, uint16_t tar_pos=0);
    void set_ctrl_mode(bool mode, bool dir, int delay_ms=10);
    uint16_t get_position(void);
    int get_speed(void);
};

/**
 * 单步控制函数
*/
void Motor::set_step(int ID)
{
  switch(ID) {
    case 1:
      digitalWrite(A1_pin,HIGH);
      digitalWrite(A2_pin,LOW);
      digitalWrite(B1_pin,LOW);
      digitalWrite(B2_pin,LOW);
      break;
    case 2:
      digitalWrite(A1_pin,LOW);
      digitalWrite(A2_pin,HIGH);
      digitalWrite(B1_pin,LOW);
      digitalWrite(B2_pin,LOW);
      break;
    case 3:
      digitalWrite(A1_pin,LOW);
      digitalWrite(A2_pin,LOW);
      digitalWrite(B1_pin,HIGH);
      digitalWrite(B2_pin,LOW);
      break;
    case 4:
      digitalWrite(A1_pin,LOW);
      digitalWrite(A2_pin,LOW);
      digitalWrite(B1_pin,LOW);
      digitalWrite(B2_pin,HIGH);
      break;
    case 0:
      digitalWrite(A1_pin,LOW);
      digitalWrite(A2_pin,LOW);
      digitalWrite(B1_pin,LOW);
      digitalWrite(B2_pin,LOW);
      break;
    case 5:
      digitalWrite(A1_pin,HIGH);
      digitalWrite(A2_pin,LOW);
      digitalWrite(B1_pin,HIGH);
      digitalWrite(B2_pin,LOW);
      break;
    case 6:
      digitalWrite(A1_pin,HIGH);
      digitalWrite(A2_pin,LOW);
      digitalWrite(B1_pin,LOW);
      digitalWrite(B2_pin,HIGH);
      break;
    case 7:
      digitalWrite(A1_pin,LOW);
      digitalWrite(A2_pin,HIGH);
      digitalWrite(B1_pin,LOW);
      digitalWrite(B2_pin,HIGH);
      break;
    case 8:
      digitalWrite(A1_pin,LOW);
      digitalWrite(A2_pin,HIGH);
      digitalWrite(B1_pin,HIGH);
      digitalWrite(B2_pin,LOW);
      break;
    default:
      break;
  }
  return;
}

void Motor::motor_run(void){
  if (ctrl_mode) set_speed();
  else set_position();
}
void Motor::set_speed(void) {
  if(dir) {
    tem_step=next[tem_step];
    set_step(tem_step);
    if(position==MAX_POSITION) position=0;
    else position++;
  }
  else {
    tem_step=last[tem_step];
    set_step(tem_step);
    if(position==0) position=MAX_POSITION;
    else position--;
  }
  sleep(freq);
  return;
}

void Motor::set_position(void)
{
  int dis = target_pos-position;
  if(dis==0) return;
  if(dis > 0){
    if(dis < MAX_POSITION/2) {
      dir = 1;
    }
    else dir = 0;
  }
  else {
    if(-dis < MAX_POSITION/2) {
      dir = 0;
    }
    else dir = 1;
  }
  set_speed();
  return;
}

void Motor::reset_position(void)
{
  position=0;
  target_pos=0;
  return;
}

void Motor::set_ctrl_mode(bool mode, uint16_t tar_pos)
{
  ctrl_mode=mode;
  target_pos=tar_pos;
  return;
}

void Motor::set_ctrl_mode(bool mode, bool dir, int delay_ms)
{
  ctrl_mode=mode;
  this->dir=dir;                                 //this用于区分同名变量
  this->freq=delay_ms;
  return;
}

int Motor::get_speed(void)
{
  if(dir) return freq;
  else return -freq;
}

uint16_t Motor::get_position(void)
{
  return position;
}

Motor M[3] = {
  Motor(motor_A1[0],motor_A2[0],motor_B1[0],motor_B2[0]),
  Motor(motor_A1[1],motor_A2[1],motor_B1[1],motor_B2[1]),
  Motor(motor_A1[2],motor_A2[2],motor_B1[2],motor_B2[2])
};
Angle_sensor AS[3] = {
  Angle_sensor(ADC_pin[0]),
  Angle_sensor(ADC_pin[1]),
  Angle_sensor(ADC_pin[2])
};
void setup() {
  for(int i=0;i<3;i++){
    pinMode(motor_A1[i],OUTPUT);
    pinMode(motor_A2[i],OUTPUT);
    pinMode(motor_B1[i],OUTPUT);
    pinMode(motor_B2[i],OUTPUT);
    pinMode(ADC_pin[i],INPUT);
  }
//  
  Serial.begin(115200);
  while(Serial.read()>=0){}                   //clear buffer
  mySCoop.start();
}

/**
 * 信息打印线程
*/
defineTaskLoop(info_Task){
  static bool LED_state = 1;
  while(1){
    //print angle
    Serial.print("angle: ");
    for(int i=0;i<3;i++){
      Serial.print(AS[i].);
      Serial.print(" ");
    }
    Serial.println();

    // delay(2000); 针对全局
    sleep(2000); // 针对线程
    if(LED_state) {
      LED_state=0;
      digitalWrite(LED_pin,HIGH);
    }
    else {
      digitalWrite(LED_pin,LOW);
      LED_state=1;   
    }
  }
}

/**
 * 电机控制线程
*/
defineTaskLoop(motor1_Task) {
  M[0].motor_run();
}
defineTaskLoop(motor2_Task) {
  M[1].motor_run();
}
defineTaskLoop(motor3_Task) {
  M[2].motor_run();
}
/**
 * 角度读取线程
*/
// defineTaskLoop(angle_Task) {

// }

//static bool dir=1, ctrl_mode=0;//0:position 1:speed
void loop() {
  // if(ctrl_mode) set_speed(freq,dir);
  // else set_position(target_pos);
  sleep(1000);
}

/**
 * 串口数据处理函数，四种消息类型
 * @param buf 串口数据
 * byte 0: 电机编号/传感器编号
 * byte 1：类型标识
 *  0x01设置速度
 *    byte 2：方向
 *    byte 3~4：速度大小，现在其实是步进时间间隔
 *  0x02设置位置
 *    byte 2~3：坐标
 *  0x03读取速度
 *  0x04读取位置
 *  0x05 stop
 *  0x06 position reset
 *  0x07 read angle
*/  
void buf_process(uint8_t *buf)
{
  uint8_t ID = buf[0];
  uint8_t type = buf[1];
  uint8_t resp_buf;
  bool dir=buf[2];
  int freq = buf[3]<<8|buf[4];
  uint16_t target_pos=buf[2]<<8|buf[3];
  Serial.print(ID);
  switch(type) {
    case 0x01:
      M[ID].set_ctrl_mode(1,dir,freq);
      Serial.print(" set speed: ");
      Serial.print(dir);
      Serial.print(" ");
      Serial.println(freq);
      break;
    case 0x02:
      M[ID].set_ctrl_mode(0,target_pos);
      Serial.print(" set target position: ");
      Serial.println(target_pos);
      break;
    case 0x03: {
      Serial.println(" read speed: ");
      Serial.println(M[ID].get_speed());
      int speed = M[ID].get_speed();
      resp_buf = 3;
      Serial.print(resp_buf,HEX);
      if(speed>0) Serial.print(1,HEX);
      else {
        Serial.print(0,HEX);
        speed = -speed;
      }
      resp_buf = (speed&0xff00)>>8;
      Serial.print(resp_buf,HEX);
      resp_buf = speed&0xff;
      Serial.print(resp_buf,HEX);
      Serial.print("AI");
    }
      break;
    case 0x04:
      Serial.println(" read position: ");
      Serial.println(M[ID].get_position());
      break;
    case 0x05:
      Serial.println(" stop");
      M[ID].set_ctrl_mode(0,target_pos);
      break;
    case 0x06:
      Serial.println(" position reset");
      M[ID].reset_position();
      break;
    case 0x07:
      Serial.println(" read angle");
      Serial.println(AS[ID].updateAngle(analogRead(ADC_pin[ID])));
      break;
    default:
      Serial.println(" error cmd type");
      Serial.println(type);
      break;
  }
  return;
}

/*
  serialEvent是一种在接收到串口数据时自动调用的函数，它可以让你在loop()函数执行完后处理串口数据。
  serialEvent会在每次loop()函数结束时被检查，如果串口缓冲区中有可用的数据，就会触发serialEvent。
  你可以使用Serial.read()等函数来读取串口数据，并进行相应的操作。
  如果你的板子有多个串口，你也可以使用serialEvent1()，serialEvent2()等函数来分别处理不同的串口数据。
*/
void serialEvent(){
  static int buf_index=0, end_flag=0;
  static uint8_t buf[10];
  while(Serial.available()>0)
  {
    uint8_t data=Serial.read();
    buf[buf_index++]=data;

    //结束标识符 0x41 0x49(星野爱呜呜呜)
    if(end_flag == 0)
    {
      if(data == 0x41)
      {
        end_flag=1;
      }
    }
    else if(end_flag == 1)
    {
      if(data == 0x49)
      {
        end_flag=0;
        buf_index=0;
        //for (int i = 0; i < 10; i++) {
        //  Serial.print(buf[i]);
        //}
        //Serial.println();

        buf_process(buf);
      }
      else {
        end_flag=0;
        buf_index=0;
        Serial.println("error data");
      }
    }

    if(buf_index > 9)
    {
      end_flag=0;
      buf_index=0;
      Serial.println("buf overflow");
    }

  }

}
