#include "DCDM1210.h"
#include "RobotSerialComm.h"

int r_count=0;
int l_count=0;
long int enc1_count=0;
long int enc2_count=0;
unsigned int number;
char data[100];
double t_rv,t_lv;
int R_direction=2;
int L_direction=2;
 
 
#define TIME_RATE 50000
#define LEAD_A 1
#define LEAD_B 2
#define WHEEL_R 0.0675

#define PULSES_TO_M 0.000734045
 
HardwareTimer timer(2);
DCDM1210 driver;
RobotSerialComm port;
int voltage;
boolean stream=false;

//Variables for motion control
double lin_speed_si=0.0;    //Linear speed in m/s
double ang_speed_si=0.0;    //Rotational speed in rad/s
 
double r_desired_velocity = 0; // 목표 속도 설정
double r_currently_velocity;
double l_desired_velocity = 0; // 목표 속도 설정
double l_currently_velocity;
double r_error,r_error_dot,r_previous_error;
double l_error,l_error_dot,l_previous_error;
double P=100;
double I=50;
double r_input,l_input;
 
int r_way,l_way;

unsigned int reply_arg[5];

void sendRobotInfo(){   
    
    driver.Answer_voltage(&voltage);
    delay(10);
    reply_arg[0] = (int)1;                // Robot ID
    SerialUSB.println(voltage);
    port.reply(ROBOT_INFO, reply_arg, 1);
}

void sendEncodersReads(){  
  
    reply_arg[0] = enc1_count;
    reply_arg[1] = enc2_count;
    //enc2_count = 0;
    //enc1_count = 0;
    port.reply(OMNI_READ_ENCODERS, reply_arg, 2);
}
 
void setup() {
  //Initialize Serial3
  driver.Serial_connect(SERIAL_3,BAUD_RATE_9600);

  SerialUSB.begin();
  pinMode(26,INPUT);
  pinMode(27,INPUT);
  pinMode(28,INPUT);
  pinMode(29,INPUT);
  attachInterrupt(26,R_intA,FALLING);
  attachInterrupt(27,L_intA,FALLING);
  attachInterrupt(28,R_intB,FALLING);
  attachInterrupt(29,L_intB,FALLING);
 
  timer.pause();
  timer.setPeriod(TIME_RATE); // in microseconds
  timer.setMode(TIMER_CH1, TIMER_OUTPUT_COMPARE);
  timer.setCompare(TIMER_CH1, 1);
  timer.attachInterrupt(TIMER_CH1, handler_led);
 
  timer.refresh();
  timer.resume();

  pinMode(BOARD_LED_PIN, OUTPUT);
}
 
int r_control=0,l_control=0;

void loop() {

  int arg[5];

  int action = port.getMsg(arg); 

  if(action==0 && stream==true){
      action=ACTION_START_STREAM;
    }

  switch(action){
          
            case OMNI_CALIBRATION:                 // "@1e", no reply
                //omni.calibrate(1,0,0);
                delay(95);
                break;
            
            case OMNI_SET_PID:                    //@2,"KP","KI","KD"e, no reply
                //omni.set_PID(arg[0], arg[1], arg[2]);
                break;
                
            case OMNI_SET_PRESCALER:              //@3,"enc","value"e, no reply
                //omni.set_prescaler(arg[0], arg[1]);
                break;

            case OMNI_SET_ENC_VALUE:            //@4,"enc","enc_value"e, no reply
                //omni.set_enc_value(arg[0], arg[1]);
                break;

            case ROBOT_INFO:                    //@5e, reply: @5,"temp","firm","bat","r_firm","r_id"e
                sendRobotInfo();
                break;

            case OMNI_READ_ENCODERS:          //@6e,  reply: @6,"enc1(R)","enc2(L)"e
                sendEncodersReads();
                break;

            case READ_SONARS:                //@7e, reply: @7,"son1(F)","son2(L)","son3(R)"e
                //sendSonarsReads();
                break;    

            case READ_ENCODERS_SONARS:       //@8e, reply: @8,"enc1(R)","enc2(L)","son1(F)","son2(L)","son3(R)"e
                //sendEncodersSonarsReads();
                break;    
                
            case LINEAR_MOVE_PID:            //@9,"speed1","speed3"e, no reply
                //omni.mov_lin3m_pid(arg[0], 0, arg[1]);              
                break;

            case LINEAR_MOVE_NOPID:          //@10,"speed1","speed2"e, no reply
                //omni.mov_lin3m_nopid(arg[0], 0, arg[1]);
                break;

            case MOVE_DIFFERENTIAL_SI:          //@11,"vel_linear","vel_angular"e, no reply
                //SerialUSB.println(arg[0]);
                //SerialUSB.println(arg[1]);
                lin_speed_si= ((double)arg[0]/1000); 
                ang_speed_si= ((double)arg[1]/1000);

                // the reason for /10 meets a kobuki's ang scale
                r_desired_velocity = lin_speed_si + ang_speed_si / WHEEL_R / 10;
                l_desired_velocity = lin_speed_si - ang_speed_si / WHEEL_R / 10;
                break;

            case MOVE_POSITIONAL:              //@12,"motor_nr","speed","encoder_Position"e, no reply
                //omni.mov_pos(arg[0], arg[1], arg[2], 1);  // move motor1 at speed1 until encoder count reaches the defined position and then stop with holding torque
                delay(1);                          // wait 1ms for Omni3MD to process information
                break;

            case STOP_MOTORS:                //@13e, no reply
                //omni.stop_motors();
                break;
                
            case ENCODERS_RESET:             //@14e, no reply
                //robot.encodersReset();
                break;                
                              
            case ACTION_GET_DEBUG:           //@15e, reply (to the console): @13,"0/1"e
                //reply_arg[0] = port.getDebug();
                //port.reply(ACTION_GET_DEBUG, reply_arg, 1);
                break;

            case ACTION_SET_DEBUG:           //@16,"0/1"e, no reply
                //port.setDebug(arg[0]);  
                break;

            case ACTION_GET_STREAM:           //@17e, reply @15,"0/1"e
                //reply_arg[0] = stream;
                //port.reply(ACTION_GET_STREAM, reply_arg, 1);
                break;
                
            case ACTION_START_STREAM:      // "@18e,  reply: @6,"enc1(R)","enc2(L)"e (repeatedly)
                stream = true;
                sendEncodersReads();
                //delay(65);                //encoders read update (+- 15Hz)
                break;
                
            case ACTION_STOP_STREAM:        // "@19e,  no reply 
                stream = false;                
                break;
                
            default:
                break;
   } // switch
   
  r_currently_velocity = t_rv;
  r_error = r_desired_velocity-r_currently_velocity;
  r_error_dot = r_error-r_previous_error;
  r_input = P * r_error + I * r_error_dot + r_input;
 
  if(r_input < 0)  {
    r_input = r_input*(-1);
    r_way = 0;
  }
  else{
    r_way = 1;
  }
 
  if(r_input >= 999) r_input = 999;
  if(r_input <= 0) r_input = 0;
 
  l_currently_velocity = t_lv;
  l_error = l_desired_velocity-l_currently_velocity;
  l_error_dot = l_error-l_previous_error;
  l_input = P * l_error + I * l_error_dot + l_input;
 
  if(l_input < 0)  {
    l_input = l_input*(-1);
    l_way = 1;
  }
  else{
    l_way = 0;
  }
 
  if(l_input >= 999) l_input = 999;
  if(l_input <= 0) l_input = 0;

  // mistake  L R change
  driver.Moter_control(LEFT, r_way ,(int)r_input,RIGHT, l_way ,(int)l_input);
 
  if(r_way == 0)  r_input = r_input*(-1);
  if(l_way == 1)  l_input = l_input*(-1);
 
  //SerialUSB.write(input*0.252525252525);
  r_previous_error = r_error;
  l_previous_error = l_error;

  reset_array();
  
  delay(10);

  
}
void reset_array()
{
  int i;
  for(i=0; i<100; i++)
    data[i] = 0;
}
void R_intA()
{
  
  if(digitalRead(28))  {
    R_direction = 0;
    r_count++;
    
    enc1_count++;
    
    if(enc1_count > 32767)
      enc1_count = 0;
  }
  else                {
    R_direction = 1;
    r_count--;
    
    enc1_count--;
    
    if(enc1_count < -32767)
      enc1_count = 0;
  }
}
void R_intB()
{
  /*
  if(R_direction)  {
    r_count--;
    enc1_count--;
    
    if(enc1_count < -32767)
      enc1_count = 0;
  }
  else                {
    r_count++;
    enc1_count++;
    
    if(enc1_count > 32767)
      enc1_count = 0;
  }
  */
}
void L_intA()
{
  if(digitalRead(29))  {
    L_direction = 0;
    l_count--;
    enc2_count--;
    
    if(enc2_count < -32768)
      enc2_count = 0;
  }
  else                {
    L_direction = 1;
    l_count++;
    enc2_count++;
    
    if(enc2_count > 32767)
      enc2_count = 0;
  }
 
}
 
void L_intB()
{
  /*
  if(L_direction)  {
    l_count++;
    enc2_count++;
    if(enc2_count > 32767)
      enc2_count = 0;
  }
  else                {
    l_count--;
    enc2_count--;
    if(enc2_count < -32767)
      enc2_count = 0;
  }
  */
}
 
void handler_led(void) { 
 
  t_rv = PULSES_TO_M * r_count * 20;
  t_lv = PULSES_TO_M * l_count * 20;
  
  r_count = 0;
  l_count = 0;
}
