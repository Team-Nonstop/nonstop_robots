nt r_count=0;
int l_count=0;
unsigned int number;
char data[100];
double t_rv,t_lv;
int R_direction=2;
int L_direction=2;
 
 
#define LED_RATE 10000
#define LEAD_A 1
#define LEAD_B 2
 
HardwareTimer timer(2);
 
double r_desired_velocity = 0; // 목표 속도 설정
double r_currently_velocity;
double l_desired_velocity = 0; // 목표 속도 설정
double l_currently_velocity;
double r_error,r_error_dot,r_previous_error;
double l_error,l_error_dot,l_previous_error;
double P=200;
double I=0;
double r_input,l_input;
 
int r_way,l_way;
 
void setup() {
  //Initialize Serial2
  Serial3.begin(9600);
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
  timer.setPeriod(LED_RATE); // in microseconds
  timer.setMode(TIMER_CH1, TIMER_OUTPUT_COMPARE);
  timer.setCompare(TIMER_CH1, 1);
  timer.attachInterrupt(TIMER_CH1, handler_led);
 
  timer.refresh();
  timer.resume();
}
 
int r_control=0,l_control=0;
void loop() {
  reset_array();
 
  number = SerialUSB.available();
 
  if(number > 0)
  {
    SerialUSB.read(data,number);
 
    //r_desired_velocity = atof(data);
    //l_desired_velocity = atof(data);
 
    if(data[0] == 0x01)
    {
      r_desired_velocity = data[3] & 0xff;
      r_desired_velocity = r_desired_velocity +
      r_desired_velocity = r_desired_velocity / 100;
      l_desired_velocity = data[5] & 0xff;
      l_desired_velocity = l_desired_velocity / 100;
      
      if(data[2])  r_desired_velocity = r_desired_velocity * (-1.0);
      if(data[4])  l_desired_velocity = l_desired_velocity * (-1.0);
    }
  }
  
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
 
  Serial3.print("<0L");
  Serial3.print(r_way);
  Serial3.println((int)r_input);
  Serial3.print("R");
  Serial3.print(l_way);
  Serial3.println((int)l_input);
  Serial3.println(">");
 
  if(r_way == 0)  r_input = r_input*(-1);
  if(l_way == 1)  l_input = l_input*(-1);
 
  //SerialUSB.write(input*0.252525252525);
  r_previous_error = r_error;
  l_previous_error = l_error;
 
  delay(100);
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
  }
  else                {
    R_direction = 1;
    r_count--;
  }
}
void R_intB()
{
  if(R_direction)  {
    r_count--;
  }
  else                {
    r_count++;
  }
}
void L_intA()
{
  if(digitalRead(29))  {
    L_direction = 0;
    l_count--;
  }
  else                {
    L_direction = 1;
    l_count++;
  }
 
}
 
void L_intB()
{
  if(L_direction)  {
    l_count++;
  }
  else                {
    l_count--;
  }
}
 
void handler_led(void) { 
 
  t_rv = 0.135 * PI * r_count / 1155 * 100;
  t_lv = 0.135 * PI * l_count / 1155 * 100;
  r_count = 0;
  l_count = 0;
}



