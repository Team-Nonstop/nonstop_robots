/*
  DCDM1210.cpp - Library for interfacing with NT-S-DCDM1210 (2 motor driver controller)
  Created by Gim Tae Hwan (Antoine)
  Released into the public domain.
*/

#include "DCDM1210.h"
#include "math.h"
#include "delay.h"
#include <stdlib.h>

DCDM1210::DCDM1210()
{
  Reset_data();
  _Serial_choice = 0;
  _baud = 0;
}
DCDM1210::~DCDM1210()
{

}

void DCDM1210::Serial_connect(int Serial_choice,int baud)
{
  switch(baud)
  {
    case BAUD_RATE_4800:    _baud = 4800;   break;
    case BAUD_RATE_9600:    _baud = 9600;   break;
    case BAUD_RATE_19200:   _baud = 19200;  break;
    case BAUD_RATE_38400:   _baud = 38400;  break;
    case BAUD_RATE_57600:   _baud = 57600;  break;
    case BAUD_RATE_115200:  _baud = 115200; break;
  }
  switch(Serial_choice)
  {
    case SERIAL_1:  Serial1.begin(_baud);  _Serial_choice = SERIAL_1; break;
    case SERIAL_2:  Serial2.begin(_baud);  _Serial_choice = SERIAL_2; break;
    case SERIAL_3:  Serial3.begin(_baud);  _Serial_choice = SERIAL_3; break;
  }
}
void DCDM1210::Send_data(char *data)
{
  switch(_Serial_choice)
  {
    case SERIAL_1:  Serial1.print(data); break;
    case SERIAL_2:  Serial2.print(data); break;
    case SERIAL_3:  Serial3.print(data); break;
  }
}

int DCDM1210::Moter_control(int which, int direction ,int duty)
{
  int temp;

  // duty size 0 ~ 999
  if((duty < 0) && (duty > 999))    return 0;   // false

  Reset_data();
  _data[0] = '<';   // Start
  _data[1] = '0';
  switch(which)
  {
    case RIGHT:   _data[2] = 'R'; break;
    case LEFT:    _data[2] = 'L'; break;
    default:  return 0;                         // false
  }
  switch(direction)
  {
    case FORWARD:   _data[3] = '1'; break;
    case BACKWARD:  _data[3] = '0'; break;
    default:  return 0;                         // false
  }
  if (duty > 99)
  {
    _data[4] = (int)(duty / 100) + 0x30;
    temp = duty % 100;
    _data[5] = (int)(temp / 10) + 0x30;
    temp = temp % 10;
    _data[6] = temp + 0x30;
    _data[7] = '>';
  }
  else if(duty > 9)
  {
    _data[4] = (int)(duty / 10) + 0x30;
    temp = duty % 10;
    _data[5] = temp + 0x30;
    _data[7] = '>';
  }
  else
  {
    _data[4] = duty + 0x30;
    _data[5] = '>';
  }
  Send_data(_data);
  return 1;
}

int DCDM1210::Moter_control(int which_1, int direction_1 ,int duty_1,int which_2, int direction_2 ,int duty_2)
{
  int temp;
  int array_count=0;

  // duty size 0 ~ 999
  if((duty_1 < 0) && (duty_1 > 999))    return 0;   // false
  if((duty_2 < 0) && (duty_2 > 999))    return 0;   // false

  Reset_data();
  _data[0] = '<';   // Start
  _data[1] = '0';

  // first_moter
  switch(which_1)
  {
    case RIGHT:   _data[2] = 'R'; break;
    case LEFT:    _data[2] = 'L'; break;
    default:  return 0;                         // false
  }
  switch(direction_1)
  {
    case FORWARD:   _data[3] = '1'; break;
    case BACKWARD:  _data[3] = '0'; break;
    default:  return 0;                         // false
  }
  if (duty_1 > 99)
  {
    _data[4] = (int)(duty_1 / 100) + 0x30;
    temp = duty_1 % 100;
    _data[5] = (int)(temp / 10) + 0x30;
    temp = temp % 10;
    _data[6] = temp + 0x30;
    array_count = 7;
  }
  else if(duty_1 > 9)
  {
    _data[4] = (int)(duty_1 / 10) + 0x30;
    temp = duty_1 % 10;
    _data[5] = temp + 0x30;
    array_count = 6;
  }
  else
  {
    _data[4] = duty_1 + 0x30;
    array_count = 5;
  }

  // second_moter
  switch(which_2)
  {
    case RIGHT:   _data[array_count] = 'R'; break;
    case LEFT:    _data[array_count] = 'L'; break;
    default:  return 0;                         // false
  }
  array_count++;
  switch(direction_2)
  {
    case FORWARD:   _data[array_count] = '1'; break;
    case BACKWARD:  _data[array_count] = '0'; break;
    default:  return 0;                         // false
  }
  array_count++;
  if (duty_2 > 99)
  {
    _data[array_count] = (int)(duty_2 / 100) + 0x30;
    temp = duty_2 % 100;
    array_count++;

    _data[array_count] = (int)(temp / 10) + 0x30;
    temp = temp % 10;
    array_count++;

    _data[array_count] = temp + 0x30;
    array_count++;
    
  }
  else if(duty_2 > 9)
  {
    _data[array_count] = (int)(duty_2 / 10) + 0x30;
    temp = duty_2 % 10;
    array_count++;

    _data[array_count] = temp + 0x30;
    array_count++;   
  }
  else
  {
    _data[array_count] = duty_2 + 0x30;
    array_count++;
  }
  _data[array_count] = '>';

  Send_data(_data);

  return 1;
}

int DCDM1210::Deadband(int duty)
{
  int temp;
  // duty size 0 ~ 500
  if((duty < 0) && (duty > 500))    return 0;   // false

  Reset_data();
  _data[0] = '<';   // Start
  _data[1] = '0';   // order
  _data[2] = 'D';   // DeadBand command

  if (duty > 99)
  {
    _data[3] = (int)(duty / 100) + 0x30;
    temp = duty % 100;
    _data[4] = (int)(temp / 10) + 0x30;
    temp = temp % 10;
    _data[5] = temp + 0x30;
    _data[6] = '>';
  }
  else if(duty > 9)
  {
    _data[3] = (int)(duty / 10) + 0x30;
    temp = duty % 10;
    _data[4] = temp + 0x30;
    _data[5] = '>';
  }
  else
  {
    _data[3] = duty + 0x30;
    _data[4] = '>';
  }
  return 1;
}

int DCDM1210::Setting_RS232C_baud(int baud)
{
  Reset_data();
  _data[0] = '<';   // Start
  _data[1] = '0';   // order
  _data[2] = 'U';   // DeadBand command
  switch(baud)
  {
    case BAUD_RATE_4800:    _data[3] = '0';   break;
    case BAUD_RATE_9600:    _data[3] = '1';   break;
    case BAUD_RATE_19200:   _data[3] = '2';   break;
    case BAUD_RATE_38400:   _data[3] = '3';   break;
    case BAUD_RATE_57600:   _data[3] = '4';   break;
    case BAUD_RATE_115200:  _data[3] = '5';   break;
    default:  return 0;   
  }
  _data[4] = '>';
  return 1;
}

int DCDM1210::Low_voltage_warning(int voltage)
{
  int temp;
  // voltage size 0 ~ 24
  if((voltage < 0) && (voltage > 24))    return 0;   // false

  Reset_data();

  _data[0] = '<';   // Start
  _data[1] = '0';   // order
  _data[2] = 'W';   // warning command

  if(voltage > 9)
  {
    _data[3] = (int)(voltage / 10) + 0x30;
    temp = voltage % 10;
    _data[4] = temp + 0x30;
    _data[5] = '>';
  }
  else
  {
    _data[3] = voltage + 0x30;
    _data[4] = '>';
  }
  return 1;
}

void DCDM1210::Joystick_calibration(void)
{
  _data[0] = '<';   // Start
  _data[1] = '0';   // order
  _data[2] = 'C';   // calibration command
  _data[3] = '>';
}

void DCDM1210::Command_reset(void)
{
  _data[0] = '<';   // Start
  _data[1] = '0';   // order
  _data[2] = 'r';   // reset command
  _data[3] = '>';
}

int DCDM1210::Moter_delay(int usec)
{
  int temp;
  // usec size 0 ~ 24
  if((usec < 0) && (usec > 100))    return 0;   // false
  _data[0] = '<';   // Start
  _data[1] = '0';   // order
  _data[2] = 'E';   // calibration command

  if (usec > 99)
  {
    _data[3] = (int)(usec / 100) + 0x30;
    temp = usec % 100;
    _data[4] = (int)(temp / 10) + 0x30;
    temp = temp % 10;
    _data[5] = temp + 0x30;
    _data[6] = '>';
  }
  else if(usec > 9)
  {
    _data[3] = (int)(usec / 10) + 0x30;
    temp = usec % 10;
    _data[4] = temp + 0x30;
    _data[5] = '>';
  }
  else
  {
    _data[3] = usec + 0x30;
    _data[4] = '>';
  }
}

int DCDM1210::Answer_PWM(int *R_direction,int *R_PWM,int *L_direction,int *L_PWM)
{
  int count=0;
  int input_count=0;

  Reset_data();

  _data[0] = '<';   // Start
  _data[1] = '1';   // answer
  _data[2] = 'M';   // Answer_pwm command
  _data[3] = '>';

  // Send_data
  Send_data(_data);
  Reset_data();
  Data_read();

  if (_data[0] == '<')
  {
    if(_data[1] == 'R')
    {
      *R_direction = _data[2] - 0x30;

      for (int i = 3; i < 6; i++)
      {
        if (_data[i] == 'L')   break;

        switch(i)
        {
          case 2: *R_PWM = _data[3] - 0x30; count = 4; break;
          case 3: *R_PWM = (_data[3] - 0x30) * 10 + (_data[4] - 0x30); count = 5; break;
          case 4: *R_PWM = (_data[3] - 0x30) * 100 + (_data[4] - 0x30) * 10 + (_data[5] - 0x30); count = 6; break;
        }
      }
      if (_data[count] == 'L')
      {
        count++;
        *L_direction = _data[count] - 0x30;
        count++;

        for (int i = count; i < count+3; i++)
        {
          if (_data[i] == '>')   break;
          if(i == count)
            *L_PWM = _data[count] - 0x30;
          else if(i == (count+1))
            *L_PWM = (_data[count] - 0x30) * 10 + (_data[count+1] - 0x30);
          else if(i == (count+2))
            *L_PWM = (_data[count] - 0x30) * 100 + (_data[count+1] - 0x30) * 10 + (_data[count+2] - 0x30);
        }
      }
      else  return 0;
    }
  }
  else return 0;

  return 1;
}

int DCDM1210::Answer_voltage(int *voltage)
{
  int count;
  int input_count=0;

  Reset_data();
  _data[0] = '<';   // Start
  _data[1] = '1';   // answer
  _data[2] = 'V';   // Answer_voltage command
  _data[3] = '>';

  // Send_data
  Send_data(_data);
  Reset_data();
  Data_read();

  
  if (_data[0] == '<')
  {
    if(_data[1] == 'V')
    {
      for (int i = 2; i < 4; i++)
      {
        if (_data[i] == '>')   break;

        switch(i)
        {
          case 2: *voltage = _data[2] - 0x30; break;
          case 3: *voltage = (_data[2] - 0x30) * 10 + (_data[3] - 0x30); break;
        }
      }
    }
  }
  else return 0;

  return 1;
}

int DCDM1210::Answer_current(double *R_current,double *L_current)
{
  char temp[10];
  int count=0;
  int input_count=0;

  Reset_data();

  _data[0] = '<';   // Start
  _data[1] = '1';   // answer
  _data[2] = 'C';   // Answer_current command
  _data[3] = '>';

  // Send_data
  Send_data(_data);
  Reset_data();
  Data_read();

  if (_data[0] == '<')
  {
    if(_data[1] == 'R')
    {
      for (int i = 2; i < 7; i++)
      {
        if (_data[i] == 'L')   
        {
          *R_current = atof(temp);
          for (int j = 0; j < 10; j++)  temp[j] = 0;

          count = i+1;
          break;
        }
        else
          temp[count] = _data[i];
      }
      for (int i = count; i < count+5; i++)
      {
        if (_data[i] == '>')   
        {
          *L_current = atof(temp);
          break;
        }
        else
          temp[count] = _data[i];
      }
    }
  }
  else return 0;

  return 1;
}

void DCDM1210::Reset_data(void)
{
  for (int i = 0; i < 20; i++)
  {
    _data[i] = 0;
  }
}

void DCDM1210::Data_read(void)
{
  int input_count=0;
  while(1)
  {
    if(_Serial_choice == SERIAL_1)
    {
      if (Serial1.available())
      {
        _data[input_count] = Serial1.read();
        if (_data[input_count] == '>')
        {
          return;
        }
        if (_data[0] == '<')
        {
          input_count++;
        }
      }
    }
    else if(_Serial_choice == SERIAL_2)
    {
      if (Serial2.available())
      {
        _data[input_count] = Serial2.read();
        if (_data[input_count] == '>')
        {
          return;
        }
        if (_data[0] == '<')
        {
          input_count++;
        }
      }
    }
    else if(_Serial_choice == SERIAL_3)
    {
      if (Serial3.available())
      {
        _data[input_count] = Serial3.read();

        if (_data[input_count] == '>')
        {
          return;
        }
        if (_data[0] == '<')
        {
          input_count++;
        }
      }
    }
  }
}
