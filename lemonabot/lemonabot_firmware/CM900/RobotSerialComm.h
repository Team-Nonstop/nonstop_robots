
#include <usb_serial.h>
#include <stdio.h>
#include "math.h"
#include "delay.h"
#include <stdlib.h>

#define SERIAL_BUFFER_SIZE  32

#define START            '@'
#define SEPARATOR        ','
#define END              'e'

enum RobotAction
{
  
  // Setup routines
  OMNI_CALIBRATION = 1,
  OMNI_SET_PID = 2,
  OMNI_SET_PRESCALER = 3,
  OMNI_SET_ENC_VALUE = 4,
    
  // Reading routines
  ROBOT_INFO = 5,
  OMNI_READ_ENCODERS = 6,
  READ_SONARS = 7,
  READ_ENCODERS_SONARS = 8,
 
  // Movement routines
  LINEAR_MOVE_PID = 9,
  LINEAR_MOVE_NOPID = 10,
  MOVE_DIFFERENTIAL_SI = 11,
  MOVE_POSITIONAL = 12,
  STOP_MOTORS = 13,
  ENCODERS_RESET = 14,
  
  // COMM
  ACTION_GET_DEBUG = 15,
  ACTION_SET_DEBUG = 16,
  ACTION_GET_STREAM = 17,
  ACTION_START_STREAM = 18,
  ACTION_STOP_STREAM = 19,
  ACTION_COUNT = 20

};

extern int ACTION_PARAM_COUNT[];


class RobotSerialComm
{
    public:
      RobotSerialComm();
    
      int getMsg(int * argv);
    
      void reply(unsigned int action, unsigned int * argv, int argc);
      void itoa(int action, char *str);
      int my_atoi(char *pStr);
};

// EOF

