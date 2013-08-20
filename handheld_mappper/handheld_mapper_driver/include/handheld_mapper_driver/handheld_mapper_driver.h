#include <cereal_port/cereal_port.h>
#include <ros/ros.h>


#define BAUDRATE_9600 1
#define BAUDRATE_19200 2
#define BAUDRATE_38400 3
#define BAUDRATE_57600 4
#define BAUDRATE_115200 5
#define BAUDRATE_230400 6
#define BAUDRATE_460800 7
#define BAUDRATE_921600 8

#define OFF 0
#define ON 1

#define GYRO_SENS_250DPS 1
#define GYRO_SENS_500DPS 2
#define GYRO_SENS_2000DPS 3

#define ACCELERO_SENS_2G 1
#define ACCELERO_SENS_4G 2
#define ACCELERO_SENS_8G 3

#define MAGNETO_SENS_1_3G 1
#define MAGNETO_SENS_1_9G 2
#define MAGNETO_SENS_2_5G 3
#define MAGNETO_SENS_4_0G 4
#define MAGNETO_SENS_4_7G 5
#define MAGNETO_SENS_5_6G 6
#define MAGNETO_SENS_8_1G 7

#define OUTPUT_FORM_EULER 1
#define OUTPUT_FORM_QUARER 2

#define CMD_BEGIN "<"
#define CMD_END ">"
#define CMD_BEGIN_CHAR '<'
#define CMD_END_CHAR '>'

#define CMD_SET_BAUDRATE "sb"
#define CMD_SET_OUTPUTRATE "sor"
#define CMD_SET_ENABLE_MAGNETO "sem"
#define CMD_SET_OUTOUT_ACCELERO "soa"
#define CMD_SET_SENS_GYRO "ssg"
#define CMD_SET_SENS_ACCELERO "ssa"
#define CMD_SET_SENS_MAGNETO "ssm"
#define CMD_SET_OUTPUT_FORMAT "sof"

#define SET_BAUDRATE 			0
#define SET_OUTPUTRATE 			1
#define SET_ENABLE_MAGNETO 		2
#define SET_OUTOUT_ACCELERO 	3
#define SET_SENS_GYRO 			4
#define SET_SENS_ACCELERO 		5
#define SET_SENS_MAGNETO 		6
#define SET_OUTPUT_FORMAT 		7

#define IMU_RESP_SEP ","
#define IMU_RESP_BEGIN "*"
#define IMU_RESP_END "\r\n"
#define IMU_RESP_BEGIN_CHAR '*'
#define IMU_RESP_END_CHAR '\n'

#define STATUS_OFF            0
#define STATUS_INITIALIZING   1
#define STATUS_STREAMING      2
#define STATUS_ERROR          3

cereal::cereal_port device;

std::string make_command(int _cmd, int _value){

	std::string cmd; 
	std::ostringstream value;

	switch(_cmd){
		case SET_BAUDRATE:
			cmd = CMD_SET_BAUDRATE;
			switch(_value){
				case 9600:
					value << BAUDRATE_9600;
				break;
				case 19200:
					value << BAUDRATE_19200;
				break;
				case 38400:
					value << BAUDRATE_38400;
				break;
				case 57600:
					value << BAUDRATE_57600;
				break;
				case 115200:
					value << BAUDRATE_115200;
				break;
				case 230400:
					value << BAUDRATE_230400;
				break;
				case 460800:
					value << BAUDRATE_460800;
				break;
				case 921600:
					value << BAUDRATE_921600;
				break;
				default:
					ROS_WARN("Baudrate ERROR :\"%d\" set 115200 as default",_value);
					value << BAUDRATE_115200;
				break;
		   		}
			break;
		case SET_OUTPUTRATE:
			cmd = 	CMD_SET_OUTPUTRATE;
			value <<  _value;
			break;
		case SET_ENABLE_MAGNETO:
			cmd = CMD_SET_ENABLE_MAGNETO;
			value << _value;
			break;
		case SET_OUTOUT_ACCELERO:
			cmd = CMD_SET_OUTOUT_ACCELERO;
			value << _value;
			break;
		case SET_SENS_GYRO:
			cmd = CMD_SET_SENS_GYRO;
			switch(_value){
				case 250:
					value << GYRO_SENS_250DPS;
				break;
				case 500:
					value << GYRO_SENS_500DPS;
				break;
				case 2000:
					value << GYRO_SENS_2000DPS;
				break;
				
				default:
					ROS_WARN("Gyroscope Sensitivity ERROR :\"%d\" 2000DPS as default",_value);
					value << GYRO_SENS_2000DPS;
				break;
		   		}
			break;
		case SET_SENS_ACCELERO:
			cmd = CMD_SET_SENS_ACCELERO;
			switch(_value){
				case 2:
					value << ACCELERO_SENS_2G;
				break;
				case 4:
					value << ACCELERO_SENS_4G;
				break;
				case 8:
					value << ACCELERO_SENS_8G;
				break;
				
				default:
					ROS_WARN("Accelero Sensitivity ERROR :\"%d\" set 8G as default",_value);
					value << ACCELERO_SENS_8G;
				break;
		   		}
			break;
		case SET_SENS_MAGNETO:
			cmd = CMD_SET_SENS_MAGNETO;
			switch(_value){
				case 13:
					value << MAGNETO_SENS_1_3G;
				break;
				case 19:
					value << MAGNETO_SENS_1_9G;
				break;
				case 25:
					value << MAGNETO_SENS_2_5G;
				break;
				case 40:
					value << MAGNETO_SENS_4_0G;
				break;
				case 47:
					value << MAGNETO_SENS_4_7G;
				break;
				case 56:
					value << MAGNETO_SENS_5_6G;
				break;
				case 81:
					value << MAGNETO_SENS_8_1G;
				break;
				default:
					ROS_WARN("Magneto Sensitivity ERROR :\"%d\" set 2.5G as default",_value);
					value << MAGNETO_SENS_2_5G;
				break;
		   		}
			break;
		case SET_OUTPUT_FORMAT:
			cmd = CMD_SET_OUTPUT_FORMAT;
			value << _value;
			break;
		default:
		    ROS_WARN("Command Code ERROR :\"%d\"",_cmd);
		    break;
	}

	std::string result = CMD_BEGIN_CHAR + cmd + value.str() + CMD_END_CHAR;
	ROS_WARN("Command Code :\"%s\"",result.c_str());    
  	return result;
}

class Gyro{
public:
  int setSens(int _statusSens){
	int res;
  	res =  device.write( make_command(SET_SENS_GYRO,_statusSens).c_str() ); 
  	if(res){statusSens = _statusSens;}
  	return res;
  }
  int getSens(){return statusSens;}
private:
  int statusSens;
};

class Accelero{
public:
  int setVisible(int _status){
	int res;
  	res =  device.write( make_command(SET_OUTOUT_ACCELERO,_status).c_str() ); 
   	if(res){status = _status;}
  	return res; 
  }
  int getVisible(){return status;}
  int setSens(int _statusSens){
	int res;
  	res =  device.write( make_command(SET_SENS_ACCELERO,_statusSens).c_str() ); 
   	if(res){statusSens = _statusSens;}
  	return res;
  }
  int getSens(){return statusSens;}
private:
  int status;
  int statusSens;
};

class Magneto{
public:
  int setEnable(int _status){
	int res;
  	res =  device.write( make_command(SET_ENABLE_MAGNETO,_status).c_str() ); 
  	if(res){status = _status;}
  	return res;
  }
  int getEnable(){return status;}
  int setSens(int _statusSens){
	int res;
  	res =  device.write( make_command(SET_SENS_MAGNETO,_statusSens).c_str() ); 
  	if(res){statusSens = _statusSens;}
  	return res;
  }
  int getSens(){return statusSens;}
private:
  int status;
  int statusSens;
};


class EBIMU9DOF {
private:
  int statusOutputForm;
  int statusBaudrate;
  int statusOutputRate; //10ms * 1
  int status;
public:
  Gyro gyro;
  Accelero accelero;
  Magneto magneto;
  int initializeIMU(){
  	accelero.setVisible(ON);  	
  }
  int setBaudrate(int _baudrate){
	int res;
  	res =  device.write( make_command(SET_BAUDRATE,_baudrate).c_str() ); 
  	if(res){statusBaudrate = _baudrate;}
  	return res;  	
  };
  int getBaudrate(){return statusBaudrate;}
  int setOutputRate(int _outputrate){
  	int res;
  	res =  device.write( make_command(SET_OUTPUTRATE,_outputrate).c_str() ); 
  	if(res){statusOutputRate = _outputrate;}
  	return res;  	
  }
  int getOutputRate(){return statusOutputRate;}
  int setOutputForm(int _outputForm){
  	int res;
  	res =  device.write( make_command(SET_OUTPUT_FORMAT,_outputForm).c_str() ); 
  	if(res){statusOutputForm = _outputForm;}
  	return res;  	
  }
  int getOutputForm(){return statusOutputForm;}


  int setEnableMagneto(int _status){
  	return magneto.setEnable(_status);
  }
  int setVisibleAccelero(int _status){
	return accelero.setVisible(_status);
  }
  int setSensGyro(int _sens){
	return gyro.setSens(_sens);
  }
  int setSensAccelero(int _sens){
	return accelero.setSens(_sens);
  }
  int setSensMagneto(int _sens){
  	return magneto.setSens(_sens);
  }

  };