
#include "RobotSerialComm.h"


int ACTION_PARAM_COUNT[] = {0, 3, 2, 2, 0, 0, 0, 0, 2, 2, 2, 3, 0, 0, 0, 1, 0, 0, 0}; //parse number of arguments from serial port (ROS in this case)
                         // 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,16,17,18,19

RobotSerialComm::RobotSerialComm()
{

}
    
int RobotSerialComm::getMsg(int * argv)
{
    int number;
    char data[20];
    char temp[10];
    int temp_count=0;
    int action=0;
    int first_separator=0;
    int argv_count=0;

    // number is data size from USB data
    number = SerialUSB.available();

    // If data exist...
    if(number > 0)
    {
        // Read an array from the SerialUSB port
        SerialUSB.read(data,number);
        
        if ( (data[0] == START) && (data[number-1] == END) ) 
        {
           for (int i = 0; i < number; ++i)
           {
               switch(data[i])
               {
                    case START:     
                    break;
                    case SEPARATOR:   
                        if(first_separator == 0)
                        {
                            action = atoi(temp);  
                            first_separator = 1;
                        }
                        else
                        {
                            argv[argv_count] = atoi(temp);
                            argv_count++;
                        }

                        temp_count = 0;
                        for (int j = 0; j < 10; j++)
                            temp[j] = 0;
                        
                    break;
                    case END:           
                        if (first_separator == 1)
                        {
                            argv[argv_count] = atoi(temp);
                            return action;
                        }
                        else
                        {
                            action = atoi(temp); 
                            return action;
                        }
                        
                    default:
                        temp[temp_count] = data[i];
                        temp[temp_count+1] = '\0';
                        temp_count++;
               }
           }
        }
        else 
            return 0;       //false
    }
    return 0;
}
    
void RobotSerialComm::reply(unsigned int action, unsigned int *argv, int argc)
{
    char str_param[10];
    int i;
    int ret;
    
    SerialUSB.print("@");
    //itoa(action,str_param); 
    ret  = sprintf( str_param, "%d", action);   
    SerialUSB.print(str_param);
    
    for(i=0 ; i<argc ; i++){       
      SerialUSB.print(",");
      ret  = sprintf( str_param, "%d", argv[i]);   
        SerialUSB.print(str_param);
    }
    SerialUSB.println("e");    
}

void RobotSerialComm::itoa(int num, char *str)
{
    int i=0; 
    int radix = 10;  // 진수 
    int deg=1; 
    int cnt = 0; 

    while(1){    // 자리수의 수를 뽑는다 
        if( (num/deg) > 0) 
            cnt++; 
        else 
            break; 
        deg *= radix; 
    } 
    deg /=radix;    // deg가 기존 자리수보다 한자리 높게 카운트 되어서 한번 나누어줌  
    // EX) 1241 ->    cnt = 4; deg = 1000; 
    for(i=0; i<cnt; i++)    {    // 자리수만큼 순회 
        *(str+i) = num/deg + '0';    // 가장 큰 자리수의 수부터 뽑음 
        num -= ((num/deg) * deg);        // 뽑은 자리수의 수를 없엠 
        deg /=radix;    // 자리수 줄임 
    } 
    *(str+i) = '\0';  // 문자열끝널.. 
} 

int RobotSerialComm::my_atoi(char *pStr){
    int sign = 1;
    int nResult = 0;
    if(pStr == NULL) return 0;
    
    if(*pStr == '-'){
        sign = -1;
        pStr++;
    }

    for( ; *pStr != '\0'; pStr++){
        if(*pStr  < '0' || *pStr > '9'){            
            return 0;
        }

        nResult *= 10;
        nResult += *pStr - '0';        
    }    

    return nResult * sign;
}


// EOF
