#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include "../basic_lib/PREDICTBasic.h"

// -------------------- Variables --------------------
float p_roll_tel = 0.45;               //Gain setting for the roll P-controller
float i_roll_tel = 0.0008;             //Gain setting for the roll I-controller
float d_roll_tel = 2.0;               //Gain setting for the roll D-controller

float p_pitch_tel = 0.45;   //Gain setting for the pitch P-controller.
float i_pitch_tel = 0.0008;   //Gain setting for the pitch I-controller.
float d_pitch_tel = 2.0;   //Gain setting for the pitch D-controller.

float p_yaw_tel = 1;                   //Gain setting for the pitch P-controller. //4.0
float i_yaw_tel = 0.002;               //Gain setting for the pitch I-controller. //0.02
float d_yaw_tel = 0;                 //Gain setting for the pitch D-controller.

// Conversion factor
const float p_angle_tel = 0.1;
const float i_angle_tel = 0.0001;
const float d_angle_tel = 0.01;
bool pid_tel;

// -------------------- Functions --------------------

void send_telemtry(char *xstr, const unsigned int msgLength){
  char *START_STR = ".FPGA1";
  char *END_STR = "!";
  char full_message[msgLength+7];
  memset(full_message, '\0', msgLength);
  snprintf(full_message, msgLength, "%s%s%s",START_STR,xstr,END_STR);
  for(int i=0;i<msgLength+7;i++)
  {
    uart3_write(full_message[i]);
    millis(1);
  }
}

void receive_telemtry(char *Outstr,  const unsigned int msgLength){
  unsigned char uart_data=0;
  char START_IN[6] = "$PC";
  char temp[6]="";
  char full_message[msgLength];
  memset(full_message, '\0', msgLength);
  bool busy = true, start_temp = false, start = false;
  int k =0, comp = 0, cnt = 0, pos = 0 ;

    while((k<msgLength) && busy){ //(busy){
      uart3_read(&uart_data);
      // find end
      if((uart_data=='!')&&start){
        start = false;
        //printf("Message: %s\n",full_message);
        cnt=0;
        pos=0;
        busy = false;
      }
      // Appending message
      if(start){
        full_message[cnt]=uart_data;
        cnt++;
      }
      // find beginning
      if((uart_data=='$') && !start){
        pos = 0;
        start = true;
        cnt = 0;
      }
      millis(3);
      k++;
    }
    strcpy(Outstr, full_message);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void exchange_telemetry_data(void){
  const unsigned int MSG_LENGTH = 256;
  char msgMotor[64], msgAngles[64], msgPID[64];
  char msgTotal[MSG_LENGTH];
  
  // ----------- Send data ----------- 
  // encode message
  sprintf(msgMotor,"%d;%d;%d;%d;", esc_1, esc_2, esc_3, esc_4);
  sprintf(msgAngles,"%4.7f;%4.7f;",roll_level_adjust,pitch_level_adjust);
  sprintf(msgPID,"%4.7f;%4.7f;%4.7f;",pid_output_roll,pid_output_pitch,pid_output_yaw);

  sprintf(msgTotal,"%s%s%s",msgMotor,msgAngles,msgPID);

  // printf("---> Sending: <---\nmsgMotor = %s \n", msgMotor);
  // printf("msgAngles = %s \n", msgAngles);
  // printf("msgPID = %s \n", msgPID);

  // printf("msgTotal = %s \n", msgTotal);
  // send to PC
  send_telemtry(msgTotal, MSG_LENGTH);

  // ----------- Receive data ----------- 
  // char msgRcv[MSG_LENGTH];
  // receive_telemtry(msgRcv);
  // decode message
}

void get_telemetry_pid(char *message, const unsigned int msgLength){
  char msgRcv[msgLength];
  memset(msgRcv, '\0', msgLength);
  snprintf(msgRcv, msgLength, "%s",message);
  // decode message
  char delC = ';';
  bool valid[7],delFound, validRoll, validPitch, validYaw;
  // roll
  valid[0] = (msgRcv[0]>='0') && (msgRcv[0]<='9');
  valid[1] = (msgRcv[1]>='0') && (msgRcv[1]<='9');

  valid[2] = (msgRcv[3]>='0') && (msgRcv[3]<='9');
  valid[3] = (msgRcv[4]>='0') && (msgRcv[4]<='9');

  valid[4] = (msgRcv[6]>='0') && (msgRcv[6]<='9');
  valid[5] = (msgRcv[7]>='0') && (msgRcv[7]<='9');
  valid[6] = (msgRcv[8]>='0') && (msgRcv[8]<='9');

  validRoll = valid[0] && valid[1] && valid[2] && valid[3] && valid[4] && valid[5] && valid[6];

  // Pitch
  valid[0] = (msgRcv[10]>='0') && (msgRcv[10]<='9');
  valid[1] = (msgRcv[11]>='0') && (msgRcv[11]<='9');

  valid[2] = (msgRcv[13]>='0') && (msgRcv[13]<='9');
  valid[3] = (msgRcv[14]>='0') && (msgRcv[14]<='9');

  valid[4] = (msgRcv[16]>='0') && (msgRcv[16]<='9');
  valid[5] = (msgRcv[17]>='0') && (msgRcv[17]<='9');
  valid[6] = (msgRcv[18]>='0') && (msgRcv[18]<='9');

  validPitch = valid[0] && valid[1] && valid[2] && valid[3] && valid[4] && valid[5] && valid[6];

  // Yaw
  valid[0] = (msgRcv[20]>='0') && (msgRcv[20]<='9');
  valid[1] = (msgRcv[21]>='0') && (msgRcv[21]<='9');

  valid[2] = (msgRcv[23]>='0') && (msgRcv[23]<='9');
  valid[3] = (msgRcv[24]>='0') && (msgRcv[24]<='9');

  valid[4] = (msgRcv[26]>='0') && (msgRcv[26]<='9');
  valid[5] = (msgRcv[27]>='0') && (msgRcv[27]<='9');
  valid[6] = (msgRcv[28]>='0') && (msgRcv[28]<='9');

  validYaw = valid[0] && valid[1] && valid[2] && valid[3] && valid[4] && valid[5] && valid[6]; 

  // find delimiters
  delFound = msgRcv[2]==delC && msgRcv[5]==delC && msgRcv[9]==delC && msgRcv[12]==delC && msgRcv[15]==delC && msgRcv[19]==delC && msgRcv[22]==delC && msgRcv[25]==delC;

  // convert values if message is valid
  pid_tel = validRoll && validPitch && validYaw && delFound;
  if (validRoll && validPitch && validYaw && delFound)
  {
    p_roll_tel = ((msgRcv[0]-'0')*10 + (msgRcv[1]-'0'))*p_angle_tel;
    i_roll_tel = ((msgRcv[3]-'0')*10 + (msgRcv[4]-'0'))*i_angle_tel;
    d_roll_tel = ((msgRcv[6]-'0')*100 + (msgRcv[7]-'0')*10 + (msgRcv[8]-'0'))*d_angle_tel;

    p_pitch_tel = ((msgRcv[10]-'0')*10 + (msgRcv[11]-'0'))*p_angle_tel;
    i_pitch_tel = ((msgRcv[13]-'0')*10 + (msgRcv[14]-'0'))*i_angle_tel;
    d_pitch_tel = ((msgRcv[16]-'0')*100 + (msgRcv[17]-'0')*10 + (msgRcv[18]-'0'))*d_angle_tel;

    p_yaw_tel = ((msgRcv[20]-'0')*10 + (msgRcv[21]-'0'))*p_angle_tel;
    i_yaw_tel = ((msgRcv[23]-'0')*10 + (msgRcv[24]-'0'))*i_angle_tel;
    d_yaw_tel = ((msgRcv[26]-'0')*100 + (msgRcv[27]-'0')*10 + (msgRcv[28]-'0'))*d_angle_tel;
  }


}