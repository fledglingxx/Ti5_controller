#ifndef CAN_HW_H
#define CAN_HW_H

#include <cstring>
#include <iostream>


typedef struct {
    float position;
    float velocity;
    float torque;
}OD_Motor_State;


extern OD_Motor_State rv_motor_msg[20];



int32_t convertHexArrayToDecimal(const std::uint8_t hexArray[4]);
void toIntArray(int number, int *res, int size);
void sendSimpleCanCommand(uint8_t numOfActuator, uint8_t *canIdList, uint8_t *commandList, int CANInd);
void sendCanCommand(uint8_t numOfActuator, uint8_t *canIdList, uint8_t *commandList, uint32_t *parameterList,int CANInd);
bool init_can();

#endif //CAN_HW_H