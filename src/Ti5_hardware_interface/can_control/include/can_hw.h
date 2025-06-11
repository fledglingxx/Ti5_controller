#ifndef CAN_HW_H
#define CAN_HW_H

#include <cstring>
#include <iostream>


int32_t convertHexArrayToDecimal(const std::uint8_t hexArray[4]);
void toIntArray(int number, int *res, int size);
void sendSimpleCanCommand(uint8_t numOfActuator, uint8_t *canIdList, uint8_t *commandList, int CANInd);
void sendCanCommand(uint8_t numOfActuator, uint8_t *canIdList, uint8_t *commandList, uint32_t *parameterList,int CANInd);
bool init_can();
