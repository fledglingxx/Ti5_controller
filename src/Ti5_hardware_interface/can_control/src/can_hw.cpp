#include "can_hw.h"

#include <iostream>
#include <cstring>


CANMotorInterface::CANMotorInterface() {}

CANMotorInterface::~CANMotorInterface() {
    VCI_CloseDevice(DeviceType, DeviceIndex);
    std::cout << "CANMotorInerface destructor called" << std::endl;
}



int32_t CANMotorInterface::convertHexArrayToDecimal(const std::uint8_t hexArray[4])
{
    std::int32_t result = 0;
    for (int i = 0; i < 4; i++)
        result = (result << 8) | hexArray[i];
    if (result > 0x7FFFFFFF)
        result -= 0x100000000;

    return result;
}

void CANMotorInterface::toIntArray(int number, int *res, int size)
{
    unsigned int unsignedNumber = static_cast<unsigned int>(number);

    for (int i = 0; i < size; ++i)
    {
        res[i] = unsignedNumber & 0xFF;
        unsignedNumber >>= 8;
    }
}


bool CANMotorInterface::initCAN() 
{
    VCI_INIT_CONFIG vic;
    vic.AccCode = 0x80000008;
    vic.AccMask = 0xFFFFFFFF;
    vic.Filter = 0;
    vic.Timing0 = 0x00;
    vic.Timing1 = 0x14;
    vic.Mode = 0;

    if (VCI_OpenDevice(DeviceType, DeviceIndex, 0) != 1)
    {
        std::cout << "aaaaaa!!! Open device failed!" << std::endl;
        return false;
    }
    else
        std::cout << "hhhhhh!!! Open device success!" << std::endl;

    // tongdao 1
    if (VCI_InitCAN(DeviceType, DeviceIndex, CANInd, &vic) != 1)
    {
        std::cout << "aaaaaa!!! Init can0 failed!" << std::endl;
        VCI_CloseDevice(DeviceType, DeviceIndex);
        return false;
    }
    else
        std::cout << "hhhhhhh!!! Init can0 success!" << std::endl;

    if (VCI_StartCAN(DeviceType, DeviceIndex, CANInd) != 1)
    {
        std::cout << "aaaaaa!!! Start can0 failed!" << std::endl;
        VCI_CloseDevice(DeviceType, DeviceIndex);
        return false;
    }
    else
        std::cout << "hhhhhhh!!! Start can0 success!" << std::endl;

    return true;
}




int32_t CANMotorInterface::sendSimpleCanCommand(uint8_t motor_id, uint8_t command)
{
    VCI_CAN_OBJ send;
    send.SendType = 0;
    send.RemoteFlag = 0;
    send.ExternFlag = 0;
    send.DataLen = 1;

    send.ID = motor_id;
    send.Data[0] = command;

    if (VCI_Transmit(DeviceType, 0, CANInd, &send, 1) == 1)
    {
        VCI_CAN_OBJ rec[3000];
        int cnt = 2;

        while (VCI_Receive(DeviceType, 0, CANInd, rec, 3000, 100) <= 0 && cnt)
            cnt--;
        if (cnt == 0)
            std::cout << "aaaaaa!!! ops! ID " << send.ID << " receive failed!" << std::endl;
        else
        {
            std::uint8_t hexArray[4] = {rec[0].Data[4], rec[0].Data[3], rec[0].Data[2], rec[0].Data[1]};
            std::int32_t decimal = convertHexArrayToDecimal(hexArray);
            return decimal;
            //std::cout << "ID: " << send.ID << " Data: " << decimal << std::endl;
        }
    }
    else
        std::cout << "aaaaaa!!! ops! ID " << send.ID << " transmit failed!" << std::endl;
    return 0;
    
}

void CANMotorInterface::sendCanCommand(uint8_t motor_id, uint8_t command, uint32_t parameter)
{
    VCI_CAN_OBJ send;
    send.SendType = 0;
    send.RemoteFlag = 0;
    send.ExternFlag = 0;
    send.DataLen = 5;

    send.ID = motor_id;
    send.Data[0] = command;
    int res[4], cnt = 2, reclen = 0;
    toIntArray(parameter, res, 4);

    for (int j = 1; j < 5; j++)
        send.Data[j] = res[j - 1];

    while (VCI_Transmit(DeviceType, 0, CANInd, &send, 1) <= 0 && cnt)
        cnt--;
    if (cnt == 0)   
            std::cout << "aaaaaa!!!  ID " << send.ID << " transmit failed!" << std::endl;
    // else
    // {
    //     //std::cout << "ID: " << send.ID << std::endl;
        //for (int c = 0; c < send.DataLen; c++)
            //printf("  %02X ", send.Data[c]);
        // //std::cout << std::endl;   
        // }
}





