#include "can_hw.h"

#include <iostream>
#include <cstring>

int can_id[14]={16,17,18,19,20,21,22,  23,24,25,26,27,28,29};

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




float CANMotorInterface::receive_angle(uint8_t motor_id, uint8_t command)
{
    VCI_CAN_OBJ send;
    send.SendType = 0;
    send.RemoteFlag = 0;
    send.ExternFlag = 0;
    send.DataLen = 1;

    send.ID = can_id[motor_id];
    send.Data[0] = command;

    if (VCI_Transmit(DeviceType, 0, CANInd, &send, 1) == 1)
    {
        VCI_CAN_OBJ rec[3000];
        int cnt = 2;

        while (VCI_Receive(DeviceType, 0, CANInd, rec, 3000, 100) <= 0 && cnt)
            cnt--;
        if (cnt == 0)
            std::cout << "aaaaaa!!! ops! ID " << send.ID << " receive failed!  command  " << command<< std::endl;
        else
        {
            std::uint8_t hexArray[4] = {rec[0].Data[4], rec[0].Data[3], rec[0].Data[2], rec[0].Data[1]};
            std::int32_t decimal = convertHexArrayToDecimal(hexArray);
            float res = decimal * 1.0/ ratio * 2 * 3.14;
            // std::cout << "ID: " << send.ID <<"  decimal: " << decimal <<" res: " << res << std::endl;
            return res;
        }
    }
    else
        std::cout << "aaaaaa!!! ops! ID " << send.ID << " transmit failed!" << std::endl;
    return 0;
    
}

uint32_t CANMotorInterface::receive_vel(uint8_t motor_id, uint8_t command)
{
    VCI_CAN_OBJ send;
    send.SendType = 0;
    send.RemoteFlag = 0;
    send.ExternFlag = 0;
    send.DataLen = 1;

    send.ID = can_id[motor_id];
    send.Data[0] = command;

    if (VCI_Transmit(DeviceType, 0, CANInd, &send, 1) == 1)
    {
        VCI_CAN_OBJ rec[3000];
        int cnt = 2;

        while (VCI_Receive(DeviceType, 0, CANInd, rec, 3000, 100) <= 0 && cnt)
            cnt--;
        if (cnt == 0)
            std::cout << "aaaaaa!!! ops! ID " << send.ID << " receive failed!  command  " << command<< std::endl;
        else
        {
            std::uint8_t hexArray[4] = {rec[0].Data[4], rec[0].Data[3], rec[0].Data[2], rec[0].Data[1]};
            std::int32_t decimal = convertHexArrayToDecimal(hexArray);
            // std::cout << "ID: " << send.ID <<"  decimal: " << decimal << std::endl;
            return decimal;
        }
    }
    else
        std::cout << "aaaaaa!!! ops! ID " << send.ID << " transmit failed!" << std::endl;
    return 0;
    
}



void CANMotorInterface::sendCanCommand(uint8_t motor_id, uint8_t command, float parameter)
{
    VCI_CAN_OBJ send;
    send.SendType = 0;
    send.RemoteFlag = 0;
    send.ExternFlag = 0;
    send.DataLen = 5;

    send.ID = can_id[motor_id];
    send.Data[0] = command;
    int res[4], cnt = 2, reclen = 0;

    uint32_t parameter_int = static_cast<uint32_t>(parameter / 2 / 3.14 * ratio);


    toIntArray(parameter_int, res, 4);

    for (int j = 1; j < 5; j++)
        send.Data[j] = res[j - 1];

    while (VCI_Transmit(DeviceType, 0, CANInd, &send, 1) <= 0 && cnt)
        cnt--;
    if (cnt == 0)   
            std::cout << "aaaaaa!!! sendCanCommand ID " << send.ID << " transmit failed!" << std::endl;
    // else
    // {
    //     std::cout << "ID: " << send.ID << std::endl;
    //     for (int c = 0; c < send.DataLen; c++)
    //         printf("  %02X ", send.Data[c]);
    //     std::cout << std::endl;   
    // }
}

void CANMotorInterface::set_vel(uint8_t motor_id, uint8_t command, uint32_t velocity)
{
    VCI_CAN_OBJ send;
    send.SendType = 0;
    send.RemoteFlag = 0;
    send.ExternFlag = 0;    
    send.DataLen = 5;

    send.ID = can_id[motor_id];
    send.Data[0] = command;
    int res[4], cnt = 4, reclen = 0;

    toIntArray(velocity, res, 4);

    for (int j = 1; j < 5; j++)
        send.Data[j] = res[j - 1];

    while (VCI_Transmit(DeviceType, 0, CANInd, &send, 1) <= 0 && cnt)
        cnt--;
    if (cnt == 0)
        std::cout << "aaaaaa!!! set_vel ID " << send.ID << " transmit failed!" << std::endl;
    // else
    // {
    //     std::cout << "ID: " << send.ID << std::endl;
    //     for (int c = 0; c < send.DataLen; c++)
    //         printf("  %02X ", send.Data[c]);
    //     std::cout << std::endl;
    // }
}



bool CANMotorInterface::closeCAN()
{
    VCI_CloseDevice(DeviceType, DeviceIndex);
}


