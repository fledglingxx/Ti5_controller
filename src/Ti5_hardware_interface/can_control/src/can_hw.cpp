#include "controlcan.h"

#include <iostream>
#include <string>


int32_t convertHexArrayToDecimal(const std::uint8_t hexArray[4])
{
    std::int32_t result = 0;
    for (int i = 0; i < 4; i++)
        result = (result << 8) | hexArray[i];
    if (result > 0x7FFFFFFF)
        result -= 0x100000000;

    return result;
}

void toIntArray(int number, int *res, int size)
{
    unsigned int unsignedNumber = static_cast<unsigned int>(number);

    for (int i = 0; i < size; ++i)
    {
        res[i] = unsignedNumber & 0xFF;
        unsignedNumber >>= 8;
    }
}


void sendSimpleCanCommand(uint8_t numOfActuator, uint8_t *canIdList, uint8_t *commandList, int CANInd)
{
    VCI_CAN_OBJ send[1];
    send[0].SendType = 0;
    send[0].RemoteFlag = 0;
    send[0].ExternFlag = 0;
    send[0].DataLen = 1;

    for (int i = 0; i < numOfActuator; i++)
    {
        send[0].ID = canIdList[i];
        send[0].Data[0] = commandList[i];

        if (VCI_Transmit(VCI_USBCAN2, 0, CANInd, send, 1) == 1)
        {
            VCI_CAN_OBJ rec[3000];
            int reclen = 0, cnt = 2;

            while (VCI_Receive(VCI_USBCAN2, 0, CANInd, rec, 3000, 100)<= 0 && cnt)
                cnt--;
            if (cnt == 0)
                std::cout << "ops! ID " << send[0].ID << " receive failed!" << std::endl;
            else
            {
                for (int j = 0; j < reclen; j++)
                {
                    std::uint8_t hexArray[4] = {rec[j].Data[4], rec[j].Data[3], rec[j].Data[2], rec[j].Data[1]};
                    std::int32_t decimal = convertHexArrayToDecimal(hexArray);
                    //std::cout << "ID: " << send[0].ID << " Data: " << decimal << std::endl;
                }
            }
        }
        else
            break;
    }
}

void sendCanCommand(uint8_t numOfActuator, uint8_t *canIdList, uint8_t *commandList, uint32_t *parameterList,int CANInd)
{
    VCI_CAN_OBJ send[1];
    send[0].SendType = 0;
    send[0].RemoteFlag = 0;
    send[0].ExternFlag = 0;
    send[0].DataLen = 5;

    for (int i = 0; i < numOfActuator; i++)
    {
        send[0].ID = canIdList[i];
        send[0].Data[0] = commandList[i];
        int res[4], cnt = 2, reclen = 0;
        toIntArray(parameterList[i], res, 4);

        for (int j = 1; j < 5; j++)
            send[0].Data[j] = res[j - 1];

        while (VCI_Transmit(VCI_USBCAN2, 0, CANInd, send, 1) <= 0 && cnt)
            cnt--;
        if (cnt == 0)
            std::cout << "ops! ID " << send[0].ID << " transmit failed!" << std::endl;
        else
        {
            //std::cout << "ID: " << send[0].ID << std::endl;
            //for (int c = 0; c < send[0].DataLen; c++)
               // printf("  %02X ", send[0].Data[c]);
            //std::cout << std::endl;
        }
    }
}





bool init_can()
{
    int nDeviceType = 4;
    int nDeviceInd = 0;
    int nCANInd_1 = 0, nCANInd_2 = 1;

    if (VCI_OpenDevice(nDeviceType, nDeviceInd, 0) != 1)
    {
        std::cout << "Open device failed!" << std::endl;
        return false;
    }
    else
        std::cout << "Open device success!" << std::endl;

    VCI_INIT_CONFIG vic;
    vic.AccCode = 0x80000008;
    vic.AccMask = 0xFFFFFFFF;
    vic.Filter = 0;
    vic.Timing0 = 0x00;
    vic.Timing1 = 0x14;
    vic.Mode = 0;

    // tongdao 1
    if (VCI_InitCAN(nDeviceType, nDeviceInd, nCANInd_1, &vic) != 1)
    {
        std::cout << "Init can 1 failed!" << std::endl;
        VCI_CloseDevice(nDeviceType, nDeviceInd);
        return false;
    }
    else
        std::cout << "Init can 1 success!" << std::endl;

    if (VCI_StartCAN(VCI_USBCAN2, 0, 0) != 1)
    {
        std::cout << "Start can 1 failed!" << std::endl;
        VCI_CloseDevice(nDeviceType, nDeviceInd);
        return false;
    }
    else
        std::cout << "Start can 1 success!" << std::endl;
        
    // tongdao 2
    if (VCI_InitCAN(nDeviceType, nDeviceInd, nCANInd_2, &vic) != 1)
    {
        std::cout << "Init can 2 failed!" << std::endl;
        VCI_CloseDevice(nDeviceType, nDeviceInd);
        return false;
    }
    else
        std::cout << "Init can 2 success!" << std::endl;

    if (VCI_StartCAN(VCI_USBCAN2, 0, 1) != 1)
    {
        std::cout << "Start can 2 failed!" << std::endl;
        VCI_CloseDevice(nDeviceType, nDeviceInd);
        return false;
    }
    else
        std::cout << "Start can 2 success!" << std::endl;

    return true;
}