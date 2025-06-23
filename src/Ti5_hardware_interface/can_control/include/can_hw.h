#ifndef CAN_HW_H
#define CAN_HW_H

#include <iostream>
#include <cstring>
#include <cstdint>
#include <cmath>

#include "controlcan.h"  

#define ratio 262144


class CANMotorInterface
{
public:
    CANMotorInterface();
    ~CANMotorInterface();

    int32_t convertHexArrayToDecimal(const std::uint8_t hexArray[4]);
    void toIntArray(int number, int *res, int size);

    bool initCAN();
    int32_t sendSimpleCanCommand(uint8_t motor_id, uint8_t command);
    void sendCanCommand(uint8_t motor_id, uint8_t command, float parameter);
    void set_vel(uint8_t motor_id, uint8_t command, uint32_t velocity);


private:
    int DeviceType = VCI_USBCAN2;
    int DeviceIndex = 0;
    int CANInd = 0;
};

#endif //CAN_HW_H