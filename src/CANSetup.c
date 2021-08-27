/*
 * CAN.c
 *
 *  Created on: Feb 22, 2019
 *      Author: mfeurtado
 */
#include <CANSetup.h>
#include "driverlib.h"
#include "device.h"

#define BTR_REGISTER (0x2593U)
    /*
        CAN Packet Description for Dual Inverter in Parallel

        Fs = 0-0255     10      8b
        Id = 0-1023     10      10b
        Td = 0-4095     1300    12b
        Ff = 0-1023     300     10b

        Power Disable UVW = 0b0123, 4b, padded 0
        Logic Enable UVW = 0b0123, 4b, padded 0
        Reset/Fault UVW = 0bR123, 4b
        Power Disable XYZ = 0b0123, 4b, padded 0
        Logic Enable XYZ = 0b0123, 4b, padded 0
        Reset/Fault XYZ = 0b0123, 4b, padded 0

        Uses all 8B with several padded 0s
        Note in TI DSP, MDH is lower 4B, and MDL is upper 4B
    */

void initCANGPIO()
{

    // CAN-A
    //
    // Enable CAN-A on GPIO30 - GPIO31
    /*
    30  CANA-RX
    31  CANA-TX *NOTE* TMDSCNCD28379D (F28379D ControlCard) GPIO31 is connected to on-board LED through buffer
    */
   // GPIO_setPadConfig(18, GPIO_PIN_TYPE_PULLUP);     // Enable pullup on GPIO18
    GPIO_setPinConfig(GPIO_30_CANRXA);               // GPIO30 = CANRXA
   // GPIO_setPadConfig(19, GPIO_PIN_TYPE_PULLUP);     // Enable pullup on GPIO19
    GPIO_setQualificationMode(31, GPIO_QUAL_ASYNC);  // asynch input
    GPIO_setPinConfig(GPIO_31_CANTXA);               // GPIO31 = CANTXA


}

void initCAN()
{
    //
    // Initialize the CAN controller
    //
    CAN_initModule(CANA_BASE);
    CAN_selectClockSource(CANA_BASE, CAN_CLOCK_SOURCE_SYS);
    //
    // Set up the CAN bus bit rate to 500kHz
    // Refer to the Driver Library User Guide for information on how to set
    // tighter timing control. Additionally, consult the device data sheet
    // for more information about the CAN module clocking.
    //
    //CAN_setBitRate(CANA_BASE, DEVICE_SYSCLK_FREQ, 1000000, 16);
    CAN_setBitTiming(CANA_BASE, 19 ,0,5,2,2);

    //
    // Enable interrupts on the CAN peripheral.
    //
   // CAN_enableInterrupt(CANA_BASE, CAN_INT_IE0 | CAN_INT_ERROR |
   //                     CAN_INT_STATUS);
    //
    // Enable CAN test mode with external loopback
    //
    //CAN_enableTestMode(CANA_BASE, CAN_TEST_EXL);

    //
    // Initialize the transmit message object used for sending CAN messages.
    // Message Object Parameters:
    //      Message Object ID Number: 1
    //      Message Identifier: 0x00FF0000
    //      Message Frame: Standard
    //      Message Type: Transmit
    //      Message ID Mask: 0x0
    //      Message Object Flags: None
    //      Message Data Length: 8 Bytes
    //
    CAN_setupMessageObject(CANA_BASE, 1, 0x00000000, CAN_MSG_FRAME_STD,
                           CAN_MSG_OBJ_TYPE_TX, 3, CAN_MSG_OBJ_NO_FLAGS,
                           8);

    //
    // Initialize the receive message object used for receiving CAN messages.
    // Message Object Parameters:
    //      Message Object ID Number: 2
    //      Message Identifier: 0x000000FF
    //      Message Frame: Standard
    //      Message Type: Receive
    //      Message ID Mask: 0x0
    //      Message Object Flags: None
    //      Message Data Length: 8 Bytes
    //
    CAN_setupMessageObject(CANA_BASE, 2, 0x00000000, CAN_MSG_FRAME_STD,
                           CAN_MSG_OBJ_TYPE_RX, 0x000000FF, CAN_MSG_OBJ_NO_FLAGS,
                           8);


    // TEMPERATURES UVW
    // Initialize the transmit message object used for sending CAN messages.
    // Message Object Parameters:
    //      Message Object ID Number: 3
    //      Message Identifier: 0x000000FF
    //      Message Frame: Standard
    //      Message Type: Transmit
    //      Message ID Mask: 0x0
    //      Message Object Flags: None
    //      Message Data Length: 8 Bytes
    //
    CAN_setupMessageObject(CANA_BASE, 3, 0x000000FF, CAN_MSG_FRAME_STD,
                           CAN_MSG_OBJ_TYPE_TX, 3, CAN_MSG_OBJ_NO_FLAGS,
                           8);

    // CURRENTS UVW
    // Initialize the transmit message object used for sending CAN messages.
    // Message Object Parameters:
    //      Message Object ID Number: 4
    //      Message Identifier: 0x000000FE
    //      Message Frame: Standard
    //      Message Type: Transmit
    //      Message ID Mask: 0x0
    //      Message Object Flags: None
    //      Message Data Length: 8 Bytes
    //
    CAN_setupMessageObject(CANA_BASE, 4, 0x000000FE, CAN_MSG_FRAME_STD,
                           CAN_MSG_OBJ_TYPE_TX, 3, CAN_MSG_OBJ_NO_FLAGS,
                           8);

    // VOLTAGES UVW
    // Initialize the transmit message object used for sending CAN messages.
    // Message Object Parameters:
    //      Message Object ID Number: 5
    //      Message Identifier: 0x000000FD
    //      Message Frame: Standard
    //      Message Type: Transmit
    //      Message ID Mask: 0x0
    //      Message Object Flags: None
    //      Message Data Length: 8 Bytes
    //
    CAN_setupMessageObject(CANA_BASE, 5, 0x000000FD, CAN_MSG_FRAME_STD,
                           CAN_MSG_OBJ_TYPE_TX, 3, CAN_MSG_OBJ_NO_FLAGS,
                           8);

    // TEMPERATURES XYZ
    // Initialize the transmit message object used for sending CAN messages.
    // Message Object Parameters:
    //      Message Object ID Number: 6
    //      Message Identifier: 0x000000FF
    //      Message Frame: Standard
    //      Message Type: Transmit
    //      Message ID Mask: 0x0
    //      Message Object Flags: None
    //      Message Data Length: 8 Bytes
    //
    CAN_setupMessageObject(CANA_BASE, 6, 0x000000FC, CAN_MSG_FRAME_STD,
                           CAN_MSG_OBJ_TYPE_TX, 3, CAN_MSG_OBJ_NO_FLAGS,
                           8);

    // CURRENTS XYZ
    // Initialize the transmit message object used for sending CAN messages.
    // Message Object Parameters:
    //      Message Object ID Number: 7
    //      Message Identifier: 0x000000FE
    //      Message Frame: Standard
    //      Message Type: Transmit
    //      Message ID Mask: 0x0
    //      Message Object Flags: None
    //      Message Data Length: 8 Bytes
    //
    CAN_setupMessageObject(CANA_BASE, 7, 0x000000FB, CAN_MSG_FRAME_STD,
                           CAN_MSG_OBJ_TYPE_TX, 3, CAN_MSG_OBJ_NO_FLAGS,
                           8);

    // VOLTAGES XYZ
    // Initialize the transmit message object used for sending CAN messages.
    // Message Object Parameters:
    //      Message Object ID Number: 8
    //      Message Identifier: 0x000000FD
    //      Message Frame: Standard
    //      Message Type: Transmit
    //      Message ID Mask: 0x0
    //      Message Object Flags: None
    //      Message Data Length: 8 Bytes
    //
    CAN_setupMessageObject(CANA_BASE, 8, 0x000000FA, CAN_MSG_FRAME_STD,
                           CAN_MSG_OBJ_TYPE_TX, 3, CAN_MSG_OBJ_NO_FLAGS,
                           8);

    //
    // Start CAN module operations
    //
    CAN_startModule(CANA_BASE);
}


