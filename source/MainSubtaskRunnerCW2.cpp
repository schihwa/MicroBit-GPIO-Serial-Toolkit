#include "MicroBit.h"

// Declare the Subtasks that are in CW2.cpp so we can call them 
extern void bitBangSerial(char *string);
extern void voteForChocolate(void);
extern int  getAccelerometerSample(char);
extern void showAccelerometerSamples(void);
extern void makeNoise(void);

// Entry point is a menu that allows any Subtask to be run
int main() {

    int in = 0; // input character

    // Create a scope for a serial port object running over USB back to the host PC
    // having received a valid character leave this scope and destroy the object before 
    // calling the relevant Subtask
    {
        // Create the serial port objects
        NRF52Pin    usbTx(ID_PIN_USBTX, MICROBIT_PIN_UART_TX, PIN_CAPABILITY_DIGITAL);
        NRF52Pin    usbRx(ID_PIN_USBRX, MICROBIT_PIN_UART_RX, PIN_CAPABILITY_DIGITAL);
        NRF52Serial serial(usbTx, usbRx, NRF_UARTE0);

        // Display instructions
        serial.printf("\r\nEnter a number 1-3 to run a CW2 subtask: ");

        // Repeatedly get and print characters until a valid selection is made
        while ((in < '1') || (in > '3')) {        // repeat until there's a valid character
            in = serial.getChar(SYNC_SPINWAIT);   // get a character from serial
            serial.printf("%c", in);              // echo it
        }
    }

    // having got a character that's in range, call the relevant Subtask
    // (which we don't expect to ever return!)
    switch (in) {
        case '1':
            voteForChocolate();
            break;
        case '2':
            showAccelerometerSamples();
            break;
        case '3':
            makeNoise();
            break;
    }
}