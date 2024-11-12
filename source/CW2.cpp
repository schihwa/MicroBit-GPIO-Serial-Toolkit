#include "MicroBit.h"

void init_timer0(void);
void custom_delay_us(int time);
void init_serial(void);
void init_I2C_accel(void);
void init_PWM(void);
void setPWMFrequency(int freq);
void intToStr(int value, char* buf, int* idx);

// Configuration Constants for various pins, baud rates, and frequency limits
#define TX_PIN_P0_06        6           // GPIO for bit-bang serial (micro:bit interface chip)
#define TX_PIN_P2           2           // GPIO for bit-bang serial (edge connector P2)
#define BAUD_RATE           115200      // Serial communication baud rate
#define BIT_TIME_US         (1000000 / BAUD_RATE) // Bit interval for 115200 baud in microseconds
#define VOTE_STRING         "Twix\n\r"  // Vote string for Subtask 1
#define VOTE_INTERVAL_MS    240         // Interval between transmissions in milliseconds
#define ACCEL_I2C_ADDR      0x19        // I2C address for accelerometer
#define CTRL_REG1           0x20        // Accelerometer control register
#define OUT_X_L             0x28        // Register for X-axis acceleration data
#define OUT_Y_L             0x2A        // Register for Y-axis acceleration data
#define OUT_Z_L             0x2C        // Register for Z-axis acceleration data
#define SAMPLE_INTERVAL_MS  200         // Accelerometer sampling interval in milliseconds
#define SPEAKER_PIN         3           // Speaker GPIO pin
#define BUTTON_A_PIN        14          // GPIO pin for Button A
#define BUTTON_B_PIN        23          // GPIO pin for Button B
#define BASE_FREQ_HZ        1000        // Base frequency for sound generation
#define MIN_FREQ_HZ         500         // Minimum frequency for PWM
#define MAX_FREQ_HZ         5000        // Maximum frequency for PWM

// Helper Macros for common initializations and button press check
#define SERIAL_INIT()       { init_timer0(); init_serial(); } // Initialize serial communication
#define ACCEL_INIT()        init_I2C_accel()                 // Initialize accelerometer
#define PWM_INIT()          init_PWM()                       // Initialize PWM for sound
#define IS_BUTTON_PRESSED() (!(NRF_GPIO->IN & (1 << BUTTON_A_PIN)) || !(NRF_GPIO->IN & (1 << BUTTON_B_PIN)))

/*=================================== Subtask 1: Bit-Bang Serial Transmission ===================================
 *  Function: bitBangSerial(char *string)
 *  - Transmit a null-terminated string via bit-banging at 115200 baud rate on GPIO pins P0.06 and P2.
 *  - No UART or interrupts used; only GPIO.
 *  - Initialize once, then transmit the data with accurate timing for each bit.
 * 
 *  Function: voteForChocolate(void)
 *  - Repeatedly transmit the predefined string (VOTE_STRING) every 240ms.
 *  - Must maintain timing and does not return.
 * ============================================================================================================*/

// Function to transmit a string bit-by-bit at 115200 baud rate using bit-banging
void bitBangSerial(const char *string) {
    static int initialized = 0;  // Flag to ensure initialization happens only once
    if (!initialized) { SERIAL_INIT(); initialized = 1; } // Initialize if not already done

    while (*string) {  // Loop through each character in the input string
        char c = *string++; // Read current character and move to next one
        uint16_t frame = (1 << 9) | (c << 1); // Frame includes start bit (0), 8 data bits, stop bit (1)

        // Transmit each bit in the frame (start, data, stop bits)
        for (int i = 0; i < 10; i++) {
            if (frame & 0x01) // Check if current bit is 1
                NRF_GPIO->OUTSET = (1 << TX_PIN_P0_06) | (1 << TX_PIN_P2); // Set GPIO pins high for '1'
            else
                NRF_GPIO->OUTCLR = (1 << TX_PIN_P0_06) | (1 << TX_PIN_P2); // Set GPIO pins low for '0'

            custom_delay_us(BIT_TIME_US); // Delay for the duration of one bit
            frame >>= 1; // Shift frame to process the next bit
        }
        NRF_GPIO->OUTSET = (1 << TX_PIN_P0_06) | (1 << TX_PIN_P2); // Set stop bit high
    }
}

// Function to continuously transmit the vote string every 240 ms
void voteForChocolate(void) {
    while (1) {  // Infinite loop for continuous transmission
        bitBangSerial(VOTE_STRING);  // Transmit predefined vote string
        custom_delay_us(VOTE_INTERVAL_MS * 1000); // Wait 240 ms before next transmission
    }
}

/*=================================== Subtask 2: Accelerometer Data Display ===================================
 *  Function: getAccelerometerSample(char axis)
 *  - Uses I2C to retrieve acceleration data from LSM303AGR for a specified axis ('X', 'Y', 'Z').
 *  - Returns values in range -512 to +511, initializing only on first call.
 * 
 *  Function: showAccelerometerSamples(void)
 *  - Continuously fetches accelerometer data for X, Y, Z, and formats it for serial output.
 *  - Sends data in the format "[X: value] [Y: value] [Z: value]" approximately 5 times per second.
 * ============================================================================================================*/

// Function to retrieve accelerometer sample for a given axis ('X', 'Y', 'Z')
int getAccelerometerSample(char axis) {
    static int accel_initialized = 0; // Flag to ensure accelerometer initializes once
    if (!accel_initialized) { ACCEL_INIT(); accel_initialized = 1; } // Initialize if not already done

    uint8_t reg = (axis == 'X') ? OUT_X_L : (axis == 'Y') ? OUT_Y_L : OUT_Z_L; // Select register based on axis
    uint8_t data[2] = {0};  // Buffer for received data (2 bytes)

    // Write the selected register address to the accelerometer to initiate reading
    NRF_TWIM0->TXD.PTR = (uint32_t)&reg;  // Point to register address
    NRF_TWIM0->TXD.MAXCNT = 1;            // Transmit only 1 byte (register address)
    NRF_TWIM0->TASKS_STARTTX = 1;         // Start the transmission
    while (!NRF_TWIM0->EVENTS_STOPPED);   // Wait until transmission completes
    NRF_TWIM0->EVENTS_STOPPED = 0;        // Reset stop event

    // Read 2 bytes of data (low and high bytes for axis sample)
    NRF_TWIM0->RXD.PTR = (uint32_t)data; // Point to data buffer
    NRF_TWIM0->RXD.MAXCNT = 2;           // Receive 2 bytes
    NRF_TWIM0->TASKS_STARTRX = 1;        // Start reception
    while (!NRF_TWIM0->EVENTS_STOPPED);  // Wait until reception completes
    NRF_TWIM0->EVENTS_STOPPED = 0;       // Reset stop event

    // Combine the two bytes to form a 16-bit sample, and constrain the result within -512 to 511
    int16_t sample = ((int16_t)data[1] << 8) | data[0]; // Combine high and low bytes
    return (sample > 511) ? 511 : (sample < -512) ? -512 : sample; // Constrain to -512 to +511
}

// Function to display accelerometer samples for X, Y, Z axes continuously
void showAccelerometerSamples(void) {
    while (1) {  // Infinite loop for continuous sampling and display
        int x = getAccelerometerSample('X'); // Get X-axis sample
        int y = getAccelerometerSample('Y'); // Get Y-axis sample
        int z = getAccelerometerSample('Z'); // Get Z-axis sample

        char buffer[50]; // Buffer for formatted output
        int index = 0;   // Buffer index

        // Format and store the X-axis value in buffer
        buffer[index++] = '['; buffer[index++] = 'X'; buffer[index++] = ':';
        intToStr(x, buffer, &index); // Convert X value to string and add to buffer
        buffer[index++] = ']';

        // Format and store the Y-axis value in buffer
        buffer[index++] = '['; buffer[index++] = 'Y'; buffer[index++] = ':';
        intToStr(y, buffer, &index);
        buffer[index++] = ']';

        // Format and store the Z-axis value in buffer
        buffer[index++] = '['; buffer[index++] = 'Z'; buffer[index++] = ':';
        intToStr(z, buffer, &index);
        buffer[index++] = ']';
        buffer[index++] = '\r'; buffer[index++] = '\n';
        buffer[index] = '\0';  // Null-terminate the string

        bitBangSerial(buffer); // Transmit formatted data
        custom_delay_us(SAMPLE_INTERVAL_MS * 1000); // Wait before next sample
    }
}

/*=================================== Subtask 3: Sound Generation Based on Y-Axis ===================================
 *  Function: makeNoise(void)
 *  - Produces a square wave sound on the speaker pin whenever either Button A or Button B is pressed.
 *  - Basic implementation produces a constant 1kHz sound.
 *  - Advanced requirement dynamically adjusts the frequency between 500Hz and 5kHz based on the Y-axis accelerometer reading.
 * =================================================================================================================*/

// Function to generate noise based on Y-axis data when a button is pressed
void makeNoise(void) {
    PWM_INIT(); // Initialize PWM for speaker output
    while (1) {
        if (IS_BUTTON_PRESSED()) { // Check if any button is pressed
            int16_t y = getAccelerometerSample('Y'); // Get Y-axis value from accelerometer
            // Map Y-axis data to a frequency between MIN_FREQ_HZ and MAX_FREQ_HZ
            int freq = MIN_FREQ_HZ + ((y + 512) * (MAX_FREQ_HZ - MIN_FREQ_HZ)) / 1023;
            setPWMFrequency(freq); // Set PWM frequency based on Y-axis data
        }
    }
}

/*=========================================== Helper Functions ===========================================
 *  - Contains utility functions for initialization, PWM configuration, and custom delays.
 *  - Utility functions:
 *      - init_timer0(): Initializes Timer0 for generating custom delays.
 *      - custom_delay_us(int time): Delays for a specified time in microseconds using Timer0.
 *      - init_serial(): Configures GPIO pins for serial transmission.
 *      - init_I2C_accel(): Sets up I2C communication with the accelerometer.
 *      - init_PWM(): Configures PWM for audio output on the speaker pin.
 *      - setPWMFrequency(int freq): Adjusts the PWM frequency on the speaker pin.
 *      - intToStr(int value, char* buf, int* idx): Converts an integer to its string representation.
 * ======================================================================================================*/

// Timer0 Initialization for custom delay generation
void init_timer0(void) {
    NRF_TIMER0->MODE = TIMER_MODE_MODE_Timer;  // Set Timer0 to Timer mode
    NRF_TIMER0->BITMODE = TIMER_BITMODE_BITMODE_32Bit;  // Set Timer0 to 32-bit mode
    NRF_TIMER0->PRESCALER = 4; // Prescaler for 1 MHz timer frequency
    NRF_TIMER0->TASKS_CLEAR = 1;  // Clear the timer count
}

// Function to create a custom delay using Timer0
void custom_delay_us(int time) {
    NRF_TIMER0->TASKS_CLEAR = 1;  // Clear the timer
    NRF_TIMER0->CC[0] = time;     // Set compare register to delay time
    NRF_TIMER0->EVENTS_COMPARE[0] = 0; // Clear compare event
    NRF_TIMER0->TASKS_START = 1;       // Start the timer
    while (!NRF_TIMER0->EVENTS_COMPARE[0]); // Wait until compare event occurs
    NRF_TIMER0->TASKS_STOP = 1;            // Stop the timer
}

// Configure GPIO pins for serial transmission
void init_serial(void) {
    NRF_GPIO->PIN_CNF[TX_PIN_P0_06] = (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos); // Configure TX_PIN_P0_06 as output
    NRF_GPIO->PIN_CNF[TX_PIN_P2] = (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos); // Configure TX_PIN_P2 as output
    NRF_GPIO->OUTSET = (1 << TX_PIN_P0_06) | (1 << TX_PIN_P2); // Set both pins high (idle state for serial)
}

// Initialize I2C for accelerometer communication
void init_I2C_accel(void) {
    NRF_TWIM0->PSEL.SCL = 27;   // Set SCL pin
    NRF_TWIM0->PSEL.SDA = 26;   // Set SDA pin
    NRF_TWIM0->FREQUENCY = TWIM_FREQUENCY_FREQUENCY_K100 << TWIM_FREQUENCY_FREQUENCY_Pos; // Set I2C frequency to 100kHz
    NRF_TWIM0->ENABLE = TWIM_ENABLE_ENABLE_Enabled << TWIM_ENABLE_ENABLE_Pos; // Enable I2C

    // Configure accelerometer control register to enable 100Hz sampling
    uint8_t config[2] = {CTRL_REG1, 0x57};
    NRF_TWIM0->TXD.PTR = (uint32_t)config;
    NRF_TWIM0->TXD.MAXCNT = 2;
    NRF_TWIM0->TASKS_STARTTX = 1;
    while (!NRF_TWIM0->EVENTS_STOPPED); // Wait until configuration is complete
    NRF_TWIM0->EVENTS_STOPPED = 0;
}

// Initialize PWM for speaker output
void init_PWM(void) {
    NRF_GPIO->PIN_CNF[SPEAKER_PIN] = (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos); // Set speaker pin as output
    NRF_PWM0->PSEL.OUT[0] = SPEAKER_PIN; // Connect PWM to speaker pin
    NRF_PWM0->MODE = PWM_MODE_UPDOWN_Up; // Set PWM mode to up counter
    NRF_PWM0->PRESCALER = PWM_PRESCALER_PRESCALER_DIV_16; // Set prescaler
    NRF_PWM0->DECODER = PWM_DECODER_LOAD_Individual; // Set decoder mode
}

// Set PWM frequency on speaker pin
void setPWMFrequency(int freq) {
    freq = (freq < MIN_FREQ_HZ) ? MIN_FREQ_HZ : (freq > MAX_FREQ_HZ) ? MAX_FREQ_HZ : freq; // Clamp frequency within range
    NRF_PWM0->COUNTERTOP = 16000000 / (freq * 2); // Set frequency based on COUNTERTOP value
    uint16_t duty = NRF_PWM0->COUNTERTOP / 2; // 50% duty cycle for square wave
    NRF_PWM0->SEQ[0].PTR = (uint32_t)&duty;
    NRF_PWM0->TASKS_SEQSTART[0] = 1; // Start PWM sequence
}

// Converts integer to string representation
void intToStr(int value, char* buf, int* idx) {
    if (value < 0) { buf[(*idx)++] = '-'; value = -value; } // Add '-' for negative values
    int digits[10];
    int digitCount = 0;
    do { digits[digitCount++] = value % 10; value /= 10; } while (value > 0); // Extract digits
    while (digitCount > 0) { buf[(*idx)++] = '0' + digits[--digitCount]; } // Convert digits to characters
}
