/* SCC.369 Coursework 1: Working with GPIO - CW1.cpp */

#include "MicroBit.h"

/*-----------------------------------------
   GPIO Pin Definitions and Configurations
------------------------------------------*/

// Define GPIO pin numbers for LED row and columns
#define LED_ROW1_PIN 21 // P0.21 - First row of the LED matrix

// Define GPIO pin numbers for LED columns (5 columns for 5 LEDs)
#define LED_COL_PIN_0 28 // P0.28 - First column
#define LED_COL_PIN_1 11 // P0.11 - Second column
#define LED_COL_PIN_2 31 // P0.31 - Third column
#define LED_COL_PIN_3 5  // P1.05 - Fourth column (on Port 1)
#define LED_COL_PIN_4 30 // P0.30 - Fifth column

// Masks for LED column pins (used for setting or clearing bits in GPIO registers)
#define LED_COL_PIN_0_MASK (1 << LED_COL_PIN_0)
#define LED_COL_PIN_1_MASK (1 << LED_COL_PIN_1)
#define LED_COL_PIN_2_MASK (1 << LED_COL_PIN_2)
#define LED_COL_PIN_3_MASK (1 << LED_COL_PIN_3)
#define LED_COL_PIN_4_MASK (1 << LED_COL_PIN_4)

// Define GPIO pin numbers for Buttons
#define BUTTON_A_PIN 14 // P0.14 - Button A input pin
#define BUTTON_B_PIN 23 // P0.23 - Button B input pin

// Define GPIO pin number for analog input (for reading analog voltage)
#define ANALOG_PIN 2 // P0.02 (AIN0) - Analog input channel 0

// Define GPIO pin numbers for RGB LED colors
#define RED_PIN 1   // P0.01 - Red LED pin
#define GREEN_PIN 9 // P0.09 - Green LED pin
#define BLUE_PIN 8  // P0.08 - Blue LED pin

// Define GPIO pin number for touch input (used in touch sensor)
#define TOUCH_INPUT_PIN 4 // P1.04 - Touch input pin

/*-----------------------------------------
   Constants and Global Variables
------------------------------------------*/

#define DEBOUNCE_DELAY_MS 50     // Debounce delay time in milliseconds
#define COUNTING_DELAY_MS 200    // Delay between counts in milliseconds
#define LONG_TOUCH_DELAY_MS 1000 // Delay to detect a long touch in milliseconds
#define POLL_INTERVAL_MS 1       // Polling interval in milliseconds for state machine

static bool gpio_initialized = false; // Flag to indicate if GPIO has been initialized

/*-----------------------------------------
   Function Prototypes
------------------------------------------*/

// Function to configure a GPIO pin with specified configuration
void configureGPIO(uint32_t pin, uint32_t config);

// Function to create a custom delay using TIMER0
void custom_delay_ms(int ms);

// Function to initialize GPIO pins for LEDs, buttons, and touch input
void initializeGPIO(void);

// Functions for Subtask 1
void updateLEDs(uint8_t value);    // Update LEDs to display a binary value
void displayBinary(uint8_t value); // Display a binary value on LEDs indefinitely
void countUpBinary(
    uint8_t initialValue); // Count up from an initial value, displaying binary on LEDs

// Functions for Subtask 2
void countWithButtonsBinary(uint8_t initialValue); // Count up or down using buttons A and B

// Functions for Subtask 3
void initSAADC(void);            // Initialize the SAADC for analog voltage measurement
uint8_t sampleVoltage(void);     // Sample voltage from the analog input
void displayVoltageBinary(void); // Display sampled voltage as binary on LEDs

// Functions for Subtask 4
void driveRGB(void); // Drive the RGB LED with varying colors based on analog input

// Functions for Subtask 5
void countWithTouchesBinary(
    uint8_t initialValue); // Count up using touch input, with reset on long touch




/*-----------------------------------------
   Function Implementations
------------------------------------------*/

// Function to configure a GPIO pin with the specified configuration
inline void configureGPIO(uint32_t pin, uint32_t config)
{
    NRF_GPIO->PIN_CNF[pin] = config; // Set the pin configuration in the GPIO register
}

// Timer configurations for custom delay
#define TIMER_PRESCALER 4    // Prescaler for TIMER0 to achieve 1 MHz frequency (16 MHz / 2^4)
#define TIMER_TICKS_1MS 1000 // Number of timer ticks for 1 millisecond delay at 1 MHz

// Function to create a custom delay using TIMER0
inline void custom_delay_ms(int ms)
{
    static bool timer_initialized = false; // Flag to indicate if TIMER0 has been initialized
    if (!timer_initialized)
    {
        // Configure TIMER0 for 32-bit timer mode with 1 MHz frequency
        NRF_TIMER0->MODE = TIMER_MODE_MODE_Timer;          // Set TIMER0 to Timer mode
        NRF_TIMER0->BITMODE = TIMER_BITMODE_BITMODE_32Bit; // Use 32-bit timer mode
        NRF_TIMER0->PRESCALER = TIMER_PRESCALER; // Set prescaler to achieve 1 MHz frequency
        NRF_TIMER0->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Msk; // Clear timer on COMPARE[0] event
        timer_initialized = true;                             // Mark TIMER0 as initialized
    }

    NRF_TIMER0->TASKS_STOP = 1;               // Stop TIMER0
    NRF_TIMER0->TASKS_CLEAR = 1;              // Clear TIMER0
    NRF_TIMER0->CC[0] = ms * TIMER_TICKS_1MS; // Set compare value for desired delay
    NRF_TIMER0->EVENTS_COMPARE[0] = 0;        // Clear compare event
    NRF_TIMER0->TASKS_START = 1;              // Start TIMER0

    while (!NRF_TIMER0->EVENTS_COMPARE[0])
        ; // Wait until compare event occurs (delay completed)

    NRF_TIMER0->TASKS_STOP = 1; // Stop TIMER0 after delay
}

// Function to initialize GPIO pins for LEDs, buttons, and touch input
void initializeGPIO(void)
{
    if (gpio_initialized)
        return;              // If GPIO already initialized, do nothing
    gpio_initialized = true; // Mark GPIO as initialized

    // Configure LED row pin as output and set it low
    NRF_GPIO->DIRSET = (1 << LED_ROW1_PIN); // Set direction to output
    NRF_GPIO->OUTCLR = (1 << LED_ROW1_PIN); // Set output low

    // Configure LED column pins (on Port 0) as outputs and set them low
    NRF_GPIO->DIRSET =
        LED_COL_PIN_0_MASK | LED_COL_PIN_1_MASK | LED_COL_PIN_2_MASK | LED_COL_PIN_4_MASK;
    NRF_GPIO->OUTCLR =
        LED_COL_PIN_0_MASK | LED_COL_PIN_1_MASK | LED_COL_PIN_2_MASK | LED_COL_PIN_4_MASK;

    // Configure LED column pin on Port 1 as output and set it low
    NRF_P1->DIRSET = (1 << LED_COL_PIN_3); // Set direction to output on Port 1
    NRF_P1->OUTCLR = (1 << LED_COL_PIN_3); // Set output low on Port 1

    // Configure Buttons as inputs with pull-up resistors
    configureGPIO(BUTTON_A_PIN, (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
                                    (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos));
    configureGPIO(BUTTON_B_PIN, (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
                                    (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos));

    // **Updated**: Configure Touch Input on Port 1 Pin 4 as input with no pull resistor
    NRF_P1->PIN_CNF[TOUCH_INPUT_PIN] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
                                       (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
                                       (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos);
}

/****************** SUBTASK 1: Display Binary on LEDs ******************/

// Function to update LEDs to display a 5-bit binary value
void updateLEDs(uint8_t value)
{
    initializeGPIO(); // Ensure GPIO is initialized

    NRF_GPIO->OUTSET = (1 << LED_ROW1_PIN); // Activate the LED row by setting the row pin high

    // Prepare masks for Port 0 LED columns (columns 0, 1, 2, 4)
    // LEDs are active low, so we need to invert the bits
    uint32_t port0MaskSet =
        ((~value & 0x01) << LED_COL_PIN_0) | // If bit is 0, set the column pin high (LED off)
        ((~(value >> 1) & 0x01) << LED_COL_PIN_1) | ((~(value >> 2) & 0x01) << LED_COL_PIN_2) |
        ((~(value >> 4) & 0x01) << LED_COL_PIN_4); // Bit 4 corresponds to column 4

    uint32_t port0MaskClear =
        ((value & 0x01) << LED_COL_PIN_0) | // If bit is 1, clear the column pin (LED on)
        (((value >> 1) & 0x01) << LED_COL_PIN_1) | (((value >> 2) & 0x01) << LED_COL_PIN_2) |
        (((value >> 4) & 0x01) << LED_COL_PIN_4);

    // Prepare masks for Port 1 LED column (column 3)
    uint32_t port1MaskSet = (~(value >> 3) & 0x01) << LED_COL_PIN_3; // For bit 3
    uint32_t port1MaskClear = ((value >> 3) & 0x01) << LED_COL_PIN_3;

    // Apply the masks to set or clear the pins
    NRF_GPIO->OUTSET = port0MaskSet;   // Set pins high (LEDs off)
    NRF_GPIO->OUTCLR = port0MaskClear; // Set pins low (LEDs on)
    NRF_P1->OUTSET = port1MaskSet;     // For Port 1 (column 3)
    NRF_P1->OUTCLR = port1MaskClear;
}

// Function to display a binary value on the LEDs indefinitely
void displayBinary(uint8_t value)
{
    updateLEDs(value); // Update LEDs with the value
    while (true)
        ; // Infinite loop to hold the display
}

// Function to count up from an initial value, displaying binary on LEDs
void countUpBinary(uint8_t initialValue)
{
    uint8_t value = initialValue; // Initialize value
    while (true)
    {
        updateLEDs(value);                  // Update LEDs with current value
        value = (value + 1) & 0x1F;         // Increment value and wrap around at 5 bits (0-31)
        custom_delay_ms(COUNTING_DELAY_MS); // Delay between counts
    }
}

/****************** SUBTASK 2: Button Controlled Counting ******************/

// Function to count up or down using buttons A and B
void countWithButtonsBinary(uint8_t initialValue)
{
    uint8_t currentValue = initialValue;                   // Current value to display
    bool buttonA_pressed = false, buttonB_pressed = false; // Flags to track button press states

    while (true)
    {
        updateLEDs(currentValue); // Update LEDs with current value

        // Check Button A for decrement
        if (!(NRF_GPIO->IN & (1 << BUTTON_A_PIN)) &&
            !buttonA_pressed) // Button A is pressed (active low)
        {
            custom_delay_ms(DEBOUNCE_DELAY_MS);        // Debounce delay
            if (!(NRF_GPIO->IN & (1 << BUTTON_A_PIN))) // Confirm button is still pressed
            {
                buttonA_pressed = true; // Set button A pressed flag
                currentValue =
                    (currentValue - 1) & 0x1F; // Decrement value and wrap around at 5 bits
            }
        }
        else if (NRF_GPIO->IN & (1 << BUTTON_A_PIN)) // Button A is released
        {
            buttonA_pressed = false; // Reset button A pressed flag
        }

        // Check Button B for increment
        if (!(NRF_GPIO->IN & (1 << BUTTON_B_PIN)) && !buttonB_pressed) // Button B is pressed
        {
            custom_delay_ms(DEBOUNCE_DELAY_MS);        // Debounce delay
            if (!(NRF_GPIO->IN & (1 << BUTTON_B_PIN))) // Confirm button is still pressed
            {
                buttonB_pressed = true; // Set button B pressed flag
                currentValue =
                    (currentValue + 1) & 0x1F; // Increment value and wrap around at 5 bits
            }
        }
        else if (NRF_GPIO->IN & (1 << BUTTON_B_PIN)) // Button B is released
        {
            buttonB_pressed = false; // Reset button B pressed flag
        }

        custom_delay_ms(10); // Short delay to prevent rapid looping
    }
}

/****************** SUBTASK 3: Measure and Display Analog Voltage ******************/

// Function to initialize the SAADC (Successive Approximation Analog-to-Digital Converter)
void initSAADC(void)
{
    static bool init = false; // Flag to check if SAADC has been initialized
    if (!init)
    {
        NRF_SAADC->RESOLUTION = SAADC_RESOLUTION_VAL_8bit; // Set ADC resolution to 8 bits
        NRF_SAADC->CH[0].CONFIG =
            (SAADC_CH_CONFIG_GAIN_Gain1_6 << SAADC_CH_CONFIG_GAIN_Pos) | // Gain of 1/6
            (SAADC_CH_CONFIG_MODE_SE << SAADC_CH_CONFIG_MODE_Pos) |      // Single-ended mode
            (SAADC_CH_CONFIG_REFSEL_Internal
             << SAADC_CH_CONFIG_REFSEL_Pos) |                        // Internal reference voltage
            (SAADC_CH_CONFIG_TACQ_10us << SAADC_CH_CONFIG_TACQ_Pos); // Acquisition time of 10 us
        NRF_SAADC->CH[0].PSELP = SAADC_CH_PSELP_PSELP_AnalogInput0; // Select analog input 0 (P0.02)
        NRF_SAADC->ENABLE = SAADC_ENABLE_ENABLE_Enabled;            // Enable SAADC
        init = true;                                                // Mark SAADC as initialized
    }
}

// Function to sample voltage from the analog input and return an 8-bit value
inline uint8_t sampleVoltage(void)
{
    initSAADC();                                // Ensure SAADC is initialized
    int16_t adc_buf = 0;                        // Buffer to store ADC result
    NRF_SAADC->RESULT.PTR = (uint32_t)&adc_buf; // Set pointer to result buffer
    NRF_SAADC->RESULT.MAXCNT = 1;               // Set number of samples to take

    NRF_SAADC->TASKS_START = 1; // Start SAADC
    while (!NRF_SAADC->EVENTS_STARTED)
        ;                          // Wait until SAADC has started
    NRF_SAADC->EVENTS_STARTED = 0; // Clear started event

    NRF_SAADC->TASKS_SAMPLE = 1; // Trigger sample task
    while (!NRF_SAADC->EVENTS_END)
        ;                      // Wait until sampling is complete
    NRF_SAADC->EVENTS_END = 0; // Clear end event

    NRF_SAADC->TASKS_STOP = 1; // Stop SAADC
    while (!NRF_SAADC->EVENTS_STOPPED)
        ;                          // Wait until SAADC has stopped
    NRF_SAADC->EVENTS_STOPPED = 0; // Clear stopped event

    return (uint8_t)adc_buf; // Return the 8-bit ADC result
}

// Function to continuously sample voltage and display it as binary on LEDs
void displayVoltageBinary(void)
{
    while (true)
    {
        uint8_t voltage = (sampleVoltage() >> 3) & 0x1F; // Get top 5 bits of ADC result
        updateLEDs(voltage);                             // Display voltage value on LEDs
        custom_delay_ms(COUNTING_DELAY_MS);              // Delay between updates
    }
}

/****************** SUBTASK 4: Drive RGB LED ******************/

// Function to drive the RGB LED, changing colors based on analog input
void driveRGB(void)
{
    initializeGPIO(); // Ensure GPIO is initialized

    // Configure RGB LED pins as outputs
    NRF_GPIO->DIRSET = (1 << RED_PIN) | (1 << GREEN_PIN) | (1 << BLUE_PIN);

    uint8_t brightness = 0; // Variable to control brightness
    bool increasing = true; // Flag to indicate if brightness is increasing

    while (true)
    {
        // Adjust brightness level
        brightness += increasing ? 1 : -1; // Increase or decrease brightness
        if (brightness == 100)             // If maximum brightness reached
            increasing = false;            // Start decreasing brightness
        else if (brightness == 0)          // If minimum brightness reached
            increasing = true;             // Start increasing brightness

        // Sample analog value from input
        uint8_t analogValue = sampleVoltage(); // Read analog input

        // Calculate intensity for each color based on analog value and brightness
        uint8_t red_intensity = (analogValue * brightness) / 255;           // Red intensity
        uint8_t green_intensity = ((255 - analogValue) * brightness) / 255; // Green intensity
        uint8_t blue_intensity = ((analogValue / 2) * brightness) / 255;    // Blue intensity

        // Software PWM loop to control the brightness of each color
        for (int i = 0; i < 100; i++) // 100 levels of brightness
        {
            // Turn on LEDs based on intensity levels
            NRF_GPIO->OUTCLR = ((i < red_intensity) ? (1 << RED_PIN) : 0) |
                               ((i < green_intensity) ? (1 << GREEN_PIN) : 0) |
                               ((i < blue_intensity) ? (1 << BLUE_PIN) : 0);

            // Turn off LEDs if intensity level is reached
            NRF_GPIO->OUTSET = ((i >= red_intensity) ? (1 << RED_PIN) : 0) |
                               ((i >= green_intensity) ? (1 << GREEN_PIN) : 0) |
                               ((i >= blue_intensity) ? (1 << BLUE_PIN) : 0);

            custom_delay_ms(1); // Short delay for PWM timing
        }
    }
}

/****************** SUBTASK 5: Touch Input Controlled Counter ******************/

/**
 * @brief Touch Input Controlled Counter with Long Press Reset
 *
 * @param initialValue The initial value to start counting from.
 */


// Forward declarations (ensure these are defined elsewhere in your code)


// Assuming NRF_P1 is correctly defined for your microcontroller
// For example, it might be something like:
// #define NRF_P1 ((NRF_GPIO_Type *) 0x50000000) // Replace with actual address

void countWithTouchesBinary(uint8_t initialValue)
{
    initializeGPIO();

    // Ensure initialValue is within 0-31 to fit 5 bits
    uint8_t count = initialValue & 0x1F;
    updateLEDs(count);

    bool wasTouched = false;

    while (1)
    {
        // Read the input register and check if the touch input pin is active
        // Assuming active low (0 means touched)
        bool isTouched = ((NRF_P1->IN & (1U << TOUCH_INPUT_PIN)) == 0U);

        // Detect rising edge: touch detected now, but wasn't in the previous check
        if (isTouched && !wasTouched)
        {
            custom_delay_ms(10); // Debounce delay

            if (isTouched)
            {
                // Increment count with wrap-around at 5 bits (0-31)
                count = (count + 1) & 0x1F;
                updateLEDs(count); // Update LED display with the new count

                custom_delay_ms(COUNTING_DELAY_MS); // Prevent rapid re-triggering
            }
        }

        // Update the previous touch state
        wasTouched = isTouched;
    }
}
