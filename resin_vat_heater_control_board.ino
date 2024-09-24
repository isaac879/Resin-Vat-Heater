/*--------------------------------------------------------------------------------------------------------------------------------------------------------
 * The code is developed for a resin vat heater control board based on an ESP32-C3 (PCB Version: V1.0) 
 * 
 * All measurements are in SI units unless otherwise specified.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS 
 * IN THE SOFTWARE
 * 
 * Code written by isaac879
 * 
 * Compiled in the Arduino IDE 2.3.2
 *  
 * Last modified: 15/09/2024
 *
 * Copyright(C) Isaac Chaseau (isaac879)
 *
 *--------------------------------------------------------------------------------------------------------------------------------------------------------*/

//Libraries
#define ENABLE_GxEPD2_GFX 0

#include <GxEPD2_BW.h>
#include <GxEPD2_3C.h>
#include <Fonts/FreeMonoBold9pt7b.h>
#include <Fonts/FreeMonoBold12pt7b.h>
#include <Fonts/FreeMonoBold18pt7b.h>
#include <Fonts/FreeMono18pt7b.h>
// #include "bitmaps/Bitmaps200x200.h"
/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/
// ESP32-C3 CS(SS)=7,SCL(SCK)=4,SDA(MOSI)=6,BUSY=3,RES(RST)=2,DC=1
//Pin declarations
#define PIN_THERMISTOR_HEATER 1//HEATER_TEMP
#define PIN_THERMISTOR_PROBE 0//PROBE_TEMP
#define PIN_INPUT_VOLTAGE 3//VIN

// #define PIN_OLED_SDA 8
// #define PIN_OLED_SCL 9

#define PIN_ENC_BTN 5
#define PIN_ENC1 21//ENC1/TXD
#define PIN_ENC2 20//ENC2/RXD

#define PIN_RES 7//2
#define PIN_SCK 4 //should be 6?
#define PIN_MOSI 6
#define PIN_CS 2//7
#define PIN_BUSY 8//BUSY/SDA
#define PIN_DC 9//DC/SCL/BOOT

#define PIN_HEAT_MOSFET 10

#define PIN_SERIAL_RX 19
#define PIN_SERIAL_TX 18

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

//Constants
// #define Serial Serial
#define BAUD_RATE 115200

#define LEDC_CHANNEL 1
#define LEDC_FREQUENCY 25000
#define LEDC_BIT_RESOLUTION 8
#define LEDC_MAX_DUTY (pow(2, LEDC_BIT_RESOLUTION) - 1)

#define SPI_FREQUENCY 4000000 //4MHz
#define BUFFER_SIZE 10
#define ADC_SAMPLES 5

#define VIN_R1 2900.0 //2.7kOhm resistor (Measured at 2.9kOhm)
#define VIN_R2 22000.0 //22kOhm

#define RESISTOR_HEATER 10000.0 //10kOhm
#define RESISTOR_PROBE 10000.0 //10kOhm

#define TEMPERATURE_NOMINAL_PLATE 25.0 
#define THERMISTOR_NOMINAL_PLATE 10000 //10k
#define THERMISTOR_B_COEFFICIENT_PLATE 3950

#define TEMPERATURE_NOMINAL_PROBE 25.0
#define THERMISTOR_NOMINAL_PROBE 10000 //10k
#define THERMISTOR_B_COEFFICIENT_PROBE 3950

#define ADC_THERMISTOR_CONNECTED_THRESHOLD (MAX_ADC_VALUE - 1) //If NTC thermistor is not connected the ADC should be pulled up by the resistor giving a value of 1023 (1022 ued to account for noise).

#define DEFAULT_TARGET_TEMPERATURE_ELEMENT 35.0
#define MAX_POWER_LIMIT 15.0 //Watts //5 for testing 30 for using.
// #define ELEMENT_RESISTANCE 2.5 //Measured the value in ohms
#define ELEMENT_RESISTANCE 1.3 //Measured the value in ohms
#define MAX_ELEMENT_TEMPERATURE 60.0
#define MAX_CURRENT 2.0 //2A

#define INPUT_VOLTAGE_MIN 7
#define INPUT_VOLTAGE_MAX 30

// #define SCREEN_WIDTH 128 // OLED display width, in pixels
// #define SCREEN_HEIGHT 64 // OLED display height, in pixels
// #define OLED_RESET -1  

#define MAX_ADC_VALUE 4095.0 //ESP32-C3 has 12bit ADCs

#define MAX_MINUTE_TIMEOUT (24 * 60) //Max of a 1 day timeout

#define MENU_TARGET_ELEMENT_TEMPERATURE 0
#define MENU_TIMEOUT 1

#define DEBOUNCE_DELAY 2 //Debounce delay in ms (Bounce measured as ~20us)

#define DISPLAY_ROTATION 1

#define LOGO_WIDTH 100 //logo width in px
#define LOGO_HEIGHT 100 //logo height in px

#define TEXT_HEIGHT 24
#define LINE_SPACE (TEXT_HEIGHT + 10) + (TEXT_HEIGHT + 8) //Line spacing for the display
#define EINK_LINE_0 (0 * LINE_SPACE)
#define EINK_LINE_1 (1 * LINE_SPACE)
#define EINK_LINE_2 (2 * LINE_SPACE)
#define EINK_LINE_3 (3 * LINE_SPACE)
#define EINK_LINE_4 (4 * LINE_SPACE)
#define EINK_LINE_5 (5 * LINE_SPACE)
#define EINK_LINE_6 (6 * LINE_SPACE)
#define EINK_LINE_7 (7 * LINE_SPACE)
#define EINK_LINE_8 (8 * LINE_SPACE)
#define EINK_LINE_9 (9 * LINE_SPACE)

// #define ERROR_CODE_NO_ERRORS 0
// #define ERROR_CODE_POWER 1
// #define ERROR_CODE_THERMISTOR_0 2
// #define ERROR_CODE_THERMISTOR_1 3
// #define ERROR_CODE_OVERTEMPERATURE 4
// #define ERROR_CODE_REATCHING_TEMPERATURE 5

//Menu value determines priority (Errors should be highest)
#define MENU_SCREEN_START 0
#define MENU_SCREEN_SPLASH 1
#define MENU_SCREEN_STATUS 2
#define MENU_SCREEN_TIMEOUT 3
#define MENU_SCREEN_ERROR 4
#define MENU_SCREEN_POWER_ERROR 5
#define MENU_SCREEN_OVERTEMPERATURE_ERROR 6
#define MENU_SCREEN_THERMISTOR_0_ERROR 7
#define MENU_SCREEN_THERMISTOR_1_ERROR 8
#define MENU_SCREEN_REATCHING_TEMPERATURE_ERROR 9

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

//Global scope variables
volatile uint16_t element_temp_buffer[BUFFER_SIZE];
volatile uint16_t probe_temp_buffer[BUFFER_SIZE];
volatile uint16_t voltage_buffer[BUFFER_SIZE];

volatile uint16_t adc_thermistor_element = 0;
volatile uint16_t adc_thermistor_probe = 0;
volatile uint16_t adc_voltage = 0;

volatile double resistance_thermistor_element = 0;
volatile double resistance_thermistor_probe = 0;

volatile double temp_steinhart_thermistor_element = 0;
volatile double temp_steinhart_thermistor_probe = 0;

volatile double temp_target_element = 35.0;
volatile double temp_target_probe = 0.0;

volatile double maximum_power_limit = MAX_POWER_LIMIT; //Max power in Watts
volatile double maximum_current_limit = MAX_CURRENT; //Max power in amps

volatile double input_voltage = 0;
volatile double minute_timeout = 60 * 8; //Time in minutes before the heating element is turned off.

volatile int16_t encoderValue = 0;  //Variable to store encoder value
volatile bool encoderDir = true; //Variable to store direction (true: clockwise, false: counterclockwise)
volatile int16_t encoder_button_count = 0;
volatile int16_t menu_item_selection = MENU_TIMEOUT;

volatile uint32_t ms_last_debounce_time_encoder = 0;
volatile uint32_t ms_last_debounce_time_button = 0;

const char splash_screen_text_0[] = "Vat Heater";
const char splash_screen_text_2[] = "isaac879";
const char splash_screen_text_1[] = "Ver: 1.0.0";

volatile int64_t msTimeoutStart = 0;
volatile int64_t msTimeoutEnd = 0;

bool full_screen_update = false;
volatile bool require_screen_update = true;

double last_displayed_thermistor0 = 0;
double last_displayed_thermistor1 = 0;
double last_displayed_target_temp = 0;
int16_t last_displayed_timer = 0;
int16_t last_displayed_screen = 0;

volatile int16_t display_screen = MENU_SCREEN_SPLASH;
const uint8_t drops_symbol_100x100[] = {
   0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0xff, 0x01, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0xff, 0xff, 0x3f, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0x00, 0x00, 0xf0, 0x03, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x0f, 0x00, 0x00, 0x00, 0x1f,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x01, 0x00, 0x00, 0x00,
   0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0x00, 0x00, 0x00,
   0x00, 0xe0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x00,
   0x00, 0x00, 0x80, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x01,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x01, 0x00, 0x00, 0x00,
   0x00, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x03, 0x00, 0x00,
   0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00,
   0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c,
   0x00, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x18, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x30, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0xc0, 0x00, 0x00,
   0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0xc0, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x0c, 0x00, 0x00, 0xc0,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x0c, 0x00, 0x00,
   0xe0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x06, 0x00,
   0x00, 0xe0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x03,
   0x00, 0x00, 0xf0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00,
   0x01, 0x00, 0x00, 0xf0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00,
   0x80, 0x01, 0x00, 0x00, 0xf8, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18,
   0x00, 0xc0, 0x00, 0x00, 0x00, 0xfc, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x30, 0x00, 0xc0, 0x00, 0x00, 0x00, 0xfe, 0x0f, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x30, 0x00, 0x60, 0x00, 0x00, 0x00, 0xfe, 0x1f, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x60, 0x00, 0x60, 0x00, 0x00, 0x00, 0xff, 0x3f, 0x00, 0x00,
   0x06, 0x00, 0x00, 0x60, 0x00, 0x30, 0x00, 0x00, 0x80, 0xff, 0x7f, 0x00,
   0x00, 0x06, 0x00, 0x00, 0xc0, 0x00, 0x30, 0x00, 0x00, 0x80, 0xff, 0x7f,
   0x00, 0x00, 0x06, 0x00, 0x00, 0xc0, 0x00, 0x18, 0x00, 0x00, 0xc0, 0xff,
   0xff, 0x00, 0x00, 0x06, 0x00, 0x00, 0x80, 0x01, 0x18, 0x00, 0x00, 0xc0,
   0xff, 0xff, 0x00, 0x00, 0x06, 0x00, 0x00, 0x80, 0x01, 0x08, 0x00, 0x00,
   0xe0, 0xff, 0xff, 0x01, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x01, 0x0c, 0x00,
   0x00, 0xe0, 0xff, 0xff, 0x01, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x03, 0x0c,
   0x00, 0x00, 0xe0, 0xff, 0xff, 0x01, 0x80, 0x1f, 0x00, 0x00, 0x00, 0x03,
   0x04, 0x00, 0x00, 0xe0, 0xff, 0xff, 0x01, 0x80, 0x1f, 0x00, 0x00, 0x00,
   0x02, 0x04, 0x00, 0x00, 0xe0, 0xff, 0xff, 0x01, 0xc0, 0x3f, 0x00, 0x00,
   0x00, 0x02, 0x06, 0x00, 0x00, 0xe0, 0xff, 0xff, 0x01, 0xe0, 0x7f, 0x00,
   0x00, 0x00, 0x06, 0x06, 0x00, 0x00, 0xc0, 0xff, 0xff, 0x00, 0xf0, 0x7f,
   0x00, 0x00, 0x00, 0x06, 0x02, 0x00, 0x00, 0xc0, 0xff, 0xff, 0x00, 0xf0,
   0xff, 0x00, 0x00, 0x00, 0x04, 0x02, 0x00, 0x00, 0x80, 0xff, 0x7f, 0x00,
   0xf8, 0xff, 0x01, 0x00, 0x00, 0x04, 0x02, 0x00, 0x00, 0x00, 0xff, 0x3f,
   0x00, 0xfc, 0xff, 0x03, 0x00, 0x00, 0x04, 0x03, 0x00, 0x00, 0x00, 0xfe,
   0x1f, 0x00, 0xfc, 0xff, 0x03, 0x00, 0x00, 0x0c, 0x03, 0x00, 0x00, 0x00,
   0xfc, 0x0f, 0x00, 0xfe, 0xff, 0x07, 0x00, 0x00, 0x0c, 0x03, 0x00, 0x00,
   0x00, 0xe0, 0x01, 0x00, 0xfe, 0xff, 0x07, 0x00, 0x00, 0x0c, 0x03, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x0f, 0x00, 0x00, 0x0c, 0x03,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x0f, 0x00, 0x00, 0x0c,
   0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x0f, 0x00, 0x00,
   0x0c, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x0f, 0x00,
   0x00, 0x0c, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x0f,
   0x00, 0x00, 0x0c, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff,
   0x0f, 0x00, 0x00, 0x0c, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe,
   0xff, 0x07, 0x00, 0x00, 0x0c, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xfe, 0xff, 0x07, 0x00, 0x00, 0x0c, 0x03, 0x00, 0x00, 0x00, 0x00, 0xc0,
   0x00, 0xfc, 0xff, 0x03, 0x00, 0x00, 0x0c, 0x03, 0x00, 0x00, 0x00, 0x00,
   0xc0, 0x00, 0xf8, 0xff, 0x01, 0x00, 0x00, 0x0c, 0x03, 0x00, 0x00, 0x00,
   0x00, 0xc0, 0x00, 0xf0, 0xff, 0x00, 0x00, 0x00, 0x0c, 0x02, 0x00, 0x00,
   0x00, 0x00, 0xc0, 0x00, 0xe0, 0x7f, 0x00, 0x00, 0x00, 0x04, 0x02, 0x00,
   0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x04, 0x02,
   0x00, 0x00, 0x00, 0x00, 0xe0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04,
   0x06, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x06, 0x06, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x03, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x06, 0x04, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x03, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x07, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x02, 0x0c, 0x00, 0x00, 0x00, 0x00, 0xfc, 0x0f, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x03, 0x0c, 0x00, 0x00, 0x00, 0x00, 0xfe, 0x0f,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x08, 0x00, 0x00, 0x00, 0x00, 0xfe,
   0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x18, 0x00, 0x00, 0x00, 0x00,
   0xff, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x80, 0x01, 0x18, 0x00, 0x00, 0x00,
   0x80, 0xff, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x80, 0x01, 0x30, 0x00, 0x00,
   0x00, 0x80, 0xff, 0x7f, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x30, 0x00,
   0x00, 0x00, 0xc0, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x60,
   0x00, 0x00, 0x00, 0xc0, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00,
   0x60, 0x00, 0x00, 0x00, 0xe0, 0xff, 0xff, 0x01, 0x00, 0x00, 0x00, 0x60,
   0x00, 0xc0, 0x00, 0x00, 0x00, 0xe0, 0xff, 0xff, 0x01, 0x00, 0x00, 0x00,
   0x30, 0x00, 0xc0, 0x00, 0x00, 0x00, 0xe0, 0xff, 0xff, 0x01, 0x00, 0x00,
   0x00, 0x30, 0x00, 0x80, 0x01, 0x00, 0x00, 0xe0, 0xff, 0xff, 0x01, 0x00,
   0x00, 0x00, 0x18, 0x00, 0x00, 0x01, 0x00, 0x00, 0xe0, 0xff, 0xff, 0x01,
   0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x03, 0x00, 0x00, 0xe0, 0xff, 0xff,
   0x01, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x06, 0x00, 0x00, 0xc0, 0xff,
   0xff, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x0c, 0x00, 0x00, 0xc0,
   0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x0c, 0x00, 0x00,
   0x80, 0xff, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x18, 0x00,
   0x00, 0x00, 0xff, 0x3f, 0x00, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x30,
   0x00, 0x00, 0x00, 0xfe, 0x1f, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00,
   0x60, 0x00, 0x00, 0x00, 0xfc, 0x0f, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00,
   0x00, 0xc0, 0x00, 0x00, 0x00, 0xe0, 0x01, 0x00, 0x00, 0x00, 0x30, 0x00,
   0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18,
   0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x0c, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x80, 0x03, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0xc0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x01, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x1e, 0x00, 0x00, 0x00, 0x00, 0x80, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x78, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x01, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0xe0, 0x01, 0x00, 0x00, 0x00, 0x78, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x80, 0x0f, 0x00, 0x00, 0x00, 0x1f, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0x00, 0x00, 0xf0, 0x03, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0xff, 0xff, 0x3f, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0xff, 0x01, 0x00,
   0x00, 0x00, 0x00, 0x00 
};

hw_timer_t *timer_control_loop_ISR = NULL;

//1.54'' EPD Module
GxEPD2_BW<GxEPD2_154_D67, GxEPD2_154_D67::HEIGHT> display(GxEPD2_154_D67(PIN_CS, PIN_DC, PIN_RES, PIN_BUSY)); //GDEH0154D67 200x200, SSD1681

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

double mapNumber(double x, double in_min, double in_max, double out_min, double out_max){//Remaps a number to a given range
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void IRAM_ATTR readEncoder(void){ //Interrupt service routine (ISR) for the encoder
    uint32_t currentTime = millis(); 
    // Serial.printf("Encoder ISR\n");

    if(currentTime - ms_last_debounce_time_encoder > DEBOUNCE_DELAY){
        bool pinA = digitalRead(PIN_ENC1); //Read both encoder pins
        bool pinB = digitalRead(PIN_ENC2);
        
        if(pinA == pinB){ //Determine direction based on the states of the pins
            encoderValue++;
        }
        else{
            encoderValue--;
        }
        ms_last_debounce_time_encoder = currentTime;  //Update the last debounce time
    }
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void IRAM_ATTR readEncoderButton(void){ //Interrupt service routine (ISR) for the encoder
    uint32_t currentTime = millis(); 
    // Serial.printf("Button ISR\n");

    if(currentTime - ms_last_debounce_time_button > DEBOUNCE_DELAY){
        encoder_button_count++; //Increment the count

        if(encoder_button_count > 2){ //Wrap around the count
            encoder_button_count = 1;
        }

        if(encoder_button_count == 1){
            menu_item_selection = MENU_TARGET_ELEMENT_TEMPERATURE;
        }
        else{
            menu_item_selection = MENU_TIMEOUT;
        }
        ms_last_debounce_time_button = currentTime; //Update the last debounce time
    }
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void readADCs(void){ //Read x samples and average then store in a circular buffer
    static uint8_t index = 0;

    uint16_t adcPlate = 0;
    uint16_t adcProbe = 0;
    uint16_t adcVin = 0;

    for(uint8_t i = 0; i < ADC_SAMPLES; i++){
        adcPlate += analogRead(PIN_THERMISTOR_HEATER);
        adcProbe += analogRead(PIN_THERMISTOR_PROBE);
        adcVin += analogRead(PIN_INPUT_VOLTAGE);
    }

    element_temp_buffer[index] = adcPlate / ADC_SAMPLES;
    probe_temp_buffer[index] = adcProbe / ADC_SAMPLES;
    voltage_buffer[index] = adcVin / ADC_SAMPLES;

    index = (index + 1) % BUFFER_SIZE;

    uint16_t adcAvgPlate = 0;
    uint16_t adcAvgProbe = 0;
    uint16_t adcAvgVin = 0;

    for(uint8_t i = 0; i < BUFFER_SIZE; i++){
        adcAvgPlate += element_temp_buffer[i];
        adcAvgProbe += probe_temp_buffer[i];
        adcAvgVin += voltage_buffer[i];
    }

    adc_thermistor_element = adcAvgPlate / BUFFER_SIZE;
    adc_thermistor_probe = adcAvgProbe / BUFFER_SIZE;
    adc_voltage = adcAvgVin / BUFFER_SIZE;

    input_voltage = (3.3 * ((double)adc_voltage / MAX_ADC_VALUE)) * ((VIN_R1 + VIN_R2) / VIN_R1);

    if((input_voltage / ELEMENT_RESISTANCE) > MAX_CURRENT){ //Make sure the max current limit cannot be exceeded
        maximum_power_limit = MAX_CURRENT * input_voltage;
    }
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

double adcToResistance(uint32_t adc, uint32_t resistorVal){
    return (resistorVal * (double)adc)/ (MAX_ADC_VALUE - constrain(adc, 0.0, MAX_ADC_VALUE - 1.0)); //Constrain to avoid dividing by zero
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

double resistanceToTemperatureSteinheart(double res, double thermistorNominal, double thermistorBCoefficient, double temperatureNominal){
    double steinhart = log(res / thermistorNominal); // ln(R/Ro)
    steinhart /= thermistorBCoefficient; // 1/B * ln(R/Ro)
    steinhart += 1.0 / (temperatureNominal + 273.15); // + (1/To)
    steinhart = 1.0 / steinhart; //Invert
    return steinhart - 273.15; //Kelvin to C
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void termistorCalcs(void){
    resistance_thermistor_element = adcToResistance(adc_thermistor_element, RESISTOR_HEATER);
    resistance_thermistor_probe = adcToResistance(adc_thermistor_probe, RESISTOR_PROBE);

    temp_steinhart_thermistor_element = (elementThermistorConnected() == true) ? resistanceToTemperatureSteinheart(resistance_thermistor_element, THERMISTOR_NOMINAL_PLATE, THERMISTOR_B_COEFFICIENT_PLATE, TEMPERATURE_NOMINAL_PLATE) : -273.15;
    temp_steinhart_thermistor_probe = (probeThermistorConnected() == true) ? resistanceToTemperatureSteinheart(resistance_thermistor_probe, THERMISTOR_NOMINAL_PROBE, THERMISTOR_B_COEFFICIENT_PROBE, TEMPERATURE_NOMINAL_PROBE) : -273.15;
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

double getVoltageIn(void){
    return input_voltage;
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

double getCurrent(void){
    if(getMaximumAchievablePower() > getPowerLimit()){
        return maximum_power_limit / getVoltageIn();
    }
    else{
        return getVoltageIn() / ELEMENT_RESISTANCE;
    }
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

double getPowerLimit(void){
    return (maximum_power_limit > MAX_POWER_LIMIT) ? MAX_POWER_LIMIT : maximum_power_limit;
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

double getMaximumAchievablePower(void){
    return (input_voltage * input_voltage) / ELEMENT_RESISTANCE;
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

double getPowerUsed(void){
    if(getMaximumAchievablePower() > getPowerLimit()){
        return getPowerLimit();
    }
    else{
        return getMaximumAchievablePower();
    }
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

double getTargetTemperature(void){
    static int16_t lastEncoderValue = 0;
    int16_t encoderDelta = encoderValue - lastEncoderValue;
    if((menu_item_selection == MENU_TARGET_ELEMENT_TEMPERATURE) && (encoderDelta != 0)){
        temp_target_element += mapNumber(encoderDelta, -1, 1, -0.5, 0.5);
        temp_target_element = constrain(temp_target_element, 0, MAX_ELEMENT_TEMPERATURE);
    }
    lastEncoderValue = encoderValue;
    return temp_target_element;
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void updateTimeout(void){
    static int16_t lastEncoderValue = 0;
    int16_t encoderDelta = encoderValue - lastEncoderValue;

    if((menu_item_selection == MENU_TIMEOUT) && (encoderDelta != 0)){
        minute_timeout += mapNumber(encoderDelta, -1, 1, -10, 10);
        minute_timeout = constrain(minute_timeout, 0, MAX_MINUTE_TIMEOUT);
    }
    if(minute_timeout == 0){
        msTimeoutStart = millis();
    }

    msTimeoutEnd = msTimeoutStart + ((int64_t)minute_timeout * (int64_t)60000);
    lastEncoderValue = encoderValue;
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

double getTimeoutMinutes(void){ //Return the minutes left until the timeout is reached
    double mTimeout = (double)(msTimeoutEnd - (int64_t)millis()) / (60000.0);
    if(mTimeout <= 0){
        mTimeout = 0;
        updateMenuScreenselector(MENU_SCREEN_TIMEOUT);
    }
    return mTimeout;
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void heatingElementOn(void){
    double maxPowerDraw = (input_voltage * input_voltage) / ELEMENT_RESISTANCE; //Max possible power given coil resistance and input voltage
    uint8_t duty = (double)LEDC_MAX_DUTY;// * (maximum_power_limit / maxPowerDraw); //Duty scaled proportional to the power limit

    if(maxPowerDraw > maximum_power_limit){
        duty = (double)LEDC_MAX_DUTY * (maximum_power_limit / maxPowerDraw); //Duty scaled proportional to the power limit
    }

     ledcWrite(LEDC_CHANNEL, duty);
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void heatingElementOff(void){
    ledcWrite(LEDC_CHANNEL, 0); //Off
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

bool inputVoltageOk(void){
    if((input_voltage > INPUT_VOLTAGE_MIN) && (input_voltage < INPUT_VOLTAGE_MAX)){
        return true;
    }
    else{
        updateMenuScreenselector(MENU_SCREEN_POWER_ERROR);
        return false;
    }
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

bool elementThermistorConnected(void){
    if(adc_thermistor_element < ADC_THERMISTOR_CONNECTED_THRESHOLD){
        return true;
    }
    else{
        updateMenuScreenselector(MENU_SCREEN_THERMISTOR_0_ERROR);
        return false;
    }
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

bool probeThermistorConnected(void){
    if(adc_thermistor_probe < ADC_THERMISTOR_CONNECTED_THRESHOLD){
        return true;
    }
    else{
        // updateMenuScreenselector(MENU_SCREEN_THERMISTOR_1_ERROR);
        return false;
    }
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void IRAM_ATTR updateControlLoop(void){
    display_screen = MENU_SCREEN_STATUS; //Set the screen to be displayed as the lowest priority
    readADCs();
    termistorCalcs();
    updateTimeout();
    bool timeout = !(getTimeoutMinutes() > 0);
    if( (temp_steinhart_thermistor_element < getTargetTemperature()) && 
        (temp_steinhart_thermistor_element < MAX_ELEMENT_TEMPERATURE) && 
        (inputVoltageOk() == true) && 
        // (getTimeoutMinutes() > 0) &&
        (timeout == false) &&
        (elementThermistorConnected() == true)){ 
        heatingElementOn();
    }
    else{
        heatingElementOff();
    }
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void printValues(void){
    Serial.println(F("\n-----------------------------------------------"));
    Serial.print(F("Target Temp: "));
    Serial.println(getTargetTemperature());
    Serial.println("");

    Serial.println(F("Element:"));
    Serial.print(F("ADC: "));
    Serial.println(adc_thermistor_element);
    Serial.print(F("Resistance: "));
    Serial.println(resistance_thermistor_element);
    Serial.print(F("Temperature Steinhart: "));
    Serial.print(temp_steinhart_thermistor_element);
    Serial.println(F("°C\n"));

    Serial.println(F("Probe:"));
    Serial.print(F("ADC: "));
    Serial.println(adc_thermistor_probe);
    Serial.print(F("Resistance: "));
    Serial.println(resistance_thermistor_probe);
    Serial.print(F("Temperature Steinhart: "));
    Serial.print(temp_steinhart_thermistor_probe);
    Serial.println(F("°C\n"));

    float mt = getTimeoutMinutes();
    uint16_t hours = (uint16_t)(mt / 60);
    uint16_t minutes = mt - (hours * 60);
    Serial.print(F("Timeout: "));
    Serial.print(hours);
    Serial.print(F("h"));
    Serial.print(minutes);
    Serial.println(F("m\n"));

    Serial.println(F("Encoder:"));
    Serial.print(F("Value: "));
    Serial.println(encoderValue);
    Serial.print(F("Button: "));
    Serial.println(digitalRead(PIN_ENC_BTN));
    Serial.print(F("Button Count: "));
    Serial.println(encoder_button_count);
    Serial.println("");

    Serial.println(F("Voltage:"));
    Serial.print(F("ADC: "));
    Serial.println(adc_voltage);
    Serial.print(F("Input Voltage: "));
    Serial.print(getVoltageIn());
    Serial.println(F("V"));

    Serial.print(F("Max Achievable Power:"));
    Serial.print(getMaximumAchievablePower());
    Serial.println(F("W"));

    Serial.print(F("Power Limit:"));
    Serial.print(getPowerLimit());
    Serial.println(F("W"));

    Serial.print(F("Power Target:"));
    Serial.print(getPowerUsed());
    Serial.println(F("W"));

    Serial.print(F("Current:"));
    Serial.print(getCurrent());
    Serial.print(F("A\n"));
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void showDisplayBuffer(void){
    if(full_screen_update == true){
        display.setFullWindow();
    }
    else{
        display.setPartialWindow(0, 0, display.width(), display.height()); //Full screen size
    }
    
    // display.setFullWindow();
    // display.setPartialWindow(0, 0, display.width(), display.height()); //Full screen size
    while(display.nextPage()){};
    display.firstPage();
    display.fillScreen(GxEPD_WHITE); //Clear the buffer to all white
    display.hibernate();
    require_screen_update = false;
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/ 

void printDegreeSymbol(void){
    int16_t cursorX = display.getCursorX();
    int16_t cursorY = display.getCursorY();

    display.setFont(&FreeMonoBold9pt7b);
    display.setCursor(cursorX, cursorY - 7);
    display.print("o");
    display.setCursor(cursorX + 11, cursorY); //Restor cursor position
    display.setFont(&FreeMonoBold12pt7b); //Restor text size
    display.print("C");
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/ 

// display.drawRFrame(PROGRESS_BAR_POSITION_X, PROGRESS_BAR_POSITION_Y, PROGRESS_BAR_LENGTH, PROGRESS_BAR_HEIGHT, 2); //Draw the bar outline
// display.drawRBox(PROGRESS_BAR_POSITION_X + 2, PROGRESS_BAR_POSITION_Y + 2, barLength, PROGRESS_BAR_HEIGHT - 4, 0);
// drawPixel(x + i, y + j, color);
//  writeImage(const uint8_t bitmap[], int16_t x, int16_t y, int16_t w, int16_t h, bool invert = false, bool mirror_y = false, bool pgm = false)
void printTextCentredX(const char* text, int16_t lineHeight){
    // int16_t tbx, tby; 
    int16_t textBoxX = 0;
    int16_t textBoxY = 0;
    // uint16_t tbw, tbh;
    uint16_t textBoxWidth = 0;;
    uint16_t textBoxHeight = 0;
    display.getTextBounds(text, 0, 0, &textBoxX, &textBoxY, &textBoxWidth, &textBoxHeight);
    uint16_t xCursorStart = ((display.width() - textBoxWidth) / 2) - textBoxX;
    display.setCursor(xCursorStart, lineHeight);
    display.print(text);
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/ 

void splashScreen(void){ //Displayed on boot
    printTextCentredX(splash_screen_text_0, EINK_LINE_0); //Vat heater
    printTextCentredX(splash_screen_text_2, EINK_LINE_4); //isaac879
    printTextCentredX(splash_screen_text_1, EINK_LINE_5 - 8); //Version

    display.drawXBitmap((display.width()  - LOGO_WIDTH) / 2, (display.height() - LOGO_HEIGHT) / 2, drops_symbol_100x100, LOGO_WIDTH, LOGO_HEIGHT, GxEPD_BLACK);
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/ 

void startScreen(void){
    printTextCentredX("SET TIMER", EINK_LINE_0); 

    uint16_t hours = (uint16_t)(minute_timeout / 60);
    uint16_t minutes = minute_timeout - (hours * 60);
    display.setCursor(0, EINK_LINE_2);
    display.print("Timer: ");

    char textBuffer[10]; //Max 10 charactars
    sprintf(textBuffer, "%uh%um", hours, minutes);
    int16_t textBoxX = 0;
    int16_t textBoxY = 0;
    uint16_t textBoxWidth = 0;;
    uint16_t textBoxHeight = 0;
    display.getTextBounds(textBuffer, 0, 0, &textBoxX, &textBoxY, &textBoxWidth, &textBoxHeight);
    display.drawRect(display.getCursorX(), EINK_LINE_2 + 4, textBoxWidth + 4, 2, GxEPD_BLACK);
    display.print(textBuffer);

    // display.print(hours);
    // display.print("h");
    // display.print(minutes);
    // display.print("m");

    display.setCursor(0, EINK_LINE_3);
    display.print("Target:");
    display.print(DEFAULT_TARGET_TEMPERATURE_ELEMENT, 1);
    printDegreeSymbol();
    // display.setCursor(0, EINK_LINE_3);
    // display.print("DEBUG: ");
    // display.print(millis());

    display.setCursor(0, EINK_LINE_4);
    display.print("Press To Start");
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/ 

void reachedTimeoutScreen(void){
    printTextCentredX("Heater Off", EINK_LINE_0);
    printTextCentredX("Timer Finished", EINK_LINE_2);
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/ 

void errorScreen(const char* errorMessage){
    printTextCentredX("ERROR", EINK_LINE_0);
    printTextCentredX(errorMessage, EINK_LINE_2);
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void errorScreen(const char* errorMessageLine1, const char* errorMessageLine2){
    printTextCentredX("ERROR", EINK_LINE_0);
    printTextCentredX(errorMessageLine1, EINK_LINE_2);
    printTextCentredX(errorMessageLine2, EINK_LINE_3);
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void setDisplayBuffer(int16_t displayScreen){
    static int16_t lastScreenDisplayed = MENU_SCREEN_SPLASH;
    full_screen_update = (lastScreenDisplayed == displayScreen) ? false : true;

    last_displayed_thermistor0 = temp_steinhart_thermistor_element;
    last_displayed_thermistor1 = temp_steinhart_thermistor_probe;
    last_displayed_target_temp = getTargetTemperature();
    last_displayed_timer = (int16_t)getTimeoutMinutes();
    last_displayed_screen = displayScreen;

    display.drawRect(0, EINK_LINE_0 + 4, display.width(), 2, GxEPD_BLACK);
    // display.drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
    // display.drawRBox(x, y, display.width(), 2, 0);

    switch(displayScreen){
        case MENU_SCREEN_STATUS:{
            display.setRotation(DISPLAY_ROTATION);
            display.setFont(&FreeMonoBold12pt7b);
            display.setTextColor(GxEPD_BLACK);

            printTextCentredX("STATUS", EINK_LINE_0);

            display.setCursor(0, EINK_LINE_2);
            display.print("Target: ");

            char textBuffer[10]; //Max 10 charactars
            sprintf(textBuffer, "%.1f C", last_displayed_target_temp);
            if(menu_item_selection == MENU_TARGET_ELEMENT_TEMPERATURE){
                int16_t textBoxX = 0;
                int16_t textBoxY = 0;
                uint16_t textBoxWidth = 0;;
                uint16_t textBoxHeight = 0;
                display.getTextBounds(textBuffer, 0, 0, &textBoxX, &textBoxY, &textBoxWidth, &textBoxHeight);
                display.drawRect(display.getCursorX(), EINK_LINE_2 + 4, textBoxWidth, 2, GxEPD_BLACK);
            }

            display.print(last_displayed_target_temp, 1);
            printDegreeSymbol();

            display.setCursor(0, EINK_LINE_3);
            display.print("Vat: ");
            display.print(last_displayed_thermistor0, 1);
            printDegreeSymbol();

            display.setCursor(0, EINK_LINE_4);
            display.print("Probe: ");
            if(probeThermistorConnected() == true){
                display.print(last_displayed_thermistor1, 1);
                printDegreeSymbol();
            }
            else{
                display.print("N/A");
            }
            
            // display.setCursor(0, EINK_LINE_3);
            // display.print("Limit:");
            // display.print(MAX_POWER_LIMIT, 1); 
            // display.print("W");

            float mt = last_displayed_timer;
            uint16_t hours = (uint16_t)(mt / 60);
            uint16_t minutes = mt - (hours * 60);
            display.setCursor(0, EINK_LINE_1);
            display.print("Timer: ");
            char textBuffer1[10]; //Max 10 charactars
            sprintf(textBuffer1, "%uh%um", hours, minutes);

            if(menu_item_selection == MENU_TIMEOUT){
                int16_t textBoxX = 0;
                int16_t textBoxY = 0;
                uint16_t textBoxWidth = 0;;
                uint16_t textBoxHeight = 0;
                display.getTextBounds(textBuffer1, 0, 0, &textBoxX, &textBoxY, &textBoxWidth, &textBoxHeight);
                display.drawRect(display.getCursorX(), EINK_LINE_1 + 4, textBoxWidth + 4, 2, GxEPD_BLACK);
            }
            display.print(textBuffer1);
            // display.print(hours);
            // display.print("h");
            // display.print(minutes);
            // display.print("m");
            break;
        }
        case MENU_SCREEN_START:{
            startScreen();
            break;
        }
        case MENU_SCREEN_TIMEOUT:{
            reachedTimeoutScreen();
            break;
        }
        case MENU_SCREEN_SPLASH:{
            splashScreen();
            break;
        }
        case MENU_SCREEN_POWER_ERROR:{
            errorScreen("Undervoltage");
            break;
        }
        case MENU_SCREEN_THERMISTOR_0_ERROR:{
            errorScreen("Thermistor 0", "Not Detected");
            break;
        }
        case MENU_SCREEN_THERMISTOR_1_ERROR:{
            errorScreen("Thermistor 1", "Not Detected");
            break;
        }
        case MENU_SCREEN_OVERTEMPERATURE_ERROR:{
            errorScreen("Vat Overheating");
            break;
        }
        case MENU_SCREEN_REATCHING_TEMPERATURE_ERROR:{
            errorScreen("Not Reaching Target Temp");
            break;
        }
        // default:{
        //     return;
        // }
    }
    lastScreenDisplayed = displayScreen;
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void updateMenuScreenselector(int16_t screen){
    if(display_screen < screen){ //Only update if it has a higher priority
        display_screen = screen;
    }
}


/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void einkInit(void){
    SPI.setFrequency(SPI_FREQUENCY);
    SPI.begin(PIN_SCK, -1, PIN_MOSI, PIN_CS); //Use the default SPI hardware assigned to the secified GPIOs
    display.init(0, true, 50, false); //Initialize the display
    display.setRotation(DISPLAY_ROTATION);
    display.setFont(&FreeMonoBold12pt7b);
    display.setTextColor(GxEPD_BLACK);
    display.setFullWindow();
    display.firstPage();
    display.fillScreen(GxEPD_WHITE);

    // splashScreen(); //Display the splash screen on boot
    // display.hibernate();
    // delay(5000);
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void setup(){
    Serial.begin(BAUD_RATE);
    Serial.print(F("Setup... "));
    pinMode(PIN_THERMISTOR_HEATER, INPUT);
    pinMode(PIN_THERMISTOR_PROBE, INPUT);
    pinMode(PIN_INPUT_VOLTAGE, INPUT);

    pinMode(PIN_HEAT_MOSFET, OUTPUT);
    digitalWrite(PIN_HEAT_MOSFET, LOW);
    ledcSetup(LEDC_CHANNEL, LEDC_FREQUENCY, LEDC_BIT_RESOLUTION);
    ledcAttachPin(PIN_HEAT_MOSFET, LEDC_CHANNEL);

    pinMode(PIN_ENC1, INPUT_PULLUP); //Rotary Encoder A/1 input
    pinMode(PIN_ENC2, INPUT_PULLUP); //Rotary Encoder B/2 input
    pinMode(PIN_ENC_BTN, INPUT_PULLUP); //Rotary Encoder button input

    attachInterrupt(digitalPinToInterrupt(PIN_ENC1), readEncoder, CHANGE); //Attach the encoder interrupt to pin A/1
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_BTN), readEncoderButton, FALLING); //Attach the encoder button interrupt
      
    delay(50); //Time for power to stabilize
    for(uint8_t i = 0; i < BUFFER_SIZE; i++){ //Fill the potentiometer buffer with readings
        readADCs();
        delay(10);
    }

    termistorCalcs();

    Serial.print(F("Max Power Limit: "));
    Serial.println(getPowerLimit());

    Serial.print(F("Achievable Power: "));
    Serial.println(getMaximumAchievablePower());

    Serial.print(F("Plate Thermistor Connected: "));
    Serial.println(elementThermistorConnected());

    Serial.print(F("Probe Thermistor Connected: "));
    Serial.println(probeThermistorConnected());

    Serial.println(F("Complete"));    

    einkInit();
    setDisplayBuffer(MENU_SCREEN_SPLASH);
    showDisplayBuffer();
    delay(3000);

    //Initialize the control loop callback
    timer_control_loop_ISR = timerBegin(0, 80, true); //Timer 0, Prescaler 80 (1 us per tick), auto-reload
    timerAttachInterrupt(timer_control_loop_ISR, &updateControlLoop, true); //Attach the updateControlLoop function to the timer as an ISR
    timerAlarmWrite(timer_control_loop_ISR, 10000, true); //Set timer alarm to trigger every 10ms (10,000us)
    timerAlarmEnable(timer_control_loop_ISR); //Enable the timer alarm

    display_screen = MENU_SCREEN_START;

    temp_target_element = 0;
    while(encoder_button_count == 0){
        static uint32_t prevTime = 0;
        uint32_t msTime = millis();

        if(msTime - prevTime > 700){ //Update every 700ms. Update cycle takes ~650ms
            prevTime = msTime;
            // displayEINK200x200PartialUpdate();
            // setDisplayBuffer(MENU_SCREEN_START); //Force the start screen to show
            // showDisplayBuffer();
            if( 
                // (last_displayed_thermistor0 != temp_steinhart_thermistor_element) ||
                // (last_displayed_thermistor1 != temp_steinhart_thermistor_probe) ||
                // (last_displayed_target_temp != getTargetTemperature()) ||
                (last_displayed_timer != (int16_t)getTimeoutMinutes())// ||
                // (last_displayed_screen != display_screen)
                ){
                require_screen_update = true;
            }

            if(require_screen_update == true){ //Only update the screen if data has changed
                setDisplayBuffer(MENU_SCREEN_START); //Force the start screen to show
                showDisplayBuffer();
            }
        }
    }
    temp_target_element = DEFAULT_TARGET_TEMPERATURE_ELEMENT;
    //Initialize the timeout
    uint32_t msNow = millis();
    msTimeoutStart = msNow;
    msTimeoutEnd = msNow + (minute_timeout * 60000L);
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void loop(){
    static uint32_t prevTime = 0;
    static uint32_t prevTime1 = 0;
    uint32_t msTime = millis();

    if(msTime - prevTime > 700){ //Update every 700ms. Update cycle takes ~650ms
        prevTime = msTime;
        // displayEINK200x200PartialUpdate();
        if( (last_displayed_thermistor0 != temp_steinhart_thermistor_element) ||
            (last_displayed_thermistor1 != temp_steinhart_thermistor_probe) ||
            (last_displayed_target_temp != getTargetTemperature()) ||
            (last_displayed_timer != (int16_t)getTimeoutMinutes()) ||
            (last_displayed_screen != display_screen)){
            // Serial.printf("Lt0: %.3f\t t0: %.3f\n", last_displayed_thermistor0, temp_steinhart_thermistor_element);
            // Serial.printf("Lt1: %.3f\t t1: %.3f\n", last_displayed_thermistor1, temp_steinhart_thermistor_probe);
            // Serial.printf("Ltt: %.3f\t tt: %.3f\n", last_displayed_target_temp, getTargetTemperature());
            // Serial.printf("LTime: %d\t Time: %d\n", last_displayed_timer, (int16_t)getTimeoutMinutes());
            // Serial.printf("LS: %d\t s: %d\n", last_displayed_screen, display_screen);

            require_screen_update = true;
        }

        if(require_screen_update == true){ //Only update the screen if data has changed
            setDisplayBuffer(display_screen);
            showDisplayBuffer();
        }
    }

    if(msTime - prevTime1 > 250){ //Print stats
        printValues();
        prevTime1 = msTime;
    }
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/
