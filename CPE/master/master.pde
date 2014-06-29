/********************************************************************************
* File Name         : UKube_Plt_Emulator_V1_05.pde
*
* Project           : UKube-1
*
* Version           : V1.05
*
* Purpose           : Provides an example scheme for the platform interface controller
*                     of the UKube-1 Cubesat. Generates all Mission Interface Computer 
*                     (MIC) commands over I2C bus as specified in UKube communications 
*                     protocol. Manual mode allows sending of individual commands.  
*                     Automated mode allows full emulation of the MIC interface from
*                     payload power on to shutdown.
*
* Inputs            : I2C and SPI data in UKube-1 comms protocol. Manual input from RS-232 
*                     terminal
*
* Outputs           : I2C commands in UKube-1 comms protocol.
*
* Notes             : This code re-defines timer2 so standard arduino PWM functions
*                     on pin 3 and pin 11 should not be used.    
*
* Created by        : Alan Kane, Clyde Space Ltd, 01/2/11
*
* Last edited       : Initial Release
*
* Changelog         : V1.0     Initial Release
*                     V1.01    Changed Default I2C Slave Address to 0x71
*                     V1.02    Standardised payload response timeout to 2ms
*                     V1.03    Updated coms protocol. Every command now includes an
*                              acknowledge CRC after the last data transmission
*                     V1.04    16 payload_parameter IDs available for use. ID numbers to
*                              be pre-programmed by user.
*                     V1.05    Update to reflect verion F of protocol document.
*
* DISCLAIMER        :
*
* This Firmware is furnished under a license and may be used and
* copied only in accordance with the terms of such license and
* with the inclusion of the above copyright notice.  This
* Firmware or any other copies thereof may not be provided or
* otherwise made available to any other person. No title to or
* ownership of the software is hereby transferred.
*
*******************************************************************************/
 


//************* #includes *************
#include <twi_long.h>
#include <avr/pgmspace.h>

//************* #defines *************

// Parameter IDs to be used
// Update HEX value to use other parameter IDs - Do not change #define text or label
// Values in the range 0x01-0xFF may be used
#define PARA0 0x01
#define PARA1 0x02
#define PARA2 0x03
#define PARA3 0x04
#define PARA4 0x05
#define PARA5 0x06
#define PARA6 0x07
#define PARA7 0x08
#define PARA8 0x09
#define PARA9 0x0A
#define PARA10 0x0B
#define PARA11 0x0C
#define PARA12 0x0D
#define PARA13 0x0E
#define PARA14 0x0F
#define PARA15 0x10


// Default I2C Slave Address
#define PAY_ADDR  0x71

// Mass memory limits allocated to a single payload
#define MASS_MEM_LIMIT       1048576  // units. 1 unit = 256 Bytes
#define PRIORITY_MEM_LIMIT   65535    // units. 1 unit = 256 Bytes

#define SERIAL_PAUSE 5                // Delay in microseconds after writing to serial port.

// Buffer Sizes
#define SERIAL_RX_BUFSIZE 5
#define TX_BUFSIZE    261
#define RX_BUFSIZE    261

// SPI Pins
#define PIN_SPI_SS 10
#define PIN_SPI_MOSI 11 
#define PIN_SPI_MISO 12
#define PIN_SPI_CLK 13

// 1Hz Sync Pulse
#define SYNC_PIN 2

// Switch control Pins
#define CTRL_3V3         6 
#define CTRL_5V          7
#define CTRL_12V         8
#define CTRL_BAT         9

// Switch status Pins
// STAT_3V3 status pin is on an ADC channel so is set up elsewhere
#define STAT_5V          3
#define STAT_12V         4
#define STAT_BAT         5

// ADC reference in mV
#define MAX_ADC          5000

// Delay between SPI bytes in uSec
#define SPI_delay        3

// Number of data bytes for I2C data transfer
#define I2C_BYTES 256
// Number of data bytes for I2C priority data transfer
#define PRIORITY_I2C_BYTES 256
// Number of data bytes for SPI data transfer
#define SPI_BYTES 256

// Flag bit definitions
#define PAY_ON 0  // Indicate if payload is powered on - allows SPI SS and SYNC pulse to activate
#define USE_TX_CRC 1 // Use CRCs on transmitted payload commands
#define AUTO_EMU 2  // Indicate if emulator is operating in automatic MIC emulation mode
#define LEDstate 3  // Indicate if LED is on or off
#define MAIN_MENU_UP 4 // indicate if main menu is up
#define USE_RX_CRC 5 // Use CRCs on received payload commands
#define UPDATE_PAYLOAD_STATUS 6 // Use automatic status updates
#define DATA_RXD 7            // data received waiting to be processed

// Payload request flag bit definitions
#define  PAYLOAD_UPDATE 0
#define  NEXT_MODE 1
#define  DISABLE_COMPRESSION  2
#define  STREAM_READY  3
#define  RESET_PAYLOAD  6
#define  SHUTDOWN  7

// Header flags in packet definition
#define CRC 0
#define ERROR 3

// Power on and off delays in milliseconds
#define POWER_ON_DELAY 30000
#define POWER_OFF_DELAY 30000
#define POWER_CYCLE_DELAY 500

// Time in microsec in which payload must respond
#define PAYLOAD_RESPONSE_TIME 2000

//************* Global Variables *************

unsigned char flags = 0;    // General flag register

uint8_t I2C_address = 0;  // I2C slave address

unsigned long previous_pay_time = 0;  // Time of last payload status update
unsigned long payload_init_time = 0;  // Initialisation delay timer

char serial_rxd[SERIAL_RX_BUFSIZE];  // Serial receive buffer
uint8_t rxd_buffer[RX_BUFSIZE];      // Shared SPI and I2C receive buffer
uint8_t txd_buffer[TX_BUFSIZE];      // Shared SPI and I2C transmit buffer

char serial_rxd_ptr = 0;             // Current position in serial receive buffer

// Setup ADC channels
int SEN_3V3 = A0; 
int SEN_5V = A1; 
int SEN_12V = A2; 
int SEN_BAT = A3;
int STAT_3V3 = A6;                  // Due to lack of available digital IO Stat_3V3 read as analogue and converted to digital value. (>half scale = 1)

unsigned char active_bus = CTRL_5V;    // hold main bus to switch during auto emulation

/* Timer2 */  
unsigned int tcnt2;           // reload value to generate 1ms interrupt
unsigned int millisecs = 0;   // counter of millis
unsigned long secs = 0;       // counter of secs since platform power on

// Header flags
uint8_t rx_header_flags = 0x00;
uint8_t tx_header_flags = 0x00;

// Payload Information fields as defined in payload protocol and packet definition
uint8_t payload_op_mode = 0;                        // Payload Operation Mode
unsigned int payload_op_flag = 0;                   // Payload Operation Flag
unsigned long init_time = 0;                        // Onboard time at MIC
unsigned int priority_limit = PRIORITY_MEM_LIMIT;   // Payload Priority Data Limit
unsigned int priority_remain = PRIORITY_MEM_LIMIT;  // Payload Priortiy Data Remaining
unsigned long mass_mem_limit = MASS_MEM_LIMIT;      // Payload Mass Memory Limit
unsigned long mass_mem_remain = MASS_MEM_LIMIT;     // Payload Mass Memory Remaining
unsigned int poll_freq = 0;                         // Payload Poll Frequency
unsigned int priority_waiting = 0;                  // Payload Priority Data Waiting
unsigned long data_waiting = 0;                     // Payload Data Waiting
uint8_t pay_request = 0;                            // Payload Request Flags; bit0 = payload update, bit1 = next mode, bit2 = disable compression, bit3 = stream ready, bit6 = reset payload, bit7 = shutdown payload

uint8_t pay_para_ID[16] = {PARA0,PARA1,PARA2,PARA3,PARA4,PARA5,PARA6,PARA7,PARA8,PARA9,PARA10,PARA11,PARA12,PARA13,PARA14,PARA15};
unsigned int pay_para_value[16] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
unsigned int pay_para_ID_flags = 0;

unsigned int crc = 0;                               // CRC bytes

unsigned char ack_length = 5;    // length of acknowledge packet.

// Store main menu text in flash.
prog_char string_0[] PROGMEM = "MIC Emulator";   
prog_char string_1[] PROGMEM = " ";
prog_char string_2[] PROGMEM = "1: PLT Command";
prog_char string_3[] PROGMEM = "2: PAY Command";
prog_char string_4[] PROGMEM = "3: Auto mode";
prog_char string_5[] PROGMEM = "4: Settings ";
prog_char string_6[] PROGMEM = "Enter Number:";
prog_char string_7[] PROGMEM = " ";

// Store command confirmations in flash
prog_char confirm_0[] PROGMEM = "<<PLT Command Received";   
prog_char confirm_1[] PROGMEM = "<<PAY Command Received";
prog_char confirm_2[] PROGMEM = "<<Automatic Emulation";
prog_char confirm_3[] PROGMEM = "<<Settings";
prog_char confirm_4[] PROGMEM = "<<Unknown Command";
prog_char confirm_5[] PROGMEM = "SPI data received:";

// Store settings menu text in flash.
prog_char set_0[] PROGMEM = "Settings";   
prog_char set_1[] PROGMEM = " ";
prog_char set_2[] PROGMEM = "1: Set slave I2C address";
prog_char set_3[] PROGMEM = "2: Set status report delay";
prog_char set_4[] PROGMEM = "3: Set active power bus";

// Store payload menu text in flash.
prog_char pay_menu_0[] PROGMEM = "Payload";   
prog_char pay_menu_1[] PROGMEM = " ";
prog_char pay_menu_2[] PROGMEM = "0: Initialise";
prog_char pay_menu_3[] PROGMEM = "1: Status";
prog_char pay_menu_4[] PROGMEM = "2: Update";
prog_char pay_menu_5[] PROGMEM = "3: Parameter Write";
prog_char pay_menu_6[] PROGMEM = "4: Parameter Read";
prog_char pay_menu_7[] PROGMEM = "5: Priority Data Transfer";
prog_char pay_menu_8[] PROGMEM = "6: Data Transfer";
prog_char pay_menu_9[] PROGMEM = "7: SPI Data Transfer";
prog_char pay_menu_10[] PROGMEM = "8: Shutdown";
prog_char pay_menu_11[] PROGMEM = " ";
prog_char pay_menu_12[] PROGMEM = "Enter Number:";

// Store emulator menu text in flash.
prog_char emu_menu_0[] PROGMEM = "1: Switch 3V3 Supply";   
prog_char emu_menu_1[] PROGMEM = "2: Switch 5V Supply";
prog_char emu_menu_2[] PROGMEM = "3: Switch 12V Supply";
prog_char emu_menu_3[] PROGMEM = "4: Switch BAT Supply";
prog_char emu_menu_4[] PROGMEM = "5: Display ADC Values";
prog_char emu_menu_5[] PROGMEM = "3V3 =";
prog_char emu_menu_6[] PROGMEM = "5V =";
prog_char emu_menu_7[] PROGMEM = "12V =";
prog_char emu_menu_8[] PROGMEM = "BAT =";
prog_char emu_menu_9[] PROGMEM = "buses(3-5-12-bat): ";

// Store emulator status text in flash.
prog_char emu_msg_0[] PROGMEM = "3V3 Supply Current: ";   
prog_char emu_msg_1[] PROGMEM = "5V Supply Current: ";
prog_char emu_msg_2[] PROGMEM = "12V Supply Current: ";
prog_char emu_msg_3[] PROGMEM = "BAT Supply Current: ";
prog_char emu_msg_4[] PROGMEM = " mA";

// Store error message text in flash.
prog_char error_0[] PROGMEM = "<<Serial Buffer Full";   
prog_char error_1[] PROGMEM = "<<Unknown Cmd";
prog_char error_2[] PROGMEM = "<<Payload Powered Off";
prog_char error_3[] PROGMEM = "<<Too Few Characters Entered";
prog_char error_4[] PROGMEM = "<<ERROR condition at payload-";
prog_char error_5[] PROGMEM = "<<Too few I2C bytes received";
prog_char error_6[] PROGMEM = "<<CRC check failed";
prog_char error_7[] PROGMEM = "<<Insufficient Storage for Priority Data";
prog_char error_8[] PROGMEM = "<<Insufficient Storage for Mass Mem Data";
prog_char error_9[] PROGMEM = "<<Failed CRC after 3 attempts";
prog_char error_10[] PROGMEM = "<<Acknowledge didnt match CRC";
prog_char error_11[] PROGMEM = "<<Timeout. Acknowledge not received";

// Store status report heading text in flash.
prog_char pay_status_0[] PROGMEM = "Payload Status";   
prog_char pay_status_1[] PROGMEM = "-------------------------";
prog_char pay_status_2[] PROGMEM = "Operation Mode: ";
prog_char pay_status_3[] PROGMEM = "Operation Flags: ";
prog_char pay_status_4[] PROGMEM = "Priority Data Waiting: ";
prog_char pay_status_5[] PROGMEM = "Data Waiting: ";
prog_char pay_status_6[] PROGMEM = "Request Flags: ";
prog_char pay_status_7[] PROGMEM = "Parameter ID: ";
prog_char pay_status_8[] PROGMEM = "Parameter Value: ";

// Store pay command text in flash.
prog_char pay_cmd_0[] PROGMEM = "Enter Parameter ID to read";   
prog_char pay_cmd_1[] PROGMEM = "Enter Parameter ID to write";
prog_char pay_cmd_2[] PROGMEM = "Enter Parameter Value to write";
prog_char pay_cmd_3[] PROGMEM = "1: Para ID :";
prog_char pay_cmd_4[] PROGMEM = "2: Para ID :";
prog_char pay_cmd_5[] PROGMEM = "3: Para ID :";
prog_char pay_cmd_6[] PROGMEM = "4: Para ID :";
prog_char pay_cmd_7[] PROGMEM = "5: Para ID :";
prog_char pay_cmd_8[] PROGMEM = "6: Para ID :";
prog_char pay_cmd_9[] PROGMEM = "7: Para ID :";
prog_char pay_cmd_10[] PROGMEM = "8: Para ID :";
prog_char pay_cmd_11[] PROGMEM = "Enter Number:";
prog_char pay_cmd_12[] PROGMEM = "(enter 4 hex digits)";
prog_char pay_cmd_13[] PROGMEM = "Enter Desired Operation Mode";
prog_char pay_cmd_14[] PROGMEM = "(enter 2 hex digits)";
prog_char pay_cmd_15[] PROGMEM = "Enter Desired Operation Flags";

// Store memory remaining text in flash
prog_char mem_stat_0[] PROGMEM = "Platform Mass Memory Status";   
prog_char mem_stat_1[] PROGMEM = "Payload Priority Memory Allocation";
prog_char mem_stat_2[] PROGMEM = "Payload Mass Memory Allocation";

// Store status update period text in flash.
prog_char stat_up_0[] PROGMEM = "Enter Status Update Period";   
prog_char stat_up_1[] PROGMEM = "1: 1 seconds (only use in auto mode)";
prog_char stat_up_2[] PROGMEM = "2: 15 seconds";
prog_char stat_up_3[] PROGMEM = "3: 30 seconds";
prog_char stat_up_4[] PROGMEM = "4: 45 seconds";
prog_char stat_up_5[] PROGMEM = "5: 60 seconds";

// Store power bus selection menu in flash.
prog_char pbus_0[] PROGMEM = "Enter Main Power bus";   
prog_char pbus_1[] PROGMEM = "1: 3V3";
prog_char pbus_2[] PROGMEM = "2: 5V";
prog_char pbus_3[] PROGMEM = "3: 12V";
prog_char pbus_4[] PROGMEM = "4: BAT";

// Store general message reports in flash
prog_char gen_msg_0[] PROGMEM = "Shutting Down Payload in 30 sec";   
prog_char gen_msg_1[] PROGMEM = "Cycling Power";
prog_char gen_msg_2[] PROGMEM = "Powering Down ";
prog_char gen_msg_3[] PROGMEM = "Acknowledge received ";
prog_char gen_msg_4[] PROGMEM = " ";
prog_char gen_msg_5[] PROGMEM = " ";


// Store LUT of CRC checksum calculation bytes
PROGMEM prog_uint16_t crc_lut[] = {
0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0,
};
 
PROGMEM const char *mic_table[] = 	   // MIC menu text
{   
  string_0,
  string_1,
  string_2,
  string_3,
  string_4,
  string_5,
  string_6,
  string_7 };

PROGMEM const char *cmd_confirmation_table[] = 	   // MIC menu confirmations
{   
  confirm_0,
  confirm_1,
  confirm_2,
  confirm_3,
  confirm_4,
  confirm_5  };

PROGMEM const char *set_table[] = 	   // Settings menu text
{   
  set_0,
  set_1,
  set_2,
  set_3,
  set_4 };
  
PROGMEM const char *pay_table[] = 	   // Payload Menu text
{   
  pay_menu_0,
  pay_menu_1,
  pay_menu_2,
  pay_menu_3,
  pay_menu_4,
  pay_menu_5,
  pay_menu_6,
  pay_menu_7,
  pay_menu_8,
  pay_menu_9,
  pay_menu_10,
  pay_menu_11,
  pay_menu_12 };
  
PROGMEM const char *emu_table[] = 	   // Emulator board menu text
{   
  emu_menu_0,
  emu_menu_1,
  emu_menu_2,
  emu_menu_3,
  emu_menu_4,
  emu_menu_5,
  emu_menu_6,
  emu_menu_7,
  emu_menu_8,
  emu_menu_9 };
  
PROGMEM const char *emu_msg_table[] = 	   // Emulator board menu feedback message text
{   
  emu_msg_0,
  emu_msg_1,
  emu_msg_2,
  emu_msg_3,
  emu_msg_4 };
  
PROGMEM const char *error_table[] = 	   // Error message text
{   
  error_0,
  error_1,
  error_2,
  error_3,
  error_4,
  error_5,
  error_6,
  error_7,
  error_8,
  error_9,
  error_10,
  error_11};
  
PROGMEM const char *status_table[] = 	   // Payload Status text
{   
  pay_status_0,
  pay_status_1,
  pay_status_2,
  pay_status_3,
  pay_status_4,
  pay_status_5,
  pay_status_6,
  pay_status_7,
  pay_status_8 };
  
PROGMEM const char *pay_cmd_table[] = 	   // Payload command confirmation text
{   
  pay_cmd_0,
  pay_cmd_1,
  pay_cmd_2,
  pay_cmd_3,
  pay_cmd_4,
  pay_cmd_5,
  pay_cmd_6,
  pay_cmd_7,
  pay_cmd_8,
  pay_cmd_9,
  pay_cmd_10,
  pay_cmd_11,
  pay_cmd_12,
  pay_cmd_13,
  pay_cmd_14,
  pay_cmd_15 };

PROGMEM const char *mem_stat_table[] = 	   // Memory status message text
{   
  mem_stat_0,
  mem_stat_1,
  mem_stat_2 };

PROGMEM const char *stat_up_table[] = 	   // Payload status update message text
{   
  stat_up_0,
  stat_up_1,
  stat_up_2,
  stat_up_3,
  stat_up_4,
  stat_up_5 };

PROGMEM const char *pbus_table[] = 	   // Primary power bus message text
{   
  pbus_0,
  pbus_1,
  pbus_2,
  pbus_3,
  pbus_4};
  
PROGMEM const char *gen_msg_table[] = 	   // text of other assorted messages used in program
{   
  gen_msg_0,
  gen_msg_1,
  gen_msg_2,
  gen_msg_3,
  gen_msg_4,
  gen_msg_5 };  

char buffer[40];    // make sure this is large enough for the largest string it must hold


//************* FUNCTIONS *************

byte data[24];
double xPosition;
double yPosition;
double zPosition;


/*----------------------------------------------------*/
// This is an obligatory arduino function. It performs the basic setup of the board and is run once on power on.
void setup() 
{
  
  int i = 0;
  
  if(PAY_ADDR==0x61)  //alternate acknowledge packet for ADR 0x61
  {
    ack_length = 4;
  }
  
  
  Serial.begin(115200); // Setup Serial Port

  pinMode(SYNC_PIN, OUTPUT);
  
  pinMode(CTRL_3V3, OUTPUT); 
  pinMode(CTRL_5V, OUTPUT); 
  pinMode(CTRL_12V, OUTPUT); 
  pinMode(CTRL_BAT, OUTPUT); 

  pinMode(STAT_3V3, INPUT); 
  pinMode(STAT_5V, INPUT); 
  pinMode(STAT_12V, INPUT); 
  pinMode(STAT_BAT, INPUT);
  
  digitalWrite(CTRL_3V3,LOW);
  digitalWrite(CTRL_5V,LOW);
  digitalWrite(CTRL_12V,LOW);
  digitalWrite(CTRL_BAT,LOW);
  
  pinMode(PIN_SPI_SS, OUTPUT); 
  pinMode(PIN_SPI_MOSI, OUTPUT); 
  pinMode(PIN_SPI_MISO, INPUT); 
  pinMode(PIN_SPI_CLK, OUTPUT);
 
  digitalWrite(PIN_SPI_SS,HIGH); 

  // Setup I2C
  I2C_address = PAY_ADDR;
  I2C_master_begin();
  
  // Setup Timer2 to generate 1Hz sync pulse and measure onboard time
  setup_tmr2();
  poll_freq = 30000;
  
  // This value is used in automatic emulation mode to determine which bus should be cycled during reset etc.
  active_bus = CTRL_5V;
  
  // Configure use of CRCs at platform
  // for UKube use of CRCs defined by standard
  bitSet(flags,USE_TX_CRC);
  bitSet(flags,USE_RX_CRC);

  Serial.println("EMU>>ON");
  
  // Clear Serial Buffer
  for (i=0; i < SERIAL_RX_BUFSIZE; i++){
      serial_rxd[i] = 0x00;
  }
}

/*----------------------------------------------------*/


/*----------------------------------------------------*/
// A test function to determine how much SRAM is available
int availableMemory() 
{
 int size = 2048;
 byte *buf;
 while ((buf = (byte *) malloc(--size)) == NULL);
 free(buf);
 return size;
} 
/*----------------------------------------------------*/


/*----------------------------------------------------*/
// This is an obligatory arduino function. Once setup is run this loop will execute continuously.
void loop() //main loop
{    
  int i = 0; 
  char main_cmd = 0;
  static uint8_t init_tries = 0;  // used to count number of times initialisation command has been sent in automatic mode
  static char initialised = 0;  // used to determine whether the payload has been initialised since power on.
  static char reset_payload = 0;  // used in automatic mode to determine whether payload was shut down or reset (and needs to be re-initialised)
  static unsigned long int shutdown_timer = 0;  // used to measure the shutdown delay in auto mode
  
  serial_rxd_ptr = 0;
  
  if(!bitRead(flags, AUTO_EMU))  // If not operating in fully automatic mode display manual menu
  {
    if(bitRead(flags,MAIN_MENU_UP)== 0)        // If menu not up display it
    {
      print_MIC_menu();    // Display main menu   
      
      Serial.flush();
    
      bitSet(flags,MAIN_MENU_UP);    // Dont repeat main menu if its already on-screen
    }
  
    if (Serial.available() > 0)     // On Serial Data response to main menu
    {
      for(i=0;i<=Serial.available();i++)
      {  
        serial_rxd[serial_rxd_ptr] = Serial.read();
        serial_rxd_ptr++;
      
        if (serial_rxd_ptr >= SERIAL_RX_BUFSIZE) 
        {
          serial_rxd_ptr = 0;
          strcpy_P(buffer, (char*)pgm_read_word(&(error_table[0]))); // error msg if serial buffer overflows
          Serial.println( buffer );
          delayMicroseconds(SERIAL_PAUSE);
        } 
      }
      bitSet(flags, DATA_RXD);
    }

    if(bitRead(flags,DATA_RXD)) // if serial cmd received process it
    {
      bitClear(flags,DATA_RXD);
    
      main_cmd = serial_rxd[0]-0x30;  // convert ASCII to integer
      
      Serial.println(main_cmd, DEC);
      
      if(main_cmd == 1) 
      {
        // Emulator Board command Recieved 
        strcpy_P(buffer, (char*)pgm_read_word(&(cmd_confirmation_table[0]))); 
        Serial.println( buffer );
        delayMicroseconds(SERIAL_PAUSE); 
        Process_EMU();
        bitClear(flags,MAIN_MENU_UP);  
      } 
      else if (main_cmd == 2) 
      {
        // Payload command Recieved 
        strcpy_P(buffer, (char*)pgm_read_word(&(cmd_confirmation_table[1]))); 
        Serial.println( buffer );
        delayMicroseconds(SERIAL_PAUSE);
        Process_PAY();
        bitClear(flags,MAIN_MENU_UP);
      }
      else if (main_cmd == 3)   // start automated MIC emulation
      {
        // Automated MIC emulation
        strcpy_P(buffer, (char*)pgm_read_word(&(cmd_confirmation_table[2]))); 
        Serial.println( buffer );
        delayMicroseconds(SERIAL_PAUSE);
        bitClear(flags,UPDATE_PAYLOAD_STATUS);  // disable automatic status updating until initialised
        auto_mic_emulator();
         
        bitClear(flags,MAIN_MENU_UP); 
      }
      else if (main_cmd == 4)   // adjust emulator settings
      {
        // Settings Menu
        strcpy_P(buffer, (char*)pgm_read_word(&(cmd_confirmation_table[3]))); 
        Serial.println( buffer );
        delayMicroseconds(SERIAL_PAUSE);
        process_SET();       
        bitClear(flags,MAIN_MENU_UP);
      }
   
      for (i=0; i < SERIAL_RX_BUFSIZE; i++){
        serial_rxd[i] = 0x00;
      }
      serial_rxd_ptr = 0; 
    }
  }  // end of manual command menu
  else  // running in automatic mode
  {
    if(initialised == 0 && init_tries < 3)  // only initialise once, attempt 3 times
    {
      if((((secs*1000)+millisecs)-payload_init_time)>POWER_ON_DELAY)  // wait 30 sec before initialisation
      {
        if (pay_ops_initialise())  // send initialise command
        {
          initialised++;           //  only update initialised flag if we receive a valid CRC ack from payload
        }
        else  // repeat initialisation command 3 times if payload fails to ack
        {
          init_tries++;
          delay(10);  //wait 10ms before trying again.
        }  
      };
    }
    else if(init_tries >= 3)
    {
      strcpy_P(buffer, (char*)pgm_read_word(&(error_table[9])));     // initialisation fail message
      Serial.println( buffer );
      delayMicroseconds(SERIAL_PAUSE);  
  
      init_tries = 0;
      bitClear(flags,MAIN_MENU_UP);  
      bitClear(flags, AUTO_EMU);  // drop out of fully automatic mode
    }
    
    // Check requests for action from payload
    // Only answer one request each time round loop to prevent slowing down main loop
    if (bitRead(pay_request,SHUTDOWN))
    {
      strcpy_P(buffer, (char*)pgm_read_word(&(gen_msg_table[0]))); // Powering down payload msg
      Serial.println( buffer );
      delayMicroseconds(SERIAL_PAUSE);
      
      delay(10);
      
      pay_shutdn();
      delayMicroseconds(500);  // wait for ack
      bitClear(pay_request,SHUTDOWN);

      pay_request = 0;  //stop responding to payload requests after shutdown sent.
      priority_waiting = 0;
      data_waiting = 0;
      shutdown_timer = (secs*1000)+millisecs;
    }
    else if(bitRead(pay_request,RESET_PAYLOAD))
    {
      strcpy_P(buffer, (char*)pgm_read_word(&(gen_msg_table[0]))); // Powering down payload msg
      Serial.println( buffer );
      delayMicroseconds(SERIAL_PAUSE);
      
      delay(1);
      pay_shutdn();
      delayMicroseconds(500);  // wait for ack
      shutdown_timer = (secs*1000)+millisecs;
      reset_payload = 1;
      bitClear(pay_request,RESET_PAYLOAD);
      pay_request = 0;  //stop responding to payload requests after shutdown sent.
      priority_waiting = 0;
      data_waiting = 0;
    }
    else if(bitRead(pay_request,NEXT_MODE))
    {
      payload_op_mode++;
      pay_ops_update();
      bitClear(pay_request,NEXT_MODE);
    }
    else if(bitRead(pay_request,PAYLOAD_UPDATE))
    {
      pay_ops_update();
      bitClear(pay_request,PAYLOAD_UPDATE);
    }
    else if(pay_para_ID_flags != 0 && shutdown_timer==0)
    {
      for(i=0;i<16;i++)
      {
        if(bitRead(pay_para_ID_flags,i))
        {
          auto_pay_parameter_write(pay_para_ID[i],pay_para_value[i]);
          pay_para_value[i]++;
          bitClear(pay_para_ID_flags,i);
        }
      }
      delay(2); //delay to allow parameter write to complete.
      
    }
    else if(priority_waiting > 0)
    {
      pay_pri_data_trans();
      priority_waiting = priority_waiting - PRIORITY_I2C_BYTES;
    }
    else if(bitRead(pay_request,STREAM_READY))
    {
      pay_SPI_data_trans();
      data_waiting = data_waiting - SPI_BYTES;      
      bitClear(pay_request,STREAM_READY);
    }
    else if(data_waiting > 0 && !bitRead(pay_request,STREAM_READY))
    {
      pay_data_trans();
      data_waiting = data_waiting - I2C_BYTES;      
    } 
  }
  
  if(shutdown_timer != 0)  // If shutdown command has been issued in auto mode
  {
    if((((secs*1000)+millisecs)-shutdown_timer)>POWER_OFF_DELAY)  // if timeout expired power off
    {
      strcpy_P(buffer, (char*)pgm_read_word(&(gen_msg_table[2]))); // Powering down payload msg
      Serial.println( buffer );
      delayMicroseconds(SERIAL_PAUSE);

      digitalWrite(active_bus, LOW);      // Power down payload
      initialised = 0;
      shutdown_timer = 0;
      
      if(reset_payload)  // if triggered by shutdown command drop out of auto mode, else wait for re-initialise
      {
        reset_payload = 0;

        strcpy_P(buffer, (char*)pgm_read_word(&(gen_msg_table[1]))); // Cycling power msg
        Serial.println( buffer );
        delayMicroseconds(SERIAL_PAUSE);

        delay(POWER_CYCLE_DELAY);
        digitalWrite(active_bus, HIGH);      // Power cycle payload
        payload_init_time = (secs*1000) + millisecs;
      }
      else
      {
        bitClear(flags, AUTO_EMU);  
      }
    }
  }

  // Send request for status update, provided not waiting to shutdown
  if(bitRead(flags,UPDATE_PAYLOAD_STATUS) && shutdown_timer == 0)
  {
    if((((secs*1000)+millisecs) - previous_pay_time) > poll_freq)       // see if period has expired
    {
      previous_pay_time = (secs*1000) + millisecs;                      // note time of last status update
      pay_ops_status(); 
      bitClear(flags,MAIN_MENU_UP);         // Redisplay main menu after update
    };
  };
}
/*----------------------------------------------------*/

/*----------------------------------------------------*/
void Blinky() {
    // if the LED is off turn it on and vice-versa:
    if (bitRead(flags,LEDstate))
      bitClear(flags,LEDstate);
    else
      bitSet(flags,LEDstate);

    // set the LED with the ledState of the variable:
    digitalWrite(SYNC_PIN, bitRead(flags,LEDstate));
};
/*----------------------------------------------------*/

/*----------------------------------------------------*/
void Process_PAY() {
  int i = 0;
  uint8_t cmd_num = 0;
  char send_data = 0, cmd  = 0;
  
  // Dispaly Payload Menu
  print_PAY_menu();
  
  Serial.flush();
 
  while(Serial.available()==0)
    ;
 
 // Payload Commands
 cmd_num = Serial.read()-0x30;
 
 
 Serial.println(cmd_num,HEX);
 Serial.println();
  
  
  // Clear I2C buffer
  for (i=0; i < TX_BUFSIZE; i++){
     txd_buffer[i] = 0x00;
  } 
            
            
  //Serial.println(cmd_num, HEX);
  switch(cmd_num) {
    case 0: ;   pay_ops_initialise();
                break;
    case 1: ;   pay_ops_status();
                break;                    
    case 2: ;   pay_ops_update();
                break;
    case 3: ;   pay_parameter_write();
                break;
    case 4: ;   pay_parameter_read();
                break;
    case 5: ;   pay_pri_data_trans();
                break;
    case 6: ;   pay_data_trans();
                break;
    case 7: ;   pay_SPI_data_trans();
                break;
    case 8: ;   pay_shutdn(); 
                break;
    default: ;  send_data = 0; 
                strcpy_P(buffer, (char*)pgm_read_word(&(error_table[1]))); 
                Serial.println( buffer );
                delayMicroseconds(SERIAL_PAUSE);
                break;
  };
   
   
};
/*----------------------------------------------------*/

/*----------------------------------------------------*/
void Process_EMU() 
{
  
  int SWT_cmd_num = 0;
  int i = 0;
  int read_3V3 = 0, read_5V = 0, read_12V = 0, read_BAT = 0;
  long calc_3V3 = 0, calc_5V = 0, calc_12V = 0, calc_BAT = 0;

  int in1 = 0,in2 = 0, in3 = 0, in4 = 0;

  strcpy_P(buffer, (char*)pgm_read_word(&(emu_table[9]))); 
  Serial.println( buffer );
  delayMicroseconds(SERIAL_PAUSE); 

  Serial.print((analogRead(STAT_3V3)>511));    //connected to analog channel instead of digital I/P
  Serial.print('-');
  Serial.print(digitalRead(STAT_5V));
  Serial.print('-');
  Serial.print(digitalRead(STAT_12V));
  Serial.print('-');
  Serial.println(digitalRead(STAT_BAT));
  
  
  // Display EMU Menu
  print_EMU_menu();
  
 
  Serial.flush();
 
  while(Serial.available()==0)
    ;
 
  SWT_cmd_num = Serial.read()-0x30;
 
 
  Serial.println(SWT_cmd_num);
  
  switch(SWT_cmd_num) 
  {
    case 1: ;   // Measure status and toggle 3V3            
                if(analogRead(STAT_3V3)>511)
                {
                  digitalWrite(CTRL_3V3, LOW);
                }
                else
                {
                   digitalWrite(CTRL_3V3, LOW);
                   delay(1);
                   digitalWrite(CTRL_3V3, HIGH);
                   bitSet(flags,PAY_ON);
                }
                delay(5);
                
                strcpy_P(buffer, (char*)pgm_read_word(&(emu_table[5]))); // 3V3 = 
                Serial.println( buffer );
                delayMicroseconds(SERIAL_PAUSE);
                Serial.println((analogRead(STAT_3V3)>511));                
                break;
    case 2: ;   // Measure status and toggle 5V
                if(digitalRead(STAT_5V))
                {
                  digitalWrite(CTRL_5V, LOW);
                }
                else
                {
                  digitalWrite(CTRL_5V, LOW);
                  delay(1);
                  digitalWrite(CTRL_5V, HIGH);
                  bitSet(flags,PAY_ON);
                }
                delay(5);
                
                strcpy_P(buffer, (char*)pgm_read_word(&(emu_table[6]))); // 5V = 
                Serial.println( buffer );
                delayMicroseconds(SERIAL_PAUSE);
                Serial.println(digitalRead(STAT_5V));
                
                break;
    case 3: ;   // Measure status and toggle 12V
                if(digitalRead(STAT_12V))
                {
                  digitalWrite(CTRL_12V, LOW);
                }
                else
                {
                  digitalWrite(CTRL_12V, LOW);
                  delay(1);
                  digitalWrite(CTRL_12V, HIGH);
                  bitSet(flags,PAY_ON);
                }
                
                delay(5);
                
                strcpy_P(buffer, (char*)pgm_read_word(&(emu_table[7]))); // 12V = 
                Serial.println( buffer );
                delayMicroseconds(SERIAL_PAUSE);
                Serial.println(digitalRead(STAT_12V));
                break;   
    case 4: ;   // Measure status and toggle BAT
                if(digitalRead(STAT_BAT))
                {
                  digitalWrite(CTRL_BAT, LOW);
                }
                else
                {
                  digitalWrite(CTRL_BAT, LOW);
                  delay(1);
                  digitalWrite(CTRL_BAT, HIGH);
                  bitSet(flags,PAY_ON);
                }
                
                delay(5);
                
                strcpy_P(buffer, (char*)pgm_read_word(&(emu_table[8]))); // BAT = 
                Serial.println( buffer );
                delayMicroseconds(SERIAL_PAUSE);
                Serial.println(digitalRead(STAT_BAT));
                break; 
    case 5: ;   // Measure ADC voltages and display
                // read ADC channels
                read_3V3 = analogRead(SEN_3V3);
                read_5V = analogRead(SEN_5V);
                read_12V = analogRead(SEN_12V);
                read_BAT = analogRead(SEN_BAT);
                  
                // convert ADC values back to voltages in mV
                calc_3V3 = map((long)read_3V3,(long)0,(long)1023,(long)0,(long)1421);
                calc_5V = map((long)read_5V,(long)0,(long)1023,(long)0,(long)913);
                calc_12V = map((long)read_12V,(long)0,(long)1023,(long)0,(long)752);
                calc_BAT = map((long)read_BAT,(long)0,(long)1023,(long)0,(long)752);

                strcpy_P(buffer, (char*)pgm_read_word(&(emu_msg_table[0]))); 
                Serial.print( buffer );
                delayMicroseconds(SERIAL_PAUSE);
                Serial.print(calc_3V3);
                strcpy_P(buffer, (char*)pgm_read_word(&(emu_msg_table[4]))); 
                Serial.println( buffer );
                delayMicroseconds(SERIAL_PAUSE);
    
                strcpy_P(buffer, (char*)pgm_read_word(&(emu_msg_table[1]))); 
                Serial.print( buffer );
                delayMicroseconds(SERIAL_PAUSE);
                Serial.print(calc_5V);
                strcpy_P(buffer, (char*)pgm_read_word(&(emu_msg_table[4]))); 
                Serial.println( buffer );
                delayMicroseconds(SERIAL_PAUSE);
   
                strcpy_P(buffer, (char*)pgm_read_word(&(emu_msg_table[2]))); 
                Serial.print( buffer );
                delayMicroseconds(SERIAL_PAUSE);
                Serial.print(calc_12V);
                strcpy_P(buffer, (char*)pgm_read_word(&(emu_msg_table[4]))); 
                Serial.println( buffer );
                delayMicroseconds(SERIAL_PAUSE);
                
                strcpy_P(buffer, (char*)pgm_read_word(&(emu_msg_table[3]))); 
                Serial.print( buffer );
                delayMicroseconds(SERIAL_PAUSE);
                Serial.print(calc_BAT);
                strcpy_P(buffer, (char*)pgm_read_word(&(emu_msg_table[4]))); 
                Serial.println( buffer );
                delayMicroseconds(SERIAL_PAUSE);
                break;  
    default: ;  strcpy_P(buffer, (char*)pgm_read_word(&(error_table[1]))); 
                Serial.println( buffer );
                delayMicroseconds(SERIAL_PAUSE);
                break;
  }    
};
/*----------------------------------------------------*/


/*----------------------------------------------------*/
void process_SET(void)
{
  char set_cmd = 0, set_cmd2 = 0;
       
  print_set_menu();
        
  Serial.flush();
 
  while(Serial.available()==0)  // wait for menu choice to be entered
    ;
 
  // Settings Option
  set_cmd = Serial.read()-0x30;
 
  Serial.println(set_cmd,HEX);
  Serial.println();

  // process settings command
  switch(set_cmd)
  {
    case 1:  ;  I2C_address = get_I2C_address();      // Set I2C slave to address
                break;
    case 2:  ;  print_stat_up_menu();                 // Set status update period
                Serial.flush();
 
                while(Serial.available()==0)  // wait for menu choice to be entered
                   ;
 
                // Settings Option
                set_cmd2 = Serial.read()-0x30;
 
                Serial.println(set_cmd2,HEX);
                Serial.println();
                     
                switch(set_cmd2)                      // Period in milliseconds
                {
                  case 1: ; poll_freq = 1000;
                            break; 
                  case 2: ; poll_freq = 15000;
                            break;
                  case 3: ; poll_freq = 30000;
                            break;
                  case 4: ; poll_freq = 45000;
                            break;
                  case 5: ; poll_freq = 60000;
                            break;        
                  default:  poll_freq = 30000; 
                            break;                       
                }
                break;
    case 3:  ;  print_pbus_menu();                    // select which power bus controls payload in auto mode
                Serial.flush();
 
                while(Serial.available()==0)  // wait for menu choice to be entered
                  ;
 
                // Settings Option
                set_cmd2 = Serial.read()-0x30;
 
                Serial.println(set_cmd2,HEX);
                Serial.println();
                     
                switch(set_cmd2)                      // choose power bus
                {
                  case 1: ; active_bus = CTRL_3V3;
                            break; 
                  case 2: ; active_bus = CTRL_5V;
                            break;
                  case 3: ; active_bus = CTRL_12V;
                            break;
                  case 4: ; active_bus = CTRL_BAT;
                            break;
                  default:  active_bus = CTRL_5V; 
                            break;                       
                }
                break;
    default: ;  break;
  }
}
/*----------------------------------------------------*/

/*----------------------------------------------------*/
// SPI data transmit
void SPI_data_transmit(unsigned int tx_num_data_bytes, uint8_t *tx_body)
{
  unsigned int SPI_tx_ptr = 0;
  unsigned int i;
 
  digitalWrite(PIN_SPI_SS, LOW);
  delay(1);
  
  // send and receive body data
  for(i = 0; i<(tx_num_data_bytes);i++)
  {
    rxd_buffer[SPI_tx_ptr] = send_spi(*(tx_body+SPI_tx_ptr));  //send current place in receive buffer
    SPI_tx_ptr++;
    delayMicroseconds(SPI_delay);
  };

  delay(1);
  digitalWrite(PIN_SPI_SS, HIGH);
  delay(10);
  // process CRC, discard data if failed
    
  return ;
}

/*----------------------------------------------------*/

/*----------------------------------------------------*/
// SPI data receive
void SPI_data_receive(unsigned int rx_num_data_bytes, uint8_t *spi_rx_buffer)
{
  unsigned int q;
  
  digitalWrite(PIN_SPI_SS, LOW);
  delayMicroseconds(100);
  
// receive body data
  for(q = 0; q<(rx_num_data_bytes);q++)
  {
    rxd_buffer[q] = send_spi(0x00);  // send blank data to clock SPI data in
    delayMicroseconds(SPI_delay);
  };

  delayMicroseconds(100);
  digitalWrite(PIN_SPI_SS, HIGH);
  delayMicroseconds(100);

}  
/*----------------------------------------------------*/ 

/*----------------------------------------------------*/
// Display received data buffer
void display_receive(unsigned int disp_num_data_bytes, uint8_t *disp_buffer)
{
  uint8_t row = 0;
  uint8_t column = 0;

// Display full rows of data
  for(row = 0; row<(disp_num_data_bytes/16);row++)
  {    
    for(column = 0; column < 16; column++)
    {
        Serial.print(*(disp_buffer+((16*row)+column)), HEX);
        Serial.print(9,BYTE);    // Horizontal tab
    }
    Serial.println(' ');
    
    delayMicroseconds(5);
  };
  
// Display any remaining data in a partial row  
  for(column = 0; column <disp_num_data_bytes%16 ; column++)
  {
      Serial.print(*(disp_buffer+((16*row)+column)), HEX);
      Serial.print(9,BYTE);    // Horizontal tab
  }
  Serial.println(' ');
}  
/*----------------------------------------------------*/ 

/*----------------------------------------------------*/ 
uint8_t send_spi(uint8_t spi_data)
{
  /* Start transmission */
  SPDR = spi_data;
  
  /* Wait for transmission complete */
  while(!(SPSR & (1<<SPIF)))
    ;

  return SPDR;
}
/*----------------------------------------------------*/ 



/*----------------------------------------------------*/  
void disable_SPI(void)
{
  //disable SPI comms  
  bitClear(SPCR,SPE);
}
/*----------------------------------------------------*/ 

/*----------------------------------------------------*/ 
void enable_SPI(void)
{
  char temp;
  //set up SPI comms  
  temp = SPSR;  // Clear WCOL and SPIF by reading SPSR then writing SPDR
  SPDR = 0x00;
  
  /* Enable SPI, Master, set clock rate fck/16 */
  SPCR = 0x00;
  SPSR = 0x00;
  SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);

}
/*----------------------------------------------------*/ 

/*----------------------------------------------------*/ 
void display_pay_status(void)
{
  uint8_t i = 0;
  
    strcpy_P(buffer, (char*)pgm_read_word(&(status_table[0]))); 
    Serial.println( buffer );
    delayMicroseconds(SERIAL_PAUSE);
  
    strcpy_P(buffer, (char*)pgm_read_word(&(status_table[1]))); 
    Serial.println( buffer );
    delayMicroseconds(SERIAL_PAUSE);
  
    strcpy_P(buffer, (char*)pgm_read_word(&(status_table[2]))); 
    Serial.print( buffer );
    delayMicroseconds(SERIAL_PAUSE);
    Serial.println(payload_op_mode,HEX);
    
    strcpy_P(buffer, (char*)pgm_read_word(&(status_table[3]))); 
    Serial.print( buffer );
    delayMicroseconds(SERIAL_PAUSE);
    Serial.println(payload_op_flag,HEX);
    
    strcpy_P(buffer, (char*)pgm_read_word(&(status_table[4]))); 
    Serial.print( buffer );
    delayMicroseconds(SERIAL_PAUSE);
    Serial.println(priority_waiting,DEC);

    strcpy_P(buffer, (char*)pgm_read_word(&(status_table[5]))); 
    Serial.print( buffer );
    delayMicroseconds(SERIAL_PAUSE);
    Serial.println(data_waiting,DEC);
    
    strcpy_P(buffer, (char*)pgm_read_word(&(status_table[6]))); 
    Serial.print( buffer );
    delayMicroseconds(SERIAL_PAUSE);
    Serial.println(pay_request,BIN);

    for(i=0;i<8;i++)
    {
      strcpy_P(buffer, (char*)pgm_read_word(&(status_table[7]))); 
      Serial.print( buffer );
      delayMicroseconds(SERIAL_PAUSE);
      Serial.println(pay_para_ID[i],HEX);
    }
  
    delay(25);  // Delay to let text display 
}
/*----------------------------------------------------*/ 

/*----------------------------------------------------*/ 
void display_pay_read(void)
{
    strcpy_P(buffer, (char*)pgm_read_word(&(status_table[7]))); 
    Serial.print( buffer );
    delayMicroseconds(SERIAL_PAUSE);
    Serial.println(rxd_buffer[2],HEX);
    
    strcpy_P(buffer, (char*)pgm_read_word(&(status_table[8]))); 
    Serial.print( buffer );
    delayMicroseconds(SERIAL_PAUSE);
    Serial.print(rxd_buffer[3],HEX);
    Serial.println(rxd_buffer[4],HEX);
    
    delay(10);  // Delay to let text display 
}
/*----------------------------------------------------*/ 

/*----------------------------------------------------*/
void print_MIC_menu(void)
{
  char i;
  
  for (int i = 0; i < 8; i++)
  {
    strcpy_P(buffer, (char*)pgm_read_word(&(mic_table[i]))); 
    Serial.println( buffer );
    delayMicroseconds(SERIAL_PAUSE);
  }
  
  delay(10);  // Delay to let text display 
}
/*----------------------------------------------------*/


/*----------------------------------------------------*/
void print_set_menu(void)
{
  char i;
  
  for (int i = 0; i <= 4; i++)
  {
    strcpy_P(buffer, (char*)pgm_read_word(&(set_table[i]))); 
    Serial.println( buffer );
    delayMicroseconds(SERIAL_PAUSE);
  }
  
  delay(10);  // Delay to let text display 
}
/*----------------------------------------------------*/


/*----------------------------------------------------*/
void print_stat_up_menu(void)
{
  char i;
  
  for (int i = 0; i < 6; i++)
  {
    strcpy_P(buffer, (char*)pgm_read_word(&(stat_up_table[i]))); 
    Serial.println( buffer );
    delayMicroseconds(SERIAL_PAUSE);
  }
  
  delay(10);  // Delay to let text display 
}
/*----------------------------------------------------*/

/*----------------------------------------------------*/
void print_pbus_menu(void)
{
  char i;
  
  for (int i = 0; i < 5; i++)
  {
    strcpy_P(buffer, (char*)pgm_read_word(&(pbus_table[i]))); 
    Serial.println( buffer );
    delayMicroseconds(SERIAL_PAUSE);
  }
  
  delay(10);  // Delay to let text display 
}
/*----------------------------------------------------*/

/*----------------------------------------------------*/
void print_PAY_menu(void)
{
  char i;
  
  for (int i = 0; i < 13; i++)
  {
    strcpy_P(buffer, (char*)pgm_read_word(&(pay_table[i]))); 
    Serial.println( buffer );
    delayMicroseconds(SERIAL_PAUSE);
  }
  
  delay(10);  // Delay to let text display 
}
/*----------------------------------------------------*/

/*----------------------------------------------------*/
void print_EMU_menu(void)
{
  char i;
  
  for (int i = 0; i < 5; i++)
  {
    strcpy_P(buffer, (char*)pgm_read_word(&(emu_table[i]))); 
    Serial.println( buffer );
    delayMicroseconds(SERIAL_PAUSE);
  }
  
  delay(10);  // Delay to let text display 
}
/*----------------------------------------------------*/

void display_para_IDs(void)
{
    strcpy_P(buffer, (char*)pgm_read_word(&(pay_cmd_table[3]))); 
    Serial.print( buffer );
    delayMicroseconds(SERIAL_PAUSE);
    Serial.println(pay_para_ID[1],HEX);

    strcpy_P(buffer, (char*)pgm_read_word(&(pay_cmd_table[4]))); 
    Serial.print( buffer );
    delayMicroseconds(SERIAL_PAUSE);
    Serial.println(pay_para_ID[2],HEX);
   
    strcpy_P(buffer, (char*)pgm_read_word(&(pay_cmd_table[5]))); 
    Serial.print( buffer );
    delayMicroseconds(SERIAL_PAUSE);
    Serial.println(pay_para_ID[3],HEX);
    
    strcpy_P(buffer, (char*)pgm_read_word(&(pay_cmd_table[6]))); 
    Serial.print( buffer );
    delayMicroseconds(SERIAL_PAUSE);
    Serial.println(pay_para_ID[4],HEX);
    
    strcpy_P(buffer, (char*)pgm_read_word(&(pay_cmd_table[7]))); 
    Serial.print( buffer );
    delayMicroseconds(SERIAL_PAUSE);
    Serial.println(pay_para_ID[5],HEX);
    
    strcpy_P(buffer, (char*)pgm_read_word(&(pay_cmd_table[8]))); 
    Serial.print( buffer );
    delayMicroseconds(SERIAL_PAUSE);
    Serial.println(pay_para_ID[6],HEX);
    
    strcpy_P(buffer, (char*)pgm_read_word(&(pay_cmd_table[9]))); 
    Serial.print( buffer );
    delayMicroseconds(SERIAL_PAUSE);
    Serial.println(pay_para_ID[7],HEX);
    
    strcpy_P(buffer, (char*)pgm_read_word(&(pay_cmd_table[10]))); 
    Serial.print( buffer );
    delayMicroseconds(SERIAL_PAUSE);
    Serial.println(pay_para_ID[8],HEX);

    strcpy_P(buffer, (char*)pgm_read_word(&(pay_cmd_table[11]))); 
    Serial.println( buffer );
    delayMicroseconds(SERIAL_PAUSE);
    
    delay(10);  // Delay to let text display 
}  
/*----------------------------------------------------*/

/*----------------------------------------------------*/
uint8_t get_para_ID(void)
{
  unsigned char i = 0;
  unsigned char value_array[3] = {0,0,0};
  unsigned char valid = 0;
  uint8_t value = 0;
  
 
  while(valid != 0xFF)          // Keep looping until valid data received
  {
    // Ask parameter ID to write
    strcpy_P(buffer, (char*)pgm_read_word(&(pay_cmd_table[1]))); 
    Serial.println( buffer );
    delayMicroseconds(SERIAL_PAUSE);
    
    strcpy_P(buffer, (char*)pgm_read_word(&(pay_cmd_table[14]))); 
    Serial.println( buffer );
    delayMicroseconds(SERIAL_PAUSE);
    
    valid = 0x00;
    
    Serial.flush();

    while(Serial.available() == 0)
      ;
    
    delay(10);
    
    // Must enter 2 HEX digits for an uint8_t
    if(Serial.available()>=2)
    {
      for(i=0;i<=1;i++)  // check each digit to ensure it is an ascii code for a valid hex digit then convert ascii code to hex
      {
        value_array[i] = Serial.read();
        if(value_array[i]>0x2F && value_array[i]<0x3A)
        {
          value_array[i] = value_array[i]-0x30;    //convert ASCII 0-9 to HEX
          valid = valid|1<<i;
        } 
        else if(value_array[i]>0x40 && value_array[i]<0x47)
        {
          value_array[i] = value_array[i]-0x37;    //convert ASCII A-F to HEX
          valid = valid|1<<i;
        }
        else if(value_array[i]>0x60 && value_array[i]<0x67)
        {
          value_array[i] = value_array[i]-0x57;    //convert ASCII a-f to HEX
          valid = valid|1<<i;
        }
        else
        {
          Serial.println("NOT HEX");
        } 
        
      }
      
      valid = valid|0xFC;  // mask top 6 bits of valid, only checking 2 digits
    }
    else
    {
      // Display error Message
      strcpy_P(buffer, (char*)pgm_read_word(&(error_table[3]))); 
      Serial.println( buffer );
      delayMicroseconds(SERIAL_PAUSE);
    }
  }
  
  value = value|value_array[0];
  value = value<<4;                  // only bit shift four since value_array[] is a char with only 1 HEX digit in lower 4 bits
  value = value|value_array[1];

  
  value = value&0xFF;
  
  Serial.println(value,HEX);
     
  return value;

}
/*----------------------------------------------------*/

/*----------------------------------------------------*/
int get_para_value(void)
{
  unsigned char i = 0;
  unsigned char value_array[5] = {0,0,0,0,0};
  unsigned char valid = 0;
  unsigned int value = 0;
  
 
  while(valid != 0xFF)          // Keep looping until valid data received
  {
    // Get value to write
    strcpy_P(buffer, (char*)pgm_read_word(&(pay_cmd_table[2]))); 
    Serial.println( buffer );
    delayMicroseconds(SERIAL_PAUSE);
    
    strcpy_P(buffer, (char*)pgm_read_word(&(pay_cmd_table[12]))); 
    Serial.println( buffer );
    delayMicroseconds(SERIAL_PAUSE);
    
    valid = 0x00;
    
    Serial.flush();

    while(Serial.available() == 0)
      ;
    
    delay(10);
    
    // Must enter 4 HEX digits for an int
    if(Serial.available()>=4)
    {
      for(i=0;i<=3;i++)  // check each digit to ensure it is an ascii code for a valid hex digit then convert ascii code to hex
      {
        value_array[i] = Serial.read();
        if(value_array[i]>0x2F && value_array[i]<0x3A)
        {
          value_array[i] = value_array[i]-0x30;    //convert ASCII 0-9 to HEX
          valid = valid|1<<i;
        } 
        else if(value_array[i]>0x40 && value_array[i]<0x47)
        {
          value_array[i] = value_array[i]-0x37;    //convert ASCII A-F to HEX
          valid = valid|1<<i;
        }
        else if(value_array[i]>0x60 && value_array[i]<0x67)
        {
          value_array[i] = value_array[i]-0x57;    //convert ASCII a-f to HEX
          valid = valid|1<<i;
        }
        else
        {
          Serial.println("NOT HEX");
        } 
        
      }
      
      valid = valid|0xF0;  // mask top four bits of valid, only checking 4 digits
    }
    else
    {
      // Display error Message
      strcpy_P(buffer, (char*)pgm_read_word(&(error_table[3]))); 
      Serial.println( buffer );
      delayMicroseconds(SERIAL_PAUSE);
    }
  }
  
  value = value|value_array[0];
  value = value<<4;                  // only bit shift four since value_array[] is a char with only 1 HEX digit in lower 4 bits
  value = value|value_array[1];
  value = value<<4;                  // only bit shift four since value_array[] is a char with only 1 HEX digit in lower 4 bits
  value = value|value_array[2];
  value = value<<4;                  // only bit shift four since value_array[] is a char with only 1 HEX digit in lower 4 bits
  value = value|(value_array[3]);
  
  value = value&0xFFFF;
  
  Serial.println(value,HEX);
     
  return value;

}
/*----------------------------------------------------*/


/*----------------------------------------------------*/
// Get serial input for desired mode
uint8_t get_desired_mode(void)
{
  unsigned char i = 0;
  unsigned char value_array[3] = {0,0,0};
  unsigned char valid = 0;
  uint8_t value = 0;

  while(valid != 0xFF)          // Keep looping until valid data received
  {
    // Get value to write
    strcpy_P(buffer, (char*)pgm_read_word(&(pay_cmd_table[13]))); 
    Serial.println( buffer );
    delayMicroseconds(SERIAL_PAUSE);
    
    strcpy_P(buffer, (char*)pgm_read_word(&(pay_cmd_table[14]))); 
    Serial.println( buffer );
    delayMicroseconds(SERIAL_PAUSE);
    
    valid = 0x00;
    
    Serial.flush();

    while(Serial.available() == 0)
      ;
    
    delay(10);
    
    if(Serial.available()>=2)
    {
      for(i=0;i<=1;i++)
      {
        value_array[i] = Serial.read();
        if(value_array[i]>0x2F && value_array[i]<0x3A)
        {
          value_array[i] = value_array[i]-0x30;    //convert ASCII 0-9 to HEX
          valid = valid|1<<i;
        } 
        else if(value_array[i]>0x40 && value_array[i]<0x47)
        {
          value_array[i] = value_array[i]-0x37;    //convert ASCII A-F to HEX
          valid = valid|1<<i;
        }
        else if(value_array[i]>0x60 && value_array[i]<0x67)
        {
          value_array[i] = value_array[i]-0x57;    //convert ASCII a-f to HEX
          valid = valid|1<<i;
        }
        else
        {
          Serial.println("NOT HEX");
        } 
        
      }
      
      valid = valid|0xFC;  // mask top four bits of valid
    }
    else
    {
      // Display error Message
      strcpy_P(buffer, (char*)pgm_read_word(&(error_table[3]))); 
      Serial.println( buffer );
      delayMicroseconds(SERIAL_PAUSE);
    }
  }
  
  value = value|value_array[0];
  value = value<<4;                  // only bit shift four since value_array[] is a char with only 1 HEX digit in lower 4 bits
  value = value|(value_array[1]);
  
  Serial.println(value,HEX);
  
  return value;  

}

/*----------------------------------------------------*/


/*----------------------------------------------------*/
// Get serial input for desired mode
int get_desired_flag(void)
{
  unsigned char i = 0;
  unsigned char value_array[5] = {0,0,0,0,0};
  unsigned char valid = 0;
  unsigned int value = 0;

  while(valid != 0xFF)          // Keep looping until valid data received
  {
    // Get value to write
    strcpy_P(buffer, (char*)pgm_read_word(&(pay_cmd_table[15]))); 
    Serial.println( buffer );
    delayMicroseconds(SERIAL_PAUSE);
    
    strcpy_P(buffer, (char*)pgm_read_word(&(pay_cmd_table[12]))); 
    Serial.println( buffer );
    delayMicroseconds(SERIAL_PAUSE);
    
    valid = 0x00;
    
    Serial.flush();

    while(Serial.available() == 0)
      ;
    
    delay(10);
    
    if(Serial.available()>=4)
    {
      for(i=0;i<=3;i++)
      {
        value_array[i] = Serial.read();
        if(value_array[i]>0x2F && value_array[i]<0x3A)
        {
          value_array[i] = value_array[i]-0x30;    //convert ASCII 0-9 to HEX
          valid = valid|1<<i;
        } 
        else if(value_array[i]>0x40 && value_array[i]<0x47)
        {
          value_array[i] = value_array[i]-0x37;    //convert ASCII A-F to HEX
          valid = valid|1<<i;
        }
        else if(value_array[i]>0x60 && value_array[i]<0x67)
        {
          value_array[i] = value_array[i]-0x57;    //convert ASCII a-f to HEX
          valid = valid|1<<i;
        }
        else
        {
          Serial.println("NOT HEX");
        } 
        
      }
      
      valid = valid|0xF0;  // mask top four bits of valid
    }
    else
    {
      // Display error Message
      strcpy_P(buffer, (char*)pgm_read_word(&(error_table[3]))); 
      Serial.println( buffer );
      delayMicroseconds(SERIAL_PAUSE);
    }
  }
  
  value = value|value_array[0];
  value = value<<4;                  // only bit shift four since value_array[] is a char with only 1 HEX digit in lower 4 bits
  value = value|value_array[1];
  value = value<<4;                  // only bit shift four since value_array[] is a char with only 1 HEX digit in lower 4 bits
  value = value|value_array[2];
  value = value<<4;                  // only bit shift four since value_array[] is a char with only 1 HEX digit in lower 4 bits
  value = value|(value_array[3]);
  
  Serial.println(value,HEX);
  
  return value;  

}

/*----------------------------------------------------*/


/*----------------------------------------------------*/
void I2C_master_begin(void)
{
  twi_init();
}
/*----------------------------------------------------*/

/*----------------------------------------------------*/
unsigned int I2C_read(int address, unsigned int bytes) {
    
    unsigned int  num_bytes = 0;
    unsigned int i;

    for (i=0; i < RX_BUFSIZE; i++){
      rxd_buffer[i] = 0;
    ////// BORIS ///////
    // rxd_buffer[i] = txd_buffer[i];
    ///// BORIS ////////
    }

    // I2C Read
    num_bytes = I2C_master_requestFrom(I2C_address, bytes, rxd_buffer);

    return num_bytes;
};
/*----------------------------------------------------*/

/*----------------------------------------------------*/
void I2C_write(char address, unsigned int bytes) {

  // transmit buffer (blocking)
  twi_writeTo(address, txd_buffer, bytes, 1);

};
/*----------------------------------------------------*/

/*----------------------------------------------------*/
unsigned int I2C_master_requestFrom(uint8_t address, unsigned int quantity, uint8_t* rxBuffer)
{
  // clamp to buffer length
  if(quantity > RX_BUFSIZE){
    quantity = RX_BUFSIZE;
  }
  // perform blocking read into buffer
  unsigned int ret = twi_readFrom(address, rxBuffer, quantity);

  return ret;
}
/*----------------------------------------------------*/

/*----------------------------------------------------*/
uint8_t process_CRC(unsigned char TRflag, unsigned int length, unsigned int checksum)
{
  // CRC processing function. Return 1 if CRC passed
  if(calculate_CRC(TRflag, length) == checksum)
    return 1;
  else
    return 0;
}
/*----------------------------------------------------*/

/*----------------------------------------------------*/
unsigned int calculate_CRC(unsigned char TRflag, unsigned int length)
{
  unsigned int checksum = 0;
  
  if(TRflag)
  {
    // Calculate CRC based on txd buffer of 'length' bytes
    checksum = crc16(0xFFFF, txd_buffer, length);  // Generate CRC-16 CCITT
  }
  else
  {
    // Calculate CRC based on rxd buffer of 'length' bytes
    checksum = crc16(0xFFFF, rxd_buffer, length);  // Generate CRC-16 CCITT
  }
  
  return checksum;
}
/*----------------------------------------------------*/

/*----------------------------------------------------*/
// Generate CRC16 checksum
unsigned int crc16(unsigned int seed, unsigned char *buf, unsigned int len)
{
  unsigned int crc, t;
  unsigned char *p;
  crc = seed;
  p = buf;  
  
  while (len--) {
    t = ((crc >> 8) ^ *p++) & 0xff;
    crc = pgm_read_word_near(crc_lut+t)^ (crc << 8);
    }

  return crc;
}
/*----------------------------------------------------*/


/*----------------------------------------------------*/
void display_mem_remain(void)
{
  strcpy_P(buffer, (char*)pgm_read_word(&(mem_stat_table[0]))); // Header msg
  Serial.println( buffer );
  delayMicroseconds(SERIAL_PAUSE);

  strcpy_P(buffer, (char*)pgm_read_word(&(mem_stat_table[1]))); // Display priority memory status 
  Serial.print( buffer );
  delayMicroseconds(SERIAL_PAUSE);
  Serial.print(": ");
  Serial.print(priority_remain);
  Serial.print(" / ");
  Serial.println(priority_limit);

  strcpy_P(buffer, (char*)pgm_read_word(&(mem_stat_table[2]))); // Display mass memory status
  Serial.print( buffer );
  delayMicroseconds(SERIAL_PAUSE);
  Serial.print(": ");
  Serial.print(mass_mem_remain);
  Serial.print(" / ");
  Serial.println(mass_mem_limit);
  
  delay(5);  // Delay to let text display 
}
/*----------------------------------------------------*/

/*----------------------------------------------------*/
uint8_t pay_ops_initialise(void)  // Payload Operations initialise 
{
  unsigned int ack_resp_timer = 0;
  unsigned char ack = 0;
  
  payload_op_mode = 0;        // Payload Operation Mode
  payload_op_flag = 0;        // Payload Operation Flag
    
  txd_buffer[0] = 0x90;  // Payload Operations initialise
  bitSet(tx_header_flags,CRC);
  txd_buffer[1] = tx_header_flags;  // Header flags
                
  txd_buffer[2] = payload_op_mode;  // Operation mode for payload - payload provider defined
                
  txd_buffer[3] = payload_op_flag>>8;  // Payload operation flags - payload provider defined
  txd_buffer[4] = payload_op_flag;
                
  init_time = secs;                    // build init time. top bits = seconds, lower 10 bits = milliseconds
  init_time = init_time<<10;
  init_time = init_time | (0x03FF | millisecs);
  
  txd_buffer[5] = byte(init_time>>24);    // send as 4 bytes. MSB sent first
  txd_buffer[6] = byte(init_time>>16);
  txd_buffer[7] = byte(init_time>>8);
  txd_buffer[8] = byte(init_time);
                
  txd_buffer[9] = priority_limit>>8;  // Total payload priority data limit in units of 256B
  txd_buffer[10] = priority_limit;
                
  txd_buffer[11] = priority_remain>>8;  // Remaining data which will be accepted as priority within current operations
  txd_buffer[12] = priority_remain;
                
  txd_buffer[13] = byte(mass_mem_limit>>24);    // Mass memory available in platform for use by payload
  txd_buffer[14] = byte(mass_mem_limit>>16);
  txd_buffer[15] = byte(mass_mem_limit>>8);
  txd_buffer[16] = byte(mass_mem_limit);
                
  txd_buffer[17] = byte(mass_mem_remain>>24);  // Remaining mass memory in platform available for use by payload
  txd_buffer[18] = byte(mass_mem_remain>>16);
  txd_buffer[19] = byte(mass_mem_remain>>8);
  txd_buffer[20] = byte(mass_mem_remain);
                
  txd_buffer[21] = poll_freq>>8;  // Frequency at which platform will issue payload status command.
  txd_buffer[22] = poll_freq; 
                
  crc = calculate_CRC(1,23);    // calculate CRC for txd buffer of 2 bytes
  txd_buffer[23] = crc>>8;      // load CRC into txd buffer MSB first
  txd_buffer[24] = crc;
  I2C_write(I2C_address, 25); 

  rxd_buffer[0] = 0x00;
  rxd_buffer[1] = 0x00;
  rxd_buffer[2] = 0x00;
  rxd_buffer[3] = 0x00;
  rxd_buffer[4] = 0x00;
  
  delayMicroseconds(PAYLOAD_RESPONSE_TIME);  // delay to allow data to be sent and processed


  while(ack_resp_timer<2000 && ack == 0)    // wait up to 2ms for response
  {
    if(I2C_read(I2C_address,ack_length)!=ack_length)  //read 5 bytes back looking for acknowledge
    {
      strcpy_P(buffer, (char*)pgm_read_word(&(error_table[5]))); // Fewer bytes received than expected error msg 
      Serial.println( buffer );
      delayMicroseconds(SERIAL_PAUSE);
    }
    else  // if five bytes received check if they are expected ack
    {  
      crc = rxd_buffer[ack_length-2];      // read new CRC from RX buffer
      crc = crc<<8;
      crc = crc|rxd_buffer[ack_length-1];
      if (!process_CRC(0,3,crc))          // If CRC fails
      {
        strcpy_P(buffer, (char*)pgm_read_word(&(error_table[6]))); // crc fail error message
        Serial.println( buffer );
        delayMicroseconds(SERIAL_PAUSE);
        rxd_buffer[0] = 0x00;
        rxd_buffer[1] = 0x00;
        rxd_buffer[2] = 0x00;
        rxd_buffer[3] = 0x00;
        rxd_buffer[4] = 0x00;
      }   
      else // if CRC pass
      {
        if((rxd_buffer[0] == txd_buffer[0]))  // If data returned is has command code of data that was sent
        {
          if(rxd_buffer[2] == 0x7E || PAY_ADDR==0x61 ) // if acknowledge received
          {
            strcpy_P(buffer, (char*)pgm_read_word(&(gen_msg_table[3]))); // Acknowledge received 
            Serial.println( buffer );
            delayMicroseconds(SERIAL_PAUSE);
            ack = 1;
           
            previous_pay_time = (secs*1000) + millisecs;      // note time of initialisation
            bitSet(flags,UPDATE_PAYLOAD_STATUS);              // now that payload is initialised start running status updates
          }
          else if(bitRead(rxd_buffer[1],ERROR)) // if not acknowledge received should be an error code so check error bit
          {
            strcpy_P(buffer, (char*)pgm_read_word(&(error_table[4]))); // Display error at payload msg
            Serial.print( buffer );
            delayMicroseconds(SERIAL_PAUSE);
            Serial.println(rxd_buffer[2],HEX);
            delayMicroseconds(SERIAL_PAUSE);
            process_pay_error(rxd_buffer[3]);           // process error byte.
            ack = 1;
          }  
        }
        else
        {
          ;// if CRC doesnt match keep waiting
        }
      }
      delayMicroseconds(50);
      ack_resp_timer = ack_resp_timer + 50; // count approximate uSec each iteration of loop
    }
  }

  if (!ack)
  {
    strcpy_P(buffer, (char*)pgm_read_word(&(error_table[11]))); // no ack within timeout
    Serial.println( buffer );
    delayMicroseconds(SERIAL_PAUSE); 
  }
  
  return ack;
}
/*----------------------------------------------------*/


/*----------------------------------------------------*/
void pay_ops_status(void)      // Payload Operations status 
{
  unsigned int crc = 0;
  unsigned int tx_pkt_length = 0;
  unsigned int rx_pkt_length = 0;
  unsigned char error_byte = 0;
  uint8_t  i = 0, j = 0;
  
  txd_buffer[0] = 0x91;    // Payload Operations Status
  txd_buffer[1] = tx_header_flags;

  bitSet(tx_header_flags,CRC);
  txd_buffer[1] = tx_header_flags;
  crc = calculate_CRC(1,2);    // calculate CRC for txd buffer of 2 bytes
  txd_buffer[2] = crc>>8;      // load CRC into txd buffer MSB first
  txd_buffer[3] = crc;
  tx_pkt_length = 4;                      // 2 bytes header, 2 bytes CRC

  I2C_write(I2C_address, tx_pkt_length);
  delayMicroseconds(PAYLOAD_RESPONSE_TIME);                // Delay to allow payload to prepare response

  if(I2C_read(I2C_address,22)!=22)
  {
    strcpy_P(buffer, (char*)pgm_read_word(&(error_table[5]))); // Fewer bytes received than expected error msg 
    Serial.println( buffer );
    delayMicroseconds(SERIAL_PAUSE);
  }
     
  // Process CRC bit but dont copy to header_flags variable yet. need to check CRC first before storing any received data
  if(bitRead(rxd_buffer[1],ERROR))  // If ERROR bit set
  {
    crc = rxd_buffer[3];           // CRC in bytes 3 and 4 or rx data
    crc = crc<<8;
    crc = crc|rxd_buffer[4];
    rx_pkt_length = 5;
  }
  else
  {
    crc = rxd_buffer[20];           // CRC in last two bytes of rx data
    crc = crc<<8;   
    crc = crc|rxd_buffer[21];   
    rx_pkt_length = 22;
  }
  
  if (!process_CRC(0,rx_pkt_length-2,crc))          // If CRC failsca
  {
    bitSet(tx_header_flags,CRC);
    txd_buffer[0] = 0x91;      //
    txd_buffer[1] = tx_header_flags;
    bitSet(txd_buffer[1],ERROR);
    txd_buffer[2] = 0x01;      // prepare error packet
    crc = calculate_CRC(1, 3);
    txd_buffer[3] = crc>>8;
    txd_buffer[4] = crc;  
    tx_pkt_length = 5;
 
    I2C_write(I2C_address, tx_pkt_length);  // send error packet
    
    strcpy_P(buffer, (char*)pgm_read_word(&(error_table[6]))); // crc fail error message
    Serial.println( buffer );
    delayMicroseconds(SERIAL_PAUSE);
    return;  // Don't process received data
  }
// End of CRC checking. If CRC failed we will have returned by now
    
  rx_header_flags = rxd_buffer[1];    // now we know data is valid (or we dont care - no CRC)
      
  if(bitRead(rx_header_flags,ERROR))  // now process error flag
  {
    //if error present acknowledge receipt of error packet
    bitSet(tx_header_flags,CRC);
    txd_buffer[0] = 0x91;      //
    txd_buffer[1] = tx_header_flags;
    txd_buffer[2] = 0x7E;      // prepare acknowledge packet
    crc = calculate_CRC(1, 3);
    txd_buffer[3] = crc>>8;
    txd_buffer[4] = crc;  
    tx_pkt_length = 5;
 
    I2C_write(I2C_address, tx_pkt_length);  // send ack packet
    
    strcpy_P(buffer, (char*)pgm_read_word(&(error_table[4]))); // Error condition at payload msg 
    Serial.print( buffer );
    delayMicroseconds(SERIAL_PAUSE); 
    Serial.println(rxd_buffer[2],HEX);
    delayMicroseconds(SERIAL_PAUSE);
                   
    error_byte = rxd_buffer[2];    // store error byte
       
    process_pay_error(error_byte);           // process error byte.
        
    bitClear(rx_header_flags,ERROR);  // clear error flag once processed
                            
  }
  else  // if no error present...
  {
    // Return acknowledgement of received status

    bitSet(tx_header_flags,CRC);
    txd_buffer[0] = 0x91;      //
    txd_buffer[1] = tx_header_flags;
    txd_buffer[2] = 0x7E;      // prepare acknowledge packet
    crc = calculate_CRC(1, 3);
    txd_buffer[3] = crc>>8;
    txd_buffer[4] = crc;   
    tx_pkt_length = 5;
 
    I2C_write(I2C_address, tx_pkt_length);  // send ack packet

    // once acknowledged reception to payload start updating stored payload status.
    payload_op_mode = rxd_buffer[2];      // Payload Operation Mode

    payload_op_flag = rxd_buffer[3];   // Payload Operation Flag 
    payload_op_flag = payload_op_flag<<8;
    payload_op_flag = payload_op_flag|rxd_buffer[4];
    
    priority_waiting = rxd_buffer[5];   // Payload Priority Data Waiting
    priority_waiting = priority_waiting<<8;
    priority_waiting = priority_waiting|rxd_buffer[6]; 
    
    data_waiting = rxd_buffer[7];   // Payload Data Waiting
    data_waiting = data_waiting<<8;
    data_waiting = data_waiting|rxd_buffer[8]; 
    data_waiting = data_waiting<<8;
    data_waiting = data_waiting|rxd_buffer[9]; 
    data_waiting = data_waiting<<8;
    data_waiting = data_waiting|rxd_buffer[10]; 
                
    pay_request = rxd_buffer[11];    // Payload Request Flags; bit0 = payload update, bit1 = next mode, bit2 = disable compression, bit3 = stream ready, bit6 = reset payload, bit7 = shutdown payload

    for(i = 0;i<8;i++)  // up to 8 para ID updates can be requested through status command
    {
      if(rxd_buffer[i+12] != 0)  // if parameter ID is not 0 this is requesting an update
      {
        for(j=0;j<16;j++)
        {
          if(pay_para_ID[j] == rxd_buffer[i+12])  // find out which of the 16 IDs on the emulator have been requested for update
          {
            bitSet(pay_para_ID_flags, j);  // set flag to indicate which IDs need update
          }
        }
      }
    }

    display_pay_status();
  }
}
/*----------------------------------------------------*/


/*----------------------------------------------------*/
void pay_ops_update(void)      // Payload Operations update 
{
  unsigned int ack_resp_timer = 0;
  unsigned char ack = 0;
  unsigned int tx_pkt_length = 0;
  
  if(!bitRead(flags, AUTO_EMU))  // If not operating in fully automatic mode display manual menu
  {
    payload_op_mode = get_desired_mode();        // Payload Operation Mode
    payload_op_flag = get_desired_flag();        // Payload Operation Mode
  }
    
  txd_buffer[0] = 0x92;    // Payload Operation Update
  txd_buffer[1] = tx_header_flags;
                
  txd_buffer[2] = payload_op_mode;  // Operation mode for payload - payload provider defined
                
  txd_buffer[3] = payload_op_flag>>8;  // Payload operation flags - payload provider defined
  txd_buffer[4] = payload_op_flag;
                
  init_time = secs;                    // build init time. top bits = seconds, lower 10 bits = milliseconds
  init_time = init_time<<10;
  init_time = init_time | (0x03FF | millisecs);
  
  txd_buffer[5] = byte(init_time>>24);
  txd_buffer[6] = byte(init_time>>16);
  txd_buffer[7] = byte(init_time>>8);
  txd_buffer[8] = byte(init_time);
                
  txd_buffer[9] = priority_limit>>8;  // Total payload priority data limit in units of 256B
  txd_buffer[10] = priority_limit;
                
  txd_buffer[11] = priority_remain>>8;  // Remaining data which will be accepted as priority within current operations
  txd_buffer[12] = priority_remain;
                
  txd_buffer[13] = byte(mass_mem_limit>>24);    // Mass memory available in platform for use by payload
  txd_buffer[14] = byte(mass_mem_limit>>16);
  txd_buffer[15] = byte(mass_mem_limit>>8);
  txd_buffer[16] = byte(mass_mem_limit);
               
  txd_buffer[17] = byte(mass_mem_limit>>24);  // Remaining mass memory in platform available for use by payload
  txd_buffer[18] = byte(mass_mem_limit>>16);
  txd_buffer[19] = byte(mass_mem_limit>>8);
  txd_buffer[20] = byte(mass_mem_limit);
                
  txd_buffer[21] = poll_freq>>8;  // Frequency at which platform will issue payload status command.
  txd_buffer[22] = poll_freq;


  bitSet(tx_header_flags,CRC);
  txd_buffer[1] = tx_header_flags;
  tx_pkt_length = 25;                      // 2 bytes header, 2 bytes CRC
  crc = calculate_CRC(1,tx_pkt_length-2);    // calculate CRC for txd buffer of 2 bytes
  txd_buffer[23] = crc>>8;      // load CRC into txd buffer MSB first
  txd_buffer[24] = crc;

  I2C_write(I2C_address, tx_pkt_length);
     
  delayMicroseconds(PAYLOAD_RESPONSE_TIME);  // delay to allow data to be sent and processed

  while(ack_resp_timer<2000 && ack == 0)    // wait up to 2ms for response
  {
      if(I2C_read(I2C_address,ack_length)!=ack_length)  //read 5 bytes back looking for CRC acknowledge
      {
        strcpy_P(buffer, (char*)pgm_read_word(&(error_table[5]))); // Fewer bytes received than expected error msg 
        Serial.println( buffer );
        delayMicroseconds(SERIAL_PAUSE);
      }
      else  // if five bytes received check if they are expected ack
      {
      crc = rxd_buffer[ack_length-2];      // read new CRC from RX buffer
      crc = crc<<8;
      crc = crc|rxd_buffer[ack_length-1];
      if (!process_CRC(0,3,crc))          // If CRC fails
      {
        strcpy_P(buffer, (char*)pgm_read_word(&(error_table[6]))); // crc fail error message
        Serial.println( buffer );
        delayMicroseconds(SERIAL_PAUSE);
        rxd_buffer[0] = 0x00;
        rxd_buffer[1] = 0x00;
        rxd_buffer[2] = 0x00;
        rxd_buffer[3] = 0x00;
        rxd_buffer[4] = 0x00;
      }   
      else // if CRC pass
      {
        if((rxd_buffer[0] == txd_buffer[0]))  // If data returned is has command code of data that was sent
        {
          if(rxd_buffer[2] == 0x7E || PAY_ADDR==0x61) // if acknowledge received
          {
            strcpy_P(buffer, (char*)pgm_read_word(&(gen_msg_table[3]))); // Acknowledge received 
            Serial.println( buffer );
            delayMicroseconds(SERIAL_PAUSE);
            ack = 1;
          }
          else if(bitRead(rxd_buffer[1],ERROR)) // if not acknowledge received should be an error code so check error bit
          {
            strcpy_P(buffer, (char*)pgm_read_word(&(error_table[4]))); // Display error at payload msg
            Serial.print( buffer );
            delayMicroseconds(SERIAL_PAUSE);
            Serial.println(rxd_buffer[2],HEX);
            delayMicroseconds(SERIAL_PAUSE);
            process_pay_error(rxd_buffer[3]);           // process error byte.
            ack = 1;
          }  
        }
        else
        {
          ;// if CRC doesnt match keep waiting
        }
      }
      delayMicroseconds(50);
      ack_resp_timer = ack_resp_timer + 50; // count approximate uSec each iteration of loop
    }
  }



  if (!ack)
  {
    strcpy_P(buffer, (char*)pgm_read_word(&(error_table[11]))); // no ack within timeout
    Serial.println( buffer );
    delayMicroseconds(SERIAL_PAUSE); 
  }     
                
  bitSet(flags,UPDATE_PAYLOAD_STATUS);
}
/*----------------------------------------------------*/


/*----------------------------------------------------*/
void pay_parameter_write(void) // Payload Parameter write
{
  unsigned int ack_resp_timer = 0;
  unsigned char ack = 0;
  unsigned int tx_pkt_length = 0;
  unsigned char cmd_num2 = 0;
  unsigned int para_value = 0;
  
  txd_buffer[0] = 0x93;    // Payload Parameter Write
  txd_buffer[1] = tx_header_flags;
                  
  txd_buffer[2] = get_para_ID();  // Get parameter ID to write
                
  para_value = get_para_value();  // get parameter value to write

  txd_buffer[3] = para_value>>8;
  txd_buffer[4] = para_value;

  bitSet(tx_header_flags,CRC);
  txd_buffer[1] = tx_header_flags;
  tx_pkt_length = 7;                      // 2 bytes header, 2 bytes CRC
  crc = calculate_CRC(1,tx_pkt_length-2);    // calculate CRC for txd buffer of 4 bytes
  txd_buffer[5] = crc>>8;      // load CRC into txd buffer MSB first
  txd_buffer[6] = crc;

  I2C_write(I2C_address, tx_pkt_length);

  delayMicroseconds(PAYLOAD_RESPONSE_TIME);  // delay to allow data to be sent and processed

  while(ack_resp_timer<2000 && ack == 0)    // wait up to 2ms for response
  {
      if(I2C_read(I2C_address,ack_length)!=ack_length)  //read 5 bytes back looking for CRC acknowledge
      {
        strcpy_P(buffer, (char*)pgm_read_word(&(error_table[5]))); // Fewer bytes received than expected error msg 
        Serial.println( buffer );
        delayMicroseconds(SERIAL_PAUSE);
      }
      else  // if five bytes received check if they are expected ack
      {
      crc = rxd_buffer[ack_length-2];      // read new CRC from RX buffer
      crc = crc<<8;
      crc = crc|rxd_buffer[ack_length-1];
      if (!process_CRC(0,3,crc))          // If CRC fails
      {
        strcpy_P(buffer, (char*)pgm_read_word(&(error_table[6]))); // crc fail error message
        Serial.println( buffer );
        delayMicroseconds(SERIAL_PAUSE);
        rxd_buffer[0] = 0x00;
        rxd_buffer[1] = 0x00;
        rxd_buffer[2] = 0x00;
        rxd_buffer[3] = 0x00;
        rxd_buffer[4] = 0x00;
      }   
      else // if CRC pass
      {
        if((rxd_buffer[0] == txd_buffer[0]))  // If data returned is has command code of data that was sent
        {
          if(rxd_buffer[2] == 0x7E || PAY_ADDR == 0x61) // if acknowledge received
          {
            strcpy_P(buffer, (char*)pgm_read_word(&(gen_msg_table[3]))); // Acknowledge received 
            Serial.println( buffer );
            delayMicroseconds(SERIAL_PAUSE);
            ack = 1;
          }
          else if(bitRead(rxd_buffer[1],ERROR)) // if not acknowledge received should be an error code so check error bit
          {
            strcpy_P(buffer, (char*)pgm_read_word(&(error_table[4]))); // Display error at payload msg
            Serial.print( buffer );
            delayMicroseconds(SERIAL_PAUSE);
            Serial.println(rxd_buffer[2],HEX);
            delayMicroseconds(SERIAL_PAUSE);
            process_pay_error(rxd_buffer[3]);           // process error byte.
            ack = 1;
          }  
        }
        else
        {
          ;// if CRC doesnt match keep waiting
        }
      }
      delayMicroseconds(50);
      ack_resp_timer = ack_resp_timer + 50; // count approximate uSec each iteration of loop
    }
  }
  
  if (!ack)
  {
    strcpy_P(buffer, (char*)pgm_read_word(&(error_table[11]))); // no ack within timeout
    Serial.println( buffer );
    delayMicroseconds(SERIAL_PAUSE); 
  }                   
}
/*----------------------------------------------------*/

/*----------------------------------------------------*/
void auto_pay_parameter_write(uint8_t paraID, unsigned int para_value) // Payload Parameter write
{
  unsigned int ack_resp_timer = 0;
  unsigned char ack = 0;
  unsigned int tx_pkt_length = 0;
  unsigned char cmd_num2 = 0;

  
  txd_buffer[0] = 0x93;    // Payload Parameter Write
  txd_buffer[1] = tx_header_flags;
              
 
  txd_buffer[2] = paraID;

  txd_buffer[3] = para_value>>8;
  txd_buffer[4] = para_value;

  bitSet(tx_header_flags,CRC);
  txd_buffer[1] = tx_header_flags;
  tx_pkt_length = 7;
  crc = calculate_CRC(1,tx_pkt_length-2);    // calculate CRC for txd buffer
  txd_buffer[5] = crc>>8;      // load CRC into txd buffer MSB first
  txd_buffer[6] = crc;

  I2C_write(I2C_address, tx_pkt_length);
 
  delayMicroseconds(PAYLOAD_RESPONSE_TIME);  // delay to allow data to be sent and processed

  while(ack_resp_timer<2000 && ack == 0)    // wait up to 2ms for response
  {
      if(I2C_read(I2C_address,ack_length)!=ack_length)  //read 5 bytes back looking for CRC acknowledge
      {
        strcpy_P(buffer, (char*)pgm_read_word(&(error_table[5]))); // Fewer bytes received than expected error msg 
        Serial.println( buffer );
        delayMicroseconds(SERIAL_PAUSE);
      }
      else  // if five bytes received check if they are expected ack
      {
      crc = rxd_buffer[ack_length-2];      // read new CRC from RX buffer
      crc = crc<<8;
      crc = crc|rxd_buffer[ack_length-1];
      if (!process_CRC(0,3,crc))          // If CRC fails
      {
        strcpy_P(buffer, (char*)pgm_read_word(&(error_table[6]))); // crc fail error message
        Serial.println( buffer );
        delayMicroseconds(SERIAL_PAUSE);
        rxd_buffer[0] = 0x00;
        rxd_buffer[1] = 0x00;
        rxd_buffer[2] = 0x00;
        rxd_buffer[3] = 0x00;
        rxd_buffer[4] = 0x00;
      }   
      else // if CRC pass
      {
        if((rxd_buffer[0] == txd_buffer[0]))  // If data returned is has command code of data that was sent
        {
          if(rxd_buffer[2] == 0x7E || PAY_ADDR == 0x61) // if acknowledge received
          {
            strcpy_P(buffer, (char*)pgm_read_word(&(gen_msg_table[3]))); // Acknowledge received 
            Serial.println( buffer );
            delayMicroseconds(SERIAL_PAUSE);
            ack = 1;
          }
          else if(bitRead(rxd_buffer[1],ERROR)) // if not acknowledge received should be an error code so check error bit
          {
            strcpy_P(buffer, (char*)pgm_read_word(&(error_table[4]))); // Display error at payload msg
            Serial.print( buffer );
            delayMicroseconds(SERIAL_PAUSE);
            Serial.println(rxd_buffer[2],HEX);
            delayMicroseconds(SERIAL_PAUSE);
            process_pay_error(rxd_buffer[3]);           // process error byte.
            ack = 1;
          }  
        }
        else
        {
          ;// if CRC doesnt match keep waiting
        }
      }
      delayMicroseconds(50);
      ack_resp_timer = ack_resp_timer + 50; // count approximate uSec each iteration of loop
    }
  }
  
  if (!ack)
  {
    Serial.println("?");
    delayMicroseconds(SERIAL_PAUSE);    
    strcpy_P(buffer, (char*)pgm_read_word(&(error_table[11]))); // no ack within timeout
    Serial.println( buffer );
    delayMicroseconds(SERIAL_PAUSE); 
  }                               
}
/*----------------------------------------------------*/

 
/*----------------------------------------------------*/
void pay_parameter_read(void)  // Payload Parameter read 
{
  unsigned char error_byte = 0;
  unsigned int tx_pkt_length = 0;
  unsigned int rx_pkt_length = 0;
  uint8_t cmd_num2 = 0 ;
  uint8_t recognised = 0;
  uint8_t i = 0;
  
  txd_buffer[0] = 0x94;    // Payload Parameter Read 
  txd_buffer[1] = tx_header_flags;
  
  txd_buffer[2] = get_para_ID();  // Get parameter ID to read
  
  bitSet(tx_header_flags,CRC);
  txd_buffer[1] = tx_header_flags;
  tx_pkt_length = 5;                      
  crc = calculate_CRC(1,tx_pkt_length-2);    // calculate CRC for txd buffer
  txd_buffer[3] = crc>>8;      // load CRC into txd buffer MSB first
  txd_buffer[4] = crc;

  I2C_write(I2C_address, tx_pkt_length);
   
  delayMicroseconds(PAYLOAD_RESPONSE_TIME);

  if(I2C_read(I2C_address,7)!=7)
  {
    strcpy_P(buffer, (char*)pgm_read_word(&(error_table[5]))); 
    Serial.println( buffer );
    delayMicroseconds(SERIAL_PAUSE);
  }
  
  // Process CRC bit but dont copy to header_flags variable yet. need to check CRC first before storing any received data
  if(bitRead(rxd_buffer[1],ERROR))  // If ERROR bit set
  {
    crc = rxd_buffer[3];           // CRC in bytes 3 and 4 or rx data
    crc = crc<<8;
    crc = crc|rxd_buffer[4];
    rx_pkt_length = 5;
  }
  else
  {
    crc = rxd_buffer[5];           // CRC in last two bytes of rx data
    crc = crc<<8;   
    crc = crc|rxd_buffer[6];   
    rx_pkt_length = 7;
  }
    
  if (!process_CRC(0,rx_pkt_length-2,crc))          // If CRC fails
  {
    bitSet(tx_header_flags,CRC);
    txd_buffer[0] = 0x94;      //
    txd_buffer[1] = tx_header_flags;
    bitSet(txd_buffer[1],ERROR);
    txd_buffer[2] = 0x01;      // prepare error packet
    crc = calculate_CRC(1, 3);
    txd_buffer[3] = crc>>8;
    txd_buffer[4] = crc;  
    tx_pkt_length = 5;
 
    I2C_write(I2C_address, tx_pkt_length);  // send error packet
    
    strcpy_P(buffer, (char*)pgm_read_word(&(error_table[6]))); // crc fail error message
    Serial.println( buffer );
    delayMicroseconds(SERIAL_PAUSE);
    return;  // Don't process received data
  }// End of CRC checking. If CRC failed we will have returned by now
  
  rx_header_flags = rxd_buffer[1];    //process header flags
  
  if(bitRead(rx_header_flags,ERROR))  // now process error flag
  {
    //if error present acknowledge receipt of error packet
    bitSet(tx_header_flags,CRC);
    txd_buffer[0] = 0x94;      //
    txd_buffer[1] = tx_header_flags;
    txd_buffer[2] = 0x7E;      // prepare acknowledge packet
    crc = calculate_CRC(1, 3);
    txd_buffer[3] = crc>>8;
    txd_buffer[4] = crc;  
    tx_pkt_length = 5;
 
    I2C_write(I2C_address, tx_pkt_length);  // send ack packet
    
    strcpy_P(buffer, (char*)pgm_read_word(&(error_table[4]))); // Error condition at payload msg 
    Serial.print( buffer );
    delayMicroseconds(SERIAL_PAUSE); 
    Serial.println(rxd_buffer[2],HEX);
    delayMicroseconds(SERIAL_PAUSE);
                   
    error_byte = rxd_buffer[2];    // store error byte
       
    process_pay_error(error_byte);           // process error byte.
        
    bitClear(rx_header_flags,ERROR);  // clear error flag once processed
                            
  }
  else  // if no error present...
  {
    // Return acknowledgement of received status

    bitSet(tx_header_flags,CRC);
    txd_buffer[0] = 0x94;      //
    txd_buffer[1] = tx_header_flags;
    txd_buffer[2] = 0x7E;      // prepare acknowledge packet
    crc = calculate_CRC(1, 3);
    txd_buffer[3] = crc>>8;
    txd_buffer[4] = crc;   
    tx_pkt_length = 5;
 
    I2C_write(I2C_address, tx_pkt_length);  // send ack packet

    // once acknowledged reception to payload store received value in appropriate register and display

    for(i=0;i<16;i++)
    {
      if(pay_para_ID[i] == rxd_buffer[2])
      {
 //       pay_para_value[i] = rxd_buffer[3];
 //       pay_para_value[i] = pay_para_value[i]<<8;
 //       pay_para_value[i] = pay_para_value[i]|rxd_buffer[4];
        recognised++;
      }
    }
    
    if(recognised == 0)
    {
      strcpy_P(buffer, (char*)pgm_read_word(&(error_table[1]))); 
      Serial.println( buffer );
      delayMicroseconds(SERIAL_PAUSE);
    }
    
    display_pay_read();    // print received data
  }

                
}
/*----------------------------------------------------*/


/*----------------------------------------------------*/
void pay_pri_data_trans(void)  // Payload Priority Data Transfer 
{
  unsigned int crc = 0;
  unsigned int tx_pkt_length = 0;
  unsigned int rx_pkt_length = 0;
  unsigned char error_byte = 0;
  unsigned char tries = 0, valid = 0;
                
  rx_pkt_length = PRIORITY_I2C_BYTES+4;   // 2 bytes header, PRIORITY_I2C_BYTES bytes date, 2 bytes CRC. Always assume max possible length
    
  if(priority_remain>PRIORITY_I2C_BYTES)    // only send request for data if there is enough space to store it. dont store cmd or crcs
  {
    while((tries < 3) && (valid == 0))  // attempt 3 times or until data received passes CRC.
    {
      txd_buffer[0] = 0x95;    // Payload Priority Data Transfer 
      txd_buffer[1] = tx_header_flags;
      bitSet(tx_header_flags,CRC);
      txd_buffer[1] = tx_header_flags;
      tx_pkt_length = 4;                      // 2 bytes header, 2 bytes CRC
      crc = calculate_CRC(1,tx_pkt_length-2);    // calculate CRC for txd buffer of 2 bytes
      txd_buffer[2] = crc>>8;      // load CRC into txd buffer MSB first
      txd_buffer[3] = crc;
      
      I2C_write(I2C_address, tx_pkt_length);    // transmit request for data
      delayMicroseconds(PAYLOAD_RESPONSE_TIME);                // expect response within 2mSec
      
      if(I2C_read(I2C_address,rx_pkt_length)!= rx_pkt_length)    // read data back, if fewer bytes than expected maximum display message but dont discard packet yet. may be an error packet or have no CRC
      {
        strcpy_P(buffer, (char*)pgm_read_word(&(error_table[5]))); // fewer bytes read than requested error msg. 
        Serial.println( buffer );
        delayMicroseconds(SERIAL_PAUSE);
        tries++;
      }
      else       
      {
        // Process CRC bit but dont copy to header_flags variable yet. need to check CRC first before storing any received data
        if(bitRead(rxd_buffer[1],ERROR))  // If ERROR bit set
        {
          crc = rxd_buffer[3];           // CRC in bytes 3 and 4 or rx data
          crc = crc<<8;
          crc = crc|rxd_buffer[4];
          rx_pkt_length = 5;
        }
        else
        {
          crc = rxd_buffer[PRIORITY_I2C_BYTES+2];           // CRC in last two bytes of rx data
          crc = crc<<8;   
          crc = crc|rxd_buffer[PRIORITY_I2C_BYTES+3];   
          rx_pkt_length = PRIORITY_I2C_BYTES+4;
        }  
          
        if (!process_CRC(0,rx_pkt_length-2,crc))          // If CRC fails
        {
          bitSet(tx_header_flags,CRC);
          txd_buffer[0] = 0x95;      //
          txd_buffer[1] = tx_header_flags;
          bitSet(txd_buffer[1],ERROR);
          txd_buffer[2] = 0x01;      // prepare error packet
          crc = calculate_CRC(1, 3);
          txd_buffer[3] = crc>>8;
          txd_buffer[4] = crc;  
          tx_pkt_length = 5;
 
          I2C_write(I2C_address, tx_pkt_length);  // send error packet
    
          strcpy_P(buffer, (char*)pgm_read_word(&(error_table[6]))); // crc fail error message
          Serial.println( buffer );
          delayMicroseconds(SERIAL_PAUSE);
          tries++;         
          delay(3);  //delay before trying again
        }
        else                                                // If CRC valid
        {
          valid++;
        }
      }
    }
    
    if(valid)
    {
      // CRC passed to get to this point
      rx_header_flags = rxd_buffer[1];    //process header flags
  
      if(bitRead(rx_header_flags,ERROR))  // now process error flag
      {
        //if error present acknowledge receipt of error packet
        bitSet(tx_header_flags,CRC);
        txd_buffer[0] = 0x95;      //
        txd_buffer[1] = tx_header_flags;
        txd_buffer[2] = 0x7E;      // prepare acknowledge packet
        crc = calculate_CRC(1, 3);
        txd_buffer[3] = crc>>8;
        txd_buffer[4] = crc;  
        tx_pkt_length = 5;
 
        I2C_write(I2C_address, tx_pkt_length);  // send ack packet
    
        strcpy_P(buffer, (char*)pgm_read_word(&(error_table[4]))); // Error condition at payload msg 
        Serial.print( buffer );
        delayMicroseconds(SERIAL_PAUSE); 
        Serial.println(rxd_buffer[2],HEX);
        delayMicroseconds(SERIAL_PAUSE);
                   
        error_byte = rxd_buffer[2];    // store error byte
       
        process_pay_error(error_byte);           // process error byte.
        
        bitClear(rx_header_flags,ERROR);  // clear error flag once processed
                            
      }
      else  // if no error present...
      {
        // Return acknowledgement of received status

        bitSet(tx_header_flags,CRC);
        txd_buffer[0] = 0x95;      //
        txd_buffer[1] = tx_header_flags;
        txd_buffer[2] = 0x7E;      // prepare acknowledge packet
        crc = calculate_CRC(1, 3);
        txd_buffer[3] = crc>>8;
        txd_buffer[4] = crc;   
        tx_pkt_length = 5;
 
        I2C_write(I2C_address, tx_pkt_length);  // send ack packet   

        display_receive(PRIORITY_I2C_BYTES+4, &rxd_buffer[0]);
        
        ///////////////////  BORIS  //////////////
        
         union xPosition_tag {byte xPosition_b[8]; double xPosition_fval;} xPosition_Union;    
      xPosition_Union.xPosition_b[0] = rxd_buffer[2];
      xPosition_Union.xPosition_b[1] = rxd_buffer[3];
      xPosition_Union.xPosition_b[2] = rxd_buffer[4];
      xPosition_Union.xPosition_b[3] = rxd_buffer[5];
      xPosition_Union.xPosition_b[3] = rxd_buffer[6];
      xPosition_Union.xPosition_b[3] = rxd_buffer[7]; 
      xPosition_Union.xPosition_b[3] = rxd_buffer[8]; 
      xPosition_Union.xPosition_b[3] = rxd_buffer[9];      
      xPosition = xPosition_Union.xPosition_fval * 60;
      
      union yPosition_tag {byte yPosition_b[8]; double yPosition_fval;} yPosition_Union;    
      yPosition_Union.yPosition_b[0] = rxd_buffer[10];
      yPosition_Union.yPosition_b[1] = rxd_buffer[11];
      yPosition_Union.yPosition_b[2] = rxd_buffer[12];
      yPosition_Union.yPosition_b[3] = rxd_buffer[13];
      yPosition_Union.yPosition_b[3] = rxd_buffer[14];      
      yPosition_Union.yPosition_b[3] = rxd_buffer[15]; 
      yPosition_Union.yPosition_b[3] = rxd_buffer[16];
      yPosition_Union.yPosition_b[0] = rxd_buffer[17];      
      yPosition = yPosition_Union.yPosition_fval * 60;
      
      union zPosition_tag {byte zPosition_b[8]; double zPosition_fval;} zPosition_Union;       
      zPosition_Union.zPosition_b[1] = rxd_buffer[18];
      zPosition_Union.zPosition_b[2] = rxd_buffer[19];
      zPosition_Union.zPosition_b[3] = rxd_buffer[20];
      zPosition_Union.zPosition_b[3] = rxd_buffer[21];
      zPosition_Union.zPosition_b[3] = rxd_buffer[22]; 
      zPosition_Union.zPosition_b[3] = rxd_buffer[23]; 
      zPosition_Union.zPosition_b[3] = rxd_buffer[24];
      zPosition_Union.zPosition_b[3] = rxd_buffer[25];       
      zPosition = zPosition_Union.zPosition_fval * 60;
      
      Serial.print("xPositionMaster ------>");
        Serial.print(xPosition); 
        Serial.println();
        Serial.print("yPositionMaster ------>"); 
        Serial.print(yPosition); 
        Serial.println();
        Serial.print("zPositionMaster ------>"); 
        Serial.print(zPosition); 
        Serial.println();
      
      /////////////// BORIS ///////////////////////
        
        
        priority_remain = priority_remain - PRIORITY_I2C_BYTES;  // Update counter of free space
        display_mem_remain();
        Serial.println();
      }
    }
    else      // if after 3 tries still hasnt passed CRC.
    {
      strcpy_P(buffer, (char*)pgm_read_word(&(error_table[9]))); // Stop trying error msg 
      Serial.println( buffer );
      delayMicroseconds(SERIAL_PAUSE);
    }
  }
  else      // If insufficient space to store response dont request data and display error msg
  {
    strcpy_P(buffer, (char*)pgm_read_word(&(error_table[7]))); // Priority mem full error msg 
    Serial.println( buffer );
    delayMicroseconds(SERIAL_PAUSE);
  }
}
/*----------------------------------------------------*/


/*----------------------------------------------------*/
void pay_data_trans(void)      // Payload Data Transfer 
{
  unsigned int crc = 0;
  unsigned int tx_pkt_length = 0;
  unsigned int rx_pkt_length = 0;
  unsigned char error_byte = 0;
  unsigned char tries = 0, valid = 0;
 
                
  rx_pkt_length = I2C_BYTES+4;   // 2 bytes header, PRIORITY_I2C_BYTES bytes date, 2 bytes CRC. Always assume max possible length
    
  if(priority_remain>I2C_BYTES)    // only send request for data if there is enough space to store it. dont store cmd or crcs
  {
    while((tries < 3) && (valid == 0))  // attempt 3 times or until data received passes CRC.
    {
      txd_buffer[0] = 0x96;    // Payload Priority Data Transfer 
      txd_buffer[1] = tx_header_flags;
      bitSet(tx_header_flags,CRC);
      txd_buffer[1] = tx_header_flags;
      tx_pkt_length = 4;                      // 2 bytes header, 2 bytes CRC
      crc = calculate_CRC(1,tx_pkt_length-2);    // calculate CRC for txd buffer of 2 bytes
      txd_buffer[2] = crc>>8;      // load CRC into txd buffer MSB first
      txd_buffer[3] = crc;
      
      I2C_write(I2C_address, tx_pkt_length);    // transmit request for data
      delayMicroseconds(PAYLOAD_RESPONSE_TIME);                // expect response within 2mSec
      
      if(I2C_read(I2C_address,rx_pkt_length)!= rx_pkt_length)    // read data back, if fewer bytes than expected maximum display message but dont discard packet yet. may be an error packet or have no CRC
      {
        strcpy_P(buffer, (char*)pgm_read_word(&(error_table[5]))); // fewer bytes read than requested error msg. 
        Serial.println( buffer );
        delayMicroseconds(SERIAL_PAUSE);
        tries++;
      }
      else       
      {
        // Process CRC bit but dont copy to header_flags variable yet. need to check CRC first before storing any received data
        if(bitRead(rxd_buffer[1],ERROR))  // If ERROR bit set
        {
          crc = rxd_buffer[3];           // CRC in bytes 3 and 4 or rx data
          crc = crc<<8;
          crc = crc|rxd_buffer[4];
          rx_pkt_length = 5;
        }
        else
        {
          crc = rxd_buffer[I2C_BYTES+2];           // CRC in last two bytes of rx data
          crc = crc<<8;   
          crc = crc|rxd_buffer[I2C_BYTES+3];   
          rx_pkt_length = I2C_BYTES+4;
        }  
          
        if (!process_CRC(0,rx_pkt_length-2,crc))          // If CRC fails
        {
          bitSet(tx_header_flags,CRC);
          txd_buffer[0] = 0x96;      //
          txd_buffer[1] = tx_header_flags;
          bitSet(txd_buffer[1],ERROR);
          txd_buffer[2] = 0x01;      // prepare error packet
          crc = calculate_CRC(1, 3);
          txd_buffer[3] = crc>>8;
          txd_buffer[4] = crc;  
          tx_pkt_length = 5;
 
          I2C_write(I2C_address, tx_pkt_length);  // send error packet
    
          strcpy_P(buffer, (char*)pgm_read_word(&(error_table[6]))); // crc fail error message
          Serial.println( buffer );
          delayMicroseconds(SERIAL_PAUSE);
          tries++;         
          delay(3);  //delay before trying again
        }
        else                                                // If CRC valid
        {
          valid++;
        }
      }
    }
    
    if(valid)
    {
      // CRC passed to get to this point
      rx_header_flags = rxd_buffer[1];    //process header flags
  
      if(bitRead(rx_header_flags,ERROR))  // now process error flag
      {
        //if error present acknowledge receipt of error packet
        bitSet(tx_header_flags,CRC);
        txd_buffer[0] = 0x96;      //
        txd_buffer[1] = tx_header_flags;
        txd_buffer[2] = 0x7E;      // prepare acknowledge packet
        crc = calculate_CRC(1, 3);
        txd_buffer[3] = crc>>8;
        txd_buffer[4] = crc;  
        tx_pkt_length = 5;
 
        I2C_write(I2C_address, tx_pkt_length);  // send ack packet
    
        strcpy_P(buffer, (char*)pgm_read_word(&(error_table[4]))); // Error condition at payload msg 
        Serial.print( buffer );
        delayMicroseconds(SERIAL_PAUSE); 
        Serial.println(rxd_buffer[2],HEX);
        delayMicroseconds(SERIAL_PAUSE);
                   
        error_byte = rxd_buffer[2];    // store error byte
       
        process_pay_error(error_byte);           // process error byte.
        
        bitClear(rx_header_flags,ERROR);  // clear error flag once processed
                            
      }
      else  // if no error present...
      {
        // Return acknowledgement of received status

        bitSet(tx_header_flags,CRC);
        txd_buffer[0] = 0x96;      //
        txd_buffer[1] = tx_header_flags;
        txd_buffer[2] = 0x7E;      // prepare acknowledge packet
        crc = calculate_CRC(1, 3);
        txd_buffer[3] = crc>>8;
        txd_buffer[4] = crc;   
        tx_pkt_length = 5;
 
        I2C_write(I2C_address, tx_pkt_length);  // send ack packet   

        //display_receive(I2C_BYTES+4, &rxd_buffer[0]);
        int rx_place;
//        for (rx_place = 2; rx_place < 26; rx_place = rx_place + 8) {
//          int part1 = rxd_buffer[rx_place];
//          int part2 = rxd_buffer[rx_place + 1];
//          int combined = (part1 << 4) | part2;
//          float temperatureC = (float)combined / 100.00;
//          Serial.print(combined, HEX); Serial.print(" ----> "); Serial.println(temperatureC); // print hex and temperature value on same line

                ////////////////////////////BORIS TANE /////////////////////////////////
//                
//   
//    I2C_master_requestFrom(2, 24, rxd_buffer);              // request 8 bytes from slave device #2
//    if(Serial.available())
//    {
//      int i = 0;
//      while(Serial.available())    // slave may send less than requested
//      { 
//        data[i] = I2C_read(I2C_address, 24); // receive a byte as character  
//        i++;
//      }
      
      //A union datatypes makes the byte and float elements share the same piece of memory, which enables conversion from a byte array to a float possible
      union xPosition_tag {byte xPosition_b[8]; double xPosition_fval;} xPosition_Union;    
      xPosition_Union.xPosition_b[0] = rxd_buffer[2];
      xPosition_Union.xPosition_b[1] = rxd_buffer[3];
      xPosition_Union.xPosition_b[2] = rxd_buffer[4];
      xPosition_Union.xPosition_b[3] = rxd_buffer[5];
      xPosition_Union.xPosition_b[3] = rxd_buffer[6];
      xPosition_Union.xPosition_b[3] = rxd_buffer[7]; 
      xPosition_Union.xPosition_b[3] = rxd_buffer[8]; 
      xPosition_Union.xPosition_b[3] = rxd_buffer[9];      
      xPosition = xPosition_Union.xPosition_fval * 60;
      
      union yPosition_tag {byte yPosition_b[8]; double yPosition_fval;} yPosition_Union;    
      yPosition_Union.yPosition_b[0] = rxd_buffer[10];
      yPosition_Union.yPosition_b[1] = rxd_buffer[11];
      yPosition_Union.yPosition_b[2] = rxd_buffer[12];
      yPosition_Union.yPosition_b[3] = rxd_buffer[13];
      yPosition_Union.yPosition_b[3] = rxd_buffer[14];      
      yPosition_Union.yPosition_b[3] = rxd_buffer[15]; 
      yPosition_Union.yPosition_b[3] = rxd_buffer[16];
      yPosition_Union.yPosition_b[0] = rxd_buffer[17];      
      yPosition = yPosition_Union.yPosition_fval * 60;
      
      union zPosition_tag {byte zPosition_b[8]; double zPosition_fval;} zPosition_Union;       
      zPosition_Union.zPosition_b[1] = rxd_buffer[18];
      zPosition_Union.zPosition_b[2] = rxd_buffer[19];
      zPosition_Union.zPosition_b[3] = rxd_buffer[20];
      zPosition_Union.zPosition_b[3] = rxd_buffer[21];
      zPosition_Union.zPosition_b[3] = rxd_buffer[22]; 
      zPosition_Union.zPosition_b[3] = rxd_buffer[23]; 
      zPosition_Union.zPosition_b[3] = rxd_buffer[24];
      zPosition_Union.zPosition_b[3] = rxd_buffer[25];       
      zPosition = zPosition_Union.zPosition_fval * 60;
   // }
//    twi_stop();
    
    Serial.println("xPositionMaster ------>"); Serial.println(xPosition);
        Serial.println("yPositionMaster ------>"); Serial.println(yPosition);
        Serial.println("zPositionMaster ------>"); Serial.println(zPosition);
    
    ///////////////////////////////////////////// BORIS /////////////////////////////////
 //         }
        
        mass_mem_remain = mass_mem_remain - I2C_BYTES;  // Update counter of free space
        display_mem_remain();
        
      }
    }
    else      // if after 3 tries still hasnt passed CRC.
    {
      strcpy_P(buffer, (char*)pgm_read_word(&(error_table[9]))); // Stop trying error msg 
      Serial.println( buffer );
      delayMicroseconds(SERIAL_PAUSE);
    }
  }
  else      // If insufficient space to store response dont request data and display error msg
  {
    strcpy_P(buffer, (char*)pgm_read_word(&(error_table[7]))); // Priority mem full error msg 
    Serial.println( buffer );
    delayMicroseconds(SERIAL_PAUSE);
  }
}
/*----------------------------------------------------*/


/*----------------------------------------------------*/
void pay_SPI_data_trans(void)  // Payload SPI data transfer 
{
  unsigned int crc = 0;
  unsigned int tx_pkt_length = 0;
  unsigned int rx_pkt_length = 0;
  unsigned char error_byte = 0;
  unsigned char tries = 0, valid = 0;
     
  bitSet(tx_header_flags,CRC);   //always use CRC        
  rx_pkt_length = SPI_BYTES+4;   // 2 bytes header, PRIORITY_I2C_BYTES bytes date, 2 bytes CRC. Always assume max possible length
     
  if(mass_mem_remain>(SPI_BYTES))    // only send request for data if there is enough space to store it
  {
    while((tries < 3) && (valid == 0))  // attempt 3 times or until data received passes CRC.
    {
      txd_buffer[0] = 0x97;    // Build Payload SPI Data Transfer Packet
      txd_buffer[1] = tx_header_flags;
      txd_buffer[2] = 0x00;
      tx_pkt_length = 5;
      crc = calculate_CRC(1,tx_pkt_length-2);    // calculate CRC for txd buffer of 2 bytes
      txd_buffer[3] = crc>>8;      // load CRC into txd buffer MSB first
      txd_buffer[4] = crc;

      I2C_write(I2C_address, tx_pkt_length);    // transmit request for data
  
      enable_SPI();
      delayMicroseconds(PAYLOAD_RESPONSE_TIME);                  // expect response within 2mSec
      SPI_data_receive(SPI_BYTES+4, &rxd_buffer[0]);

      // Process CRC bit but dont copy to header_flags variable yet. need to check CRC first before storing any received data
      if(bitRead(rxd_buffer[1],ERROR))  // If ERROR bit set
      {
        crc = rxd_buffer[3];           // CRC in bytes 3 and 4 or rx data
        crc = crc<<8;
        crc = crc|rxd_buffer[4];
        rx_pkt_length = 5;
      }
      else
      {
        crc = rxd_buffer[SPI_BYTES+2];           // CRC in last two bytes of rx data
        crc = crc<<8;   
        crc = crc|rxd_buffer[SPI_BYTES+3];   
        rx_pkt_length = SPI_BYTES+4;
      }
    
      if (process_CRC(0,rx_pkt_length-2,crc))          // If CRC valid
      {
          valid++;
      }
      else                                                // If CRC fails
      {
        txd_buffer[0] = 0x97;      //
        txd_buffer[1] = tx_header_flags;
        bitSet(txd_buffer[1],ERROR);
        txd_buffer[2] = 0x01;      // prepare error packet
        crc = calculate_CRC(1, 3);
        txd_buffer[3] = crc>>8;
        txd_buffer[4] = crc;  
        tx_pkt_length = 5;
        I2C_write(I2C_address, tx_pkt_length);  // send error packet 
          
        strcpy_P(buffer, (char*)pgm_read_word(&(error_table[6]))); // crc fail error message
        Serial.println( buffer );
        delayMicroseconds(SERIAL_PAUSE);
        tries++;         
        delay(3);  //delay before trying again
      }
    }
 
    if(valid)
    {
      // CRC passed to get to this point
      if(bitRead(rxd_buffer[1],ERROR))  // now process error flag
      {
        //if error present acknowledge receipt of error packet
        txd_buffer[0] = 0x97;      //
        txd_buffer[1] = tx_header_flags;
        txd_buffer[2] = 0x7E;      // prepare acknowledge packet
        crc = calculate_CRC(1, 3);
        txd_buffer[3] = crc>>8;
        txd_buffer[4] = crc;  
        tx_pkt_length = 5;
 
        I2C_write(I2C_address, tx_pkt_length);  // send ack packet
    
        strcpy_P(buffer, (char*)pgm_read_word(&(error_table[4]))); // Error condition at payload msg 
        Serial.print( buffer );
        delayMicroseconds(SERIAL_PAUSE); 
        Serial.println(rxd_buffer[2],HEX);
        delayMicroseconds(SERIAL_PAUSE);
                   
        error_byte = rxd_buffer[2];    // store error byte
       
        process_pay_error(error_byte);           // process error byte.
        
        bitClear(rx_header_flags,ERROR);  // clear error flag once processed
                            
      }
      else  // if no error present...
      {
        // Return acknowledgement of received status
        txd_buffer[0] = 0x97;      //
        txd_buffer[1] = tx_header_flags;
        txd_buffer[2] = 0x7E;      // prepare acknowledge packet
        crc = calculate_CRC(1, 3);
        txd_buffer[3] = crc>>8;
        txd_buffer[4] = crc;   
        tx_pkt_length = 5;
 
        I2C_write(I2C_address, tx_pkt_length);  // send ack packet   
    
        strcpy_P(buffer, (char*)pgm_read_word(&(cmd_confirmation_table[5]))); 
        Serial.println( buffer );
        delayMicroseconds(SERIAL_PAUSE);
        display_receive(SPI_BYTES+4, &rxd_buffer[0]);
        disable_SPI();
        mass_mem_remain = mass_mem_remain - I2C_BYTES;  // Update counter of free space
        display_mem_remain();
        Serial.println();
      }
    }
    else      // if after 3 tries still hasnt passed CRC.
    {
      strcpy_P(buffer, (char*)pgm_read_word(&(error_table[9]))); // Stop trying error msg 
      Serial.println( buffer );
      delayMicroseconds(SERIAL_PAUSE);
    }
  }
  else      // If insufficient space to store response dont request data and display error msg
  {
    strcpy_P(buffer, (char*)pgm_read_word(&(error_table[7]))); // Priority mem full error msg 
    Serial.println( buffer );
    delayMicroseconds(SERIAL_PAUSE);
  }
}
/*----------------------------------------------------*/


/*----------------------------------------------------*/
void pay_shutdn(void)         // Payload Shutdown
{
  unsigned int ack_resp_timer = 0;
  unsigned char ack = 0;
  unsigned int tx_pkt_length = 0;
  unsigned int crc = 0;
 
  txd_buffer[0] = 0x9F;    // Payload Shutdown 
  
  bitSet(tx_header_flags,CRC);
  txd_buffer[1] = tx_header_flags;
  tx_pkt_length = 4;           // 2 bytes header, 2 bytes CRC
  crc = calculate_CRC(1,tx_pkt_length-2);    // calculate CRC for txd buffer of 2 bytes
  txd_buffer[2] = crc>>8;      // load CRC into txd buffer MSB first
  txd_buffer[3] = crc;

  I2C_write(I2C_address, tx_pkt_length);
  
  delayMicroseconds(PAYLOAD_RESPONSE_TIME);  // delay to allow data to be sent and processed

  while(ack_resp_timer<2000 && ack == 0)    // wait up to 2ms for response
  {
    if(I2C_read(I2C_address,ack_length)!=ack_length)  //read 5 bytes back looking for CRC acknowledge
    {
      strcpy_P(buffer, (char*)pgm_read_word(&(error_table[5]))); // Fewer bytes received than expected error msg 
      Serial.println( buffer );
      delayMicroseconds(SERIAL_PAUSE);
    }
    else  // if five bytes received check if they are expected ack
    {
      crc = rxd_buffer[ack_length-2];      // read new CRC from RX buffer
      crc = crc<<8;
      crc = crc|rxd_buffer[ack_length-1];
      if (!process_CRC(0,3,crc))          // If CRC fails
      {
        strcpy_P(buffer, (char*)pgm_read_word(&(error_table[6]))); // crc fail error message
        Serial.println( buffer );
        delayMicroseconds(SERIAL_PAUSE);
        rxd_buffer[0] = 0x00;
        rxd_buffer[1] = 0x00;
        rxd_buffer[2] = 0x00;
        rxd_buffer[3] = 0x00;
        rxd_buffer[4] = 0x00;
      }   
      else // if CRC pass
      {
        if((rxd_buffer[0] == txd_buffer[0]))  // If data returned is has command code of data that was sent
        {  
          if(rxd_buffer[2] == 0x7E || PAY_ADDR == 0x61) // if acknowledge received
          {
            strcpy_P(buffer, (char*)pgm_read_word(&(gen_msg_table[3]))); // Acknowledge received 
            Serial.println( buffer );
            delayMicroseconds(SERIAL_PAUSE);
            ack = 1;
          }
          else if(bitRead(rxd_buffer[1],ERROR)) // if not acknowledge received should be an error code so check error bit
          {
            strcpy_P(buffer, (char*)pgm_read_word(&(error_table[4]))); // Display error at payload msg
            Serial.print( buffer );
            delayMicroseconds(SERIAL_PAUSE);
            Serial.println(rxd_buffer[2],HEX);
            delayMicroseconds(SERIAL_PAUSE);
            process_pay_error(rxd_buffer[3]);           // process error byte.
            ack = 1;
          }  
        }
        else
        {
          ;// if CRC doesnt match keep waiting
        }
      }
      delayMicroseconds(50);
      ack_resp_timer = ack_resp_timer + 50; // count approximate uSec each iteration of loop
    }
  }
 

  if (!ack)
  {
    strcpy_P(buffer, (char*)pgm_read_word(&(error_table[11]))); // no ack within timeout
    Serial.println( buffer );
    delayMicroseconds(SERIAL_PAUSE); 
  }     
  
  bitClear(flags,UPDATE_PAYLOAD_STATUS);
}
/*----------------------------------------------------*/


/*----------------------------------------------------*/
void process_pay_error(unsigned char error)
{
  ; // insert error processing once details received from payload teams.
}
/*----------------------------------------------------*/

/*----------------------------------------------------*/
void auto_mic_emulator(void)
{
  // Power down payload to enter known state
  digitalWrite(active_bus, LOW);
 
  delay(250);
  
  // Power on payload
  digitalWrite(active_bus, HIGH);     
  
  payload_init_time = (secs*1000) + millisecs;
    
  bitSet(flags, AUTO_EMU);  // Tell main loop we are operating in automatic mode  
}
/*----------------------------------------------------*/



/*----------------------------------------------------*/
uint8_t get_I2C_address(void)
{
  unsigned char i = 0;
  unsigned char value_array[3] = {0,0,0};
  unsigned char valid = 0;
  uint8_t value = 0;

  while(valid != 0xFF)          // Keep looping until valid data received
  {
    // Get value to write
    strcpy_P(buffer, (char*)pgm_read_word(&(pay_cmd_table[11]))); 
    Serial.println( buffer );
    delayMicroseconds(SERIAL_PAUSE);
    
    strcpy_P(buffer, (char*)pgm_read_word(&(pay_cmd_table[14]))); 
    Serial.println( buffer );
    delayMicroseconds(SERIAL_PAUSE);
    
    valid = 0x00;
    
    Serial.flush();

    while(Serial.available() == 0)
      ;
    
    delay(10);
    
    // Must enter 2 HEX digits for an uint8_t
    if(Serial.available()>=2)
    {
      for(i=0;i<=1;i++)  // check each digit to ensure it is an ascii code for a valid hex digit then convert ascii code to hex
      {
        value_array[i] = Serial.read();
        if(value_array[i]>0x2F && value_array[i]<0x3A)
        {
          value_array[i] = value_array[i]-0x30;    //convert ASCII 0-9 to HEX
          valid = valid|1<<i;
        } 
        else if(value_array[i]>0x40 && value_array[i]<0x47)
        {
          value_array[i] = value_array[i]-0x37;    //convert ASCII A-F to HEX
          valid = valid|1<<i;
        }
        else if(value_array[i]>0x60 && value_array[i]<0x67)
        {
          value_array[i] = value_array[i]-0x57;    //convert ASCII a-f to HEX
          valid = valid|1<<i;
        }
        else
        {
          Serial.println("NOT HEX");
        } 
        
      }
      
      valid = valid|0xFC;  // mask top six bits of valid, only checking 2 digits
    }
    else
    {
      // Display error Message
      strcpy_P(buffer, (char*)pgm_read_word(&(error_table[3]))); 
      Serial.println( buffer );
      delayMicroseconds(SERIAL_PAUSE);
    }
  }
  
  value = value|value_array[0];
  value = value<<4;                  // only bit shift four since value_array[] is a char with only 1 HEX digit in lower 4 bits
  value = value|value_array[1];

  value = value&0xFF;
  
  Serial.println(value,HEX);
     
  return value;
}
/*----------------------------------------------------*/

/*----------------------------------------------------*/ 
/* Setup Timer2 to generate 1ms interrupt */
void setup_tmr2() 
{   
   /* First disable the timer overflow interrupt while we're configuring */  
  TIMSK2 &= ~(1<<TOIE2);   
  
  /* Configure timer2 in normal mode (pure counting, no PWM etc.) */  
  TCCR2A &= ~((1<<WGM21) | (1<<WGM20));   
  TCCR2B &= ~(1<<WGM22);   
  
  /* Select clock source: internal I/O clock */  
  ASSR &= ~(1<<AS2);   
  
  /* Disable Compare Match A interrupt enable (only want overflow) */  
  TIMSK2 &= ~(1<<OCIE2A);   
  
  /* Now configure the prescaler to CPU clock divided by 128 */  
  TCCR2B |= (1<<CS22)  | (1<<CS20); // Set bits   
  TCCR2B &= ~(1<<CS21);             // Clear bit   
  
  /* We need to calculate a proper value to load the timer counter.  
   * The following loads the value 131 into the Timer 2 counter register  
   * The math behind this is:  
   * (CPU frequency) / (prescaler value) = 125000 Hz = 8us.  
   * (desired period) / 8us = 125.  
   * MAX(uint8) + 1 - 125 = 131;  
   */  
  /* Save value globally for later reload in ISR */  
  tcnt2 = 131;    
  
  /* Finally load end enable the timer */  
  TCNT2 = tcnt2;   
  TIMSK2 |= (1<<TOIE2);   
}   
/*----------------------------------------------------*/

/*----------------------------------------------------*/  
// Timer2 Interrupt Handlet
ISR(TIMER2_OVF_vect) 
{   
  /* Reload the timer */  
  TCNT2 = tcnt2;   
  
  millisecs++;
  
  if(millisecs>999) // every second increment secs
  {
    secs++;
    millisecs = 0;
    /* Write to a digital pin so that we can confirm our timer */  
    digitalWrite(2, LOW);   
  }
  else if(millisecs == 500)  // generate 1Hz sync pulse
  {
    digitalWrite(2, HIGH);   
  } 
} 
/*----------------------------------------------------*/


