
/********************************************************************************
* File Name         : UKube_Pay_Emulator_V1_05.pde
*
* Project           : UKube-1
*
* Version           : V1.05
*
* Purpose           : Provides an example scheme for the payload interface controller
*                     of the UKube-1 Cubesat. Responds to all UKube-1 comms protocol 
*                     commands over I2C bus and allows data transfer over I2C and SPI
*                     buses.
*
* Inputs            : I2C commands in UKube-1 comms protocol. Manual input from RS-232 
*                     terminal
*
* Outputs           : I2C and SPI data transmissions in accordance with UKube-1 protocol
*
* Notes             : Uses 
*
* Created by        : Alan Kane, Clyde Space Ltd, 01/2/11
*
* Last edited       : Initial Release
*
* Changelog         : V1.0     Initial Release
*                     V1.01    Changed Default I2C Slave Address to 0x71
*                     V1.02    Payload response timeout set to 2ms.
*                     V1.03    Updated coms protocol. Every command now includes an
*                              acknowledge CRC after the last data transmission
*                     V1.04    16 payload_parameter IDs available for use. ID numbers to
*                              be pre-programmed by user.
*                     V1.05    Updated to reflect version F of protocol document.
*
* DISCLAIMER        :
*
* This Firmware is provided for use with the UKube-1 Payload Emulator but may 
* be reused for other purposes within the UKube programme. No support or warranty
* will be provided by Clyde Space beyond the original use.
* 
*
*******************************************************************************/

//************* TEMPRATURE setup ************* 
#define aref_voltage 5
//TMP36 Pin Variables
int tempPin = 1;        //the analog pin the TMP36's Vout (sense) pin is connected to
                        //the resolution is 10 mV / degree centigrade with a
                        //500 mV offset to allow for negative temperatures
int tempReading;        // the analog reading from the sensor

//int temperature_array[4];  //array to store hex temperature values
int temp_counter;            //counter to keep place in the array defined on the above line
 
byte xPosition[8];  //Float 1 Array Pointer (series of bytes, 8 bytes each double)
byte yPosition[8];  //Float 2 Array Pointer (series of bytes, 8 bytes each float)
byte zPosition[8];  //Float 3 Array Pointer (series of bytes, 8 bytes each float)

double doubleData[3]; //Your float array

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

// I2C slave address
#define PAY_ADDR  0x71

// Flag bit definitions
#define SPI_write 0  // set this bit is a flag to indicate whether SPI is being used to transmit or receive.
#define ledState 1   // current state of LED
#define i2c_gone 2   // flag to indicate i2c transmission has been sent
#define initialised 3 // flag to indicate initialisation command received
#define i2c_new_cmd_rx 4  // flag to indicate new I2C command received
#define USE_TX_CRC 5 // Use CRCs on transmitted payload commands
#define USE_RX_CRC 6 // Use CRCs on received payload commands

// Buffer Sizes
#define SERIAL_RX_BUFFER 5
#define BUFSIZE   265

// Number of data bytes for I2C data transfer
#define I2C_BYTES 256
// Number of data bytes for I2C priority data transfer
#define PRIORITY_I2C_BYTES 256
// Number of data bytes for I2C priority data transfer
#define SPI_BYTES 256

// I/O Pin directions
#define SYNC_PIN 2
#define PIN_SPI_SS 10
#define PIN_SPI_MOSI 11 
#define PIN_SPI_MISO 12
#define PIN_SPI_CLK 13

#define SERIAL_PAUSE 5

// Header flags in packet definition
#define CRC 0
#define ERROR 3

// Payload request flag bit definitions
#define  PAYLOAD_UPDATE 0
#define  NEXT_MODE 1
#define  DISABLE_COMPRESSION  2
#define  STREAM_READY  3
#define  RESET_PAYLOAD  6
#define  SHUTDOWN  7

//************* Global Variables *************

// Rx and Tx buffers rxd_buffer and txd_buffer used for SPI and I2C, serial_rxd for serial comms.
uint8_t rxd_buffer[BUFSIZE];
uint8_t txd_buffer[BUFSIZE];
char serial_rxd[SERIAL_RX_BUFFER];

// Rx and Tx pointers
unsigned int received=0;
unsigned int i2c_tx_available = 0;

// Timer variables
unsigned int init_millisecs = 0;   // counter of millis
unsigned long init_secs = 0;       // counter of secs since platform power on

// Flags
uint8_t flags = 0;  // bit 0 = SPI write mode;

// Payload Information fields as defined in payload protocol and packet definition
uint8_t payload_op_mode = 0;        // Payload Operation Mode
unsigned int payload_op_flag = 0;   // Payload Operation Flag
unsigned long init_time = 0;        // Onboard time at MIC
unsigned int priority_limit = 0;    // Payload Priority Data Limit
unsigned int priority_remain = 0;   // Payload Priortiy Data Remaining
unsigned long mass_mem_limit = 0;         // Payload Mass Memory Limit
unsigned long mass_mem_remain = 0;        // Payload Mass Memory Remaining
unsigned int poll_freq = 0;         // Payload Poll Frequency
unsigned int priority_waiting = 0;  // Payload Priority Data Waiting
unsigned long data_waiting = 0;      // Payload Data Waiting
uint8_t pay_request = 0;            // Payload Request Flags; bit0 = payload update, bit1 = next mode, bit2 = disable compression, bit3 = stream ready, bit6 = reset payload, bit7 = shutdown payload
uint8_t pay_para_ID[16] = {0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
unsigned int pay_para_value[16] = {0};    // Payload Parameter value
uint8_t CRC_1 = 0, CRC_2 = 0;             // CRC bytes
uint8_t para_ID_temp = 0;

// Header flags
uint8_t header_flags = 0x00;

char buffer[45];    // Holds text loaded from flash prior to display. make sure this is large enough for the largest string it must hold

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

// Store status msg text in flash.
prog_char status_0[] PROGMEM = "<<PayLoad Connected...";   
prog_char status_1[] PROGMEM = "<<Awaiting Initialisation...";
prog_char status_2[] PROGMEM = ">>END Data Received";
prog_char status_3[] PROGMEM = ">>ACK received";
prog_char status_4[] PROGMEM = " ";
prog_char status_5[] PROGMEM = " ";

// Store payload commands text in flash.
prog_char pay_cmd_0[] PROGMEM = ">>Payload operations initialise";   
prog_char pay_cmd_1[] PROGMEM = ">>Payload operations status";
prog_char pay_cmd_2[] PROGMEM = ">>Payload operations update";
prog_char pay_cmd_3[] PROGMEM = ">>Payload parameter write";
prog_char pay_cmd_4[] PROGMEM = ">>Payload parameter read";
prog_char pay_cmd_5[] PROGMEM = ">>Priority data transfer";
prog_char pay_cmd_6[] PROGMEM = ">>Data transfer";
prog_char pay_cmd_7[] PROGMEM = ">>SPI data transfer";
prog_char pay_cmd_8[] PROGMEM = ">>Payload Shutdown";

// Store status report heading text in flash.
prog_char pay_status_0[] PROGMEM = "Payload Status";   
prog_char pay_status_1[] PROGMEM = "-------------------------";
prog_char pay_status_2[] PROGMEM = "Payload Operation Mode: ";
prog_char pay_status_3[] PROGMEM = "Payload Operation Flags: ";
prog_char pay_status_4[] PROGMEM = "Priority Data Waiting: ";
prog_char pay_status_5[] PROGMEM = "Payload Data Waiting: ";
prog_char pay_status_6[] PROGMEM = "Payload Request Flags: ";
prog_char pay_status_7[] PROGMEM = "Payload Parameter ID: ";
prog_char pay_status_8[] PROGMEM = "Value for Parameter ID ";
prog_char pay_status_9[] PROGMEM = "Payload Data Limit: ";
prog_char pay_status_10[] PROGMEM = "Payload Data Remaining: ";
prog_char pay_status_11[] PROGMEM = "Payload Priority Data Limit: ";
prog_char pay_status_12[] PROGMEM = "Payload Priority Data Remaining: ";


// Store error message text in flash.
prog_char error_0[] PROGMEM = "<<Serial Buffer Full";   
prog_char error_1[] PROGMEM = "<<Unknown Cmd";
prog_char error_2[] PROGMEM = "<<Parameter ID Unknown";
prog_char error_3[] PROGMEM = "<<Too Few Characters Entered";
prog_char error_4[] PROGMEM = "<<CRC Check Failed";
prog_char error_5[] PROGMEM = "<<No ACK received";
prog_char error_6[] PROGMEM = "";

// Store update flag menu text in flash.
prog_char flags_0[] PROGMEM = "Press any key below to set flag";   
prog_char flags_1[] PROGMEM = "0: Set Payload Update Flag";   
prog_char flags_2[] PROGMEM = "1: Set Next Mode Flag";
prog_char flags_3[] PROGMEM = "2: Set Disable Compression Flag";
prog_char flags_4[] PROGMEM = "3: +256 Bytes Data Waiting for SPI transfer";
prog_char flags_5[] PROGMEM = "4: Toggle ERROR flag in header";
prog_char flags_6[] PROGMEM = "6: Set Payload Reset Flag";
prog_char flags_7[] PROGMEM = "7: Set Payload Shutdown Flag";
prog_char flags_8[] PROGMEM = "8: Priority Data Waiting +256 Bytes";
prog_char flags_9[] PROGMEM = "9: Data Waiting +256 Bytes";
                  
// Store update flag response text in flash.
prog_char flags_resp_0[] PROGMEM = " ";   
prog_char flags_resp_1[] PROGMEM = "<<Payload Update Flag Set";   
prog_char flags_resp_2[] PROGMEM = "<<Next Mode Flag Set";
prog_char flags_resp_3[] PROGMEM = "<<Disable Compression Flag Set";
prog_char flags_resp_4[] PROGMEM = "<<+256 Bytes Data Waiting for SPI transfer";
prog_char flags_resp_5[] PROGMEM = "<<Payload Reset Flag Set";
prog_char flags_resp_6[] PROGMEM = "<<Payload Shutdown Flag Set";
prog_char flags_resp_7[] PROGMEM = "<<Priority Data Waiting +256 Bytes";
prog_char flags_resp_8[] PROGMEM = "<<Data Waiting +256 Bytes";                  
prog_char flags_resp_9[] PROGMEM = "<<ERROR flag set";                  
prog_char flags_resp_10[] PROGMEM = "<<ERROR flag clear";

PROGMEM const char *error_table[] = 	   // Table of error msgs
{   
  error_0,
  error_1,
  error_2,
  error_3,
  error_4,
  error_5,
  error_6 };
  
  
PROGMEM const char *status_msg_table[] =     // Table of status msgs
{   
  status_0,
  status_1,
  status_2,
  status_3,
  status_4,
  status_5  };
  
PROGMEM const char *pay_cmd_table[] = 	   // Payload cmd msgs
{   
  pay_cmd_0,
  pay_cmd_1,
  pay_cmd_2,
  pay_cmd_3,
  pay_cmd_4,
  pay_cmd_5,
  pay_cmd_6,
  pay_cmd_7,
  pay_cmd_8  };  

PROGMEM const char *status_table[] = 	   // Payload status update operation headings
{   
  pay_status_0,
  pay_status_1,
  pay_status_2,
  pay_status_3,
  pay_status_4,
  pay_status_5,
  pay_status_6,
  pay_status_7,
  pay_status_8,
  pay_status_9,
  pay_status_10,
  pay_status_11,
  pay_status_12 };

PROGMEM const char *flags_table[] = 	   // flag setting menu
{   
  flags_0,
  flags_1,
  flags_2,
  flags_3,
  flags_4,
  flags_5,
  flags_6,
  flags_7,
  flags_8,
  flags_9 };
  
PROGMEM const char *flags_resp_table[] = 	   // flag setting responses
{   
  flags_resp_0,
  flags_resp_1,
  flags_resp_2,
  flags_resp_3,
  flags_resp_4,
  flags_resp_5,
  flags_resp_6,
  flags_resp_7,
  flags_resp_8, 
  flags_resp_9,
  flags_resp_10};  


//************* Functions *************

/*----------------------------------------------------*/
// This is an obligatory arduino function. It performs the basic setup of the board and is run once on power on.
void setup()
{
  int i = 0;
  unsigned char temp = 0;

  pinMode(SYNC_PIN, INPUT);  

  pinMode(PIN_SPI_SS, INPUT); 
  pinMode(PIN_SPI_MOSI, INPUT); 
  pinMode(PIN_SPI_MISO, OUTPUT); 
  pinMode(PIN_SPI_CLK, INPUT); 

  digitalWrite(PIN_SPI_SS,HIGH);   // Enable weak pull-up on SS

  I2C_begin(PAY_ADDR);      // Start I2C

  // Configure use of CRCs at platform
  // for UKube use of CRCs defined by standard
  bitSet(flags,USE_TX_CRC);
  bitSet(flags,USE_RX_CRC);

  Serial.begin(115200); // Setup Serial Port 
  strcpy_P(buffer, (char*)pgm_read_word(&(status_msg_table[0]))); // Payload Connected msg
  Serial.println( buffer );
  delayMicroseconds(SERIAL_PAUSE);

  // Setup SPI 
  temp = SPSR;  // Clear WCOL and SPIF by reading SPSR then writing SPDR
  SPDR = 0x00;
  
  SPCR = 0x00;
  SPSR = 0x00;
  SPCR = (1<<SPE)|(1<<SPIE);
  
  // Clear Buffers
  for (i=0; i < SERIAL_RX_BUFFER; i++){
      serial_rxd[i] = 0x00;
  } 
  
  for(i = 0; i < BUFSIZE; i++)
  {
    txd_buffer[i] = 0;
  }
  
  clear_rxd_buf();

  bitClear(flags,initialised);

  strcpy_P(buffer, (char*)pgm_read_word(&(status_msg_table[1]))); // Awaiting initialisation msg
  Serial.println( buffer );
  delayMicroseconds(SERIAL_PAUSE);
 
  // Wait for initialisation command 
  while (bitRead(flags,initialised) == 0)
  {
    delay(1);
    
    if(rxd_buffer[0] == 0x90)
    {
      if(Process_i2c_rxd()==1)
      {
        bitSet(flags,initialised);
      }
    }
  }  

  // Display list of commands that can be entered to set flags
  for(i=0; i<10;i++)
  {
    strcpy_P(buffer, (char*)pgm_read_word(&(flags_table[i]))); // Display flag bit options. 
    Serial.println( buffer );
    delayMicroseconds(SERIAL_PAUSE);
  }
  
    I2C_begin(PAY_ADDR);
    I2C_onRequestService();
}


/*----------------------------------------------------*/

/*----------------------------------------------------*/
ISR(SPI_STC_vect)
{
  SPDR = txd_buffer[received+1];    // [received+1] because of preloading 

  if (received == BUFSIZE) 
  {
      received = 0;
  } 
  else 
  {
    if(!bitRead(flags,SPI_write))  // Do not overwrite receive buffer if write flag set
    {
     rxd_buffer[received] = SPDR;
    }
    received++;
  };  
  
}
/*----------------------------------------------------*/

/*----------------------------------------------------*/
void display_buf(volatile unsigned char *buffer_name, unsigned int length)
{
  char row = 0;
  char column = 0;
    
// Display full rows of data
  for(row = 0; row<(length/16);row++)
  {
    noInterrupts();

    for(column = 0; column < 16; column++)
    {
        Serial.print(*(buffer_name+((16*row)+column)), HEX);
        Serial.print(9,BYTE);    // Horizontal tab
    }
    Serial.println(' ');
    interrupts();
    
    delayMicroseconds(5);
  };
  
// Display any remaining data in a partial row
  noInterrupts();
  
  for(column = 0; column <length%16 ; column++)
  {
      Serial.print(*(buffer_name+((16*row)+column)), HEX);
      Serial.print(9,BYTE);    // Horizontal tab
  }

  interrupts();
  
  Serial.println(' ');
  strcpy_P(buffer, (char*)pgm_read_word(&(status_msg_table[2]))); // End data received msg 
  Serial.println( buffer );
  delayMicroseconds(SERIAL_PAUSE);
  
  // Set AREF to 5V for temperature readings
  analogReference(DEFAULT);
  temp_counter = 2;  //set counter to zero
}
/*----------------------------------------------------*/

/*----------------------------------------------------*/
void loop() 
{ 
  //tempReading = analogRead(tempPin);  
   doubleData[0] = 10;    //your code here
   doubleData[1] = 20;   //your code here
   doubleData[2] = 30;   //your code here
//  // converting that reading to voltage, which is based off the reference voltage
//  float voltage = tempReading * aref_voltage;
//  voltage /= 1024.0; 
// 
//  // now print out the temperature in degrees C
//  float temperatureC = (voltage - 0.5) * 100 ;  //converting from 10 mv per degree wit 500 mV offset
//                                               //to degrees ((volatge - 500mV) times 100)
//  //Serial.print(temperatureC); Serial.println(" degrees C");
//  
//  int whole_number = round(temperatureC * 100); //convert temp reading to a rounded whole number
//  //Serial.println(whole_number);
// 
//   //split whole number into two hexadecimal parts
//  int part2 = whole_number & 0xf;
//  int part1 = (whole_number >> 4);
//  //Serial.print(part1); Serial.print(" and "); Serial.println(part2);
//  Serial.print(temperatureC); Serial.print(" ----> "); Serial.print(part1, HEX); Serial.println(part2, HEX); //print decimal value side by side with hex value

//////////////////////// BORIS //////////////////////////////

  byte Data[24];
  
  *(&Data[0]) = doubleData[0];
  *(&Data[8]) = doubleData[1];
  *(&Data[16]) = doubleData[2];
  
//  Data[0] = xPosition[0]; 
//  Data[1] = xPosition[1]; 
//  Data[2] = xPosition[2]; 
//  Data[3] = xPosition[3];
//  Data[4] = xPosition[4];
//  Data[5] = xPosition[5];
//  Data[6] = xPosition[6];
//  Data[7] = xPosition[7]; 
//  yPosition = (byte) doubleData[1];
//  Data[8] = yPosition[0];
//  Data[9] = yPosition[1];
//  Data[10] = yPosition[2];
//  Data[11] = yPosition[3];
//  Data[12] = yPosition[4];
//  Data[13] = yPosition[5];
//  Data[14] = yPosition[6];
//  Data[15] = yPosition[7];
//  zPosition = (byte) doubleData[2];
//  Data[16] = zPosition[0]; 
//  Data[17] = zPosition[1]; 
//  Data[18] = zPosition[2]; 
//  Data[19] = zPosition[3];
//  Data[20] = zPosition[4];
//  Data[21] = zPosition[5];
//  Data[22] = zPosition[6];
//  Data[23] = zPosition[7]; 
  
//  twi_transmit(Data, 24); // Fills slave tx Buffer with Data
//  twi_releaseBus(); // Releases Bus control to the master
  
  
        Serial.print("xPositionSlave ------>");
        Serial.print(doubleData[0]); 
        Serial.println();
        Serial.print("yPositionSlave ------>"); 
        Serial.print(doubleData[1]); 
        Serial.println();
        Serial.print("zPositionSlave ------>"); 
        Serial.print(doubleData[2]); 
        Serial.println();
        
        for(int boris_counter = 0; boris_counter <= 23;  boris_counter++){
          txd_buffer[boris_counter + 2] = Data[boris_counter];
          if (boris_counter == 23)
          {
            Serial.println(" HAHAHA !!txd_buffer now full");
            delay(3000);
          }
        }
  
  ////////////////////////  BORIS /////////////////////////////
  
   // assign hex part to two consecutive places in the array
//  txd_buffer[temp_counter] = part1;
//  txd_buffer[temp_counter + 1] = part2;
  //Serial.print(txd_buffer[0], HEX); Serial.print(" and "); Serial.println(txd_buffer[1], HEX);
  // print out hex values when array if full
// if (temp_counter == 252){
//  //int place;
//  //for (place = 2; place < 255; place = place + 2) {
//  //    Serial.print(txd_buffer[place], HEX);
//  //    Serial.print(txd_buffer[place + 1], HEX);
//  //    Serial.println();
//  //   }
//  Serial.println("txd_buffer now full");
//  delay(3000);
//  temp_counter = 2;
//   }
  
//  temp_counter = temp_counter + 2;
temp_counter = temp_counter + 1;
  delay(100); // change this to change frequency of temperature readings
  
  char cmd = 0;
  
  // new i2c data received?
  if(bitRead(flags,i2c_new_cmd_rx))
  {
    if(Process_i2c_rxd()==0)
    {
      strcpy_P(buffer, (char*)pgm_read_word(&(error_table[4]))); // CRC check failed msg 
      Serial.println( buffer );
      delayMicroseconds(SERIAL_PAUSE);
    }
  }
  
  // Read commands to set response bits
  if (Serial.available()>0)
  {
    cmd = Serial.read();  // read cmd
    cmd = cmd-0x30;       // convert ASCII to digit
    switch(cmd)
    {
      case 0:  ;  strcpy_P(buffer, (char*)pgm_read_word(&(flags_resp_table[1]))); // Payload update flag set
                  Serial.println( buffer );
                  delayMicroseconds(SERIAL_PAUSE);
                  bitSet(pay_request, PAYLOAD_UPDATE);
                  para_ID_temp = 0x00;
                  break;
      case 1:  ;  strcpy_P(buffer, (char*)pgm_read_word(&(flags_resp_table[2]))); // Next mode flag set
                  Serial.println( buffer );
                  delayMicroseconds(SERIAL_PAUSE);
                  bitSet(pay_request, NEXT_MODE);
                  para_ID_temp = 0x00;
                  break;
      case 2:  ;  strcpy_P(buffer, (char*)pgm_read_word(&(flags_resp_table[3]))); // Disable compression flag set
                  Serial.println( buffer );
                  delayMicroseconds(SERIAL_PAUSE);
                  bitSet(pay_request, DISABLE_COMPRESSION);
                  break;
      case 3:  ;  strcpy_P(buffer, (char*)pgm_read_word(&(flags_resp_table[4]))); // +256 bytes waiting for spi
                  Serial.println( buffer );
                  delayMicroseconds(SERIAL_PAUSE);
                  data_waiting = data_waiting+256;
                  bitSet(pay_request, STREAM_READY);
                  break;
      case 4:  ;  if(bitRead(header_flags,ERROR))
                  {
                    strcpy_P(buffer, (char*)pgm_read_word(&(flags_resp_table[10]))); // Error flag clear
                    Serial.println( buffer );
                    delayMicroseconds(SERIAL_PAUSE);
                    bitClear(header_flags,ERROR);
                  }
                  else
                  {
                    strcpy_P(buffer, (char*)pgm_read_word(&(flags_resp_table[9]))); // Error flag set
                    Serial.println( buffer );
                    delayMicroseconds(SERIAL_PAUSE);
                    bitSet(header_flags,ERROR);
                  }
                  break;
      case 6:  ;  strcpy_P(buffer, (char*)pgm_read_word(&(flags_resp_table[5]))); // Reset flag set
                  Serial.println( buffer );
                  delayMicroseconds(SERIAL_PAUSE);
                  bitSet(pay_request, RESET_PAYLOAD);
                  break;
      case 7:  ;  strcpy_P(buffer, (char*)pgm_read_word(&(flags_resp_table[6]))); // Shutdown flag set
                  Serial.println( buffer );
                  delayMicroseconds(SERIAL_PAUSE);
                  bitSet(pay_request, SHUTDOWN);
                  break;
      case 8:  ;  strcpy_P(buffer, (char*)pgm_read_word(&(flags_resp_table[7]))); // priority data waiting +256 Bytes
                  Serial.println( buffer );
                  delayMicroseconds(SERIAL_PAUSE);
                  priority_waiting = priority_waiting+256;
                  break;
      case 9:  ;  strcpy_P(buffer, (char*)pgm_read_word(&(flags_resp_table[8]))); // Data Waiting +256 Bytes
                  Serial.println( buffer );
                  delayMicroseconds(SERIAL_PAUSE);
                  data_waiting = data_waiting+256;
                  break;
      default:    ;
                  break;
    }
  }
}
/*----------------------------------------------------*/


/*----------------------------------------------------*/
// Process I2C received buffer
// Assumes buffer is cleared before each data packet, rxd_buffer[0] is first byte of command
uint8_t Process_i2c_rxd(void)
{
  uint8_t i2c_command;
  unsigned int crc = 0;  //store checksum for processing
  uint8_t end_cmd = 0;  //store end of current command
  uint8_t i2c_cmd_lengths[16] = {23,3,23,6,4,3,3,4,0,0,0,0,0,0,0,2};
  uint8_t CRC_pass = 0;
  
  // Only process command if first nibble is 9 indicating payload command
  if((rxd_buffer[0]&0xF0) == 0x90)
  {
    i2c_command = rxd_buffer[0]&0x0F;

    end_cmd = i2c_cmd_lengths[i2c_command];

    if(bitRead(flags,USE_RX_CRC))
    {
      crc = rxd_buffer[end_cmd];  // CRC is 2 bytes following cmd content
      crc = crc<<8;
      crc = crc|rxd_buffer[end_cmd+1];
    
      CRC_pass = process_CRC(0,end_cmd,crc);  // set CRC pass = 1 if applicable
    }
    else
    {
      CRC_pass = 1; // if CRC disabled always set as 1
    }
  
    if(CRC_pass == 1)
    {
      // Decode command
      switch(i2c_command)
      {
        case 0: ;   end_cmd = pay_ops_initialise();  // Payload Operations initialise 
                    break;
                
        case 1: ;   end_cmd = pay_ops_status();      // Payload Operations status 
                    break; 
                
        case 2: ;   end_cmd = pay_ops_update();      // Payload Operations update 
                    break;
     
        case 3: ;   end_cmd = pay_parameter_write(); // Payload Parameter write
                    break;
     
        case 4: ;   end_cmd = pay_parameter_read();  // Payload Parameter read 
                    break;
     
        case 5: ;   end_cmd = pay_pri_data_trans();  // Payload Priority Data Transfer 
                    break;
     
        case 6: ;   end_cmd = pay_data_trans();      // Payload Data Transfer 
                    break;
     
        case 7: ;   end_cmd = pay_SPI_data_trans();  // Payload SPI data transfer 
                    break;
     
        case 8: ;  end_cmd = pay_shutdn();         // Payload Shutdown
                    break;
     
        default:    strcpy_P(buffer, (char*)pgm_read_word(&(error_table[1]))); // Unknown Cmd msg
                    Serial.println( buffer );
                    delayMicroseconds(SERIAL_PAUSE);
                    break;
      }
    }
    else
    {
      strcpy_P(buffer, (char*)pgm_read_word(&(error_table[4]))); // CRC check fail error msg. 
      Serial.println( buffer );
      delayMicroseconds(SERIAL_PAUSE);
    }
  }
  else
  {
       ; // take no action if not a payload command received
      // add error handler if required
  }
 
  clear_rxd_buf();
  
  bitClear(flags,i2c_new_cmd_rx);

  return CRC_pass;
}
/*----------------------------------------------------*/

/*----------------------------------------------------*/
// Payload operations initialise
uint8_t pay_ops_initialise(void)
{ 
  unsigned int crc = 0;
  char error = 0;      // test variable to trigger an error condition response to initialise command
  
  if(error == 0)
  {
    // If data received with valid CRC return acknowledge packet to platform.
    txd_buffer[0] = rxd_buffer[0];      // 
    txd_buffer[1] = rxd_buffer[1];      //
    txd_buffer[2] = 0x7E;               // Identifier byte of 0x7E = acknowledge
    i2c_tx_available = 5;
    crc = calculate_CRC(1, i2c_tx_available-2);
    txd_buffer[3] = crc>>8;
    txd_buffer[4] = crc;
  }
  else
  {
    // If simulating an error at payload return error packet to platform.
    txd_buffer[0] = rxd_buffer[0];      // 
    txd_buffer[1] = rxd_buffer[1];      //
    bitSet(txd_buffer[1],ERROR);        // set error bit in header flags
    txd_buffer[2] = 0xAA;               // Identifier byte of 0xAA = example error code
    i2c_tx_available = 5;
    crc = calculate_CRC(1, i2c_tx_available-2);
    txd_buffer[3] = crc>>8;
    txd_buffer[4] = crc;  
  }
  
  bitSet(flags,i2c_gone);
   
  delay(1);

  while(bitRead(flags,i2c_gone))  //wait for i2c transmission to complete
    delayMicroseconds(50);

  delay(10);
  
  i2c_tx_available = 0;
  
  if(error = 0)  // if no error initialise as per command
  {
    payload_op_mode = rxd_buffer[2];        // Payload Operation Mode

    payload_op_flag = rxd_buffer[3];       // Payload Operation Flag   
    payload_op_flag = payload_op_flag<<8;
    payload_op_flag = payload_op_flag|rxd_buffer[4];
    
    init_time = rxd_buffer[5];             // Onboard time at MIC
    init_time = init_time<<8;
    init_time = init_time|rxd_buffer[6];
    init_time = init_time<<8;
    init_time = init_time|rxd_buffer[7];
    init_time = init_time<<8;
    init_time = init_time|rxd_buffer[8];
  
    init_secs = init_time>>10;                    // build init time. top bits = seconds, lower 10 bits = milliseconds
    init_millisecs = init_time&0x03FF;
    
    priority_limit = rxd_buffer[9];       // Payload Priority Data Limit   
    priority_limit = priority_limit<<8;
    priority_limit = priority_limit|rxd_buffer[10];

    priority_remain = rxd_buffer[11];       // Payload Priority Data Limit   
    priority_remain = priority_remain<<8;
    priority_remain = priority_remain|rxd_buffer[12];
    
    mass_mem_limit = rxd_buffer[13];         // Payload Mass Memory Limit
    mass_mem_limit = mass_mem_limit<<8;
    mass_mem_limit = mass_mem_limit|rxd_buffer[14];
    mass_mem_limit = mass_mem_limit<<8;
    mass_mem_limit = mass_mem_limit|rxd_buffer[15];
    mass_mem_limit = mass_mem_limit<<8;
    mass_mem_limit = mass_mem_limit|rxd_buffer[16];
  
    mass_mem_remain = rxd_buffer[17];        // Payload Mass Memory Remaining
    mass_mem_remain = mass_mem_remain<<8;
    mass_mem_remain = mass_mem_remain|rxd_buffer[18];
    mass_mem_remain = mass_mem_remain<<8;
    mass_mem_remain = mass_mem_remain|rxd_buffer[19];
    mass_mem_remain = mass_mem_remain<<8;
    mass_mem_remain = mass_mem_remain|rxd_buffer[20];

    poll_freq = rxd_buffer[21];               // Payload Poll Frequency  
    poll_freq = poll_freq<<8;
    poll_freq = poll_freq|rxd_buffer[22];     
  }
  else
  {
    ; //ignore initialise command if we are simulating an error at the payload.
  }
  
  pay_request = 0;
 
  strcpy_P(buffer, (char*)pgm_read_word(&(pay_cmd_table[0]))); // payload ops initialise
  Serial.println( buffer );
  delayMicroseconds(SERIAL_PAUSE);

  display_status();
   
  return 15; // return length of command
}
/*----------------------------------------------------*/                
 
/*----------------------------------------------------*/
// Payload operations status
uint8_t pay_ops_status(void)
{  
  unsigned char ack = 0;
  unsigned int crc_resp_timer = 0;
  unsigned int crc = 0;
  int i;
  
  clear_txd_buf();
    
  txd_buffer[0] = 0x91;
  txd_buffer[1] = header_flags;
  
  if(bitRead(header_flags,ERROR))  // if error bit set send error packet
  {
    txd_buffer[2] = 0xFF;  // insert error byte
    
    bitSet(header_flags,CRC);
    txd_buffer[1] = header_flags;
    i2c_tx_available = 5;
    crc = calculate_CRC(1, i2c_tx_available-2);
    txd_buffer[3] = crc>>8;
    txd_buffer[4] = crc;
  }
  else
  { 
    
    txd_buffer[2] = payload_op_mode;      // Payload Operation Mode

    txd_buffer[3] = payload_op_flag>>8;   // Payload Operation Flag   
    txd_buffer[4] = payload_op_flag;
    
    txd_buffer[5] = priority_waiting>>8;  // Payload Priority Data Waiting
    txd_buffer[6] = priority_waiting; 
    
    txd_buffer[7] = data_waiting>>24;      // Payload Data Waiting
    txd_buffer[8] = data_waiting>>16;      // Payload Data Waiting 
    txd_buffer[9] = data_waiting>>8;      // Payload Data Waiting
    txd_buffer[10] = data_waiting;   
    
    txd_buffer[11] = pay_request;      // Payload Request Flags; bit0 = payload update, bit1 = next mode, bit2 = disable compression, bit3 = stream ready, bit6 = reset payload, bit7 = shutdown payload

    txd_buffer[12] = para_ID_temp;          // Payload Parameter ID1
    txd_buffer[13] = 0x00;          // Payload Parameter ID2
    txd_buffer[14] = 0x00;          // Payload Parameter ID3
    txd_buffer[15] = 0x00;          // Payload Parameter ID4
    txd_buffer[16] = 0x00;          // Payload Parameter ID5
    txd_buffer[17] = 0x00;          // Payload Parameter ID6
    txd_buffer[18] = 0x00;          // Payload Parameter ID7
    txd_buffer[19] = 0x00;          // Payload Parameter ID8    
 
    bitSet(header_flags,CRC);
    txd_buffer[1] = header_flags;
    i2c_tx_available = 22;
    crc = calculate_CRC(1, i2c_tx_available-2);
    txd_buffer[20] = crc>>8;
    txd_buffer[21] = crc;
 }
 
  bitSet(flags,i2c_gone);
   
  delay(1);

  while(bitRead(flags,i2c_gone))  //wait for i2c transmission to complete
    delayMicroseconds(50);
 
    
  delay(10);
    
  i2c_tx_available = 0;
  
  
 // wait for acknowledgement
  while(crc_resp_timer<2000 && ack == 0)    // wait up to 2ms for response
  {
    // new i2c data received?
    if(bitRead(flags,i2c_new_cmd_rx))
    {
      if(rxd_buffer[0] == 0x91)  // If data returned does it match cmd byte expected
      {
        crc = rxd_buffer[3];           // CRC in bytes 3 and 4 of rx data
        crc = crc<<8;
        crc = crc|rxd_buffer[4];
        
        if (!process_CRC(0,3,crc))          // Check CRC of ACK packet
        {
          //if CRC failed - action taken here is up to payload designer, this is just an example
          strcpy_P(buffer, (char*)pgm_read_word(&(error_table[4]))); // crc fail error message
          Serial.println( buffer );
          delayMicroseconds(SERIAL_PAUSE);
        }
        else
        {
          // if CRC passed check for error
          if(bitRead(rxd_buffer[1],ERROR))  // now process error flag
          {
            if(rxd_buffer[2] == 0x01)
              ; // CRC check on last packet sent by payload was failed. Might want to store current data for update in next status cmd?
            else if(rxd_buffer[2] == 0x02)
              ; // last packet sent by payload unrecognised
            else
              ; // decode other error codes here.
          }
          else
          {
            ; // no error, carry on with operations.
            
              para_ID_temp = pay_para_ID[0];  // in auto mode request update next time round
          }
          
          strcpy_P(buffer, (char*)pgm_read_word(&(status_msg_table[3]))); // Acknowledge received 
          Serial.println( buffer );
          delayMicroseconds(SERIAL_PAUSE);
          ack = 1;
          bitClear(flags,i2c_new_cmd_rx);  // Clear new cmd bit once CRC is read, dont clear if cmd received is not CRC
        }
      }
    }
    delayMicroseconds(50);
    crc_resp_timer = crc_resp_timer + 50; // count approximate uSec each iteration of loop
  }

  
  // If no ack received print status msg
  if(ack ==0)
  {
    strcpy_P(buffer, (char*)pgm_read_word(&(error_table[5]))); // acknowledge match failed
    Serial.println( buffer );
    delayMicroseconds(SERIAL_PAUSE);
    bitClear(flags,i2c_new_cmd_rx);  // Clear new cmd bit once CRC is read, dont clear if cmd received is not CRC
  }
  
  strcpy_P(buffer, (char*)pgm_read_word(&(pay_cmd_table[1]))); // Pay ops status 
  Serial.println( buffer );
  delayMicroseconds(SERIAL_PAUSE);
 
  return 0; // return length of command
}
/*----------------------------------------------------*/ 

/*----------------------------------------------------*/
// Payload operations update
uint8_t pay_ops_update(void)
{
  unsigned int crc = 0;
    
  if(!bitRead(header_flags,ERROR))  // if error bit clear send ack packet
  {
    // If data received with valid CRC return acknowledge packet to platform.
    txd_buffer[0] = rxd_buffer[0];      // 
    txd_buffer[1] = rxd_buffer[1];      //
    txd_buffer[2] = 0x7E;               // Identifier byte of 0x7E = acknowledge
    i2c_tx_available = 5;
    crc = calculate_CRC(1, i2c_tx_available-2);
    txd_buffer[3] = crc>>8;
    txd_buffer[4] = crc;
  }
  else
  {
    // If simulating an error at payload return error packet to platform.
    txd_buffer[0] = rxd_buffer[0];      // 
    txd_buffer[1] = rxd_buffer[1];      //
    bitSet(txd_buffer[1],ERROR);        // set error bit in header flags
    txd_buffer[2] = 0xAA;               // Identifier byte of 0xAA = example error code
    i2c_tx_available = 5;
    crc = calculate_CRC(1, i2c_tx_available-2);
    txd_buffer[3] = crc>>8;
    txd_buffer[4] = crc;  
  }
  
  bitSet(flags,i2c_gone);
   
  delay(1);

  while(bitRead(flags,i2c_gone))  //wait for i2c transmission to complete
    delayMicroseconds(50);

  delay(2);
  
  i2c_tx_available = 0;
    
  if(!bitRead(header_flags,ERROR))  // if no error bit set process as normal
  {
    payload_op_mode = rxd_buffer[2];        // Payload Operation Mode

    payload_op_flag = rxd_buffer[3];       // Payload Operation Flag   
    payload_op_flag = payload_op_flag<<8;
    payload_op_flag = payload_op_flag|rxd_buffer[4];
    
    init_time = rxd_buffer[5];             // Onboard time at MIC
    init_time = init_time<<8;
    init_time = init_time|rxd_buffer[6];
    init_time = init_time<<8;
    init_time = init_time|rxd_buffer[7];
    init_time = init_time<<8;
    init_time = init_time|rxd_buffer[8];
    
    priority_limit = rxd_buffer[9];       // Payload Priority Data Limit   
    priority_limit = priority_limit<<8;
    priority_limit = priority_limit|rxd_buffer[10];

    priority_remain = rxd_buffer[11];       // Payload Priority Data Limit   
    priority_remain = priority_remain<<8;
    priority_remain = priority_remain|rxd_buffer[12];
    
    mass_mem_limit = rxd_buffer[13];         // Payload Mass Memory Limit
    mass_mem_limit = mass_mem_limit<<8;
    mass_mem_limit = mass_mem_limit|rxd_buffer[14];
    mass_mem_limit = mass_mem_limit<<8;
    mass_mem_limit = mass_mem_limit|rxd_buffer[15];
    mass_mem_limit = mass_mem_limit<<8;
    mass_mem_limit = mass_mem_limit|rxd_buffer[16];

    mass_mem_remain = rxd_buffer[17];        // Payload Mass Memory Remaining
    mass_mem_remain = mass_mem_remain<<8;
    mass_mem_remain = mass_mem_remain|rxd_buffer[18];
    mass_mem_remain = mass_mem_remain<<8;
    mass_mem_remain = mass_mem_remain|rxd_buffer[19];
    mass_mem_remain = mass_mem_remain<<8;
    mass_mem_remain = mass_mem_remain|rxd_buffer[20];

    poll_freq = rxd_buffer[21];               // Payload Poll Frequency  
    poll_freq = poll_freq<<8;
    poll_freq = poll_freq|rxd_buffer[22];  

    bitClear(pay_request, NEXT_MODE);    // This in automated mode a payload update command will have been triggered by a request for the next mode, clear the flag once the request is serviced
    bitClear(pay_request, PAYLOAD_UPDATE);    // This in automated mode a payload update command will have been triggered by a request for update, clear the flag once the request is serviced
  }
  else
  {
    ; //ignore command if we are simulating an error at the payload.
  } 
  strcpy_P(buffer, (char*)pgm_read_word(&(pay_cmd_table[2]))); // Pay ops update
  Serial.println( buffer );
  delayMicroseconds(SERIAL_PAUSE);
   
//  display_status();
    
  return 15; // return length of command
}
/*----------------------------------------------------*/ 

/*----------------------------------------------------*/
// Payload Parameter write
uint8_t pay_parameter_write(void)
{
  uint8_t i=0;
  unsigned int crc = 0;
    
  if(!bitRead(header_flags,ERROR))  // if error bit clear send ack packet
  {
    // If data received with valid CRC return acknowledge packet to platform.
    txd_buffer[0] = rxd_buffer[0];      // 
    txd_buffer[1] = rxd_buffer[1];      //
    txd_buffer[2] = 0x7E;               // Identifier byte of 0x7E = acknowledge
    i2c_tx_available = 5;
    crc = calculate_CRC(1, i2c_tx_available-2);
    txd_buffer[3] = crc>>8;
    txd_buffer[4] = crc;
  }
  else
  {
    // If simulating an error at payload return error packet to platform.
    txd_buffer[0] = rxd_buffer[0];      // 
    txd_buffer[1] = rxd_buffer[1];      //
    bitSet(txd_buffer[1],ERROR);        // set error bit in header flags
    txd_buffer[2] = 0xAA;               // Identifier byte of 0xAA = example error code
    i2c_tx_available = 5;
    crc = calculate_CRC(1, i2c_tx_available-2);
    txd_buffer[3] = crc>>8;
    txd_buffer[4] = crc;  
  }
  
  bitSet(flags,i2c_gone);
   
  delay(1);

  while(bitRead(flags,i2c_gone))  //wait for i2c transmission to complete
    delayMicroseconds(50);

  delay(2);
  
  i2c_tx_available = 0;
    
  if(!bitRead(header_flags,ERROR))  // if no error bit set process as normal
  {
    strcpy_P(buffer, (char*)pgm_read_word(&(pay_cmd_table[3]))); // Pay parameter write
    Serial.println( buffer );
    delayMicroseconds(SERIAL_PAUSE);
  
    for(i=0;i<16;i++)  //  check received parameter ID against stored values
    {
      if(rxd_buffer[2]==pay_para_ID[i])
      {
        pay_para_value[i] = rxd_buffer[3];
        pay_para_value[i] = pay_para_value[i]<<8;
        pay_para_value[i] = pay_para_value[i]|rxd_buffer[4];
      }
    }
  }
  else
  {
    ; //ignore command if we are simulating an error at the payload.
  }    
// TEST
//  display_status();
// TEST    
  return 3; // return length of command
}
/*----------------------------------------------------*/  

/*----------------------------------------------------*/
// Payload Parameter read
uint8_t pay_parameter_read(void)
{
  unsigned char ack = 0;
  unsigned int ack_resp_timer = 0;
  unsigned char id_requested = 0;
  unsigned int crc = 0;
  unsigned int value_requested = 0;
  uint8_t i = 0;
  
  clear_txd_buf();

  
  if(bitRead(header_flags,ERROR))  // if error bit set send error packet
  {
    txd_buffer[0] = 0x94;
    txd_buffer[2] = 0xAA;  // insert error byte
    
    bitSet(header_flags,CRC);
    txd_buffer[1] = header_flags;
    i2c_tx_available = 5;
    crc = calculate_CRC(1, i2c_tx_available-2);
    txd_buffer[3] = crc>>8;
    txd_buffer[4] = crc;
  }
  else //send data packet
  {
    id_requested = rxd_buffer[2];
    
    for(i=0;i<16;i++)  //  check received parameter ID against stored values
    {
      if(rxd_buffer[2]==pay_para_ID[i])
      {
        value_requested = pay_para_value[i];
      }
    }  
    
    clear_txd_buf();
    
    txd_buffer[0] = 0x94;
    txd_buffer[1] = header_flags; 
    txd_buffer[2] = id_requested;        // Parameter ID requested

    txd_buffer[3] = value_requested>>8;  // Parameter Value requested
    txd_buffer[4] = value_requested;
    
    bitSet(header_flags,CRC);
    txd_buffer[1] = header_flags;
    i2c_tx_available = 7;
    crc = calculate_CRC(1, i2c_tx_available-2);
    txd_buffer[5] = crc>>8;
    txd_buffer[6] = crc;   
  }
 
  bitSet(flags,i2c_gone);
    
  delay(1);

  while(bitRead(flags,i2c_gone))  //wait for i2c transmission to complete
    delayMicroseconds(50);
     
  i2c_tx_available = 0;
 
   
 // wait for acknowledgement
  while(ack_resp_timer<2000 && ack == 0)    // wait up to 2ms for response
  {
    // new i2c data received?
    if(bitRead(flags,i2c_new_cmd_rx))
    {
      if(rxd_buffer[0] == 0x94)  // If data returned does it match cmd byte expected
      {
        crc = rxd_buffer[3];           // CRC in bytes 3 and 4 of rx data
        crc = crc<<8;
        crc = crc|rxd_buffer[4];
        
        if (!process_CRC(0,3,crc))          // Check CRC of ACK packet
        {
          //if CRC failed - action taken here is up to payload designer, this is just an example
          strcpy_P(buffer, (char*)pgm_read_word(&(error_table[4]))); // crc fail error message
          Serial.println( buffer );
          delayMicroseconds(SERIAL_PAUSE);
        }
        else
        {
          // if CRC passed check for error
          if(bitRead(rxd_buffer[1],ERROR))  // now process error flag
          {
            if(rxd_buffer[2] == 0x01)
              ; // CRC check on last packet sent by payload was failed. Might want to store current data for update in next status cmd?
            else if(rxd_buffer[2] == 0x02)
              ; // last packet sent by payload unrecognised
            else
              ; // decode other error codes here.
          }
          else
          {
            ; // no error, carry on with operations.
          }
          
          strcpy_P(buffer, (char*)pgm_read_word(&(status_msg_table[3]))); // Acknowledge received 
          Serial.println( buffer );
          delayMicroseconds(SERIAL_PAUSE);
          ack = 1;
          bitClear(flags,i2c_new_cmd_rx);  // Clear new cmd bit once CRC is read, dont clear if cmd received is not CRC
        }
      }
    }
    delayMicroseconds(50);
    ack_resp_timer = ack_resp_timer + 50; // count approximate uSec each iteration of loop
  }

  
  // If no ack received print status msg
  if(ack ==0)
  {
    strcpy_P(buffer, (char*)pgm_read_word(&(error_table[5]))); // acknowledge match failed
    Serial.println( buffer );
    delayMicroseconds(SERIAL_PAUSE);
    bitClear(flags,i2c_new_cmd_rx);  // Clear new cmd bit once CRC is read, dont clear if cmd received is not CRC
  }
 
  strcpy_P(buffer, (char*)pgm_read_word(&(pay_cmd_table[4]))); // pay parameter read 
  Serial.println( buffer );
  delayMicroseconds(SERIAL_PAUSE);
 
  return 1; // return length of command
}
/*----------------------------------------------------*/

/*----------------------------------------------------*/
// Payload Priority data transfer
uint8_t pay_pri_data_trans(void)
{
  unsigned char ack = 0;
  unsigned int n;
  unsigned int crc = 0;
  unsigned int ack_resp_timer = 0;
  
  if(bitRead(header_flags,ERROR))  // if error bit set send error byte
  {
    txd_buffer[0] = 0x95;
    txd_buffer[2] = 0xAA;  // insert error byte

    bitSet(header_flags,CRC);
    txd_buffer[1] = header_flags;
    i2c_tx_available = 5;
    crc = calculate_CRC(1, i2c_tx_available-2);
    txd_buffer[3] = crc>>8;
    txd_buffer[4] = crc;   
  }
  else  // if error bit not set send full packet as normal
  {
    txd_buffer[0] = 0x95;
        
    for(n = 0; n < (PRIORITY_I2C_BYTES); n++)
    {
      txd_buffer[n+2] = n;    // data into txd_buffer[]
    }
    
    bitSet(header_flags,CRC);
    txd_buffer[1] = header_flags;
    i2c_tx_available = PRIORITY_I2C_BYTES+4;
    crc = calculate_CRC(1, i2c_tx_available-2);
    txd_buffer[PRIORITY_I2C_BYTES+2] = crc>>8;
    txd_buffer[PRIORITY_I2C_BYTES+3] = crc;   
  }
  
  
  bitSet(flags,i2c_gone);
  bitClear(flags,i2c_new_cmd_rx);// clear new command flag in anticipation of response

  
  while(bitRead(flags,i2c_gone))  //wait for i2c transmission to buffer
      delayMicroseconds(50);

  delay(25);  // Delay to allow I2C to complete sending
  
  i2c_tx_available = 0;

  while(ack_resp_timer<2000 && ack == 0)    // wait up to 2ms for response
  {
    // new i2c data received?
    if(bitRead(flags,i2c_new_cmd_rx))
    {
      if(rxd_buffer[0] == 0x95)  // If data returned does it match cmd byte expected
      {
        crc = rxd_buffer[3];           // CRC in bytes 3 and 4 of rx data
        crc = crc<<8;
        crc = crc|rxd_buffer[4];
        
        if (!process_CRC(0,3,crc))          // Check CRC of ACK packet
        {
          //if CRC failed - action taken here is up to payload designer, this is just an example
          strcpy_P(buffer, (char*)pgm_read_word(&(error_table[4]))); // crc fail error message
          Serial.println( buffer );
          delayMicroseconds(SERIAL_PAUSE);
        }
        else  // crc passed on ack
        {
          // if CRC passed check for error
          if(bitRead(rxd_buffer[1],ERROR))  // now process error flag
          {
            if(rxd_buffer[2] == 0x01)
              ; // CRC check on last packet sent by payload was failed. Might want to store current data for update in next status cmd?
            else if(rxd_buffer[2] == 0x02)
              ; // last packet sent by payload unrecognised
            else
              ; // decode other error codes here.
          }
          else  // no error, carry on with operations.
          {
            if(priority_waiting>PRIORITY_I2C_BYTES)
            {
              priority_waiting = priority_waiting - PRIORITY_I2C_BYTES;  // Update count of data waiting for transmission.
            }
            else
            {
              priority_waiting = 0;  // for test purposes allow payload to keep sending data once counter reaches 0.
            }
          }
          
          strcpy_P(buffer, (char*)pgm_read_word(&(status_msg_table[3]))); // Acknowledge received 
          Serial.println( buffer );
          delayMicroseconds(SERIAL_PAUSE);
          ack = 1;
          bitClear(flags,i2c_new_cmd_rx);  // Clear new cmd bit once CRC is read, dont clear if cmd received is not CRC
        }
      }
   }
    delayMicroseconds(50);
    ack_resp_timer = ack_resp_timer + 50; // count approximate uSec each iteration of loop
  }
  
  // If no ack received print status msg
  if(ack ==0)
  {
    strcpy_P(buffer, (char*)pgm_read_word(&(error_table[5]))); // acknowledge match failed
    Serial.println( buffer );
    delayMicroseconds(SERIAL_PAUSE);
    bitClear(flags,i2c_new_cmd_rx);  // Clear new cmd bit once CRC is read, dont clear if cmd received is not CRC
  }
  
  strcpy_P(buffer, (char*)pgm_read_word(&(pay_cmd_table[5]))); // priority data transfer 
  Serial.println( buffer );
  delayMicroseconds(SERIAL_PAUSE);
  
  return 0; // return length of command
}
/*----------------------------------------------------*/ 


/*----------------------------------------------------*/
// Payload Data Transfer
uint8_t pay_data_trans(void)
{
  unsigned char ack = 0;
  unsigned int n;
  unsigned int crc = 0;
  unsigned int ack_resp_timer = 0;
   

  if(bitRead(header_flags,ERROR))  // if error bit set send error byte
  {
    txd_buffer[0] = 0x96;
    txd_buffer[2] = 0xAA;  // insert error byte

    bitSet(header_flags,CRC);
    txd_buffer[1] = header_flags;
    i2c_tx_available = 5;
    crc = calculate_CRC(1, i2c_tx_available-2);
    txd_buffer[3] = crc>>8;
    txd_buffer[4] = crc;   
  }
  else  // if error bit not set send full packet as normal
  {
    txd_buffer[0] = 0x96;
    
   // for(n = 0; n < (I2C_BYTES); n++)
   // {
    //  txd_buffer[n+2] = 0xFF-n;    // data into txd_buffer[]
    //}
   
    bitSet(header_flags,CRC);
    txd_buffer[1] = header_flags;
    i2c_tx_available = PRIORITY_I2C_BYTES+4;
    crc = calculate_CRC(1, i2c_tx_available-2);
    txd_buffer[PRIORITY_I2C_BYTES+2] = crc>>8;
    txd_buffer[PRIORITY_I2C_BYTES+3] = crc;   
  }
  
  bitSet(flags,i2c_gone);
  bitClear(flags,i2c_new_cmd_rx);// clear new command flag in anticipation of response

  while(bitRead(flags,i2c_gone))  //wait for i2c transmission to buffer
      delayMicroseconds(50);

  delay(25);  // Delay to allow I2C to complete sending
  
  i2c_tx_available = 0;

  while(ack_resp_timer<2000 && ack == 0)    // wait up to 2ms for response
  {
    // new i2c data received?
    if(bitRead(flags,i2c_new_cmd_rx))
    {
      if(rxd_buffer[0] == 0x96)  // If data returned does it match cmd byte expected
      {
        crc = rxd_buffer[3];           // CRC in bytes 3 and 4 of rx data
        crc = crc<<8;
        crc = crc|rxd_buffer[4];
        
        if (!process_CRC(0,3,crc))          // Check CRC of ACK packet
        {
          //if CRC failed - action taken here is up to payload designer, this is just an example
          strcpy_P(buffer, (char*)pgm_read_word(&(error_table[4]))); // crc fail error message
          Serial.println( buffer );
          delayMicroseconds(SERIAL_PAUSE);
        }
        else  // crc passed on ack
        {
          // if CRC passed check for error
          if(bitRead(rxd_buffer[1],ERROR))  // now process error flag
          {
            if(rxd_buffer[2] == 0x01)
              ; // CRC check on last packet sent by payload was failed. Might want to store current data for update in next status cmd?
            else if(rxd_buffer[2] == 0x02)
              ; // last packet sent by payload unrecognised
            else
              ; // decode other error codes here.
          }
          else  // no error, carry on with operations.
          {
            if(data_waiting>I2C_BYTES)
            {
              data_waiting = data_waiting - I2C_BYTES;  // Update count of data waiting for transmission.
            }
            else
            {
              data_waiting = 0;  // for test purposes allow payload to keep sending data once counter reaches 0.
            }
          }
          
          strcpy_P(buffer, (char*)pgm_read_word(&(status_msg_table[3]))); // Acknowledge received 
          Serial.println( buffer );
          delayMicroseconds(SERIAL_PAUSE);
          ack = 1;
          bitClear(flags,i2c_new_cmd_rx);  // Clear new cmd bit once CRC is read, dont clear if cmd received is not CRC
        }
      }
   }
    delayMicroseconds(50);
    ack_resp_timer = ack_resp_timer + 50; // count approximate uSec each iteration of loop
  }
  
  // If no ack received print status msg
  if(ack ==0)
  {
    strcpy_P(buffer, (char*)pgm_read_word(&(error_table[5]))); // acknowledge match failed
    Serial.println( buffer );
    delayMicroseconds(SERIAL_PAUSE);
    bitClear(flags,i2c_new_cmd_rx);  // Clear new cmd bit once CRC is read, dont clear if cmd received is not CRC
  }
  
  strcpy_P(buffer, (char*)pgm_read_word(&(pay_cmd_table[6]))); // Data transfer msg 
  Serial.println( buffer );
  delayMicroseconds(SERIAL_PAUSE);
  
  return 0; // return length of command  
}
/*----------------------------------------------------*/

/*----------------------------------------------------*/
// Payload SPI data transfer
uint8_t pay_SPI_data_trans(void)
{
  unsigned int i = 0;
  unsigned int crc = 0 ;
  unsigned int no_to_display = 0;
  unsigned int spi_tx_available = 0;
  unsigned char ack = 0;
  unsigned int ack_resp_timer = 0;

  clear_txd_buf();
  
  txd_buffer[0] = 0x97;
  txd_buffer[1] = header_flags;
  
  // Preload SPI transmit register
  received = 0;
  SPDR = txd_buffer[received];
  
  if(bitRead(header_flags,ERROR))  // if error bit set send error byte
  {
    txd_buffer[2] = 0xAA;  // insert error byte

    bitSet(header_flags,CRC);
    txd_buffer[1] = header_flags;
    crc = calculate_CRC(1, 3);
    txd_buffer[3] = crc>>8;
    txd_buffer[4] = crc;   
  }
  else  // if error bit not set send full packet as normal
  {
    // fill transmit buffer to count down from 255 to 0
    for(i = 0; i < SPI_BYTES; i++)
    {
      txd_buffer[i+2] = 0xFF-i;
    }
     
    bitSet(header_flags,CRC);
    txd_buffer[1] = header_flags;
    crc = calculate_CRC(1, SPI_BYTES+2);
    txd_buffer[SPI_BYTES+2] = crc>>8;
    txd_buffer[SPI_BYTES+3] = crc;  
  }  

  spi_tx_available = SPI_BYTES+4;
  
  bitClear(TIMSK0,0);
  
  bitSet(flags,SPI_write);

  // if SS is high wait for it to be pulled low
  while(digitalRead(PIN_SPI_SS) == 1)
      ;

  // wait for end of transmission when SS is released
  // SPI interupt is transmitting data during this wait
  while(digitalRead(PIN_SPI_SS) == 0)
    ;
   
  bitSet(TIMSK0,0);    // Re-enable Timer0 overflow interrupt

  bitClear(flags,i2c_new_cmd_rx);  

  while(ack_resp_timer<2000 && ack == 0)    // wait up to 2ms for response
  {
    // new i2c data received?
    if(bitRead(flags,i2c_new_cmd_rx))
    {
      if(rxd_buffer[0] == 0x97)  // If data returned does it match cmd byte expected
      {
        crc = rxd_buffer[3];           // CRC in bytes 3 and 4 of rx data
        crc = crc<<8;
        crc = crc|rxd_buffer[4];
        
        if (!process_CRC(0,3,crc))          // Check CRC of ACK packet
        {
          //if CRC failed - action taken here is up to payload designer, this is just an example
          strcpy_P(buffer, (char*)pgm_read_word(&(error_table[4]))); // crc fail error message
          Serial.println( buffer );
          delayMicroseconds(SERIAL_PAUSE);
        }
        else  // crc passed on ack
        {
          // if CRC passed check for error
          if(bitRead(rxd_buffer[1],ERROR))  // now process error flag
          {
            if(rxd_buffer[2] == 0x01)
              ; // CRC check on last packet sent by payload was failed. Might want to store current data for update in next status cmd?
            else if(rxd_buffer[2] == 0x02)
              ; // last packet sent by payload unrecognised
            else
              ; // decode other error codes here.
          }
          else  // no error, carry on with operations.
          {
            if(data_waiting>I2C_BYTES)
            {
              data_waiting = data_waiting - I2C_BYTES;  // Update count of data waiting for transmission
            }
            else
            {
              data_waiting = 0;  // for test purposes allow payload to keep sending data once counter reaches 0.
            }  
            bitClear(pay_request,STREAM_READY);        // Clear request for data streaming
          }
          
          strcpy_P(buffer, (char*)pgm_read_word(&(status_msg_table[3]))); // Acknowledge received 
          Serial.println( buffer );
          delayMicroseconds(SERIAL_PAUSE);
          ack = 1;
          bitClear(flags,i2c_new_cmd_rx);  // Clear new cmd bit once CRC is read, dont clear if cmd received is not CRC
        }
      }
   }
    delayMicroseconds(50);
    ack_resp_timer = ack_resp_timer + 50; // count approximate uSec each iteration of loop
  }
  
  // If no ack received print status msg
  if(ack ==0)
  {
    strcpy_P(buffer, (char*)pgm_read_word(&(error_table[5]))); // acknowledge match failed
    Serial.println( buffer );
    delayMicroseconds(SERIAL_PAUSE);
    bitClear(flags,i2c_new_cmd_rx);  // Clear new cmd bit once CRC is read, dont clear if cmd received is not CRC
  }
  
  strcpy_P(buffer, (char*)pgm_read_word(&(pay_cmd_table[7]))); // SPI data transfer msg 
  Serial.println( buffer );
  delayMicroseconds(SERIAL_PAUSE);

  return 1; // return length of command
}
/*----------------------------------------------------*/   

/*----------------------------------------------------*/
// Payload Shutdown
uint8_t pay_shutdn(void)
{
 unsigned int crc = 0;
    
  if(!bitRead(header_flags,ERROR))  // if error bit clear send ack packet
  {
    // If data received with valid CRC return acknowledge packet to platform.
    txd_buffer[0] = rxd_buffer[0];      // 
    txd_buffer[1] = rxd_buffer[1];      //
    txd_buffer[2] = 0x7E;               // Identifier byte of 0x7E = acknowledge
    i2c_tx_available = 5;
    crc = calculate_CRC(1, i2c_tx_available-2);
    txd_buffer[3] = crc>>8;
    txd_buffer[4] = crc;
  }
  else
  {
    // If simulating an error at payload return error packet to platform.
    txd_buffer[0] = rxd_buffer[0];      // 
    txd_buffer[1] = rxd_buffer[1];      //
    bitSet(txd_buffer[1],ERROR);        // set error bit in header flags
    txd_buffer[2] = 0xAA;               // Identifier byte of 0xAA = example error code
    i2c_tx_available = 5;
    crc = calculate_CRC(1, i2c_tx_available-2);
    txd_buffer[3] = crc>>8;
    txd_buffer[4] = crc;  
  }
  
  bitSet(flags,i2c_gone);
   
  delay(1);

  while(bitRead(flags,i2c_gone))  //wait for i2c transmission to complete
    delayMicroseconds(50);

  delay(2);
  
  i2c_tx_available = 0;
    
  if(!bitRead(header_flags,ERROR))  // if no error bit set process as normal
  {
    ; // take payload shutdown actions here
  }
  else
  {
    ; //ignore command if we are simulating an error at the payload.
  } 

  strcpy_P(buffer, (char*)pgm_read_word(&(pay_cmd_table[8]))); // payload shutdown 
  Serial.println( buffer );
  delayMicroseconds(SERIAL_PAUSE);
    
  return 0; // return length of command
}
/*----------------------------------------------------*/


/*----------------------------------------------------*/
// Clear I2C receive buffer
void clear_rxd_buf(void)
{
  int m;

  for(m = 0; m < (BUFSIZE); m++)
  {
    rxd_buffer[m] = 0x00;    // clear rxd_buffer[]
  }
}
/*----------------------------------------------------*/

/*----------------------------------------------------*/
// Clear I2C transmit buffer
void clear_txd_buf(void)
{
  int m;

  for(m = 0; m < (BUFSIZE); m++)
  {
    txd_buffer[m] = 0x00;    // clear txd_buffer[]
  }
}
/*----------------------------------------------------*/

/*----------------------------------------------------*/
void display_status(void)
{ 
    strcpy_P(buffer, (char*)pgm_read_word(&(status_table[0]))); // payload status heading 
    Serial.println( buffer );
    delayMicroseconds(SERIAL_PAUSE);
   
    strcpy_P(buffer, (char*)pgm_read_word(&(status_table[1]))); // divider
    Serial.println( buffer );
    delayMicroseconds(SERIAL_PAUSE); 
    
    strcpy_P(buffer, (char*)pgm_read_word(&(status_table[2]))); // op mode
    Serial.print( buffer );
    delayMicroseconds(SERIAL_PAUSE);
    Serial.println(payload_op_mode,HEX);
    
    strcpy_P(buffer, (char*)pgm_read_word(&(status_table[3]))); // op flags
    Serial.print( buffer );
    delayMicroseconds(SERIAL_PAUSE);
    Serial.println(payload_op_flag,HEX);
    
    strcpy_P(buffer, (char*)pgm_read_word(&(status_table[4]))); // priority data waiting
    Serial.print( buffer );
    delayMicroseconds(SERIAL_PAUSE);
    Serial.println(priority_waiting,DEC);

    strcpy_P(buffer, (char*)pgm_read_word(&(status_table[5]))); // payload data waiting 
    Serial.print( buffer );
    delayMicroseconds(SERIAL_PAUSE);
    Serial.println(data_waiting,DEC);
    
    strcpy_P(buffer, (char*)pgm_read_word(&(status_table[6]))); // payload request flags
    Serial.print( buffer );
    delayMicroseconds(SERIAL_PAUSE);
    Serial.println(pay_request,BIN);

    strcpy_P(buffer, (char*)pgm_read_word(&(status_table[7]))); // parameter ID
    Serial.print( buffer );
    delayMicroseconds(SERIAL_PAUSE);
    Serial.println(pay_para_ID[0],HEX);

    strcpy_P(buffer, (char*)pgm_read_word(&(status_table[7]))); // parameter ID
    Serial.print( buffer );
    delayMicroseconds(SERIAL_PAUSE);
    Serial.println(pay_para_ID[1],HEX);

    strcpy_P(buffer, (char*)pgm_read_word(&(status_table[7]))); // parameter ID
    Serial.print( buffer );
    delayMicroseconds(SERIAL_PAUSE);
    Serial.println(pay_para_ID[2],HEX);
   
    strcpy_P(buffer, (char*)pgm_read_word(&(status_table[7]))); // parameter ID
    Serial.print( buffer );
    delayMicroseconds(SERIAL_PAUSE);
    Serial.println(pay_para_ID[3],HEX);
    
    strcpy_P(buffer, (char*)pgm_read_word(&(status_table[7]))); // parameter ID
    Serial.print( buffer );
    delayMicroseconds(SERIAL_PAUSE);
    Serial.println(pay_para_ID[4],HEX);
    
    strcpy_P(buffer, (char*)pgm_read_word(&(status_table[7]))); // parameter ID
    Serial.print( buffer );
    delayMicroseconds(SERIAL_PAUSE);
    Serial.println(pay_para_ID[5],HEX);
    
    strcpy_P(buffer, (char*)pgm_read_word(&(status_table[7]))); // parameter ID
    Serial.print( buffer );
    delayMicroseconds(SERIAL_PAUSE);
    Serial.println(pay_para_ID[6],HEX);
    
    strcpy_P(buffer, (char*)pgm_read_word(&(status_table[7]))); // parameter ID
    Serial.print( buffer );
    delayMicroseconds(SERIAL_PAUSE);
    Serial.println(pay_para_ID[7],HEX);
 
    strcpy_P(buffer, (char*)pgm_read_word(&(status_table[8]))); // parameter value
    Serial.print( buffer );
    Serial.print(pay_para_ID[0],HEX);
    Serial.print(": ");
    delayMicroseconds(SERIAL_PAUSE);
    Serial.println(pay_para_value[0],HEX);  
   
    strcpy_P(buffer, (char*)pgm_read_word(&(status_table[8]))); // parameter value
    Serial.print( buffer );
    Serial.print(pay_para_ID[1],HEX);
    Serial.print(": ");
    delayMicroseconds(SERIAL_PAUSE);
    Serial.println(pay_para_value[1],HEX); 
    
    strcpy_P(buffer, (char*)pgm_read_word(&(status_table[8]))); // parameter value
    Serial.print( buffer );
    Serial.print(pay_para_ID[2],HEX);
    Serial.print(": ");
    delayMicroseconds(SERIAL_PAUSE);
    Serial.println(pay_para_value[2],HEX); 
    
    strcpy_P(buffer, (char*)pgm_read_word(&(status_table[8]))); // parameter value
    Serial.print( buffer );
    Serial.print(pay_para_ID[3],HEX);
    Serial.print(": ");
    delayMicroseconds(SERIAL_PAUSE);;
    Serial.println(pay_para_value[3],HEX); 
    
    strcpy_P(buffer, (char*)pgm_read_word(&(status_table[8]))); // parameter value
    Serial.print( buffer );
    Serial.print(pay_para_ID[4],HEX);
    Serial.print(": ");
    delayMicroseconds(SERIAL_PAUSE);
    Serial.println(pay_para_value[4],HEX); 
    
    strcpy_P(buffer, (char*)pgm_read_word(&(status_table[8]))); // parameter value
    Serial.print( buffer );
    Serial.print(pay_para_ID[5],HEX);
    Serial.print(": ");
    delayMicroseconds(SERIAL_PAUSE);
    Serial.println(pay_para_value[5],HEX); 
    
    strcpy_P(buffer, (char*)pgm_read_word(&(status_table[8]))); // parameter value
    Serial.print( buffer );
    Serial.print(pay_para_ID[6],HEX);
    Serial.print(": ");
    delayMicroseconds(SERIAL_PAUSE);
    Serial.println(pay_para_value[6],HEX); 

    strcpy_P(buffer, (char*)pgm_read_word(&(status_table[8]))); // parameter value
    Serial.print( buffer );
    Serial.print(pay_para_ID[7],HEX);
    Serial.print(": ");
    delayMicroseconds(SERIAL_PAUSE);
    Serial.println(pay_para_value[7],HEX); 

    strcpy_P(buffer, (char*)pgm_read_word(&(status_table[9]))); // payload data limit
    Serial.print( buffer );
    Serial.print(": ");
    delayMicroseconds(SERIAL_PAUSE);
    Serial.println(mass_mem_limit,DEC); 
    
    strcpy_P(buffer, (char*)pgm_read_word(&(status_table[10]))); // payload data remaining
    Serial.print( buffer );
    Serial.print(": ");
    delayMicroseconds(SERIAL_PAUSE);
    Serial.println(mass_mem_remain,DEC); 
    
    strcpy_P(buffer, (char*)pgm_read_word(&(status_table[11]))); // priority data limit
    Serial.print( buffer );
    Serial.print(": ");
    delayMicroseconds(SERIAL_PAUSE);
    Serial.println(priority_limit,DEC); 
    
    strcpy_P(buffer, (char*)pgm_read_word(&(status_table[12]))); // priority data remaining
    Serial.print( buffer );
    Serial.print(": ");
    delayMicroseconds(SERIAL_PAUSE);
    Serial.println(priority_remain,DEC); 
    
    Serial.println();
    delayMicroseconds(50);
}
/*----------------------------------------------------*/

/*----------------------------------------------------*/
int availableMemory() 
{
 int size = 2048;
 byte *buf;
 while ((buf = (byte *) malloc(--size)) == NULL);
 free(buf);
 return size;
} 
/*----------------------------------------------------*/

//********************************************************
//******************* START I2C lib  mods ****************
//********************************************************

/*----------------------------------------------------*/
void I2C_begin(uint8_t address)
{
  twi_setAddress(address);
  twi_attachSlaveTxEvent(I2C_onRequestService);
  twi_attachSlaveRxEvent(I2C_onReceiveService);

  twi_init();
}
/*----------------------------------------------------*/

/*----------------------------------------------------*/
// behind the scenes function that is called when data is received
void I2C_onReceiveService(uint8_t* inBytes, int numBytes)
{
  // copy twi rx buffer into local read buffer
  // this enables new reads to happen in parallel
  for(uint8_t i = 0; i < numBytes; ++i){
    rxd_buffer[i] = inBytes[i];    
  }

  // alert user program
  bitSet(flags,i2c_new_cmd_rx);
  delayMicroseconds(10);

}
/*----------------------------------------------------*/

/*----------------------------------------------------*/
// behind the scenes function that is called when data is requested
void I2C_onRequestService(void)
{
  twi_transmit(txd_buffer, i2c_tx_available);
  delayMicroseconds(500);                            // delay needed for Wire.send to work reliably
  bitClear(flags,i2c_gone); 
}
/*----------------------------------------------------*/

/*----------------------------------------------------*/
uint8_t process_CRC(unsigned char TRflag, unsigned int length, unsigned int checksum)
{    
  if(calculate_CRC(TRflag, length) == checksum)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}
/*----------------------------------------------------*/

//*----------------------------------------------------*/
unsigned int calculate_CRC(unsigned char TRflag, unsigned int length)
{
  unsigned int checksum = 0;
  
  if(TRflag)
  {
    ;                   // Calculate CRC based on txd buffer of 'length' bytes
    checksum = crc16(0xFFFF, txd_buffer, length);  // Generate CRC-16 CCITT
  }
  else
  {
    ;                   // Calculate CRC based on rxd buffer of 'length' bytes
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
