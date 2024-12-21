//#include "Arduino.h"

/*
 * MEGA 2560 HARDWARE PORT/PIN MAPPING
 * PORT A0 A1 A2 A3 A4 A5 A6 A7
 * PIN  22,23,24,25,26,27,28,29
 * 
 * PORT B0 B1 B2 B3 B4 B5 B6 B7
 * PIN  53,52,51,50,10,11,12,13
 * 
 * PORT C0 C1 C2 C3 C4 C5 C6 C7
 * PIN  37,36,35,34,33,32,31,30
 * 
 * PORT D0 D1 D2 D3[D4 D5 D6]D7
 * PIN  21,20,19,18,--,--,--,38
 * 
 * PORT E0 E1 E2 E3 E4 E5 E6 E7
 * PIN   0, 1,--, 5, 2, 3,--,--
 * 
 * PORT F0 F1 F2 F3 F4 F5 F6 F7 ANALOG SECTION
 * PIN   0, 1, 2, 3, 4, 5, 6, 7
 * 
 * PORT G0 G1 G2 G3 G4 G5 G6 G7
 * PIN  41,40,39,--,--, 4,--,--
 * 
 * PORT H0 H1 H2 H3 H4 H5 H6 H7
 * PIN  17,16,--, 6, 7, 8, 9,--
 * 
 * PORT J0 J1 J2 J3 J4 J5 J6 J7
 * PIN  15,14,--,--,--,--,--,--
 * 
 * PORT K0 K1 K2 K3 K4 K5 K6 K7
 * PIN   8, 9,10,11,12,13,14,15 ANALOG SECTION
 * 
 * PORT L0 L1 L2 L3 L4 L5 L6 L7
 * PIN  49,48,47,46,45,44,43,42
 */
//-----------------------------------------------------------------------------------------------------
const String PROG_ID = "AmigaFloppyDrive Controller (Arduino Mega 2560)\n";
//-----------------------------------------------------------------------------------------------------
// d.d. 20241220 now local command handling added
// LOCAL: "DISK2ADF" works and saves .ADF file to /adf subdir in 59..65 seconds
// LOCAL: "00".."79" works as command and moves the head to the desired track number
// LOCAL: "=" works and saves a single track to /trk subdir as .ADF file
// TODO IDEAS:
// - skip bad tracks to save as much data as possible
// - repair function if encounter bad track
//
// early concept ideas: mostly just stripping AmigaFDD to get it working with HoodLoader2 (2MBaud)
// preserve core routines because we use a REAL (converted SFD-321B) Amiga drive
// emulating Paula & feeding Paula with ADF  should be possible
// goals:
// read/write ADF just like Amiga Floppy Reader ( Mega 2560 uses different HW addresses)
// 
// add ESP32 to process the MFM data stream just like on the PC using ADF library adapted for ESP32
// analyze track in memory
// save/load tracks to/from SD card
// 
// "let's see where the ship strands"
//-----------------------------------------------------------------------------------------------------
#define BAUDRATE_UART0 115200 //2000000                 // The baudrate that we want to communicate over (2M)
#define BAUDRATE_UART1 2000000                 // The baudrate that we want to communicate over (2M)
#define BAUD_PRESCALLER_NORMAL_MODE0      (((F_CPU / (BAUDRATE_UART0 * 16UL))) - 1)
#define BAUD_PRESCALLER_DOUBLESPEED_MODE0 (((F_CPU / (BAUDRATE_UART0 * 8UL))) - 1)
#define UART_USE_DOUBLESPEED_MODE0        // We're using double speed mode
#define BAUD_PRESCALLER_NORMAL_MODE1      (((F_CPU / (BAUDRATE_UART1 * 16UL))) - 1)
#define BAUD_PRESCALLER_DOUBLESPEED_MODE1 (((F_CPU / (BAUDRATE_UART1 * 8UL))) - 1)
#define UART_USE_DOUBLESPEED_MODE1        // We're using double speed mode
//-----------------------------------------------------------------------------------------------------
#define MOTOR_TRACK_DECREASE   HIGH      // Motor directions for PIN settings
#define MOTOR_TRACK_INCREASE   LOW
#define MODE_DEFAULT 1
#define MODE_DEBUG 0
//-----------------------------------------------------------------------------------------------------
//SINGLE BIT BITFIELD MASK generation (experimental)
//-----------------------------------------------------------------------------------------------------
#define _BV(bit) (1 << (bit))
//-----------------------------------------------------------------------------------------------------
// AmigaFDDControl Name | Pin   | FDD/Cable
//-----------------------------------------------------------------------------------------------------
// PIN 2 - INDEX PULSE     2    | 8    used to detect a specific point on the track for sync.  
//-----------------------------------------------------------------------------------------------------
#define FD_INDEX_PULSE     2     //08     // Not used yet some Amiga copy protection uses it.
#define PIN_INDEX_PORT           PINE
#define PIN_INDEX_MASK           _BV(PE4) //B00000100 // The mask used to set this pin high or low
//-----------------------------------------------------------------------------------------------------
// PIN 3 - WRITE DATA      3    | 22
//-----------------------------------------------------------------------------------------------------
#define FD_WRITE_DATA      3     //22     // Raw triggering of writing data to the disk
#define PIN_WRITE_DATA_PORT      PORTE    // The actual port the above pin is on
#define PIN_WRITE_DATA_MASK      _BV(PA0) //B00001000 // The mask used to set this pin high or low
//-----------------------------------------------------------------------------------------------------
// PIN 4 - READ DATA       4    | 30
//-----------------------------------------------------------------------------------------------------
#define FD_READ_DATA       4     //30     // Reads RAW floppy data on this pin
#define PIN_READ_DATA_PORT       PING     // The port the above pin is on
#define PIN_READ_DATA_MASK       _BV(PG5) //B00010000 // The mask used to set this pin high or low
//-----------------------------------------------------------------------------------------------------
// PIN 5, 6 and 7 - DRIVE, HEAD MOTOR DIRECTION and CONTROL
//-----------------------------------------------------------------------------------------------------
#define FD_MOTOR_ENABLE    5    //16
#define FD_MOTOR_DIRECTION 6    //18
#define FD_MOTOR_STEP      7    //20
//-----------------------------------------------------------------------------------------------------
// PIN 8 - Used to detect track 0 while moving the head
//-----------------------------------------------------------------------------------------------------
#define FD_TRACK_0         8    //26
//-----------------------------------------------------------------------------------------------------
// PIN 9 - HEAD SELECTION
//-----------------------------------------------------------------------------------------------------
#define FD_HEAD_SELECT     9    //32
//-----------------------------------------------------------------------------------------------------
// PIN 10 - CABLE SELECT /DRIVE SELECT
//-----------------------------------------------------------------------------------------------------
#define FD_DRV_SELECT_0    10   //10 AMIGA DF0: drive
//#define FD_DRV_SELECT_1  10   //12 PC FDD A: drive
//-----------------------------------------------------------------------------------------------------
// PIN 11 - DISK READY SIGNAL 
//-----------------------------------------------------------------------------------------------------
#define FD_DISK_READY      11   //34
//-----------------------------------------------------------------------------------------------------
// PIN 12 - DISK CHANGE SIGNAL AFTER REMOVE
//-----------------------------------------------------------------------------------------------------
#define FD_DISK_REMOVED    12   //02
//-----------------------------------------------------------------------------------------------------
// PIN A0 - WRITE GATE (Floppy Write Enable)
//-----------------------------------------------------------------------------------------------------
#define FD_WRITE_GATE      A0   //24 //ANALOG 0  // This pin enables writing to the disk
  #define PIN_WRITE_GATE_PORT      PORTF      // The actual port the above pin is on
  #define PIN_WRITE_GATE_MASK     _BV(PF0)    // B00000001  // The port pin mask for the gate
//-----------------------------------------------------------------------------------------------------
// PIN A1 - CHECK WRITE PROTECTION
//-----------------------------------------------------------------------------------------------------
#define FD_WRITE_PROTECT   A1  //28 //ANALOG 1  // To check if the disk is write protected
//-----------------------------------------------------------------------------------------------------
// CTS on 16u2 seems to be not possible with the original Amiga Floppy Reader ????
// CTS with FTDI solution should work fine with (hacked) nano or FTDI module
// multiple CTS ports is available on ESP32 so that should work
//-----------------------------------------------------------------------------------------------------
// PIN A2 - CTS Pin from UART ESP32-FTDI
//-----------------------------------------------------------------------------------------------------
#define PIN_CTS            A2   //ANALOG 2  // Pin linked to the CTS pin
#define PIN_CTS_PORT            PORTF       // Port the CTS pin is on
#define PIN_CTS_MASK            _BV(PF2)    //B00000100        // Binary mask to control it with
//-----------------------------------------------------------------------------------------------------
// AMIGA CASE - LED CONTROL
//-----------------------------------------------------------------------------------------------------
// PIN 13 - Activity LED
//-----------------------------------------------------------------------------------------------------
#define ACTIVITY_LED       13   //LED
//-----------------------------------------------------------------------------------------------------
// PIN 22 - Power LED
//-----------------------------------------------------------------------------------------------------
#define POWER_LED          22   //LED
//-----------------------------------------------------------------------------------------------------
// PIN 23 - Floppy Drive LED
//-----------------------------------------------------------------------------------------------------
#define FDD_LED            23   //LED
//-----------------------------------------------------------------------------------------------------
// PIN 24 - Hard Drive LED
//-----------------------------------------------------------------------------------------------------
#define HDD_LED            24   //LED
//-----------------------------------------------------------------------------------------------------
// AMIGA CASE - EXTRA / MIDI / MIXER
//-----------------------------------------------------------------------------------------------------
// PIN A8 - POTMETER 1 resolution 1/1024
//-----------------------------------------------------------------------------------------------------
#define PIN_POTMETER_1     A8   //ANALOG 8
//-----------------------------------------------------------------------------------------------------
bool DEBUG = false;
// These are read from the EEPROM so do not modifiy them
bool advancedControllerMode     = false;   // DO NOT CHANGE THIS, its automatically detected. If you can't connect pin12 to GND because you want to use the ISP headers, then see https://amiga.robsmithdev.co.uk/isp
bool drawbridgePlusMode         = false;   // DO NOT CHANGE THIS, its automatically set via EEPROM.  See the Windows Tool for more information
bool disableDensityDetection    = false;   // DO NOT CHANGE THIS, its automatically set vis EEPROM.  See the Windows Tool for more information
bool slowerDiskSeeking          = false;   // DO NOT CHANGE THIS, its automatically set vis EEPROM.  See the Windows Tool for more information
bool alwaysIndexAlignWrites     = false;   // DO NOT CHANGE THIS, its automatically set vis EEPROM.  See the Windows Tool for more information
bool DRIVE_BUSY = false;
//ADVANCED
// Detect advanced mode
//-----------------------------------------------------------------------------------------------------
// The timings here could be changed.  These are based on F_CPU=16Mhz, which leaves the resolution at 1 tick = 0.0625usec, hence 16=1uSec
// There's approx 4 clock ticks on average between noticing the flux transition and the counter value being read/reset
//-----------------------------------------------------------------------------------------------------
#define TIMING_OVERHEAD               -4
//-----------------------------------------------------------------------------------------------------
// Calculate the bit-timing windows.  These are the ideal exact centre of the next flux transition since the previous.
#define TIMING_DD_MIDDLE_2us     (2 * 16)
#define TIMING_DD_MIDDLE_4us     (4 * 16)
#define TIMING_DD_MIDDLE_6us     (6 * 16)
#define TIMING_DD_MIDDLE_8us     (8 * 16)
//-----------------------------------------------------------------------------------------------------
// Work out the upper window of the timing.  Most PLL allow for about 5% drift, but we're not interested in that and just want to recover the data
//-----------------------------------------------------------------------------------------------------
#define TIMING_DD_UPPER_2us     (TIMING_DD_MIDDLE_2us + 16 + TIMING_OVERHEAD) 
#define TIMING_DD_UPPER_4us     (TIMING_DD_MIDDLE_4us + 16 + TIMING_OVERHEAD) 
#define TIMING_DD_UPPER_6us     (TIMING_DD_MIDDLE_6us + 16 + TIMING_OVERHEAD) 
#define TIMING_DD_UPPER_8us     (TIMING_DD_MIDDLE_8us + 16 + TIMING_OVERHEAD) 
//-----------------------------------------------------------------------------------------------------
// HD versions
//-----------------------------------------------------------------------------------------------------
#define TIMING_HD_UPPER_2us     ((TIMING_DD_MIDDLE_4us/2) + 8 + TIMING_OVERHEAD) 
#define TIMING_HD_UPPER_4us     ((TIMING_DD_MIDDLE_6us/2) + 8 + TIMING_OVERHEAD) 
#define TIMING_HD_UPPER_6us     ((TIMING_DD_MIDDLE_8us/2) + 8 + TIMING_OVERHEAD) 
//-----------------------------------------------------------------------------------------------------
// LOW LEVEL
//-----------------------------------------------------------------------------------------------------
//#define LW_SIZE 4 //?????
#define POLLING_INTERVAL 2
//int RUNMODE = MODE_DEBUG;
//-----------------------------------------------------------------------------------------------------
// Paula on the Amiga used to find the SYNC WORDS and then read 0x1900 further WORDS.  
// A dos track is 11968 bytes in size, the critical revolution is 12800 bytes. 
/* The ATARI ST could format a track with up to 11 Sectors, so the AMIGA settings are OK. */
//-----------------------------------------------------------------------------------------------------
#define RAW_TRACKDATA_LENGTH   (0x1900*2+0x440)  // Paula assumed it was 12868 bytes, so we read that, plus the size of a sector, to find overlap
//-----------------------------------------------------------------------------------------------------
/* For the HD (1.4 MBytes) Disks the amount of data should be about 26688: */
#define RAW_HD_TRACKDATA_LENGTH   (0x1900*2*2+0x440)
//-----------------------------------------------------------------------------------------------------
// The current track that the head is over. Starts with -1 to identify an unknown head position.
//-----------------------------------------------------------------------------------------------------
int currentTrack = -1;
//-----------------------------------------------------------------------------------------------------
// If the drive has been switched on or not
//-----------------------------------------------------------------------------------------------------
bool driveEnabled  = 0;
//-----------------------------------------------------------------------------------------------------
/* Where there should be a HD Disk been read (1) or a DD and SD Disk (0).*/
//-----------------------------------------------------------------------------------------------------
bool disktypeHD = 0;
//-----------------------------------------------------------------------------------------------------
// If we're in WRITING mode or not
//-----------------------------------------------------------------------------------------------------
bool inWriteMode = 0;
//-----------------------------------------------------------------------------------------------------
int SETUP_COUNTER = 0;
//int data_pulse_count = 0;
//int DATA_PULSE = 1;
//int PREV_DATA_PULSE = 0;
//bool HAS_DATA_PULSE = false;
//-----------------------------------------------------------------------------------------------------
//int index_pulse_count = 0;
//int INDEX_PULSE = 0;
//int PREV_INDEX_PULSE = -1;
//bool HAS_INDEX_PULSE = false;
//-----------------------------------------------------------------------------------------------------
bool DISK_NOT_REMOVED = false;
bool DISK_READY = false;
//-----------------------------------------------------------------------------------------------------
int potmeter1Value = 0;  // variable to store the value coming from the sensor
int prev_potmeter1Value = 0;  // variable to store the last value coming from the sensor
//-----------------------------------------------------------------------------------------------------
int POLLING_COUNT = 0; // was int
//-----------------------------------------------------------------------------------------------------
unsigned long previousMillis = 0;        // will store last time LED was updated
unsigned long currentMillisecs = 0; //counter
unsigned long interval = 1000;
//-----------------------------------------------------------------------------------------------------
String MOTD = "";
String statusMessage = "";
String prevstatusMessage = "";
String inputString = "";         // a String to hold incoming data
String msgString = "";         // a String to hold any message data
//bool stringComplete = false;  // whether the string is complete
//String input1String = "";         // a String to hold incoming data from serialport 1
//bool string1Complete = false;  // whether the string is complete from serialport 1
String ADOS_CMD = "";
String ADOS_ARGS = "";
byte FLOPPY_SIDE = 0;
void HWserialEvent0();
void HWserialEvent1();
// 256 byte circular buffer - don't change this, we abuse the unsigned char to overflow back to zero!
#define SERIAL_BUFFER_SIZE 256
#define SERIAL_BUFFER_START (SERIAL_BUFFER_SIZE-16)
unsigned char SERIAL_BUFFER[SERIAL_BUFFER_SIZE];
#define SERIAL1_BUFFER_SIZE 256
#define SERIAL1_BUFFER_START (SERIAL1_BUFFER_SIZE-16)
unsigned char SERIAL1_BUFFER[SERIAL1_BUFFER_SIZE];
//-----------------------------------------------------------------------------------------------------
const char* DISK_READ_ONLY = "DISK_READ_ONLY";
const char* DISK_READ_WRITE = "DISK_READ_WRITE";
const char*  ERROR_GOTO_TRACK_ZERO = "ECHO ERROR_GOTO_TRACK_0\n";
const char*  ABORT_CURRENT_OPERATION = "ABORT CURRENT OPERATION (TOO MANY RETRIES FAILED!)\n";
// Read the command from the PC
byte command='0';
//-----------------------------------------------------------------------------------------------------
// Because we turned off interrupts delay() doesnt work!
//-----------------------------------------------------------------------------------------------------
// Because we turned off interrupts delay() doesnt work! This is accurate at the millisecond level
void smalldelay(unsigned long delayTime) {
    // Use timer 0 to count the correct number of ms
    TCCR0A = 0;         // Simple counter
    TCCR0B = bit(CS01) | bit(CS00); // Prescaler of divide by 64.  So if F_CPU=16000000, 250 clock ticks occur in 1ms second

    for (unsigned long i=0; i<delayTime; i++) {
       TCNT0 = 0;             // Reset counter;
       while (TCNT0<250) {};  // wait 1ms, we could do this more accuratly, but im not bothered
    }
    TCCR0B = 0; // turn off
}
/*void smalldelay(unsigned long delayTime) {
    delayTime*=(F_CPU/(9*1000L));
    for (unsigned long loops=0; loops<delayTime; ++loops) {
        asm volatile("nop\n\t"::);
    }
}*/
//-----------------------------------------------------------------------------------------------------
// Step the head once.  This seems to be an acceptable speed for the head
// Drive spec says pulse should be at least 3ms, but the time between pulses must be greater than 1us.  16 NOPS is approx 1us, so im just being cautious
//-----------------------------------------------------------------------------------------------------
void stepDirectionHead() {
    asm volatile("nop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\t"::);
    digitalWrite(FD_MOTOR_STEP,LOW);
    smalldelay(3);    // drive pulse must be at least 3ms long
    digitalWrite(FD_MOTOR_STEP,HIGH);
    asm volatile("nop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\t"::);
}
//-----------------------------------------------------------------------------------------------------
// Prepare serial port - We dont want to use the arduino serial library as we want to use faster speeds and no serial interrupts
//-----------------------------------------------------------------------------------------------------
void prepSerialInterface_UART0() { 
#ifdef UART_USE_DOUBLESPEED_MODE0
    UBRR0H = (uint8_t)(BAUD_PRESCALLER_DOUBLESPEED_MODE0>>8);
    UBRR0L = (uint8_t)(BAUD_PRESCALLER_DOUBLESPEED_MODE0);
    UCSR0A |= 1<<U2X0;
#else
    UBRR0H = (uint8_t)(BAUD_PRESCALLER_NORMAL_MODE0>>8);
    UBRR0L = (uint8_t)(BAUD_PRESCALLER_NORMAL_MODE0);
    UCSR0A &= ~(1<<U2X0);
#endif 

    // UCSROA is a status register only (apart from U2Xn):
    //  • Bit 7 – RXCn: USART Receive Complete 
    //  • Bit 6 – TXCn: USART Transmit Complete
    //  • Bit 5 – UDREn: USART Data Register Empty
    //  • Bit 4 – FEn: Frame Error
    //  • Bit 3 – DORn: Data OverRun/
    //  • Bit 2 – UPEn: USART Parity Error
    //  • Bit 1 – U2Xn: Double the USART Transmission Speed
    //  • Bit 0 – MPCMn: Multi-processor Communication Mode

    UCSR0B  = (0<<RXCIE0)  |   // Disable ReceiveCompleteInteruptEnable
              (0<<TXCIE0)  |   // Disable TransmitCompleteInteruptEnable
              (0<<UDRIE0)  |   // Disable UsartDataRegisterEmptyInteruptEnable
              (1<<RXEN0)   |   // Enable RX
              (1<<TXEN0)   |   // Enable TX
              (0<<UCSZ02)  ;   // Clear the 9-bit character mode bit

    UCSR0C =  (0<<UMSEL01) | (0<<UMSEL00) |   // UsartModeSelect - Asynchronous (00=Async, 01=Sync, 10=Reserved, 11=Master SPI)
              (0<<UPM01)   | (0<<UPM00)   |   // UsartParatyMode - Disabled  (00=Off, 01=Reserved, 10=Even, 11=Odd)
              (0<<USBS0)   |   // UsartStopBitSelect (0=1 Stop bit, 1 = 2Stop Bits)
              (1<<UCSZ01)  | (1<<UCSZ00);    // UsartCharacterSiZe  - 8-bit (00=5Bit, 01=6Bit, 10=7Bit, 11=8Bit, must be 11 for 9-bit)
}
//-----------------------------------------------------------------------------------------------------
// Prepare serial port UART1 - Not using arduino serial library as we want to use faster speeds and no serial interrupts
//-----------------------------------------------------------------------------------------------------
void prepSerialInterface_UART1() { 
#ifdef UART_USE_DOUBLESPEED_MODE1
    UBRR1H = (uint8_t)(BAUD_PRESCALLER_DOUBLESPEED_MODE1>>8);
    UBRR1L = (uint8_t)(BAUD_PRESCALLER_DOUBLESPEED_MODE1);
    UCSR1A |= 1<<U2X1;
#else
    UBRR1H = (uint8_t)(BAUD_PRESCALLER_NORMAL_MODE1>>8);
    UBRR1L = (uint8_t)(BAUD_PRESCALLER_NORMAL_MODE1);
    UCSR1A &= ~(1<<U2X1);
#endif 
//-----------------------------------------------------------------------------------------------------
    // UCSR1A is a status register only (apart from U2Xn):
    //  • Bit 7 – RXCn: USART Receive Complete 
    //  • Bit 6 – TXCn: USART Transmit Complete
    //  • Bit 5 – UDREn: USART Data Register Empty
    //  • Bit 4 – FEn: Frame Error
    //  • Bit 3 – DORn: Data OverRun/
    //  • Bit 2 – UPEn: USART Parity Error
    //  • Bit 1 – U2Xn: Double the USART Transmission Speed
    //  • Bit 0 – MPCMn: Multi-processor Communication Mode
//-----------------------------------------------------------------------------------------------------
    UCSR1B  = (0<<RXCIE1)  |   // Disable ReceiveCompleteInteruptEnable
              (0<<TXCIE1)  |   // Disable TransmitCompleteInteruptEnable
              (0<<UDRIE1)  |   // Disable UsartDataRegisterEmptyInteruptEnable
              (1<<RXEN1)   |   // Enable RX
              (1<<TXEN1)   |   // Enable TX
              (0<<UCSZ12)  ;   // Clear the 9-bit character mode bit
//-----------------------------------------------------------------------------------------------------
    UCSR1C =  (0<<UMSEL11) | (0<<UMSEL10) |   // UsartModeSelect - Asynchronous (00=Async, 01=Sync, 10=Reserved, 11=Master SPI)
              (0<<UPM11)   | (0<<UPM10)   |   // UsartParityMode - Disabled  (00=Off, 01=Reserved, 10=Even, 11=Odd)
              (0<<USBS1)   |   // UsartStopBitSelect (0=1 Stop bit, 1 = 2Stop Bits)
              (1<<UCSZ11)  | (1<<UCSZ10);    // UsartCharacterSiZe  - 8-bit (00=5Bit, 01=6Bit, 10=7Bit, 11=8Bit, must be 11 for 9-bit)
}
//-----------------------------------------------------------------------------------------------------
// Directly READ a byte from the UART0 (NON BLOCKING)
//-----------------------------------------------------------------------------------------------------
inline byte readByteFromUART0_NonBlocking() {
    if (!( UCSR0A & ( 1 << RXC0 ))){    // Wait for data to be available
      return 0;
    }
    else{
      return UDR0;                                 // Read it
    }
}
//-----------------------------------------------------------------------------------------------------
// Directly READ a byte from the UART0 (BLOCKING)
//-----------------------------------------------------------------------------------------------------
inline byte readByteFromUART() {
    while (!( UCSR0A & ( 1 << RXC0 ))){};    // Wait for data to be available
    return UDR0;                                 // Read it
}
//-----------------------------------------------------------------------------------------------------
// Directly WRITE a byte to the UART0 (MEGA2560)
//-----------------------------------------------------------------------------------------------------
inline void writeByteToUART(const char value) {
    while(!(UCSR0A & (1<<UDRE0)));                // Wait until the last byte has been sent
    UDR0 = value;                                 // And send another
}
//-----------------------------------------------------------------------------------------------------
// Directly READ a byte from the UART1 (NON BLOCKING)
//-----------------------------------------------------------------------------------------------------
inline byte readByteFromUART1_NonBlocking() {
    if (!( UCSR1A & ( 1 << RXC1 ))){    // Wait for data to be available
      return 0;
    }
    else{
      return UDR1;                                 // Read it
    }
}
//-----------------------------------------------------------------------------------------------------
// Directly READ a byte from the UART1 (BLOCKING)
//-----------------------------------------------------------------------------------------------------
inline byte readByteFromUART1() {
    while (!( UCSR1A & ( 1 << RXC1 ))) {};    // Wait for data to be available
    return UDR1;                                 // Read it
}
//-----------------------------------------------------------------------------------------------------
// Directly WRITE a byte to the UART1 (MEGA2560->ESP32)
//-----------------------------------------------------------------------------------------------------
inline void writeByteToUART1(const char value) {
    while(!(UCSR1A & (1<<UDRE1))) {};                // Wait until the last byte has been sent
    UDR1 = value;                                 // And send another
}
//-----------------------------------------------------------------------------------------------------
// Nasty string sending function
//-----------------------------------------------------------------------------------------------------
void sendString(const char* str) {
  while (*str!='\0') {
    writeByteToUART(*str);
    str++;
  }
}
//-----------------------------------------------------------------------------------------------------
// Nasty string sending function
//-----------------------------------------------------------------------------------------------------
void sendString1(const char* str) {
  while (*str!='\0') {
//    writeByteToUART(*str);
    writeByteToUART1(*str);
    str++;
  }
}
//-----------------------------------------------------------------------------------------------------
// Nasty int sending function
//-----------------------------------------------------------------------------------------------------
void sendInt(unsigned int i) {
  char buffer[6];
  buffer[0] = '0' + (i/10000)%10;
  buffer[1] = '0' + (i/1000)%10;
  buffer[2] = '0' + (i/100)%10;
  buffer[3] = '0' + (i/10)%10;
  buffer[4] = '0' + (i/1)%10;
  buffer[5] = '\0';
  for (int a=0; a<4; a++)
    if (buffer[a]=='0') buffer[a] = ' '; else break;
   sendString(buffer);
}
//-----------------------------------------------------------------------------------------------------
// Nasty send 'ticks' as uSec
//-----------------------------------------------------------------------------------------------------
void sendTickAsuSec(unsigned int i) {
  i*=62.5;
  int h = (i/10000)%10;
  writeByteToUART(h ? '0' + h : ' ');
  writeByteToUART('0' + (i/1000)%10);
  writeByteToUART('.');
  writeByteToUART('0' + (i/100)%10);
  writeByteToUART('0' + (i/10)%10);
  writeByteToUART('0' + (i/1)%10);
}
//-----------------------------------------------------------------------------------------------------
// goToTrack0() - Rewinds the head back to Track 0
//-----------------------------------------------------------------------------------------------------
bool goToTrack0() {
    //sendString("RESET_TO_TRACK_ZERO\n");
    digitalWrite(FD_MOTOR_DIRECTION, MOTOR_TRACK_DECREASE);   // Set the direction to go backwards
    int counter=0;
    while (digitalRead(FD_TRACK_0) != LOW) {
       stepDirectionHead();   // Keep moving the head until we see the TRACK 0 detection pin
       smalldelay(1);         // slow down a little
       counter++;
       // If this happens we;ve steps twice as many as needed and still havent found track 0
       if (counter>170) {
          //stopDriveForOperation();
          return false;
       }
    }
    currentTrack = 0;    // Reset the track number
//    sendString1("RESET_TO_TRACK_ZERO\n");
   return true;
}
//-----------------------------------------------------------------------------------------------------
// ADOS_CMD from ESP32 Goto to a specific track.
// During testing it was easier for the track number to be supplied as two ASCII characters, so I left it like this
//-----------------------------------------------------------------------------------------------------
//bool gotoTrackXX(const String &newtrack, bool reportDiskChange) {
bool gotoTrackXX(const String &newtrack) {
    sendString(String("TRACK_"+newtrack+"\n").c_str());
    int track = newtrack.toInt();
  //sendString1((String(track)+("_int\n")).c_str());
    byte flags = 1;  // default to normal speed
    // Work so its compatiable with previous versions
    const unsigned char delayTime = 4 - (flags & 3);
    // Exit if its already been reached
    if (track == currentTrack) {
      return true;
    }
    // If current track is unknown go to track 0 first
    if (currentTrack == -1) goToTrack0();
//    startDriveForOperation();
    // And step the head until we reach this track number
    if (currentTrack < track) {
        digitalWrite(FD_MOTOR_DIRECTION,MOTOR_TRACK_INCREASE);   // Move IN
        while (currentTrack < track) {
            stepDirectionHead();
            if (delayTime) smalldelay(delayTime);
            currentTrack++;         
        }
    } else {
     digitalWrite(FD_MOTOR_DIRECTION,MOTOR_TRACK_DECREASE);   // Move OUT
     while (currentTrack > track) {
      stepDirectionHead();
      if (delayTime) smalldelay(delayTime);
       currentTrack--;
      }
     }
     return true;
}
//-----------------------------------------------------------------------------------------------------
// Check if the disk is write protected.  Sends '#' if its write protected, or '1' if its not.  If theres no disk in the drive this number is meaningless
//-----------------------------------------------------------------------------------------------------
void checkWriteProtectStatus1() {
  if (digitalRead(FD_WRITE_PROTECT) == LOW) {
        // Drive is write protected
        sendString1(DISK_READ_ONLY);
        //writeByteToUART('1');
  } else {
        // Drive can be written to
        sendString1(DISK_READ_WRITE);
        //writeByteToUART('#');
  }                
}
//-----------------------------------------------------------------------------------------------------
#define CHECKSERIAL1_ONLY() if (UCSR1A & bit(RXC0)) {\
                            SERIAL1_BUFFER[serial1WritePos++] = UDR1;\
                            serial1BytesInUse++;\
                          }   
//-----------------------------------------------------------------------------------------------------
#define CHECKSERIAL_ONLY() if (UCSR0A & bit(RXC0)) {\
                            SERIAL_BUFFER[serialWritePos++] = UDR0;\
                            serialBytesInUse++;\
                          }   
//-----------------------------------------------------------------------------------------------------
// 14 is the minimum number here.  Any less than this and the CHECKSERIAL_ONLY() code will impact the output.  The pulse width doesn't matter as long as its at least 0.125uSec (its the falling edge that triggers a bit write)   
// Because only the falling edge is important we achieve precomp by shifting the pulse starting position back or forward two clock ticks      
// Because it may go back 2 ticks, we increase this number here by 2.  12 ticks is 750 ns, 14 ticks is 875 ns and 16 is 1000ns (1us) 
// By doing this, the bit cell timing remains constant, but the actual write position is shifted +/- 125ns as required
#define PULSE_WIDTH 14
// This is where the above starts from the end of the timer
#define PULSE_WIDTH_VALUE (0xFF - (PULSE_WIDTH-1))
// This is where to start the counter from compensating for code delay of 6 ticks (measured) 
#define PULSE_BREAK (58-PULSE_WIDTH) 
//-----------------------------------------------------------------------------------------------------
#define CHECK_SERIAL1()          if (UCSR1A & ( 1 << RXC0 )) {\
                                    SERIAL1_BUFFER[serial1WritePos++] = UDR1;\
                                    serial1BytesInUse++;\
                                }\
                                if (serial1BytesInUse<SERIAL1_BUFFER_START)\
                                    PIN_CTS_PORT &= (~PIN_CTS_MASK);\
                                else PIN_CTS_PORT|=PIN_CTS_MASK;                   
//-----------------------------------------------------------------------------------------------------
#define CHECK_SERIAL()          if (UCSR0A & ( 1 << RXC0 )) {\
                                    SERIAL_BUFFER[serialWritePos++] = UDR0;\
                                    serialBytesInUse++;\
                                }\
                                if (serialBytesInUse<SERIAL_BUFFER_START)\
                                    PIN_CTS_PORT &= (~PIN_CTS_MASK);\
                                else PIN_CTS_PORT|=PIN_CTS_MASK;                   
//-----------------------------------------------------------------------------------------------------
// Small Macro to write a '1' pulse to the drive if a bit is set based on the supplied bitmask
//-----------------------------------------------------------------------------------------------------
#define WRITE_BIT(value,bitmask) if (currentByte & bitmask) {\
                                     while (TCNT2<value) {};\
                                     PIN_WRITE_DATA_PORT&=~PIN_WRITE_DATA_MASK;\
                                 } else {\
                                     while (TCNT2<value) {};\
                                     PIN_WRITE_DATA_PORT|=PIN_WRITE_DATA_MASK;\
                                 }                                                       
//-----------------------------------------------------------------------------------------------------
// void initSerialInterfaces()
//-----------------------------------------------------------------------------------------------------
void initSerialInterfaces() {
   prepSerialInterface_UART0();
   prepSerialInterface_UART1();
} 
//-----------------------------------------------------------------------------------------------------
// Write a track to disk from the UART - This works like the precomp version as the old method isn't fast enough.
// This version is 100% jitter free
// This runs the PWM for writing a single pulse
//-----------------------------------------------------------------------------------------------------
#define RUN_PULSE_DD1()                                                                                 \
        while (!(TIFR2 &  bit(TOV2))) {                                                                \
          if (UCSR1A & bit(RXC0)) {                                                                    \
             PIN_CTS_PORT|=PIN_CTS_MASK;                                                               \
             SERIAL1_BUFFER[serial1WritePos++] = UDR1;                                                   \
             serial1BytesInUse++;                                                                       \
          }                                                                                            \
        };                                                                                             \
        OCR2A = counter;    /* reset to zero when we get to counter */                                 \
        OCR2B = pulseStart;                                                                            \
        TIFR2 |= bit(TOV2);        
//-----------------------------------------------------------------------------------------------------
// void writePrecompTrack()
//-----------------------------------------------------------------------------------------------------
void writePrecompTrack() {
    // Check if its write protected.  You can only do this after the write gate has been pulled low
    if (digitalRead(FD_WRITE_PROTECT) == LOW) {
        writeByteToUART1('N'); 
        return;
    } else
    writeByteToUART1('Y');

    unsigned char highByte = readByteFromUART1();//range 0-65535
    unsigned char lowByte = readByteFromUART1();
    unsigned char waitForIndex = readByteFromUART1();
    unsigned short numBytes = (((unsigned short)highByte)<<8) | lowByte;
    
    unsigned char serial1ReadPos = 0;
    unsigned char serial1WritePos = SERIAL1_BUFFER_START;
    unsigned char serial1BytesInUse = SERIAL1_BUFFER_START;
    
    writeByteToUART('!');     
    digitalWrite(HDD_LED,HIGH);//the case is closed so we cannot see the default ACTIVITY_LED on the MEGA2560
    register unsigned char currentByte = readByteFromUART1();

    // Fill our buffer to give us a head start
    for (int a=0; a<SERIAL1_BUFFER_START; a++) {
        // Wait for it
        while (!( UCSR1A & ( 1 << RXC0 ))){}; 
        // Save the byte
        SERIAL1_BUFFER[a] = UDR1;
    }  

    numBytes--;

    PIN_CTS_PORT|=PIN_CTS_MASK;                // stop any more data coming in!

    // Reset the counter, ready for writing - abuse the Fast PWM to produce the wayforms we need.  All we do is change the value it resets at!
    TCCR2A = bit(COM2B1) | bit(WGM20) | bit(WGM21);  // (COM2B0|COM2B1) Clear OC2B. on compare match, set OC2B at BOTTOM.  WGM20|WGM21 is Fast PWM. 
    TCCR2B = bit(WGM22)| bit(CS20);         // WGM22 enables waveform generation.  CS20 starts the counter runing at maximum speed

    // Enable writing
    PIN_WRITE_DATA_PORT|=PIN_WRITE_DATA_MASK;
    
    // While the INDEX pin is high wait.  Might as well write from the start of the track
    if (waitForIndex || alwaysIndexAlignWrites) 
        while (PIN_INDEX_PORT & PIN_INDEX_MASK)  {};

    PIN_WRITE_GATE_PORT&=~PIN_WRITE_GATE_MASK;

    // Get it ready    
    OCR2A = 64;      // Just setup something to get this going        
    OCR2B = 62;         
    TCNT2 = 0;      
    TIFR2 |= bit(TOV2) | bit(OCF2B);

    // Loop through all bytes of data required.  Each byte contains two sequences to write
    do {
        // Group 1
        register unsigned char counter = 63 + ((currentByte&0x03) * 32);
        register unsigned char pulseStart = counter - 5;
        if (currentByte & 0x04) pulseStart-=2;    // Pulse should be early, so just move the pulse start back
        if (currentByte & 0x08) pulseStart+=2;    // Pulse should be late, so move the pulse start forward
        
        RUN_PULSE_DD1();
        if (serial1BytesInUse<SERIAL1_BUFFER_START) PIN_CTS_PORT &= (~PIN_CTS_MASK); else PIN_CTS_PORT|=PIN_CTS_MASK;    
              
        numBytes--;
        // Check for overflow errors
        if (UCSR1A & (bit(FE0)|bit(DOR0))) break;

        // Group 2
        counter = 63 + ((currentByte&0x30) * 2);
        pulseStart = counter - 5;
        if (currentByte & 0x40) pulseStart-=2;    // Pulse should be early, so just move the pulse start back
        if (currentByte & 0x80) pulseStart+=2;    // Pulse should be late, so move the pulse start forward
        RUN_PULSE_DD1();

        if (!serial1BytesInUse) break;
        
        currentByte = SERIAL1_BUFFER[serial1ReadPos++]; 
        serial1BytesInUse--;
    } while (numBytes);    

    PIN_WRITE_GATE_PORT|=PIN_WRITE_GATE_MASK;
    // Turn off the write head
    PIN_WRITE_DATA_PORT|=PIN_WRITE_DATA_MASK;

    TCCR2A = 0;              // disable and reset everything
    TCCR2B = 0;              // Stop timer 2     

    // Data wouldnt have been read quick enough
    if (numBytes) {
        if (UCSR1A & bit(FE0)) {
           writeByteToUART1('Y');   // Serial Framing Error
        } else
        if (UCSR1A & bit(DOR0)) {
            writeByteToUART1('Z');   // Serial IO overrun
        } else {
            writeByteToUART1('X');   // Serial data not received quickly enough
        }
    } else {
      // Done!
      writeByteToUART1('1');      
    }
    digitalWrite(HDD_LED,LOW);
    PIN_CTS_PORT &= (~PIN_CTS_MASK);     
}
//-----------------------------------------------------------------------------------------------------
// void ADF2DISK() - record tracks to floppy
//-----------------------------------------------------------------------------------------------------
void ADF2DISK() {
 FLOPPY_SIDE=HIGH;
 int newtrack=0;
 bool aborted=false;
 bool track_completed=false;
//-----------------------------------------------------------------------------------------------------
// SEND COMMAND ONCE TO ESP32, 
//-----------------------------------------------------------------------------------------------------
 sendString(String("WRITE_TRACK "+ADOS_ARGS+"\n").c_str());
 sendString1(String("WRITE_TRACK "+ADOS_ARGS+"\n").c_str());
//-----------------------------------------------------------------------------------------------------
// Check if its write protected.  You can only do this after the write gate has been pulled low
//-----------------------------------------------------------------------------------------------------
 if (digitalRead(FD_WRITE_PROTECT) == LOW) {
        writeByteToUART1('N'); 
        return;
 } else writeByteToUART1('Y');
//-----------------------------------------------------------------------------------------------------
 while (track_completed==false && aborted==false) {
//-----------------------------------------------------------------------------------------------------
// we need a loop where we itterate through the tracks, normally 0..79 2 sides starting with UPPER SIDE
//-----------------------------------------------------------------------------------------------------
 while (newtrack<80 && aborted==false) {
  if (FLOPPY_SIDE==HIGH){
   gotoTrackXX(String(newtrack));
  }
//-----------------------------------------------------------------------------------------------------
  digitalWrite(FD_HEAD_SELECT, FLOPPY_SIDE);
//-----------------------------------------------------------------------------------------------------
//we need a loop where get the valid sectors for this track
  track_completed=false;
//-----------------------------------------------------------------------------------------------------
    unsigned char highByte = readByteFromUART1();
    unsigned char lowByte = readByteFromUART1();
    unsigned char waitForIndex = readByteFromUART1();
    unsigned short numBytes = (((unsigned short)highByte)<<8) | lowByte;
    
    unsigned char serial1ReadPos = 0;
    unsigned char serial1WritePos = SERIAL1_BUFFER_START;
    unsigned char serial1BytesInUse = SERIAL1_BUFFER_START;
    //SEND CONFIRM TO ESP32
    writeByteToUART('!');     
    digitalWrite(HDD_LED,HIGH);//the A600 case is closed so we cannot see the default ACTIVITY_LED on the MEGA2560
    //LETS GO AND RUN WILD
    register unsigned char currentByte = readByteFromUART1();

    // Fill our buffer to give us a head start
    for (int a=0; a<SERIAL1_BUFFER_START; a++) {
        // Wait for it
        while (!( UCSR1A & ( 1 << RXC0 ))){}; 
        // Save the byte
        SERIAL1_BUFFER[a] = UDR1;
    }  

    numBytes--;

    PIN_CTS_PORT|=PIN_CTS_MASK;                // stop any more data coming in!

    // Reset the counter, ready for writing - abuse the Fast PWM to produce the wayforms we need.  All we do is change the value it resets at!
    TCCR2A = bit(COM2B1) | bit(WGM20) | bit(WGM21);  // (COM2B0|COM2B1) Clear OC2B. on compare match, set OC2B at BOTTOM.  WGM20|WGM21 is Fast PWM. 
    TCCR2B = bit(WGM22)| bit(CS20);         // WGM22 enables waveform generation.  CS20 starts the counter runing at maximum speed

    // Enable writing
    PIN_WRITE_DATA_PORT|=PIN_WRITE_DATA_MASK;
    
    // While the INDEX pin is high wait.  Might as well write from the start of the track
    if (waitForIndex || alwaysIndexAlignWrites) 
        while (PIN_INDEX_PORT & PIN_INDEX_MASK)  {};

    PIN_WRITE_GATE_PORT&=~PIN_WRITE_GATE_MASK;

    // Get it ready    
    OCR2A = 64;      // Just setup something to get this going        
    OCR2B = 62;         
    TCNT2 = 0;      
    TIFR2 |= bit(TOV2) | bit(OCF2B);

    // Loop through all bytes of data required.  Each byte contains two sequences to write
    do {
        // Group 1
        register unsigned char counter = 63 + ((currentByte&0x03) * 32);
        register unsigned char pulseStart = counter - 5;
        if (currentByte & 0x04) pulseStart-=2;    // Pulse should be early, so just move the pulse start back
        if (currentByte & 0x08) pulseStart+=2;    // Pulse should be late, so move the pulse start forward
        
        RUN_PULSE_DD1();
        if (serial1BytesInUse<SERIAL1_BUFFER_START) PIN_CTS_PORT &= (~PIN_CTS_MASK); else PIN_CTS_PORT|=PIN_CTS_MASK;    
              
        numBytes--;
        // Check for overflow errors
        if (UCSR1A & (bit(FE0)|bit(DOR0))) break;

        // Group 2
        counter = 63 + ((currentByte&0x30) * 2);
        pulseStart = counter - 5;
        if (currentByte & 0x40) pulseStart-=2;    // Pulse should be early, so just move the pulse start back
        if (currentByte & 0x80) pulseStart+=2;    // Pulse should be late, so move the pulse start forward
        RUN_PULSE_DD1();

        if (!serial1BytesInUse) break;
        
        currentByte = SERIAL1_BUFFER[serial1ReadPos++]; 
        serial1BytesInUse--;
    } while (numBytes);    

    PIN_WRITE_GATE_PORT|=PIN_WRITE_GATE_MASK;
    // Turn off the write head
    PIN_WRITE_DATA_PORT|=PIN_WRITE_DATA_MASK;

    TCCR2A = 0;              // disable and reset everything
    TCCR2B = 0;              // Stop timer 2     

    // Data wouldnt have been read quick enough
    if (numBytes) {
        if (UCSR1A & bit(FE0)) {
           writeByteToUART1('Y');   // Serial Framing Error
        } else
        if (UCSR1A & bit(DOR0)) {
            writeByteToUART1('Z');   // Serial IO overrun
        } else {
            writeByteToUART1('X');   // Serial data not received quickly enough
        }
    } else {
      // Done!
      writeByteToUART1('1');      
    }
    digitalWrite(HDD_LED,LOW);
    PIN_CTS_PORT &= (~PIN_CTS_MASK);     
//-----------------------------------------------------------------------------------------------------
// check track recording result from ESP32
//-----------------------------------------------------------------------------------------------------
   char cResult=readByteFromUART1();
   if (cResult=='!'){ //IF THE TRACK WAS NOT OK THEN TRY AGAIN WITH SAME SETTINGS OR FORCED ABORT
   }
   else if (cResult=='#'){
    aborted=true;
    sendString(ABORT_CURRENT_OPERATION);
    //ABORT PROCEDURE
   }
   else{track_completed=true;}
  }
//-----------------------------------------------------------------------------------------------------
  //we received a completed any-token we are done
  if (track_completed){
   if (FLOPPY_SIDE==HIGH){FLOPPY_SIDE=LOW;} // same track same recording procedure but now for the other side
   else{newtrack++; FLOPPY_SIDE=HIGH;}
  }
 }
}
//-----------------------------------------------------------------------------------------------------
//void DECODE_TRACK_MEGA_2560()
//-----------------------------------------------------------------------------------------------------
void DECODE_TRACK_MEGA_2560() {
// FLOPPY_SIDE=LOW;
// int newtrack=0;
 bool aborted=false;
 bool completed=false;
// if (goToTrack0()==false){
//  sendString(ERROR_GOTO_TRACK_ZERO);
///  return;
// }
//-----------------------------------------------------------------------------------------------------
// SEND COMMAND ONCE TO ESP32, 
 sendString1(String("DECODE_TRACK TRACK_"+String((currentTrack<10) ? "0" : "")+String(currentTrack)+"_SIDE_"+String(FLOPPY_SIDE)+".RAW\n").c_str());
// sendString1("READ_DISK DUMMY\n");


//-----------------------------------------------------------------------------------------------------
// we need a loop where get the valid sectors for this track
//-----------------------------------------------------------------------------------------------------
 while (completed==false && aborted==false) {
//  if (FLOPPY_SIDE==LOW){
//   gotoTrackXX(String(newtrack));
//  }
//  digitalWrite(FD_HEAD_SELECT, FLOPPY_SIDE);
//-----------------------------------------------------------------------------------------------------
 // Configure timer 2 just as a counter in NORMAL mode
//-----------------------------------------------------------------------------------------------------
    TCCR2A = 0 ;              // No physical output port pins and normal operation
    TCCR2B = bit(CS20);       // Precale = 1  
    OCR2A = 0x00;
    OCR2B = 0x00;
//-----------------------------------------------------------------------------------------------------
//  First wait for the serial port to be available
    while(!(UCSR1A & (1<<UDRE1)));   
//-----------------------------------------------------------------------------------------------------
//  Signal we're active via FDD_LED
    digitalWrite(FDD_LED,HIGH);
//-----------------------------------------------------------------------------------------------------
//  Force data to be stored in a register
    register unsigned char DataOutputByte = 0;
//-----------------------------------------------------------------------------------------------------
//  While the INDEX pin is high wait if the other end requires us to
    readByteFromUART1();
    //while (PIN_INDEX_PORT & PIN_INDEX_MASK)  {}; //deprecate this leads to more accurate data according to GRAB_TRACK_MEGA2560()
    register unsigned char counter;
    long totalBits=0;
    long target = ((long)RAW_TRACKDATA_LENGTH)*(long)8;
    TCNT2=0;       // Reset the counter
    while (totalBits<target) {
        for (register unsigned char bits=0; bits<4; bits++) {
            // Wait while pin is high
            while (PIN_READ_DATA_PORT & PIN_READ_DATA_MASK) {
            };
            counter = TCNT2, TCNT2 = 0;  // reset - must be done with a COMMA
            DataOutputByte<<=2;   
            // DO NOT USE BRACES HERE, use the "," or the optomiser messes it up.  Numbers changed as these are best centered around where the bitcels actually are
            if (counter<TIMING_DD_UPPER_4us) DataOutputByte|=B00000001,totalBits+=2; else    // this accounts for just a '1' or a '01' as two '1' arent allowed in a row
            if (counter<TIMING_DD_UPPER_6us) DataOutputByte|=B00000010,totalBits+=3; else            
            if (counter<TIMING_DD_UPPER_8us) DataOutputByte|=B00000011,totalBits+=4; else      
                                                                       totalBits+=5;   // this is treated as an 00001, which isnt allowed, but does work
            // Wait until pin is high again
            while (!(PIN_READ_DATA_PORT & PIN_READ_DATA_MASK)) {};
        }
        if (!DataOutputByte) {
          // sending 0 here is wrong and will cause the PC side to get confused.  so we set one ot be a 8us rather than the fake 10
          DataOutputByte = B00000011; 
          totalBits--;  // account for one less bit
        }
        UDR1 = DataOutputByte;
    }
    // Because of the above rules the actual valid two-bit sequences output are 01, 10 and 11, so we use 00 to say "END OF DATA"
    writeByteToUART1(0);
    // turn off the status FDD LED
    digitalWrite(FDD_LED,LOW);
    // Disable the counter
    TCCR2B = 0;      // No Clock (turn off)    
//-----------------------------------------------------------------------------------------------------
// check result from ESP32
   char cResult=readByteFromUART1();
   if (cResult=='!'){
    //IF THERE ARE NOT 11 VALID SECTORS YET SO CAPTURE AGAIN WITH SAME SETTINGS
   }
   else if (cResult=='#'){
    aborted=true;
    sendString(ABORT_CURRENT_OPERATION);
    //ABORT PROCEDURE
   }
   else{ //we received a completed any-token we are done
     //if (FLOPPY_SIDE==LOW){FLOPPY_SIDE=HIGH;} // same track same procedure but now for the other side
     //else{newtrack++; FLOPPY_SIDE=LOW;}
     completed=true;
   }
 }
}
//-----------------------------------------------------------------------------------------------------
// void DISK2ADF() GRAB WHOLE FLOPPYDISK / 80 Tracks and save to ADF 
//-----------------------------------------------------------------------------------------------------
void DISK2ADF() {
//-----------------------------------------------------------------------------------------------------
 FLOPPY_SIDE=HIGH;
 int newtrack=0;
 bool aborted=false;
 bool track_completed=false;
//-----------------------------------------------------------------------------------------------------
// SEND COMMAND ONCE TO ESP32, 
//-----------------------------------------------------------------------------------------------------
 sendString1("DISK2ADF TEST.ADF\n");
//-----------------------------------------------------------------------------------------------------
 // we need a loop where we itterate through the tracks, normally 0..79 2 sides starting with UPPER SIDE
//-----------------------------------------------------------------------------------------------------
 while (newtrack<80 && aborted==false) {
  if (FLOPPY_SIDE==HIGH){
   gotoTrackXX(String(newtrack));
  }
  digitalWrite(FD_HEAD_SELECT, FLOPPY_SIDE);
//-----------------------------------------------------------------------------------------------------
//we need a loop where get the valid sectors for this track
  track_completed=false;
//-----------------------------------------------------------------------------------------------------
  while (track_completed==false && aborted==false) {
//-----------------------------------------------------------------------------------------------------
 // Configure timer 2 just as a counter in NORMAL mode
//-----------------------------------------------------------------------------------------------------
    TCCR2A = 0 ;              // No physical output port pins and normal operation
    TCCR2B = bit(CS20);       // Precale = 1  
    OCR2A = 0x00;
    OCR2B = 0x00;
//-----------------------------------------------------------------------------------------------------
//  First wait for the serial port to be available
    while(!(UCSR1A & (1<<UDRE1)));   
//-----------------------------------------------------------------------------------------------------
//  Signal we're active via FDD_LED
    digitalWrite(FDD_LED,HIGH);
//-----------------------------------------------------------------------------------------------------
//  Force data to be stored in a register
    register unsigned char DataOutputByte = 0;
//-----------------------------------------------------------------------------------------------------
//  While the INDEX pin is high wait if the other end requires us to
    readByteFromUART1();
    //while (PIN_INDEX_PORT & PIN_INDEX_MASK)  {}; //deprecate this leads to more accurate data according to GRAB_TRACK_MEGA2560()
    register unsigned char counter;
    long totalBits=0;
    long target = ((long)RAW_TRACKDATA_LENGTH)*(long)8;
    TCNT2=0;       // Reset the counter
    while (totalBits<target) {
        for (register unsigned char bits=0; bits<4; bits++) {
            // Wait while pin is high
            while (PIN_READ_DATA_PORT & PIN_READ_DATA_MASK) {
            };
            counter = TCNT2, TCNT2 = 0;  // reset - must be done with a COMMA
            DataOutputByte<<=2;   
            // DO NOT USE BRACES HERE, use the "," or the optomiser messes it up.  Numbers changed as these are best centered around where the bitcels actually are
            if (counter<TIMING_DD_UPPER_4us) DataOutputByte|=B00000001,totalBits+=2; else    // this accounts for just a '1' or a '01' as two '1' arent allowed in a row
            if (counter<TIMING_DD_UPPER_6us) DataOutputByte|=B00000010,totalBits+=3; else            
            if (counter<TIMING_DD_UPPER_8us) DataOutputByte|=B00000011,totalBits+=4; else      
                                                                       totalBits+=5;   // this is treated as an 00001, which isnt allowed, but does work
            // Wait until pin is high again
            while (!(PIN_READ_DATA_PORT & PIN_READ_DATA_MASK)) {};
        }
        if (!DataOutputByte) {
          // sending 0 here is wrong and will cause the PC side to get confused.  so we set one ot be a 8us rather than the fake 10
          DataOutputByte = B00000011; 
          totalBits--;  // account for one less bit
        }
        UDR1 = DataOutputByte;
    }
    // Because of the above rules the actual valid two-bit sequences output are 01, 10 and 11, so we use 00 to say "END OF DATA"
    writeByteToUART1(0);
    // turn off the status FDD LED
    digitalWrite(FDD_LED,LOW);
    // Disable the counter
    TCCR2B = 0;      // No Clock (turn off)    
//-----------------------------------------------------------------------------------------------------
// check track capturing result from ESP32
//-----------------------------------------------------------------------------------------------------
   char cResult=readByteFromUART1();
   if (cResult=='!'){ //IF THERE ARE NOT 11 VALID SECTORS YET SO CAPTURE AGAIN WITH SAME SETTINGS
   }
   else if (cResult=='#'){
    aborted=true;
    sendString(ABORT_CURRENT_OPERATION);
    //ABORT PROCEDURE
   }
   else{track_completed=true;}
  }
//-----------------------------------------------------------------------------------------------------
  //we received a completed any-token we are done
  if (track_completed){
   if (FLOPPY_SIDE==HIGH){FLOPPY_SIDE=LOW;} // same track same procedure but now for the other side
   else{newtrack++; FLOPPY_SIDE=HIGH;}
  }
 }
}
//-----------------------------------------------------------------------------------------------------
// void GRAB_TRACK_MEGA_2560() - Read the track using a timings to calculate which MFM sequence has been triggered
//-----------------------------------------------------------------------------------------------------
void GRAB_TRACK_MEGA_2560() {
//    sendString1(String("DECODE_TRACK TRACK_"+String((currentTrack<10) ? "0" : "")+String(currentTrack)+"_SIDE_"+String(FLOPPY_SIDE)+".RAW\n").c_str());
//    sendString1(String("READ_TRACK_TO_BUFFER TRACK_"+String((currentTrack<10) ? "0" : "")+String(currentTrack)+"_SIDE_"+String(FLOPPY_SIDE)+".RAW\n").c_str());
    sendString1(String("READ_TRACK TRACK_"+String((currentTrack<10) ? "0" : "")+String(currentTrack)+"_SIDE_"+String(FLOPPY_SIDE)+".RAW\n").c_str());
    // Configure timer 2 just as a counter in NORMAL mode
    TCCR2A = 0 ;              // No physical output port pins and normal operation
    TCCR2B = bit(CS20);       // Precale = 1  
    OCR2A = 0x00;
    OCR2B = 0x00;
    // First wait for the serial port to be available
    while(!(UCSR1A & (1<<UDRE1)));   
   
    // Signal we're active via FDD_LED
    digitalWrite(FDD_LED,HIGH);

    // Force data to be stored in a register
    register unsigned char DataOutputByte = 0;
    //register unsigned int cnt = 0;
    // While the INDEX pin is high wait if the other end requires us to
    //the converted PC2AMIGA drive acts a bit differently since from Amiga-only-perspective NO INDEX is needed
    //SO I DEACTIVATED THE INDEX PART. 'CUZ MESSES UP THE BITSTREAM FROM MEGA2560
    //NOW THE GRABBING GIVES BETTER RESULTS finding the SECTOR KEY $44894489
    //as observed it is needed to grab multiple times to get ALL the VALID SECTORS!
    // now uses DMA buffer on the ESP32 lets see
    if (readByteFromUART1()) //here sOK="1" ANY BYTE or GIVE BYTE A FUNCTION
    while (PIN_INDEX_PORT & PIN_INDEX_MASK)  {}; //todo: experiment with random timer/delay VS a fixed amount of bytes 
    register unsigned char counter;
    long totalBits=0;
    long target = ((long)RAW_TRACKDATA_LENGTH)*(long)8;
    TCNT2=0;       // Reset the counter DIRECT AFTER THE ESP32 SIGNALS READY_TO_RECEIVE
    while (totalBits<target) {
        for (register unsigned char bits=0; bits<4; bits++) {
            // Wait while pin is high
            while (PIN_READ_DATA_PORT & PIN_READ_DATA_MASK) {};
            counter = TCNT2, TCNT2 = 0;  // reset - must be done with a COMMA
            DataOutputByte<<=2;   
            // DO NOT USE BRACES HERE, use the "," or the optomiser messes it up.
            //Numbers changed as these are best centered around where the bitcels actually are
            if (counter<TIMING_DD_UPPER_4us) DataOutputByte|=B00000001,totalBits+=2; else    // this accounts for just a '1' or a '01' as two '1' arent allowed in a row
            if (counter<TIMING_DD_UPPER_6us) DataOutputByte|=B00000010,totalBits+=3; else            
            if (counter<TIMING_DD_UPPER_8us) DataOutputByte|=B00000011,totalBits+=4; else      
                                                                       totalBits+=5;   // this is treated as an 00001, which isnt allowed, but does work
            // Wait until pin is high again
            while (!(PIN_READ_DATA_PORT & PIN_READ_DATA_MASK)) {};
        }
        if (!DataOutputByte) {
          // sending 0 here is wrong and will cause the PC side to get confused.  so we set one ot be a 8us rather than the fake 10
          DataOutputByte = B00000011; 
          totalBits--;  // account for one less bit
        }
        UDR1 = DataOutputByte;//cnt++;
    }
    // Because of the above rules the actual valid two-bit sequences output are 01, 10 and 11, so we use 00 to say "END OF DATA"
    writeByteToUART1(0);
    //sendString1(String(cnt).c_str());
    // turn off the status FDD LED
    digitalWrite(FDD_LED,LOW);
    // Disable the counter
    TCCR2B = 0;      // No Clock (turn off)    
}
//-----------------------------------------------------------------------------------------------------
// unsigned long millisecs() - replacement for millis() which does not work with interrupts turned-off
//-----------------------------------------------------------------------------------------------------
unsigned long millisecs(){
  smalldelay(1);
  currentMillisecs+=1;
  return currentMillisecs;
  };
//-----------------------------------------------------------------------------------------------------
// Polling() - Start Amiga trackdisk.device alike polling on Track 0/1 (THE CLICK IS BACK!)
//-----------------------------------------------------------------------------------------------------
bool Polling() {
   digitalWrite(FDD_LED,HIGH);
//   digitalWrite(FD_DRV_SELECT_0,LOW);
   digitalWrite(FD_MOTOR_ENABLE,LOW);
   if (currentTrack==-1){goToTrack0();}
   digitalWrite(FD_MOTOR_DIRECTION, MOTOR_TRACK_INCREASE);   // Set the direction to go forward
   stepDirectionHead();   // move the head 1 step ahead
   DISK_NOT_REMOVED=digitalRead(FD_DISK_REMOVED);
   digitalWrite(FD_MOTOR_DIRECTION, MOTOR_TRACK_DECREASE);   // Set the direction to go backwards
   stepDirectionHead();   // move the head 1 step back
   digitalWrite(FD_MOTOR_ENABLE,HIGH);
//   digitalWrite(FD_DRV_SELECT_0,HIGH);
   digitalWrite(FDD_LED,LOW);
  return true;
}
//-----------------------------------------------------------------------------------------------------
// void setup() - MEGA2560 SETUP
//-----------------------------------------------------------------------------------------------------
void setup() {
  // put your setup code here, to run once:
  // Do these right away to prevent the disk being written to
  digitalWrite(FD_WRITE_GATE, HIGH);
  digitalWrite(FD_WRITE_DATA, HIGH);
  //CTS_A2 connects to ESP_27
  pinMode(PIN_CTS,OUTPUT);
  pinMode(FD_WRITE_GATE,OUTPUT);
  pinMode(FD_WRITE_DATA,OUTPUT);
  pinMode(FD_WRITE_PROTECT, INPUT_PULLUP);
  pinMode(FD_TRACK_0, INPUT_PULLUP);
  pinMode(FD_READ_DATA,INPUT_PULLUP);
  pinMode(FD_INDEX_PULSE,INPUT_PULLUP);
  pinMode(FD_DISK_REMOVED,INPUT_PULLUP);
  pinMode(FD_DISK_READY,INPUT_PULLUP);
  // Prepare the pin inputs and outputs
  pinMode(FD_DRV_SELECT_0, OUTPUT);
  pinMode(FD_MOTOR_ENABLE, OUTPUT);
  pinMode(FD_HEAD_SELECT, OUTPUT);
  digitalWrite(FD_DRV_SELECT_0,LOW);//TURN ON DRIVE CABLE SELECT
  digitalWrite(FD_MOTOR_ENABLE,HIGH);//TURN OFF
  digitalWrite(FD_HEAD_SELECT,LOW);    
  pinMode(FD_MOTOR_DIRECTION, OUTPUT);
  pinMode(FD_MOTOR_STEP,OUTPUT);
  //LEDS
  pinMode(ACTIVITY_LED,OUTPUT);
   digitalWrite(ACTIVITY_LED,LOW);
  pinMode(POWER_LED, OUTPUT);
   digitalWrite(POWER_LED,HIGH);
  pinMode(FDD_LED, OUTPUT);
   digitalWrite(FDD_LED,LOW);
  pinMode(HDD_LED, OUTPUT);
   digitalWrite(HDD_LED,LOW);
//-----------------------------------------------------------------------------------------------------
// Disable all interrupts - we dont want them!
//-----------------------------------------------------------------------------------------------------
    cli();
    TIMSK0=0;
    TIMSK1=0;
    TIMSK2=0;
    PCICR = 0;
    PCIFR = 0;
    PCMSK0 = 0;
    PCMSK2 = 0;
    PCMSK1 = 0;
  initSerialInterfaces();
  sendString(PROG_ID.c_str());
}
//-----------------------------------------------------------------------------------------------------
// ACTIONS
//-----------------------------------------------------------------------------------------------------
enum consoleactionESP32
{
  wait,
  do_cmd,
  draw_bridge,
  do_debug
};
consoleactionESP32 actionESP32 = wait;
//-----------------------------------------------------------------------------------------------------
void loop() {
 PIN_CTS_PORT &= (~PIN_CTS_MASK);            // Allow data incoming
 PIN_WRITE_GATE_PORT|=PIN_WRITE_GATE_MASK;   // always turn writing off    
//-----------------------------------------------------------------------------------------------------
// do not send commands from both ports at the same time!
HWserialEvent1(); //Read SERIAL 1 for ADOS_CMD
if (ADOS_CMD=="") HWserialEvent0(); //if ADOS_CMD is not set then read SERIAL 0 (commands from LOCAL)
 if (actionESP32==do_cmd){
  if (DEBUG==true) sendString(String("DEBUG_MEGA2560: CMD="+ADOS_CMD+" ARG="+ADOS_ARGS+"\n").c_str());
//-----------------------------------------------------------------------------------------------------
// ADOS - COMMANDS
//-----------------------------------------------------------------------------------------------------
// ? - SHOW VERSION                                1
//--------------------------------------------------
if (ADOS_CMD == "?")  { //    writeByteToUART1('1');  // Success
  if (ADOS_ARGS="LOCAL"){
    sendString( String("AmigaFloppyDrive Controller v1.8\n").c_str() );
  }else{
   writeByteToUART1('V');  // Followed
    writeByteToUART1('1');  // By
     writeByteToUART1(advancedControllerMode ? ',' : '.');  // Advanced controller version
      writeByteToUART1('8');  // Number    
  }
}
//--------------------------------------------------
// #xx - GOTO TRACK xx    00/79                    3
//--------------------------------------------------
else if (ADOS_CMD[0]=='#'){
   if (ADOS_CMD.length()==3){
    ADOS_ARGS=String(ADOS_CMD[1])+String(ADOS_CMD[2]);
    if (driveEnabled==true) {
     sendString1("DRIVE_BUSY"); //to prevent the timeout on ESP32
     DRIVE_BUSY=true;
     smalldelay(10);//test
     if (gotoTrackXX(ADOS_ARGS)==true){
      sendString1(String("ECHO CURRENT_TRACK:#"+String((currentTrack<10) ? "0" : "")+String(currentTrack)+"\n").c_str());//writeByteToUART1('1');
     }
     else{
      sendString1(String("ECHO ERROR_SETTING_TRACK_TO:#"+ADOS_ARGS+"\n").c_str());//writeByteToUART1('0');
     }
      DRIVE_BUSY=false;
    }else{sendString1("NO_DISK");}
   }else{sendString1("SYNTAX_ERROR");}
  }
//--------------------------------------------------
// [ - SELECT LOWER SIDE                           1
//--------------------------------------------------
  else if (ADOS_CMD=="["){
   digitalWrite(FD_HEAD_SELECT,LOW);
   FLOPPY_SIDE=0;
   sendString1("ECHO LOWER_SIDE_SELECTED\n");
  }
//--------------------------------------------------
// [ - SELECT UPPER SIDE                           1
//--------------------------------------------------
  else if (ADOS_CMD=="]"){
   digitalWrite(FD_HEAD_SELECT,HIGH);
   FLOPPY_SIDE=1;
   sendString1("ECHO UPPER_SIDE_SELECTED\n");
  }
//--------------------------------------------------
// < - READ TRACK/SIDE                             1
//--------------------------------------------------
/*  else if (ADOS_CMD=="<"){
   if(!driveEnabled){
    sendString1("DRIVE_NOT_ENABLED");
   } 
   else {
    sendString1(String("READ_TRACK_"+String(currentTrack)+"_SIDE_"+String(FLOPPY_SIDE)).c_str());
    smalldelay(20);
    GRAB_TRACK_MEGA_2560();
    //readContinuousStream1();
    smalldelay(20);
    sendString1("ECHO READ_TRACK_OK\n");
   }    
  }*/
//--------------------------------------------------
// DECODE_TRACK - GRAB&DECODE CURRENT TRACK/SIDE  12
//--------------------------------------------------
  else if (ADOS_CMD=="="){
   if(!driveEnabled){
    sendString1("DRIVE_NOT_ENABLED");
   } 
   else {
    sendString1(String("DECODE_TRACK_"+String(currentTrack)+"_SIDE_"+String(FLOPPY_SIDE)).c_str());
    smalldelay(20);
    DECODE_TRACK_MEGA_2560();
    //readContinuousStream1();
    smalldelay(50);
    sendString1("ECHO DECODE_TRACK_OK\n");
   }    
  }
//--------------------------------------------------
// ADF2DISK - WRITE WHOLE DISK 80 TRACKS/SIDE      8
//--------------------------------------------------
  else if (ADOS_CMD=="ADF2DISK"){
   if (DISK_READY==true) {
    if(!driveEnabled){
      sendString1("DRIVE_NOT_ENABLED"); 
    }
    else {
     sendString1("ADF2DISK_INIT");
     smalldelay(20);
     ADF2DISK();
     smalldelay(20);
     sendString1("ECHO ADF2DISK_OK\n");
    }    
   }else{
    sendString1("ECHO NO_DISK\n");
   }
  }
//--------------------------------------------------
// DISK2ADF - GRAB WHOLE DISK 80 TRACKS/SIDE      8
//--------------------------------------------------
  else if (ADOS_CMD=="DISK2ADF"){
   if (DISK_READY==true) {
    if(!driveEnabled){
      sendString1("DRIVE_NOT_ENABLED"); 
    }
    else {
     sendString1("DISK2ADF_INIT");
     smalldelay(20);
     DISK2ADF();
     smalldelay(20);
     sendString1("ECHO DISK2ADF_OK\n");
    }    
   }else{
    sendString1("ECHO NO_DISK\n");
   }
  }
//--------------------------------------------------
// $ - CHECK WRITE PROTECTION STATUS               1
//--------------------------------------------------
  else if (ADOS_CMD=="$"){
   if (DISK_READY==true) {
    checkWriteProtectStatus1();
   }
   else{
    sendString1("NO_DISK");
   }
  }
//--------------------------------------------------
// + = DRIVE MOTOR ENABLE                          1
//--------------------------------------------------
  else if (ADOS_CMD == "+"){
                  if (!driveEnabled) { //digitalWrite(FD_DRV_SELECT_0,LOW);
                     digitalWrite(FDD_LED,HIGH);
                     digitalWrite(FD_MOTOR_ENABLE,LOW);
                     driveEnabled = 1;
                     smalldelay(500);  // wait for drive
                  }
    
  }
//--------------------------------------------------
// . = GOTO TRACK ZERO                             1
//--------------------------------------------------
  else if (ADOS_CMD == "."){
   if (DISK_READY==true) {
    if (currentTrack!=0){  
     if (ADOS_ARGS!="LOCAL"){sendString1("DRIVE_BUSY");smalldelay(20);} //to prevent the timeout on ESP32
     if (goToTrack0()==true){
      msgString=String("TRACK_"+String((currentTrack<10) ? "0" : "")+String(currentTrack)+"\n");
     }
     else{ msgString=String("ECHO ERROR_SETTING_TRACK_TO:#"+ADOS_ARGS+"\n");
     } 
    }else{
      msgString=String("TRACK_"+String((currentTrack<10) ? "0" : "")+String(currentTrack));
     } 
   }else{msgString="NO_DISK";}
   if (ADOS_ARGS="LOCAL"){
    sendString(msgString.c_str());
   }
   else{
    sendString1(msgString.c_str());
   }
  }
//--------------------------------------------------
// ECHO = ECHO THE ARGS BACK TO ESP32 OR LOCAL     4
//--------------------------------------------------
  else if (ADOS_CMD == "ECHO")  {
   // echoes the command received from ESP32 back to the ESP32 for testing purposes
   sendString1(ADOS_ARGS.c_str());
  }
//--------------------------------------------------
// UNKNOWN COMMAND HANDLER                         ?
//--------------------------------------------------
  else if (ADOS_CMD!=""){
   if (ADOS_ARGS="LOCAL"){
    sendString(String("ADOS_UNKNOWN_CMD: ["+ADOS_CMD+"] ARGS: ["+ADOS_ARGS+"]\n").c_str());
   }
   else{
    sendString1(String("ECHO ADOS_UNKNOWN_CMD: ["+ADOS_CMD+"] ARGS: ["+ADOS_ARGS+"]\n").c_str());
   }
  }
  // clean up the strings and action modes
  ADOS_CMD=""; ADOS_ARGS=""; inputString=""; actionESP32=wait;
 }else if (actionESP32==do_debug){
  //DEBUG SECTION
  actionESP32=wait;
 }else if (actionESP32==draw_bridge){
  //full implementation of DRAW_BRIDGE for MEGA2560 to keep as compatible as possible
  actionESP32=wait;
 }; //CONSOLE_ACTION_END
//------------------------------------------------------------------------------------------------------------------- 
// HARDWARE SECTION
//------------------------------------------------------------------------------------------------------------------- 
 unsigned long currentMillis = millisecs();
 if (currentMillis - previousMillis >= interval) {
  previousMillis = currentMillis;
  POLLING_COUNT++;
  if ( (POLLING_COUNT>=POLLING_INTERVAL) && (driveEnabled==false)) {
  //sendString(String("DEBUG_MEGA2560_POLLING\n").c_str());
   Polling();
   POLLING_COUNT=0;
  }
  // put ECHO in front if sending the msg to the ESP32
  if (currentTrack!=-1){
  statusMessage = String("TRACK_"+String((currentTrack<10) ? "0" : "")+String(currentTrack)+"/");
  //"TRACK"+String(currentTrack)+"/";
  }
//  index_pulse_count=0;
//  if (data_pulse_count >> 0) {HAS_DATA_PULSE=true;}else {HAS_DATA_PULSE=false;}
//  data_pulse_count=0;
  switch (digitalRead(FD_DISK_READY)){
   case 1:DISK_NOT_REMOVED=digitalRead(FD_DISK_REMOVED);
    if (DISK_NOT_REMOVED){
     statusMessage = "DISK_INSERTED\n";
     //sendString(String("DEBUG_MEGA2560_DISK_INSERTED\n").c_str());
     if (!driveEnabled){
      statusMessage = statusMessage +"DISK_MOTOR_ON#\n";
      ADOS_CMD="+"; ADOS_ARGS="DUMMY";
      actionESP32=do_cmd;
      break;
     }else {
      statusMessage = statusMessage +"DISK_EJECTED#\n";
      driveEnabled=false; //enable polling again
      //currentTrack=-1;
      break;
     }
    }else{
      //statusMessage = statusMessage + "DISK_REMOVED\n";
     //sendString(String("DEBUG_MEGA2560_DISK_REMOVED\n").c_str());
     if (currentTrack==-1) {
      //statusMessage = statusMessage +"DRIVE_INIT#\n";
     }else{
      statusMessage = "DISK_EJECTED#\n";
      driveEnabled=false; //enable polling again
      currentTrack=-1; //reset currentTrack
      statusMessage = "NO_DISK#\n";
     }
    }
    break;
   case 0:
    if (driveEnabled){
     if (currentTrack == -1) goToTrack0();
     DISK_READY=true;
     digitalWrite(FDD_LED,LOW); // the disk is detected so we signal this phase by turning the LED off
     statusMessage = statusMessage + "DISK_READY#\n";
    }else{statusMessage = statusMessage + "DRIVE_INIT#\n";} //sometimes it misdetects too early 
    break;
  }
//------------------------------------------------------------------------------------------------------------------- 
//SHOW STATUS IN MEGA2560 CONSOLE
//------------------------------------------------------------------------------------------------------------------- 
  if (statusMessage!="")
   if (statusMessage!=prevstatusMessage) {
    prevstatusMessage=statusMessage;
    // sendString1(statusMessage.c_str());
    sendString(statusMessage.c_str());
   }
   statusMessage = ""; 
 }
}
//-----------------------------------------------------------------------------------------------------
// MAIN LOOP (REMEMBER TO RE-POWER THE DRIVE IF ANY PROBLEM SHOULD OCCUR)
//-----------------------------------------------------------------------------------------------------
// void HWserialEvent() - UART0 read routine is called from the main loop
// this nonblocking routine only detects 3 characters in a row terminated with "\n"
// switching to default serial routines might help but I tried to maintain an uniform approach so
// for now we use "shortcuts" as command and reserve numeric pairs 00-79 for tracknumber..
// some commands need just 1 character just like with drawbridge but since our system interfaces with
// an Amiga floppy drive (real or converted pc-drive) all the extra stuff became superfluous for now
// even though with hoodloader the usb hw can reach 2MBAUD the MFM streams go to the ESP32 for handling
// which is by design.. maybe we could also utilise a FTDI-adapter on serialport 0..
//-----------------------------------------------------------------------------------------------------
void HWserialEvent0() {
 char inChar = readByteFromUART0_NonBlocking();
 if (inChar!=char(0)){
  if (inChar!='\r'){
   if (inChar!='\n'){
    inputString += inChar;
   }
  }
  if (inChar == '\n') {
    if ( isDigit(inputString[0]) && isDigit(inputString[1]) ){//MOVE HEAD TO TRACK 00-79
     if ( String("89").indexOf(inputString[0])==-1){
     ADOS_CMD="#"+inputString;}else{ADOS_CMD="ERROR_BAD_TRACK_NUM:"+inputString;}  
    }
    else if (inputString=="?"){ //FIRMWARE VERSION
     ADOS_CMD="?"; 
    }
    else if (inputString=="."){ //MOVE HEAD TO TRACK 0
     ADOS_CMD="."; 
    }
    else if (inputString=="LO"){ //DISK_LOWER_SIDE_0
     ADOS_CMD="["; 
    }
    else if (inputString=="UP"){ //DISK_UPPER_SIDE_1
     ADOS_CMD="]"; 
    }
    else if (inputString=="TA"){ //TRACK2ADF
     ADOS_CMD="="; 
    }
    else if (inputString=="DA"){ //DISK2ADF
     ADOS_CMD="DISK2ADF"; 
    }
    else {
      ADOS_CMD="CMD_UNDEFINED";
    }
    ADOS_ARGS="LOCAL";
    inputString="";
    actionESP32 = do_cmd; // let the mainloop handle the ADOS command
  }
 }
}
//-----------------------------------------------------------------------------------------------------
// void HWserialEvent1() - UART1 read routine is called from the main loop
//-----------------------------------------------------------------------------------------------------
void HWserialEvent1() {
// Configure timer 2 just as a counter in NORMAL mode
 TCCR2A = 0 ;              // No physical output port pins and normal operation
 TCCR2B = bit(CS20);       // Precale = 1  
 OCR2A = 0x00;
 OCR2B = 0x00;
//-----------------------------------------------------------------------------------------------------
// checking if receiver is equipped with a character
// since there were problems with rogue chars being fired by faulty connections
// some kind of protection is needed for 1 ghost-byte can screw it up...
//-----------------------------------------------------------------------------------------------------
 if(bitRead(UCSR1A, 7) == HIGH){
  bitClear(UCSR1A, 7);     //clear the reception flag
  // using eval to prevent rogue byte effect as described above
  byte eval = readByteFromUART1();
  if (eval=='|'){
   // Find out how many bytes the ESP32 wants to send
   unsigned char highByte = readByteFromUART1();
   unsigned char lowByte = readByteFromUART1();
   int numBytes = (((unsigned short)highByte)<<8) | lowByte; //unsigned short range 0..65535
   for (int a=0; a<numBytes; a++) {
   // Wait for it
    while (!( UCSR1A & ( 1 << RXC1 ))){}; 
    // Save the byte
    SERIAL1_BUFFER[a] = UDR1;
   }
   String msg = String((const char*)&SERIAL1_BUFFER[0]).substring(0,numBytes);
   int _SPACE_POS = msg.indexOf(' ');
   if ((_SPACE_POS)!=-1){
    ADOS_CMD=msg.substring(0, _SPACE_POS);
    ADOS_CMD.toUpperCase();
    ADOS_ARGS= msg.substring(_SPACE_POS+1);//, msg.length()-_SPACE_POS);
    //accounting for ESP32 prefix command "ados "
   }
   else
   {
    ADOS_CMD=msg; ADOS_CMD.toUpperCase();
    ADOS_ARGS="EMPTY_ARGS";
   }
   actionESP32 = do_cmd; // let the mainloop handle the ADOS command
   TCCR2B = 0;   // No Clock (turn off)    
   //if (actionESP32 != do_cmd){actionESP32 = wait;}
  }else{
   command = eval;
   actionESP32 = draw_bridge;
  }
 }
}
/*
   // clear the string:
   inputString = "";
   stringComplete = false;
   // Read the command from the PC
// }
  DATA_PULSE=digitalRead(FD_READ_DATA);
    switch (DATA_PULSE){
      case 0:
       if (PREV_DATA_PULSE!=DATA_PULSE){
       data_pulse_count++; //LOW
       }
       PREV_DATA_PULSE=DATA_PULSE;
       break;
      case 1: PREV_DATA_PULSE=DATA_PULSE;
      break;
    }
  INDEX_PULSE=digitalRead(FD_INDEX_PULSE);
    switch (INDEX_PULSE){
      case 0:
       if (PREV_INDEX_PULSE!=INDEX_PULSE){
       index_pulse_count++; //LOW
       }
       PREV_INDEX_PULSE=INDEX_PULSE;
       break;
      case 1: PREV_INDEX_PULSE=INDEX_PULSE;
      break;
    }
 //let's speed it up if a disk is inserted (implicates higher speed for the FTP server since the loop becomes "lighter")
    if (!DISK_READY){
      // read the value from the sensor: (only active if NO_DISK in drive)
      potmeter1Value = (analogRead(PIN_POTMETER_1) / 4);
      if (prev_potmeter1Value!=potmeter1Value){
        prev_potmeter1Value = potmeter1Value;
        //Serial.println("POTMETER_1: "+String(potmeter1Value));
        sendString(("POTMETER_1: "+String(potmeter1Value)+"\n").c_str());
      }
    }
//------------------------------------------------------------------------------------------------------------------- 
    
      statusMessage = "ECHO TRACK"+String(currentTrack)+"/";
      index_pulse_count=0;
      if (data_pulse_count >> 0) {HAS_DATA_PULSE=true;}else {HAS_DATA_PULSE=false;}
      data_pulse_count=0;
}*/
