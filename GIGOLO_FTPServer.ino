/*
   Amiga ESP32 getLastErrorStr() const {re here at last sector number %d\n", lastSectorNumber);rgetOS: for Amiga's ONLY!
   note: various code sources assimilated as part of the AmiGoDOS project
   release via: amiga.amigoxpe.net
   
   DopusFTP Server ESP32 features:
   Custom FTP Server which handles 1 exclusive (returning) client (Amiga with Dopus 5.91 tested)
   DUAL Serial Port Command/Communication Interface:
   L - to display the contents of the SD_MMC FileSystem
   R - Reboot the ESP32 FTP Server

   Thanks to various coders for sharing functional versions of a FTP Server library
   which was used as base for a slightly customized Amiga friendly library
   as a token of 0nsch appreciation we will share the code with you
   to return the favor to keep our momentum going IYKWIM!
   
   With kind regards, PTAH

   Used ESP32 Hardware: 
   - ESP32 DEVKITC v1.0 (using Capacitor 2,2 uF between EN & GND) to upload sketch automatically
   - a DIY MicroSD card reader/writer using a MicroSD2SD adapter card (easy to make, minimal soldering)
 * HARDWARE INTERFACE - DIY MicroSD  to SD-Adapter (max 32GB)
 * 1-bit SD_MMC bus mode ONLY! (minimal wiring/good performance) from Amiga perspective (400KB/s UP and 500-800KB/s DOWN)
 * feels like WARP although throughput (if using the plipbox equipped Amiga) is limited to approx. 250KB/s...
 * Connection scheme:
 * ESP32 |SD Card/Shield
 *        _______________________________
 * NC    | D1                            |
 * 2     | D0   D6                       |
 * GND   | VSS  GND                      |_
 * 14    | CLK  D5                       | | MicroSD
 * 3.3V  | VDD  3.3V  SD-CARD-ADAPTER    | | SDSC
 * GND   | VSS                           | | SDHC
 * 15    | CMD  D7                       |_|
 * NC    | D3                            |
 * NC     \_D2___________________________|
 * 
 * ESP32 UART2 HARDWARE INTERFACE test
 * Try to initialize UART2 via custom initialization routines to get CTS functionality
 * which is being rerouted from pin GPIO-8 (reserved for internal flash) to a free pin
 * VERY EXPERIMENTAL ! first test to init IDF UART2 method and skip the default Serial2 initialization
 * 
 * CTS draad aangebracht tussen MEGA2560-GPIO-A12 via de levelconverter naar ESP32-GPIO-27
 * hoop dat de 2mbaud niet gecompromitteerd is hierdoor
 * initialize correctly is mandatory for the ESP32
 * 
 * Observations (as Amiga DOPUSFTP user): 
 * if wiggle the usb cable the ESP32 can become unresponsive if your USB port is totally worn off like with my m4400 dev/testPC so make good connection..
 * the ESP32 FTPServer can't handle DOPUSFTP std. settings .. resulting in lost SD context and vague errors which disappeared after cold reboot
 * warm reboot with ESP.Restart() not always gave back a context (maybe internal pullups are to weak)
 * so now trying DOPUSFTP Custom Options NO RECONNECT/AUTOCONNECT,
 * it seems ESP32 is sometimes unresponsive
 * , NO INDEX & AUTOLOAD,
 * otherwise the FileSystem crashes eventually not able to find INDEX
 * for now it seems stable enough to move megabytes and folders from/to ESP32/AMIGA
 * now working on some logic flaws like IF NO WIFI OR SD CARD THEN
 * because there were some nasty errors, maybe awake from sleep functions loses the SD context
 * but that is speculation...
 * it also looks like the ESP32 is unresponsive after a timeout/client kick (not confirmed)
 * 2MEGABAUD serial2 connection to MEGA2560 seems to be fine so next phase is to use the ESP32 inside CAT
 * to get the Mega2560 connected Floppy drive working , the ESP32 acts like a PC handling MFM streams via ESP32 serial2
 * convert MFM and write as ADF files available on the FTP Server for an Amiga Client...
 * some background info/wishes/targets:
 * - Mega2560, The (pc converted to amiga)floppy drive should be able to read/write, and visible for UAE adapted emulators as REALTIME FLOPPYDRIVE
 * - ESP32 converts stream to/from ADF and writes from/to SD_MMC avoiding the internal flash as wear-out protection
 * - maybe the internal flash can become usable to act as storage for WB-install disks or even
 * 
 * remark: if the DOPUSFTP module would be recompiled without the first MLSD entry while connecting
 * the workaround presented here would be not needed (old style 5.11)
 * yet it is still needed to use Custom Options until this FTPServer is somewhat more advanced for OOTB usage..
*/
#include <Arduino.h>
#include <stdlib.h>
#include <stdint.h>
#include <stddef.h>
#include <driver/uart.h>
#include <WiFi.h>
#include <time.h> 
#include <SD_MMC.h>
#include <ESP32Time.h>
#include "FTPServer.h"
#include "ESP.h"
#include <fstream>
#include <sstream>
#include <vector>
//#include "adflib.h"
//#include "FloppyDrive.h"
//#include <Streaming.h>
//------------------------------------------------------------------------------
// MAKE REVISION ON PAR WITH MEGA2560
const String REVISION = "v1.0b rev.20220804-esp32-001";
const String PROG_ID = "GIGOLO FTPServer ("+REVISION+")";
//------------------------------------------------------------------------------
// SERIAL PORT 0 COMMUNICATION
//------------------------------------------------------------------------------
#define BAUDRATE 115200 //38400//31250 for MIDI
// note Amiga 1200 takes max. 38400 when using the default amigados serial device
// to be safe use a 232 serial chip to convert from 5v to 3.3v and vice versa
//#define RXD0 and TXD0 in case of NO USB SERIAL CONNECTION used
//though POWER comes from USB one should use other solutions via VIN or a modified USB cable aka charger cable
//------------------------------------------------------------------------------
// SERIAL PORT 2 COMMUNICATION
//------------------------------------------------------------------------------
#define BAUDRATE_2MBAUD 2000000 //yes this seems to work but needs to be tested more, it must function flawless
#define RXD2 16 //the HW constant crashed when uses moved the ESP.h up, might work now
#define TXD2 17
#define ESP32_UART_PORT UART_NUM_2
#define ESP32_UART2_TXD   TXD2 //(UART_PIN_NO_CHANGE)
#define ESP32_UART2_RXD   RXD2 //(UART_PIN_NO_CHANGE)
#define ESP32_UART2_RTS   (26)
#define ESP32_UART2_CTS   (27)
#define BUF_SIZE         (127)
#define PACKET_READ_TICS (10 / portTICK_RATE_MS)
static const char* TAG = "GIGOLO FTPServer";
//------------------------------------------------------------------------------
// WIFI CREDENTIALS
//------------------------------------------------------------------------------
const char* ssid = "YOURSSID";
const char* password = "YOURPASSWORD";
//------------------------------------------------------------------------------
// EXTERNAL TIME SERVER (better use a router with NTP service build in)
// alternatively add a battery backed RTC so the time gets preserved after power faillure
//------------------------------------------------------------------------------
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 3600;
const int   daylightOffset_sec = 3600;
ESP32Time rtc;
//------------------------------------------------------------------------------
// PREP FTPSERVER TO USE SD_MMC FILESYSTEM
//------------------------------------------------------------------------------
FTPServer ftpSrv(SD_MMC);
String ADOS_CMD = "";
String ADOS_ARGS = "";
String FTP_ADOS_CMD = "";
String FTP_ADOS_ARGS = "";
String inString = "";      // a String to hold incoming data from HWSerial1
bool DEBUG = false;  // whether the string is complete
//------------------------------------------------------------------------------
// LOCAL FUNCTIONS
//------------------------------------------------------------------------------
void serialEvent();
void serial2Event();
const uart_port_t uart_num = ESP32_UART_PORT;
//------------------------------------------------------------------------------
/* * AMIGA ADF Floppy Disk Library
 * SIZE 901120 BYTES
 */
//------------------------------------------------------------------------------
#define MFM_MASK    0x55555555L    
#define AMIGA_WORD_SYNC  0x4489              // Disk SYNC code for the Amiga start of sector
#define SECTOR_BYTES  512                // Number of bytes in a decoded sector
#define NUM_SECTORS_PER_TRACK_DD 11            // Number of sectors per track
#define NUM_SECTORS_PER_TRACK_HD 22             // Same but for HD disks
#define RAW_SECTOR_SIZE (8+56+SECTOR_BYTES+SECTOR_BYTES)      // Size of a sector, *Including* the sector sync word longs
#define ADF_TRACK_SIZE_DD (SECTOR_BYTES*NUM_SECTORS_PER_TRACK_DD)   // Bytes required for a single track dd
#define ADF_TRACK_SIZE_HD (SECTOR_BYTES*NUM_SECTORS_PER_TRACK_HD)   // Bytes required for a single track hd
//------------------------------------------------------------------------------
#define FLOPPY_SIDES  2
#define FLOPPY_TRACKS_PER_SIDE 80
#define FLOPPY_SECTORS_PER_TRACK 11
#define FLOPPY_BYTES_PER_SECTOR 512
const long FLOPPY_SIZE = FLOPPY_SIDES * FLOPPY_TRACKS_PER_SIDE * FLOPPY_SECTORS_PER_TRACK * FLOPPY_BYTES_PER_SECTOR;
const long FLOPPY_SECTORS = FLOPPY_SIDES * FLOPPY_TRACKS_PER_SIDE * FLOPPY_SECTORS_PER_TRACK;
//#define TRACK_BUFFER_SIZE 13888
//------------------------------------------------------------------------------
#define RAW_TRACKDATA_LENGTH    (0x1900*2+0x440)
//------------------------------------------------------------------------------
// Optional how to respond to the callback from the writeADF command.
//------------------------------------------------------------------------------
  enum class WriteResponse {
                wrContinue,       // Continue working as normal
                wrAbort,        // Abort thr process and stop
                wrSkipBadChecksums,   // Request that the process ignores bad checksums (not recommended unless the retryCounter gets beyond RETRYS_PER_PHASE*16)
                wrRetry                 // Retry!
              };
//------------------------------------------------------------------------------
  enum class ADFResult {
            adfrComplete,         // Process completed successfully
            adfrAborted,          // Process was aborted
            adfrFileError,          // Error opening the file to write to
            adfrFileIOError,        // Error writing to the ADF file
            adfrCompletedWithErrors,    // Completed but with sector errors
            adfrDiskWriteProtected,         // Disk is write protected!
            adfrDriveError,         // Something wrong with reading the disk
            adfrFirmwareTooOld        // Firmware is too old
          };

//------------------------------------------------------------------------------
// Represent which side of the disk we're looking at
//------------------------------------------------------------------------------
  enum class DiskSurface {
    dsUpper,            // The upper side of the disk
    dsLower             // The lower side of the disk
  };
  

  // Speed at which the head will seek to a track
  enum class TrackSearchSpeed {
    tssSlow,
    tssNormal,
    tssFast,
    tssVeryFast
  };
//------------------------------------------------------------------------------
// Diagnostic responses from the interface
//------------------------------------------------------------------------------
  enum class DiagnosticResponse {
    drOK,

    // Responses from openPort
    drPortInUse,
    drPortNotFound,
    drPortError,
    drAccessDenied,
    drComportConfigError,
    drBaudRateNotSupported,
    drErrorReadingVersion,
    drErrorMalformedVersion,
    drOldFirmware,

    // Responses from commands
    drSendFailed,
    drSendParameterFailed,
    drReadResponseFailed,
    drWriteTimeout,
    drSerialOverrun,
    drFramingError,
    drError,

    // Response from selectTrack
    drTrackRangeError,
    drSelectTrackError,
    drWriteProtected,
    drStatusError,
    drSendDataFailed,
    drTrackWriteResponseError,

    // Returned if there is no disk in the drive
    drNoDiskInDrive,

    drDiagnosticNotAvailable,
    drUSBSerialBad,
    drCTSFailure,
    drRewindFailure
  };
//    DiagnosticResponse m_lastError;
  enum class LastCommand {
    lcOpenPort,
    lcGetVersion,
    lcEnableWrite,
    lcRewind,
    lcDisableMotor,
    lcEnableMotor,
    lcGotoTrack,
    lcSelectSurface,
    lcReadTrack,
    lcWriteTrack,
    lcRunDiagnostics,
    lcSwitchDiskMode,
    lcReadTrackStream,
    lcCheckDiskInDrive,
    lcCheckDiskWriteProtected,
    lcEraseTrack
  };

    // Uses the above fields to constructr a suitable error message, hopefully useful in diagnosing the issue
//    const std::string: getLastErrorStr() const;

// Convert the last executed command that had an error to a string
std::string lastCommandToName(LastCommand cmd) {
  switch (cmd) {
  case LastCommand::lcOpenPort:   return "OpenPort";
  case LastCommand::lcGetVersion:   return "GetVersion";
  case LastCommand::lcEnableWrite:  return "EnableWrite";
  case LastCommand::lcRewind:     return "Rewind";
  case LastCommand::lcDisableMotor: return "DisableMotor";
  case LastCommand::lcEnableMotor:  return "EnableMotor";
  case LastCommand::lcGotoTrack:    return "GotoTrack";
  case LastCommand::lcSelectSurface:  return "SelectSurface";
  case LastCommand::lcReadTrack:    return "ReadTrack";
  case LastCommand::lcWriteTrack:   return "WriteTrack";
  case LastCommand::lcRunDiagnostics: return "RunDiagnostics";
  case LastCommand::lcSwitchDiskMode: return "SetCapacity";
  case LastCommand::lcReadTrackStream: return "ReadTrackStream";
  case LastCommand::lcCheckDiskInDrive: return "CheckDiskInDrive";
  case LastCommand::lcCheckDiskWriteProtected: return "CheckDiskWriteProtected";
  case LastCommand::lcEraseTrack: return "EraseTrack";

  default:              return "Unknown";
  }
}
// Sketch firmware version
  struct FirmwareVersion {
    unsigned char major, minor;
    bool fullControlMod;
  };
    FirmwareVersion m_version;
    bool      m_inWriteMode;
    LastCommand   m_lastCommand;
    DiagnosticResponse m_lastError;
    bool      m_abortStreaming;
    bool      m_isWriteProtected;
    bool      m_diskInDrive;
    bool      m_abortSignalled;
    bool      m_isStreaming;

// Uses the above fields to constructr a suitable error message, hopefully useful in diagnosing the issue
const std::string getLastErrorStr() {//const {
  std::stringstream tmp;
  switch (m_lastError) {
  case DiagnosticResponse::drOldFirmware: return "The Arduino is running an older version of the firmware/sketch.  Please re-upload.";
  case DiagnosticResponse::drOK: return "Last command completed successfully.";
  case DiagnosticResponse::drPortInUse: return "The specified COM port is currently in use by another application.";
  case DiagnosticResponse::drPortNotFound: return "The specified COM port was not found.";
  case DiagnosticResponse::drAccessDenied: return "The operating system denied access to the specified COM port.";
  case DiagnosticResponse::drComportConfigError: return "We were unable to configure the COM port using the SetCommConfig() command.";
  case DiagnosticResponse::drBaudRateNotSupported: return "The COM port does not support the 2M baud rate required by this application.";
  case DiagnosticResponse::drErrorReadingVersion: return "An error occured attempting to read the version of the sketch running on the Arduino.";
  case DiagnosticResponse::drErrorMalformedVersion: return "The Arduino returned an unexpected string when version was requested.  This could be a baud rate mismatch or incorrect loaded sketch.";
  case DiagnosticResponse::drCTSFailure: return "Diagnostics reports the CTS line is not connected correctly or is not behaving correctly.";
  case DiagnosticResponse::drTrackRangeError: return "An error occured attempting to go to a track number that was out of allowed range.";
  case DiagnosticResponse::drWriteProtected: return "Unable to write to the disk.  The disk is write protected.";
  case DiagnosticResponse::drPortError: return "An unknown error occured attempting to open access to the specified COM port.";
  case DiagnosticResponse::drDiagnosticNotAvailable: return "CTS diagnostic not available, command GetCommModemStatus failed to execute.";
  case DiagnosticResponse::drSelectTrackError: return "Arduino reported an error seeking to a specific track.";
  case DiagnosticResponse::drTrackWriteResponseError: return "Error receiving status from Arduino after a track write operation.";
  case DiagnosticResponse::drSendDataFailed: return "Error sending track data to be written to disk.  This could be a COM timeout.";
  case DiagnosticResponse::drRewindFailure: return "Arduino was unable to find track 0.  This could be a wiring fault or power supply failure.";
  case DiagnosticResponse::drNoDiskInDrive: return "No disk in drive";
  case DiagnosticResponse::drWriteTimeout: return "The Arduino could not receive the data quick enough to write to disk. Try connecting via USB2 and not using a USB hub.\n\nIf this still does not work, turn off precomp if you are using it.";
  case DiagnosticResponse::drFramingError: return "The Arduino received bad data from the PC. This could indicate poor connectivity, bad baud rate matching or damaged cables.";
  case DiagnosticResponse::drSerialOverrun: return "The Arduino received data faster than it could handle. This could either be a fault with the CTS connection or the USB/serial interface is faulty";
  case DiagnosticResponse::drUSBSerialBad: return "The USB->Serial converter being used isn't suitable and doesnt run consistantly fast enough.  Please ensure you use a genuine FTDI adapter.";
  case DiagnosticResponse::drError: tmp << "Arduino responded with an error running the " << lastCommandToName(m_lastCommand) << " command.";
    return tmp.str();
  case DiagnosticResponse::drReadResponseFailed:
    switch (m_lastCommand) {
    case LastCommand::lcGotoTrack: return "Unable to read response from Arduino after requesting to go to a specific track";
    case LastCommand::lcReadTrack: return "Gave up trying to read a full track from the disk.";
    case LastCommand::lcWriteTrack: return "Unable to read response to requesting to write a track.";
    default: tmp << "Error reading response from the Arduino while running command " << lastCommandToName(m_lastCommand) << ".";
      return tmp.str();
    }
  case DiagnosticResponse::drSendFailed:
    if (m_lastCommand == LastCommand::lcGotoTrack)
      return "Unable to send the complete select track command to the Arduino.";
    else {
      tmp << "Error sending the command " << lastCommandToName(m_lastCommand) << " to the Arduino.";
      return tmp.str();
    }
  case DiagnosticResponse::drSendParameterFailed: tmp << "Unable to send a parameter while executing the " << lastCommandToName(m_lastCommand) << " command.";
    return tmp.str();
  case DiagnosticResponse::drStatusError: tmp << "An unknown response was was received from the Arduino while executing the " << lastCommandToName(m_lastCommand) << " command.";
    return tmp.str();
  default: return "Unknown error.";
  }
}


    
//------------------------------------------------------------------------------
      
//------------------------------------------------------------------------------
// Command that the ARDUINO Sketch understands
//------------------------------------------------------------------------------
#define COMMAND_VERSION            '?'
#define COMMAND_REWIND             '.'
#define COMMAND_GOTOTRACK          '#'
#define COMMAND_HEAD0              '['
#define COMMAND_HEAD1              ']'
#define COMMAND_READTRACK          '<'
#define COMMAND_ENABLE             '+'
#define COMMAND_DISABLE            '-'
#define COMMAND_WRITETRACK         '>'
#define COMMAND_ENABLEWRITE        '~'
#define COMMAND_DIAGNOSTICS        '&'
#define COMMAND_ERASETRACK       'X'
#define COMMAND_SWITCHTO_DD      'D'   // Requires Firmware V1.6
#define COMMAND_SWITCHTO_HD      'H'   // Requires Firmware V1.6
#define COMMAND_DETECT_DISK_TYPE   'M'   // currently not implemented here

// New commands for more direct control of the drive.  Some of these are more efficient or dont turn the disk motor on for modded hardware
#define COMMAND_READTRACKSTREAM    '{'    // Requires Firmware V1.8
#define COMMAND_WRITETRACKPRECOMP  '}'    // Requires Firmware V1.8
#define COMMAND_CHECKDISKEXISTS    '^'    // Requires Firmware V1.8 (and modded hardware for fast version)
#define COMMAND_ISWRITEPROTECTED   '$'    // Requires Firmware V1.8
#define COMMAND_ENABLE_NOWAIT      '*'    // Requires Firmware V1.8
#define COMMAND_GOTOTRACK_REPORT   '='    // Requires Firmware V1.8
#define COMMAND_DO_NOCLICK_SEEK    'O'    // Requires Firmware V1.8a

#define SPECIAL_ABORT_CHAR       'x'
//------------------------------------------------------------------------------
//    RawTrackData *TMP_RawTrackData = (RawTrackData *) malloc(RAW_TRACKDATA_LENGTH);//13888;
//    RawEncodedSector *
//------------------------------------------------------------------------------
// Buffer to hold raw data for just a single sector
//------------------------------------------------------------------------------
typedef unsigned char RawEncodedSector[RAW_SECTOR_SIZE];
typedef unsigned char RawDecodedSector[SECTOR_BYTES];
typedef RawDecodedSector RawDecodedTrack[NUM_SECTORS_PER_TRACK_DD];
typedef unsigned char RawMFMData[SECTOR_BYTES + SECTOR_BYTES];
//------------------------------------------------------------------------------
// When workbench formats a disk, it write 13630 bytes of mfm data to the disk.  So we're going to write this amount, and then we dont need an erase first
//------------------------------------------------------------------------------
typedef struct alignas(1) {
  unsigned char filler1[1654];  // Padding at the start of the track.  This will be set to 0xaa
  // Raw sector data
  RawEncodedSector sectors[NUM_SECTORS_PER_TRACK_DD];   // 11968 bytes
  // Blank "Filler" gap content. (this may get overwritten by the sectors a little)
  unsigned char filler2[8];
} FullDiskTrack;
//------------------------------------------------------------------------------
// Structure to hold data while we decode it
//------------------------------------------------------------------------------
typedef struct alignas(8)  {
  unsigned char trackFormat;        // This will be 0xFF for Amiga
  unsigned char trackNumber;        // Current track number (this is actually (tracknumber*2) + side
  unsigned char sectorNumber;       // The sector we just read (0 to 11)
  unsigned char sectorsRemaining;   // How many more sectors remain until the gap (0 to 10)
  uint32_t sectorLabel[4];     // OS Recovery Data, we ignore this
  uint32_t headerChecksum;    // Read from the header, header checksum
  uint32_t dataChecksum;      // Read from the header, data checksum
  uint32_t headerChecksumCalculated;   // The header checksum we calculate
  uint32_t dataChecksumCalculated;     // The data checksum we calculate
  RawDecodedSector data;          // decoded sector data
  RawMFMData rawSector;   // raw track data, for analysis of invalid sectors
} DecodedSector;
//------------------------------------------------------------------------------
// To hold a list of valid and checksum failed sectors
//------------------------------------------------------------------------------
struct DecodedTrack {
  // A list of valid sectors where the checksums are OK
  std::vector<DecodedSector> validSectors;
  // A list of sectors found with invalid checksums.  These are used if ignore errors is triggered
  // We keep copies of each one so we can perform a statistical analysis to see if we can get a working one based on which bits are mostly set the same
  std::vector<DecodedSector> invalidSectors[NUM_SECTORS_PER_TRACK_DD];
};
//------------------------------------------------------------------------------
  // Array to hold data from a floppy disk read
  typedef unsigned char RawTrackData[RAW_TRACKDATA_LENGTH];
//------------------------------------------------------------------------------
uint16_t allocateTMP_RawTrackData(uint16_t desiredBytes = RAW_TRACKDATA_LENGTH); // allocate buffer for transfer
void freeTMP_RawTrackData();
RawTrackData *TMP_RawTrackData = NULL; // pointer to buffer for file transfer (by allocateBuffer)
uint16_t TMP_RawTrackDataSize;    // size of buffer
//------------------------------------------------------------------------------
uint16_t allocateTMP_RawTrackData(uint16_t desiredBytes)
{
    while (TMP_RawTrackData == NULL && desiredBytes > 0)
    {
        TMP_RawTrackData = (RawTrackData *)malloc(desiredBytes);
        if (NULL == TMP_RawTrackData)
        {
            Serial.print("Cannot allocate TMP_RawTrackData , re-trying");
            // try with less bytes
            desiredBytes--;
        }
        else
        {
            TMP_RawTrackDataSize = desiredBytes;
        }
    }
    return TMP_RawTrackDataSize;
}
//------------------------------------------------------------------------------
void freeTMP_RawTrackData()
{
    free(TMP_RawTrackData);
    TMP_RawTrackData = NULL;
}
//------------------------------------------------------------------------------
uint16_t allocateMFM_RawTrackData(uint16_t desiredBytes = RAW_TRACKDATA_LENGTH); // allocate buffer for transfer
void freeMFM_RawTrackData();
RawTrackData *MFM_RawTrackData = NULL; // pointer to buffer for file transfer (by allocateBuffer)
uint16_t MFM_RawTrackDataSize;    // size of buffer
//------------------------------------------------------------------------------
uint16_t allocateMFM_RawTrackData(uint16_t desiredBytes)
{
    while (MFM_RawTrackData == NULL && desiredBytes > 0)
    {
        MFM_RawTrackData = (RawTrackData *)malloc(desiredBytes);
        if (NULL == TMP_RawTrackData)
        {
            Serial.print("Cannot allocate MFM_RawTrackData , re-trying");
            // try with less bytes
            desiredBytes--;
        }
        else
        {
            MFM_RawTrackDataSize = desiredBytes;
        }
    }
    return MFM_RawTrackDataSize;
}
//------------------------------------------------------------------------------
void freeMFM_RawTrackData()
{
    free(MFM_RawTrackData);
    MFM_RawTrackData = NULL;
}
//------------------------------------------------------------------------------
    // Writes an ADF file back to a floppy disk.  Return FALSE in the callback to abort this operation.  If verify is set then the track isread back and and sector checksums are checked for 11 valid sectors
    // IF using precomp mode then DO NOT connect the Arduino via a USB hub, and try to plug it into a USB2 port
    ADFResult ADFToDisk(const std::string& inputFile, bool verify, bool usePrecompMode, std::function < WriteResponse(const int currentTrack, const DiskSurface currentSide, const bool isVerifyError) > callback);
    // Read a desired number of bytes into the target pointer
    bool deviceRead(void* target, const unsigned int numBytes, const bool failIfNotAllRead = false);
    // Writes a desired number of bytes from the the pointer
    bool deviceWrite(const void* source, const unsigned int numBytes);
    bool decodeSector(const RawEncodedSector& rawSector, const unsigned int trackNumber, const DiskSurface surface, DecodedTrack& decodedTrack, bool ignoreHeaderChecksum, int& lastSectorNumber);
    void encodeSector(const unsigned int trackNumber, const DiskSurface surface, const unsigned int sectorNumber, const RawDecodedSector& input, RawEncodedSector& encodedSector, unsigned char& lastByte);  
    void alignSectorToByte(const unsigned char* inTrack, const int dataLength, int byteStart, int bitStart, RawEncodedSector& outSector);
    bool attemptFixSector(const DecodedTrack& decodedTrack, DecodedSector& outputSector);
    void findSectors(const unsigned char* track, unsigned int trackNumber, DiskSurface side, unsigned short trackSync, DecodedTrack& decodedTrack, bool ignoreHeaderChecksum);
    void writeBit(RawTrackData& output, int& pos, int& bit, int value);
    void unpack(const RawTrackData& data, RawTrackData& output);
    DiagnosticResponse readCurrentTrack(RawTrackData& trackData, const bool readFromIndexPulse);

    std::string getLastError() { return getLastErrorStr(); };
    LastCommand getLastFailedCommand(){ return m_lastCommand; };
    //const DiagnosticResponse getLastError() /*const*/ { return m_lastError; };
    const std::string getLastErrorStr();// const;
// Select the track, this makes the motor seek to this position
    DiagnosticResponse  selectTrack(const unsigned char trackIndex, const TrackSearchSpeed searchSpeed = TrackSearchSpeed::tssNormal, bool ignoreDiskInsertCheck = false);
    // Version of the above where the command has a parameter on the end (as long as its not char 0)
    DiagnosticResponse runCommand(const char command, const char parameter = '\0', char* actualResponse = nullptr);
//-----------------------------------------------------------------------------
// MFM decoding algorithm
// *input;  MFM coded data buffer (size == 2*data_size) 
// *output; decoded data buffer (size == data_size) 
// Returns the checksum calculated over the data
//-----------------------------------------------------------------------------
uint32_t decodeMFMdata(const uint32_t* input, uint32_t* output, const unsigned int data_size) {
  uint32_t odd_bits, even_bits;
  uint32_t chksum = 0L;
  unsigned int count;

  // the decoding is made here long by long : with data_size/4 iterations 
  for (count = 0; count < data_size / 4; count++) {
    odd_bits = *input;          // longs with odd bits 
    even_bits = *(uint32_t*)(((unsigned char*)input) + data_size);   // longs with even bits - located 'data_size' bytes after the odd bits

    chksum ^= odd_bits;              // XOR Checksum
    chksum ^= even_bits;

    *output = ((even_bits & MFM_MASK) | ((odd_bits & MFM_MASK) << 1));
    input++;      /* next 'odd' long and 'even bits' long  */
    output++;     /* next location of the future decoded long */
  }
  return chksum & MFM_MASK;
}

//------------------------------------------------------------------------------
// MFM encoding algorithm part 1 - this just writes the actual data bits in the right places
// *input;  RAW data buffer (size == data_size) 
// *output; MFM encoded buffer (size == data_size*2) 
// Returns the checksum calculated over the data
//------------------------------------------------------------------------------
unsigned long encodeMFMdataPart1(const unsigned long* input, unsigned long* output, const unsigned int data_size) {
  unsigned long chksum = 0L;
  unsigned int count;

  unsigned long* outputOdd = output;
  unsigned long* outputEven = (unsigned long*)(((unsigned char*)output) + data_size);

  // Encode over two passes.  First split out the odd and even data, then encode the MFM values, the /4 is because we're working in longs, not bytes
  for (count = 0; count < data_size / 4; count++) {
    *outputEven = *input & MFM_MASK;
    *outputOdd = ((*input)>>1) & MFM_MASK;
    outputEven++;
    outputOdd++;
    input++;
  }
  
  // Checksum calculator
  // Encode over two passes.  First split out the odd and even data, then encode the MFM values, the /4 is because we're working in longs, not bytes
  for (count = 0; count < (data_size / 4) * 2; count++) {
    chksum ^= *output;
    output++;
  }

  return chksum & MFM_MASK;
}

//------------------------------------------------------------------------------
// Copies the data from inTrack into outTrack but fixes the bit/byte alignment so its aligned on the start of a byte 
//------------------------------------------------------------------------------
// Copys the data from inTrack into outTrack but fixes the bit/byte alignment so its aligned on the start of a byte 
void alignSectorToByte(const unsigned char* inTrack, const int dataLength, int byteStart, int bitStart, RawEncodedSector& outSector) {
  unsigned char byteOut = 0;
  unsigned int byteOutPosition = 0;
  // Bit counter output
  unsigned int counter = 0;
  // The position supplied is the last bit of the track sync.  
  bitStart--;   // goto the next bit
  if (bitStart < 0) {
    // Could do a MEMCPY here, but for now just let the code below run
    bitStart = 7;
    byteStart++;
  }
  byteStart -= 8;   // wind back 8 bytes
  // This is mis-aligned.  So we need to shift the data into byte boundarys
  for (;;) {
    for (int bitCounter = bitStart; bitCounter >= 0; bitCounter--) {
      byteOut <<= 1;
      if (inTrack[byteStart % dataLength] & (1 << bitCounter)) byteOut |= 1;

      if (++counter >= 8) {
        outSector[byteOutPosition] = byteOut;
        byteOutPosition++;
        if (byteOutPosition >= RAW_SECTOR_SIZE) return;
        counter = 0;
      }
    }
    // Move along and reset
    byteStart++;
    bitStart = 7;
  }
}
//------------------------------------------------------------------------------
// Looks at the history for this sector number and creates a new sector where the bits are set to whatever occurs more.
// We then do a checksum and if it succeeds we use it
//------------------------------------------------------------------------------
bool attemptFixSector(const DecodedTrack& decodedTrack, DecodedSector& outputSector) {
  int sectorNumber = outputSector.sectorNumber;
  if (decodedTrack.invalidSectors[sectorNumber].size() < 2) return false;
  typedef struct {
    int zeros = 0;
    int ones = 0;
  } SectorCounter[8];
  SectorCounter sectorSum[SECTOR_BYTES + SECTOR_BYTES];
  memset(sectorSum, 0, sizeof(SectorCounter));
  // Calculate the number of '1's and '0's in each block
  for (const DecodedSector& sec : decodedTrack.invalidSectors[sectorNumber]) 
    for (int byteNumber = 0; byteNumber < SECTOR_BYTES + SECTOR_BYTES; byteNumber++) 
      for (int bit = 0; bit <= 7; bit++) 
        if (sec.rawSector[byteNumber] & (1 << bit))
          sectorSum[byteNumber][bit].ones++; else sectorSum[byteNumber][bit].zeros++;
  // Now create a sector based on this data
  memset(outputSector.rawSector, 0, sizeof(RawMFMData));
  for (int byteNumber = 0; byteNumber < SECTOR_BYTES + SECTOR_BYTES; byteNumber++)
    for (int bit = 0; bit <= 7; bit++)
      if (sectorSum[byteNumber][bit].ones >= sectorSum[byteNumber][bit].zeros)
        outputSector.rawSector[byteNumber] |= (1 << bit);
  return true;
}
//------------------------------------------------------------------------------
// Extract and convert the sector.  This may be a duplicate so we may reject it.  Returns TRUE if it was valid, or false if not
//------------------------------------------------------------------------------
bool decodeSector(const RawEncodedSector& rawSector, const unsigned int trackNumber, const DiskSurface surface, DecodedTrack& decodedTrack, bool ignoreHeaderChecksum, int& lastSectorNumber) {
  DecodedSector sector;
  lastSectorNumber = -1;
  memcpy(sector.rawSector, rawSector, sizeof(RawMFMData));
  // Easier to operate on
  unsigned char* sectorData = (unsigned char*)rawSector;
  // Read the first 4 bytes (8).  This  is the track header data  
  sector.headerChecksumCalculated = decodeMFMdata((uint32_t*)(sectorData + 8), (uint32_t*)&sector, 4);
  //if(DEBUG) Serial.printf("Read the first 4 bytes (8).  This  is the track header data->$%8.8X\n",sector.headerChecksumCalculated);
  // Decode the label data and update the checksum
  sector.headerChecksumCalculated ^= decodeMFMdata((uint32_t*)(sectorData + 16), (uint32_t*)&sector.sectorLabel[0], 16);
  //if(DEBUG)   Serial.printf("Decode the label data and update the checksum data->$%8.8X\n",sector.headerChecksumCalculated);
  // Get the checksum for the header
  decodeMFMdata((uint32_t*)(sectorData + 48), (uint32_t*)&sector.headerChecksum, 4);  // (computed on mfm longs, longs between offsets 8 and 44 == 2 * (1 + 4) longs)
  //if(DEBUG) Serial.printf("Get the checksum for the header data->$%8.8X\n",sector.headerChecksum);
  // If the header checksum fails we just cant trust anything we received, so we just drop it
  if ((sector.headerChecksum != sector.headerChecksumCalculated) && (!ignoreHeaderChecksum)) {
    //if(DEBUG) Serial.print("If the header checksum fails we just cant trust anything we received, so we just drop it\n");
    return false;
  }
  // Check if the header contains valid fields
  if (sector.trackFormat != 0xFF) 
    return false;  // not valid
  //if(DEBUG) Serial.printf("sector.trackFormat=0x%2.2X\n",sector.trackFormat);  
  if (sector.sectorNumber > 10)
    return false;
  //if(DEBUG) Serial.printf("sector.sectorNumber=%d\n",sector.sectorNumber);  
  if (sector.trackNumber > 162) 
    return false;   // 81 tracks * 2 for both sides
  //if(DEBUG) Serial.printf("sector.trackNumber=%d\n",sector.trackNumber);  
  if (sector.sectorsRemaining > 11)
    return false;  // this isnt possible either
  if (sector.sectorsRemaining < 1)
    return false;  // or this
  //if(DEBUG) Serial.printf("sector.sectorsRemaining=%d\n",sector.sectorsRemaining);  
  // And is it from the track we expected?
  const unsigned char targetTrackNumber = (trackNumber << 1) | ((surface == DiskSurface::dsUpper) ? 1 : 0);
  //if(DEBUG) Serial.printf("targetTrackNumber=%d\n",targetTrackNumber);  
  if (sector.trackNumber != targetTrackNumber) return false; // this'd be weird
  // Get the checksum for the data
  decodeMFMdata((uint32_t*)(sectorData + 56), (uint32_t*)&sector.dataChecksum, 4);
  //if(DEBUG) Serial.printf("dataChecksum->$%8.8X\n",sector.dataChecksum);  

  //if(DEBUG) Serial.printf("validsectors vector capacity->%d\n", decodedTrack.validSectors.capacity());  
  //if(DEBUG) Serial.printf("validsectors vector max size->%u\n", decodedTrack.validSectors.max_size());  
  //if(DEBUG) Serial.printf("invalidsectors vector capacity->%d\n", decodedTrack.invalidSectors[0].capacity());  
  //if(DEBUG) Serial.printf("invalidsectors vector max_size->%d\n", decodedTrack.invalidSectors[0].max_size());  

  // Lets see if we already have this one
  const int searchSector = sector.sectorNumber;
  auto index = std::find_if(decodedTrack.validSectors.begin(), decodedTrack.validSectors.end(), [searchSector](const DecodedSector& sector) -> bool {
    return (sector.sectorNumber == searchSector);
  });
//  //if(DEBUG) Serial.printf("Lets see if we already have this sector %d\n",sector.sectorNumber);  
  // We already have it as a GOOD VALID sector, so skip, we don't need it.
  if (index != decodedTrack.validSectors.end()) return true;
  // We have a new candidate sector so decode the data and receive it's checksum
  sector.dataChecksumCalculated = decodeMFMdata((uint32_t*)(sectorData + 64), (uint32_t*)&sector.data[0], SECTOR_BYTES); // (from 64 to 1088 == 2*512 bytes)
  //if(DEBUG) Serial.printf("dataChecksumCalculated->$%8.8X\n",sector.dataChecksumCalculated);  
  lastSectorNumber = sector.sectorNumber;
  // Is the data valid?
  if (sector.dataChecksum != sector.dataChecksumCalculated) {
    /* SKIP IT
    // Keep a copy
     //if(DEBUG) Serial.printf("invalid dataChecksumCalculated->Keep a copy of sectorNumber=%d\n",sector.sectorNumber);  
    decodedTrack.invalidSectors[sector.sectorNumber].emplace_back(sector);
     //if(DEBUG) Serial.print("invalid sector stored in list\n");*/
    return false;
  }
  else {
    // Its a good sector, and we dont have it yet
    decodedTrack.validSectors.push_back(sector);
//    decodedTrack.validSectors.emplace_back(sector);
     //if(DEBUG)Serial.printf("valid sector (%d) stored in list\n",sector.sectorNumber);
    // Lets delete it from invalid sectors list
    ///decodedTrack.invalidSectors[sector.sectorNumber].clear();
    // //if(DEBUG) Serial.print("invalid sector (if exists) removed from list\n");
    return true;
  }
}
//------------------------------------------------------------------------------
// Encode a sector into the correct format for disk
//------------------------------------------------------------------------------
void encodeSector(const unsigned int trackNumber, const DiskSurface surface, const unsigned int sectorNumber, const RawDecodedSector& input, RawEncodedSector& encodedSector, unsigned char& lastByte) {
  // Sector Start
  encodedSector[0] = (lastByte & 1) ? 0x2A : 0xAA;
  encodedSector[1] = 0xAA;
  encodedSector[2] = 0xAA;
  encodedSector[3] = 0xAA;
  // Sector Sync
  encodedSector[4] = 0x44;
  encodedSector[5] = 0x89;
  encodedSector[6] = 0x44;
  encodedSector[7] = 0x89;

  // MFM Encoded header
  DecodedSector header;
  memset(&header, 0, sizeof(header));

  header.trackFormat = 0xFF;
  header.trackNumber = (trackNumber << 1) | ((surface == DiskSurface::dsUpper) ? 1 : 0);
  header.sectorNumber = sectorNumber; 
  header.sectorsRemaining = NUM_SECTORS_PER_TRACK_DD - sectorNumber;  //1..11
  
  
  header.headerChecksumCalculated = encodeMFMdataPart1((const unsigned long*)&header, (unsigned long*)&encodedSector[8], 4);
  // Then theres the 16 bytes of the volume label that isnt used anyway
  header.headerChecksumCalculated ^= encodeMFMdataPart1((const unsigned long*)&header.sectorLabel, (unsigned long*)&encodedSector[16], 16);
  // Thats 40 bytes written as everything doubles (8+4+4+16+16). - Encode the header checksum
  encodeMFMdataPart1((const unsigned long*)&header.headerChecksumCalculated, (unsigned long*)&encodedSector[48], 4);
  // And move on to the data section.  Next should be the checksum, but we cant encode that until we actually know its value!
  header.dataChecksumCalculated = encodeMFMdataPart1((const unsigned long*)&input, (unsigned long*)&encodedSector[64], SECTOR_BYTES);
  // And add the checksum
  encodeMFMdataPart1( (const unsigned long*)&header.dataChecksumCalculated, (unsigned long*)&encodedSector[56], 4);

  // Now fill in the MFM clock bits
  bool lastBit = encodedSector[7] & (1 << 0);
  bool thisBit = lastBit;

  // Clock bits are bits 7, 5, 3 and 1
  // Data is 6, 4, 2, 0
  for (int count = 8; count < RAW_SECTOR_SIZE; count++) {
    for (int bit = 7; bit >= 1; bit -= 2) {
      lastBit = thisBit;      
      thisBit = encodedSector[count] & (1 << (bit-1));
  
      if (!(lastBit || thisBit)) {
        // Encode a 1!
        encodedSector[count] |= (1 << bit);
      }
    }
  }

  lastByte = encodedSector[RAW_SECTOR_SIZE - 1];
}

//------------------------------------------------------------------------------
// Find sectors within raw data read from the drive by searching bit-by-bit for the SYNC bytes
//------------------------------------------------------------------------------
void findSectors(const unsigned char* track, unsigned int trackNumber, DiskSurface side, unsigned short trackSync, DecodedTrack& decodedTrack, bool ignoreHeaderChecksum) {
  // Work out what we need to search for which is syncsync
  const uint32_t search = (trackSync | (((uint32_t)trackSync) << 16));
  // Prepare our test buffer
  uint32_t decoded = 0;
  // Keep runnign until we run out of data
  unsigned int byteIndex = 0;
  int nextTrackBitCount = 0;
  const unsigned int dataLength = RAW_TRACKDATA_LENGTH;
  const int maxSectors =  NUM_SECTORS_PER_TRACK_DD;
  // run the entire track length
  while (byteIndex < dataLength) {
    // Check each bit, the "decoded" variable slowly slides left providing a 32-bit wide "window" into the bitstream
    for (int bitIndex = 7; bitIndex >= 0; bitIndex--) {
      decoded <<= 1;   // shift off one bit to make room for the new bit
      if (track[byteIndex] & (1 << bitIndex)) decoded |= 1;
      // Have we matched the sync words correctly
      ++nextTrackBitCount;
      int lastSectorNumber = -1;
      if (decoded == search) {
        //if(DEBUG)Serial.printf("Sector found at BYTE_INDEX=$%4.4X(%d) BIT_INDEX=%d\n",byteIndex, byteIndex, bitIndex);
        RawEncodedSector alignedSector;
        // We extract ALL of the track data from this BIT to byte align it properly, then pass it onto the code to read the sector (from the start of the sync code)
        alignSectorToByte(track, dataLength, byteIndex, bitIndex, alignedSector);
        // Now see if there's a valid sector there.  We now only skip the sector if its valid, incase rogue data gets in there
        if (decodeSector(alignedSector, trackNumber, side, decodedTrack, ignoreHeaderChecksum, lastSectorNumber)) {
          // We know the size of this buffer, so we can skip by exactly this amount
          byteIndex += RAW_SECTOR_SIZE - 8; // skip this many bytes as we know this is part of the track! minus 8 for the SYNC
          if (byteIndex >= dataLength) break;
          // We know that 8 bytes from here should be another track. - we allow 1 bit either way for slippage, but this allows an extra check incase the SYNC pattern is damaged
          nextTrackBitCount = 0;
        }
        else {
          // Decode failed.  Lets try a "homemade" one
          DecodedSector newTrack;
          //memset(newTrack,0,sizeof(newTrack));
          if ((lastSectorNumber >= 0) && (lastSectorNumber < maxSectors)) {
            newTrack.sectorNumber = lastSectorNumber;
            if (attemptFixSector(decodedTrack, newTrack)) {
              memcpy(newTrack.rawSector, alignedSector, sizeof(newTrack.rawSector));
              // See if our makeshift data will decode or not
              if (decodeSector(alignedSector, trackNumber, side, decodedTrack, ignoreHeaderChecksum, lastSectorNumber)) {
                // We know the size of this buffer, so we can skip by exactly this amount
                byteIndex += RAW_SECTOR_SIZE - 8; // skip this many bytes as we know this is part of the track! minus 8 for the SYNC
                if (byteIndex >= dataLength) break;
              }
            }
          }
          if (decoded == search) nextTrackBitCount = 0;
        }
      }
    }
    byteIndex++;
  }
}
//------------------------------------------------------------------------------
// Merges any invalid sectors into the valid ones as a last resort
//------------------------------------------------------------------------------
/*void mergeInvalidSectors(DecodedTrack& track) {
  for (unsigned char sector = 0; sector < NUM_SECTORS_PER_TRACK_DD; sector++) {
    if (track.invalidSectors[sector].size()) {
      // Lets try to make the best sector we can
      DecodedSector sec = track.invalidSectors[sector][0];
      // Repair maybe!?
      attemptFixSector(track, sec);
      track.validSectors.push_back(sec);
    }
    track.invalidSectors[sector].clear();
  }
}*/
//------------------------------------------------------------------------------
// WRITEBIT
//------------------------------------------------------------------------------
void writeBit(RawTrackData& output, int& pos, int& bit, int value) {
  if (pos >= RAW_TRACKDATA_LENGTH) return;
  output[pos] <<= 1;
  output[pos] |= value;
  bit++;
  if (bit >= 8) {
    pos++;
    bit = 0;
  }
}
//------------------------------------------------------------------------------
// UNPACK
//------------------------------------------------------------------------------
void unpack(const RawTrackData& data, RawTrackData& output) {
  int pos = 0;
  size_t index = 0;
  int p2 = 0;
  memset(output, 0, sizeof(output));
  while (pos < RAW_TRACKDATA_LENGTH) {
    // Each byte contains four pairs of bits that identify an MFM sequence to be encoded
    for (int b = 6; b >= 0; b -= 2) {
      unsigned char value = (data[index] >> b) & 3;
      switch (value) {
      case 0:
        // This can't happen, its invalid data but we account for 4 '0' bits
        writeBit(output, pos, p2, 0);
        writeBit(output, pos, p2, 0);
        writeBit(output, pos, p2, 0);
        writeBit(output, pos, p2, 0);
        break;
      case 1: // This is an '01'
        writeBit(output, pos, p2, 0);
        writeBit(output, pos, p2, 1);
        break;
      case 2: // This is an '001'
        writeBit(output, pos, p2, 0);
        writeBit(output, pos, p2, 0);
        writeBit(output, pos, p2, 1);
        break;
      case 3: // this is an '0001'
        writeBit(output, pos, p2, 0);
        writeBit(output, pos, p2, 0);
        writeBit(output, pos, p2, 0);
        writeBit(output, pos, p2, 1);
        break;
      }
    }
    index++;
    if (index >= sizeof(data)) return;
  }
  // There will be left-over data
}
//------------------------------------------------------------------------------
// Read RAW data from the current track and surface 
//------------------------------------------------------------------------------
DiagnosticResponse readCurrentTrack(RawTrackData& trackData, const bool readFromIndexPulse) {
  m_lastError = runCommand(COMMAND_READTRACK);

  RawTrackData tmp;

  // Allow command retry
  if (m_lastError != DiagnosticResponse::drOK) {
    // Clear the buffer
    deviceRead(&tmp, RAW_TRACKDATA_LENGTH);
    m_lastError = runCommand(COMMAND_READTRACK);
    if (m_lastError != DiagnosticResponse::drOK) {
      m_lastCommand = LastCommand::lcReadTrack;
      return m_lastError;
    }
  }

  unsigned char signalPulse = readFromIndexPulse ? 1 : 0;
  if (!deviceWrite(&signalPulse, 1)) {
    m_lastCommand = LastCommand::lcReadTrack;
    m_lastError = DiagnosticResponse::drSendParameterFailed;
    return m_lastError;
  }

  // Keep reading until he hit RAW_TRACKDATA_LENGTH or a null byte is recieved
  int bytePos = 0;
  int readFail = 0;
  for (;;) {
    unsigned char value;
    if (deviceRead(&value, 1, true)) {
      if (value == 0) break; else
        if (bytePos < RAW_TRACKDATA_LENGTH) tmp[bytePos++] = value;
    }
    else {
      readFail++;
      if (readFail > 4) {
        m_lastCommand = LastCommand::lcReadTrack;
        m_lastError = DiagnosticResponse::drReadResponseFailed;
        return m_lastError;
      }
    }
  }
  unpack(tmp, trackData);
  m_lastError = DiagnosticResponse::drOK;
  return m_lastError;
}
//------------------------------------------------------------------------------
#define PRECOMP_NONE 0x00
#define PRECOMP_ERLY 0x04   
#define PRECOMP_LATE 0x08   
//------------------------------------------------------------------------------
/* If we have a look at the previous and next three bits, assume the current is a '1', then these are the only valid combinations that coudl be allowed
  I have chosen the PRECOMP rule based on trying to get the current '1' as far away as possible from the nearest '1'

     LAST    CURRENT      NEXT      PRECOMP     Hex Value
  0 0 0     1    0  0 0   Normal
  0 0 0     1    0  0 1       Early     0x09
  0 0 0     1    0  1 0       Early     0x0A
  0 1 0     1    0  0 0       Late      0x28
  0 1 0     1    0  0 1       Late      0x29
  0 1 0     1    0  1 0       Normal  
  1 0 0     1    0  0 0       Late      0x48  
  1 0 0     1    0  0 1       Normal  
  1 0 0     1    0  1 0       Early     0x4A
*/
//------------------------------------------------------------------------------
// Read a bit from the data provided
//------------------------------------------------------------------------------
inline int readBit(const unsigned char* buffer, const unsigned int maxLength, int& pos, int& bit) {
  if (pos >= (int)maxLength) {
    bit--;
    if (bit < 0) {
      bit = 7;
      pos++;
    }
    return (bit & 1) ? 0 : 1;
  }

  int ret = (buffer[pos] >> bit) & 1;
  bit--;
  if (bit < 0) {
    bit = 7;
    pos++;
  }

  return ret;
}

//------------------------------------------------------------------------------
// The precomp version of the above. Dont use the above function directly to write precomp mode, it wont work.  Data must be passed with an 0xAA each side at least
//------------------------------------------------------------------------------
DiagnosticResponse writeCurrentTrackPrecomp(const unsigned char* mfmData, const unsigned short numBytes, const bool writeFromIndexPulse, bool usePrecomp) {
  m_lastCommand = LastCommand::lcWriteTrack;
  //if ((m_version.major == 1) && (m_version.minor < 8)) return DiagnosticResponse::drOldFirmware;

  // First step is we need to re-encode the supplied buffer into a packed format with precomp pre-calculated.
  // Each nybble looks like: xxyy
  // where xx is: 0=no precomp, 1=-ve, 2=+ve
  //     yy is: 0: 4us,   1: 6us,   2: 8us,   3: 10us

  // *4 is a worse case situation, ie: if everything was a 01.  The +128 is for any extra padding
  const unsigned int maxOutSize = (numBytes * 4) + 16;
  unsigned char* outputBuffer = (unsigned char*)malloc(maxOutSize);

  if (!outputBuffer) {
    m_lastError = DiagnosticResponse::drSendParameterFailed;
    return m_lastError;
  }

  // Original data was written from MSB downto LSB
  int pos = 0;
  int bit = 7;
  unsigned int outputPos = 0;
  unsigned char sequence = 0xAA;  // start at 10101010
  unsigned char* output = outputBuffer;
  int lastCount = 2;

  // Re-encode the data into our format and apply precomp.  The +8 is to ensure theres some padding around the edge which will come out as 010101 etc
  while (pos < numBytes + 8) {
    *output = 0;

    for (int i = 0; i < 2; i++) {
      int b, count = 0;

      // See how many zero bits there are before we hit a 1
      do {
        b = readBit(mfmData, numBytes, pos, bit);
        sequence = ((sequence << 1) & 0x7F) | b;
        count++;
      } while (((sequence & 0x08) == 0) && (pos < numBytes + 8));

      // Validate range
      if (count < 2) count = 2;  // <2 would be a 11 sequence, not allowed
      if (count > 5) count = 5;  // max we support 01, 001, 0001, 00001

      // Write to stream. Based on the rules above we apply some precomp
      int precomp = 0;
      if (usePrecomp) {
        switch (sequence) {
        case 0x09:
        case 0x0A:
        case 0x4A: // early;
          precomp = PRECOMP_ERLY;
          break;

        case 0x28:
        case 0x29:
        case 0x48: // late
          precomp = PRECOMP_LATE;
          break;

        default:
          precomp = PRECOMP_NONE;
          break;
        }
      }
      else precomp = PRECOMP_NONE;

      *output |= ((lastCount - 2) | precomp) << (i * 4);
      lastCount = count;
    }

    output++;
    outputPos++;
    if (outputPos >= maxOutSize) {
      // should never happen
      free(outputBuffer);
      m_lastError = DiagnosticResponse::drSendParameterFailed;
      return m_lastError;
    }
  }

  m_lastError = internalWriteTrack(outputBuffer, outputPos, writeFromIndexPulse, true);

  free(outputBuffer);

  return m_lastError;
}
//------------------------------------------------------------------------------
// Run a command that returns 1 or 0 for its response
//------------------------------------------------------------------------------
DiagnosticResponse runCommand(const char command, const char parameter, char* actualResponse) {
  unsigned char response;

  // Pause for I/O
  //std::this_thread::sleep_for(std::chrono::milliseconds(1));

  // Send the command
  if (!deviceWrite(&command, 1)) {
    m_lastError = DiagnosticResponse::drSendFailed;
    return m_lastError;
  }

  // Only send the parameter if its not NULL
  if (parameter != '\0')
    if (!deviceWrite(&parameter, 1)) {
      m_lastError = DiagnosticResponse::drSendParameterFailed;
      return m_lastError;
    }

  // And read the response
  if (!deviceRead(&response, 1, true)) {
    m_lastError = DiagnosticResponse::drReadResponseFailed;
    return m_lastError;
  }

  if (actualResponse) *actualResponse = response;

  // Evaluate the response
  switch (response) {
  case '1': m_lastError = DiagnosticResponse::drOK;
    break;
  case '0': m_lastError = DiagnosticResponse::drError;
    break;
  default:  m_lastError = DiagnosticResponse::drStatusError;
    break;
  }
  return m_lastError;
}
//------------------------------------------------------------------------------
// Read a desired number of bytes into the target pointer
//------------------------------------------------------------------------------
bool deviceRead(void* target, const unsigned int numBytes, const bool failIfNotAllRead) {
//  if (!m_comPort.isPortOpen()) return false;
  unsigned long read=uart_read_bytes(uart_num, (uint8_t*)target, numBytes, PACKET_READ_TICS+100);
  if (read < numBytes) {
    if (failIfNotAllRead) return false;
    // Clear the unread bytes
    char* target2 = ((char*)target) + read;
    memset(target2, 0, numBytes - read);
  }
  return true;
}
//------------------------------------------------------------------------------
// Writes a desired number of bytes from the the pointer
//------------------------------------------------------------------------------
bool deviceWrite(const void* source, const unsigned int numBytes) {
  return uart_write_bytes(uart_num, (const char*)source , numBytes)==numBytes;
  //m_comPort.write(source, numBytes) == numBytes;
}
//------------------------------------------------------------------------------
// Writes RAW data onto the current track
//------------------------------------------------------------------------------
DiagnosticResponse writeCurrentTrack(const unsigned char* data, const unsigned short numBytes, const bool writeFromIndexPulse) {
  return internalWriteTrack(data, numBytes, writeFromIndexPulse, false);
}
//------------------------------------------------------------------------------
// Writes RAW data onto the current track
//------------------------------------------------------------------------------
DiagnosticResponse internalWriteTrack(const unsigned char* data, const unsigned short numBytes, const bool writeFromIndexPulse, bool usePrecomp) {
  /*
  // Fall back if older firmware
  if ((m_version.major == 1) && (m_version.minor < 8) && usePrecomp) {
    return DiagnosticResponse::drOldFirmware;
  }*/
  m_lastError = runCommand(usePrecomp ? COMMAND_WRITETRACKPRECOMP : COMMAND_WRITETRACK);
  if (m_lastError != DiagnosticResponse::drOK) {
    m_lastCommand = LastCommand::lcWriteTrack;
    return m_lastError;
  }
  unsigned char chr;
  if (!deviceRead(&chr, 1, true)) {
    m_lastCommand = LastCommand::lcWriteTrack;
    m_lastError = DiagnosticResponse::drReadResponseFailed;
    return m_lastError;
  }

  // 'N' means NO Writing, aka write protected
  if (chr == 'N') {
    m_lastCommand = LastCommand::lcWriteTrack;
    m_lastError = DiagnosticResponse::drWriteProtected;
    return m_lastError;
  }
  if (chr != 'Y') {
    m_lastCommand = LastCommand::lcWriteTrack;
    m_lastError = DiagnosticResponse::drStatusError;
    return m_lastError;
  }

  // Now we send the number of bytes we're planning on transmitting
  unsigned char b = (numBytes >> 8);
  if (!deviceWrite(&b, 1)) {
    m_lastCommand = LastCommand::lcWriteTrack;
    m_lastError = DiagnosticResponse::drSendParameterFailed;
    return m_lastError;
  }
  b = numBytes & 0xFF;
  if (!deviceWrite(&b, 1)) {
    m_lastCommand = LastCommand::lcWriteTrack;
    m_lastError = DiagnosticResponse::drSendParameterFailed;
    return m_lastError;
  }

  // Explain if we want index pulse sync writing (slower and not required by normal AmigaDOS disks)
  b = writeFromIndexPulse ? 1 : 0;
  if (!deviceWrite(&b, 1)) {
    m_lastCommand = LastCommand::lcWriteTrack;
    m_lastError = DiagnosticResponse::drSendParameterFailed;
    return m_lastError;
  }

  unsigned char response;
  if (!deviceRead(&response, 1, true)) {
    m_lastCommand = LastCommand::lcWriteTrack;
    m_lastError = DiagnosticResponse::drReadResponseFailed;
    return m_lastError;
  }

  if (response != '!') {
    m_lastCommand = LastCommand::lcWriteTrack;
    m_lastError = DiagnosticResponse::drStatusError;

    return m_lastError;
  }

  if (!deviceWrite((const void*)data, numBytes)) {
    m_lastCommand = LastCommand::lcWriteTrack;
    m_lastError = DiagnosticResponse::drSendDataFailed;
    return m_lastError;
  }

  if (!deviceRead(&response, 1, true)) {
    m_lastCommand = LastCommand::lcWriteTrack;
    m_lastError = DiagnosticResponse::drTrackWriteResponseError;
    return m_lastError;
  }

  // If this is a '1' then the Arduino didn't miss a single bit!
  if (response != '1') {
    m_lastCommand = LastCommand::lcWriteTrack;
    switch (response) {
    case 'X': m_lastError = DiagnosticResponse::drWriteTimeout;  break;
    case 'Y': m_lastError = DiagnosticResponse::drFramingError; break;
    case 'Z': m_lastError = DiagnosticResponse::drSerialOverrun; break;
    default:
      m_lastError = DiagnosticResponse::drStatusError;
      break;
    }
    return m_lastError;
  }

  m_lastError = DiagnosticResponse::drOK;
  return m_lastError;
}

//------------------------------------------------------------------------------
DiagnosticResponse findTrack0() {
  // And rewind to the first track
  char status = '0';
  m_lastError = runCommand(COMMAND_REWIND, '\000', &status);
  if (m_lastError != DiagnosticResponse::drOK) {
    m_lastCommand = LastCommand::lcRewind;
    if (status == '#') return DiagnosticResponse::drRewindFailure;
    return m_lastError;
  }
  return m_lastError;
}

//------------------------------------------------------------------------------
// Choose which surface of the disk to read from
//------------------------------------------------------------------------------
DiagnosticResponse selectSurface(const DiskSurface side) {
  m_lastError = runCommand((side == DiskSurface::dsUpper) ? COMMAND_HEAD0 : COMMAND_HEAD1);
  if (m_lastError != DiagnosticResponse::drOK) {
    m_lastCommand = LastCommand::lcSelectSurface;
    return m_lastError;
  }
  return m_lastError;
}

//------------------------------------------------------------------------------
// Turns on and off the writing interface.  If irError is returned the disk is write protected
//------------------------------------------------------------------------------
DiagnosticResponse enableWriting(const bool enable, const bool reset) {
  if (enable) {
    // Enable the device
    m_lastError = runCommand(COMMAND_ENABLEWRITE);
    if (m_lastError == DiagnosticResponse::drError) {
      m_lastCommand = LastCommand::lcEnableWrite;
      m_lastError = DiagnosticResponse::drWriteProtected;
      return m_lastError;
    }
    if (m_lastError != DiagnosticResponse::drOK) {
      m_lastCommand = LastCommand::lcEnableWrite;
      return m_lastError;
    }
    m_inWriteMode = true;

    // Reset?
    if (reset) {
      // And rewind to the first track
      m_lastError = findTrack0();
      if (m_lastError != DiagnosticResponse::drOK) return m_lastError;

      // Lets know where we are
      return selectSurface(DiskSurface::dsUpper);
    }
    m_lastError = DiagnosticResponse::drOK;
    return m_lastError;
  }
  else {
    // Disable the device
    m_lastError = runCommand(COMMAND_DISABLE);
    if (m_lastError != DiagnosticResponse::drOK) {
      m_lastCommand = LastCommand::lcDisableMotor;
      return m_lastError;
    }
    m_inWriteMode = false;

    return m_lastError;
  }
}

//------------------------------------------------------------------------------
// Select the track, this makes the motor seek to this position
//------------------------------------------------------------------------------
DiagnosticResponse selectTrack(const unsigned char trackIndex, const TrackSearchSpeed searchSpeed, bool ignoreDiskInsertCheck) {
  if (trackIndex > 81) {
    m_lastError = DiagnosticResponse::drTrackRangeError;
    return m_lastError; // no chance, it can't be done.
  }
  // And send the command and track.  This is sent as ASCII text as a result of terminal testing.  Easier to see whats going on
  bool isV18 = (m_version.major > 1) || ((m_version.major == 1) && (m_version.minor >= 8));
  char buf[8];
  if (isV18) {
    char flags = 0;
    switch (searchSpeed) {
    case TrackSearchSpeed::tssNormal:   flags = 1; break;                    // Speed - 1
    case TrackSearchSpeed::tssFast:     flags = 2; break;                    // Speed - 2
    case TrackSearchSpeed::tssVeryFast: flags = 3; break;         // Speed - 3
    default: break;
    }
    if (!ignoreDiskInsertCheck) flags |= 4;
    sprintf(buf, "%c%02i%c", COMMAND_GOTOTRACK_REPORT, trackIndex, flags);
  }
  else {
    sprintf(buf, "%c%02i", COMMAND_GOTOTRACK, trackIndex);
  }

  // Send track number. 
  if (!deviceWrite(buf, strlen(buf))) {
    m_lastCommand = LastCommand::lcGotoTrack;
    m_lastError = DiagnosticResponse::drSendFailed;
    return m_lastError;
  }

  // Get result
  char result;

  if (!deviceRead(&result, 1, true)) {
    m_lastCommand = LastCommand::lcGotoTrack;
    m_lastError = DiagnosticResponse::drReadResponseFailed;
    return m_lastError;
  }

  switch (result) {
  case '2':  m_lastError = DiagnosticResponse::drOK; break; // already at track.  No op needed.  V1.8 only
  case '1':   m_lastError = DiagnosticResponse::drOK;
    if (isV18) {
      // Read extended data
      char status;
      if (!deviceRead(&status, 1, true)) {
        m_lastError = DiagnosticResponse::drReadResponseFailed;
        return m_lastError;
      }
      // 'x' means we didnt check it
      if (status != 'x') m_diskInDrive = status == '1';

      // Also read the write protect status
      if (!deviceRead(&status, 1, true)) {
        m_lastError = DiagnosticResponse::drReadResponseFailed;
        return m_lastError;
      }
      m_isWriteProtected = status == '1';
    }
    break;
  case '0':   m_lastCommand = LastCommand::lcGotoTrack;
    m_lastError = DiagnosticResponse::drSelectTrackError;
    break;
  default:  m_lastCommand = LastCommand::lcGotoTrack;
    m_lastError = DiagnosticResponse::drStatusError;
    break;
  }

  return m_lastError;
}
//------------------------------------------------------------------------------
// Writes an ADF file back to a floppy disk.  Return FALSE in the callback to abort this operation 
// IF using precomp mode then DO NOT connect the Arduino via a USB hub, and try to plug it into a USB2 port
//------------------------------------------------------------------------------
ADFResult ADFToDisk(const std::string& inputFile, bool verify, bool usePrecompMode, std::function < WriteResponse(const int currentTrack, const DiskSurface currentSide, const bool isVerifyError) > callback) {
//  if (isOpen()) return ADFResult::adfrDriveError;

  // Upgrade to writing mode
  if (enableWriting(true, true)!=DiagnosticResponse::drOK) return ADFResult::adfrDriveError;

  // Attempt to open the file
  std::string inputFileA;
//  quickw2a(inputFile,inputFileA);
  std::ifstream hADFFile(inputFileA, std::ifstream::in | std::ifstream::binary);
  if (!hADFFile.is_open()) return ADFResult::adfrFileError;

  unsigned int currentTrack = 0;
  unsigned int surfaceIndex = 0;

  // Buffer to hold the track
  RawDecodedTrack track;
  FullDiskTrack disktrack;
  memset(disktrack.filler1, 0xAA, sizeof(disktrack.filler1));  // Pad with "0"s which as an MFM encoded byte is 0xAA
  memset(disktrack.filler2, 0xAA, sizeof(disktrack.filler2));  // Pad with "0"s which as an MFM encoded byte is 0xAA

  // Just make sure nothing weird is going on
  assert(sizeof(track) == ADF_TRACK_SIZE_DD);
  bool errors = false;


  while (hADFFile.good()) {
    hADFFile.read((char*)&track, ADF_TRACK_SIZE_DD);
    std::streamsize bytesRead = hADFFile.gcount();

    // Stop if we didnt read a full track
    if (bytesRead != ADF_TRACK_SIZE_DD) break;

    // Select the track we're working on
    if (selectTrack(currentTrack)!= DiagnosticResponse::drOK) {
      hADFFile.close();
      return ADFResult::adfrDriveError;
    }

    DiskSurface surface = (surfaceIndex == 1) ? DiskSurface::dsUpper : DiskSurface::dsLower;
    // Change the surface we're targeting
    if (selectSurface(surface) != DiagnosticResponse::drOK) {
      hADFFile.close();
      return ADFResult::adfrDriveError;
    }

    // Handle callback
    if (callback)
      if (callback(currentTrack, surface, false) == WriteResponse::wrAbort) {
        hADFFile.close();
        return ADFResult::adfrAborted;
      }

    unsigned char lastByte = disktrack.filler1[sizeof(disktrack.filler1)-1];
    // Now encode the sector into the output buffer
    for (unsigned int sector = 0; sector < NUM_SECTORS_PER_TRACK_DD; sector++)
      encodeSector(currentTrack, surface, sector, track[sector], disktrack.sectors[sector], lastByte);
    if (lastByte & 1) disktrack.filler2[7] = 0x2F; else disktrack.filler2[7] = 0xFF;

    // Keep looping until it wrote correctly
    DecodedTrack trackRead;
    trackRead.validSectors.clear();
    for (unsigned int a = 0; a < NUM_SECTORS_PER_TRACK_DD; a++) trackRead.invalidSectors[a].clear();

    int failCount = 0;
    while (trackRead.validSectors.size()<NUM_SECTORS_PER_TRACK_DD) {

      DiagnosticResponse resp = writeCurrentTrackPrecomp((const unsigned char*)(&disktrack), sizeof(disktrack), false, (currentTrack >= 40) && usePrecompMode);
      if (resp == DiagnosticResponse::drOldFirmware) resp = writeCurrentTrack((const unsigned char*)(&disktrack), sizeof(disktrack), false);   

      switch (resp) {
      case DiagnosticResponse::drWriteProtected:  hADFFile.close();
                            return ADFResult::adfrDiskWriteProtected;
      case DiagnosticResponse::drOK: break;
      default: hADFFile.close();
           return ADFResult::adfrDriveError;
      }

      if (verify) {       
        for (int retries=0; retries<10; retries++) {
          RawTrackData data;
          // Read the track back
          if (readCurrentTrack(data, false) == DiagnosticResponse::drOK) {
            // Find hopefully all sectors
            findSectors(data, currentTrack, surface, AMIGA_WORD_SYNC, trackRead, false);
          }
          if (trackRead.validSectors.size() == NUM_SECTORS_PER_TRACK_DD) break;
        } 

        // So we found all 11 sectors, but were they the ones we actually wrote!?
        if (trackRead.validSectors.size() == NUM_SECTORS_PER_TRACK_DD) {
          int sectorsGood = 0;
          for (int sector = 0; sector < NUM_SECTORS_PER_TRACK_DD; sector++) {
            auto index = std::find_if(trackRead.validSectors.begin(), trackRead.validSectors.end(), [sector](const DecodedSector& sectorfound) -> bool {
              return (sectorfound.sectorNumber == sector);
            });

            // We found this sector.
            if (index != trackRead.validSectors.end()) {
              DecodedSector& rec = trackRead.validSectors[index - trackRead.validSectors.begin()];
              if (memcmp(rec.data, track[sector], SECTOR_BYTES) == 0) {
                sectorsGood++;  // this one matches on read!
              }
            }
          }
          if (sectorsGood != NUM_SECTORS_PER_TRACK_DD) {
            // Something went wrong, so we clear them all so it gets reported as an error
            trackRead.validSectors.clear();
          }
        }


        // We failed to verify this track.
        if (trackRead.validSectors.size() < NUM_SECTORS_PER_TRACK_DD) {
          failCount++;
          if (failCount >= 5) {
            if (!callback) break;
            bool breakOut = false;

            switch (callback(currentTrack, surface, true)) {
            case WriteResponse::wrAbort: hADFFile.close();
              return ADFResult::adfrAborted;
            case WriteResponse::wrSkipBadChecksums: breakOut = true; errors = true; break;
            case WriteResponse::wrContinue: break;
            default: break;
            }
            if (breakOut) break;
            failCount = 0;
          }
        }
      }
      else break;
    }

    // Goto the next surface/track
    surfaceIndex++;
    if (surfaceIndex > 1) {
      surfaceIndex = 0;
      currentTrack++;
    }
    // But there is a physical limit
    if (currentTrack > 81) break; 
  }

  return errors? ADFResult::adfrCompletedWithErrors: ADFResult::adfrComplete;
}
//------------------------------------------------------------------------------
// Read an ADF file and write it to disk
//------------------------------------------------------------------------------
void adf2Disk(const std::string& filename, bool verify) {
  Serial.printf("\nWrite disk from ADF mode\n\n");
  if (!verify) Serial.printf("WARNING: It is STRONGLY recommended to write with verify support turned on.\r\n\r\n");

  ADFResult result = ADFToDisk(filename,verify,true, [](const int currentTrack, const DiskSurface currentSide, bool isVerifyError) ->WriteResponse {
    if (isVerifyError) {
      char input='A';
      /*do {
        printf("\rDisk write verify error on track %i, %s side. [R]etry, [S]kip, [A]bort?                                   ", currentTrack, (currentSide == DiskSurface::dsUpper) ? "Upper" : "Lower");
        input = toupper(_getChar());
      } while ((input != 'R') && (input != 'S') && (input != 'A'));
      */
      switch (input) {
      case 'R': return WriteResponse::wrRetry;
      case 'I': return WriteResponse::wrSkipBadChecksums;
      case 'A': return WriteResponse::wrAbort;
      }
    }
    Serial.printf("\rWriting Track %i, %s side     ", currentTrack, (currentSide == DiskSurface::dsUpper) ? "Upper" : "Lower");
    return WriteResponse::wrContinue;
  });

  switch (result) {
  case ADFResult::adfrComplete:         Serial.printf("\rADF file written to disk                                                           "); break;
  case ADFResult::adfrCompletedWithErrors:    Serial.printf("\rADF file written to disk but there were errors during verification                 "); break;
  case ADFResult::adfrAborted:          Serial.printf("\rWriting ADF file to disk                                                           "); break;
  case ADFResult::adfrFileError:          Serial.printf("\rError opening ADF file.                                                            "); break;
  case ADFResult::adfrDriveError:         Serial.printf("\rError communicating with the Arduino interface.                                    "); 
                          Serial.printf("\n%s                                                  ", getLastError().c_str()); break;
  case ADFResult::adfrDiskWriteProtected:     Serial.printf("\rError, disk is write protected!                                                    "); break;
  default:                    Serial.printf("\rAn unknown error occured                                                           "); break;
  }
}
//------------------------------------------------------------------------------
// SETUP
//------------------------------------------------------------------------------
void setup(void){
//------------------------------------------------------------------------------
// INITIALIZE DEFAULT SERIAL INTERFACE
//------------------------------------------------------------------------------
  Serial.begin(BAUDRATE); 
//------------------------------------------------------------------------------
// INITIALIZE PULLUPS SLOT1 SD CARD INTERFACE
//  SD-card interface can not initialize properly without PULLUPS
//   Start in 1-bit SD_MMC bus mode
//    TODO: read wifi credentials from SD or use function to put it in EPROM which needs user interaction
//------------------------------------------------------------------------------
  pinMode(2, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
  pinMode(15  , INPUT_PULLUP);
  bool fsok = SD_MMC.begin("/sdcard", true);
  Serial.printf("SD_MMC FileSystem: %s\n", fsok ? "OK" : "FAIL!");
//------------------------------------------------------------------------------
// what todo in case of FAILURE ??
//------------------------------------------------------------------------------
  if (!fsok) {
    Serial.println("FATAL ERROR INITIALIZING SD_MMC");
    Serial.println("Check the SD card and [R]eboot");
  }else{  
   WiFi.begin(ssid, password);
  // Wait for connection
   while (WiFi.status() != WL_CONNECTED)
   {
    delay(500);
    Serial.printf(".");
   }
   Serial.printf("\nConnected to %s, IP address is %s\n", ssid, WiFi.localIP().toString().c_str());
//------------------------------------------------------------------------------
// Init and get the TIME from NTPSERVER which for now uses external ntp server
// we should only do this when the time stored in RTC is too far off
// to prevent outgoing traffic it's best to use a router with build-in NTP service
//------------------------------------------------------------------------------
   configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
   struct tm tmstruct ;
   delay(3000);
   tmstruct.tm_year = 0;
   getLocalTime(&tmstruct, 5000);
   Serial.printf("Now is : %d-%02d-%02d %02d:%02d:%02d\n",(tmstruct.tm_year)+1900,( tmstruct.tm_mon)+1, tmstruct.tm_mday,tmstruct.tm_hour , tmstruct.tm_min, tmstruct.tm_sec);
   rtc.setTime(tmstruct.tm_sec, tmstruct.tm_min, tmstruct.tm_hour, tmstruct.tm_mday, ( tmstruct.tm_mon)+1, (tmstruct.tm_year)+1900);
//------------------------------------------------------------------------------
// ESP32 has "lots" of RAM compared to MEGA2560 but 160kb is a tight workspace
// we also run a FTP server remember..
// for the core-routines "ported" from the PC app. we need 2 big buffers and 
// we re-use them over-and-over again (TMP_RawTrackData and MFM_RawTrackData)
//------------------------------------------------------------------------------
   if (allocateTMP_RawTrackData()){
    Serial.printf("TMP_RawTrackData [%d] initialized\n",TMP_RawTrackDataSize);
   }
   else {
    Serial.println("ERROR: not enough memory for TMP_RawTrackData");
   }
//------------------------------------------------------------------------------
   if (allocateMFM_RawTrackData()){
    Serial.printf("MFM_RawTrackData [%d] initialized\n",MFM_RawTrackDataSize);
   }
   else {
    Serial.println("ERROR: not enough memory for MFM_RawTrackData");
   }
//------------------------------------------------------------------------------
// SETUP THE FTPSERVER with username and password
// ports are defined in FTPCommon.h
//------------------------------------------------------------------------------
   ftpSrv.begin("amiga", "amiga");
//------------------------------------------------------------------------------
    uart_config_t uart_config = {
        .baud_rate  = BAUDRATE_2MBAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_CTS,//UART_HW_FLOWCTRL_DISABLE, //UART_HW_FLOWCTRL_CTS,
        .rx_flow_ctrl_thresh = 122,
    };
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_num, ESP32_UART2_TXD, ESP32_UART2_RXD, ESP32_UART2_RTS, ESP32_UART2_CTS));
    //uart_driver_install(uart_num, BUF_SIZE,0, 0, NULL, 0);
    uart_driver_install(uart_num, BUF_SIZE*2,0, 0, NULL, 0);
    Serial.println("INIT_UART2_OK (2MBaud)");
    Serial.printf("Welcome to %s\n", TAG);
  }
}
//------------------------------------------------------------------------------
//MEGA2560 floppy related commands come/go from/to serial2
//hardware port clocked at 2M-baud
//the floppydrive's READY SIGNAL could trigger
//some functions like auto-read to ADF file
//or any other processing
//for example if the disklabel is read we could start
//the auto-backup process if the ADF is not available
//and startup UAE if the ADF is available which reduces game-loading time drastically
//& that is the functionality needed for the Amiga CAT project
//in a modern fashion using mixed emulated and original hardware
//to keep the momentum going..
//maybe a database comes in handy to keep track
//it the intention is to rescue 100s of disks
//and share the workload/result with the Classic Amiga community
// TODO make the MEGA2560/ESP32 combo emulate a floppy drive on the real Amiga as external drive
// if this is possible we could ditch the gotek ;-)
//------------------------------------------------------------------------------
// ACTIONS
//------------------------------------------------------------------------------
enum consoleaction
{
  show,
  idle,
  reboot,
  checkserial2,
  format,
  do_cmd,
  do_ftp_ados_cmd,
  toggle_debug,
//  create_dir,
  showtime,
  //examine_file,
  info
};
//------------------------------------------------------------------------------
consoleaction action = show;
//------------------------------------------------------------------------------
// LOOP - keep it non-blocking
//------------------------------------------------------------------------------
void loop(void){
//------------------------------------------------------------------------------
// make sure to call handleFTP() frequently!
//------------------------------------------------------------------------------
  ftpSrv.handleFTP();
//------------------------------------------------------------------------------
// the _ADOS_CMD is a way of injecting a command into the main loop
// not very elegant but it works todo: send result back as answer on FTP request 
//------------------------------------------------------------------------------
  if (ftpSrv._ADOS_CMD != "") action=do_ftp_ados_cmd;
//------------------------------------------------------------------------------
// it would show every loop because of it's non-blocking nature so turned it off
// maybe a place for Q&A request
// Serial.printf("Enter 'R' to Reboot, 'L' to list the content of the SD card\n");
///Show Once the ID of this program
//------------------------------------------------------------------------------
  if (action == show){ action = idle; }
//------------------------------------------------------------------------------
  else if (action == reboot){ ESP.restart(); /*use only if there are problems*/}
//------------------------------------------------------------------------------
  else if (action == idle){ serialEvent();}
//------------------------------------------------------------------------------
  else if (action == checkserial2){ serial2Event();}
//------------------------------------------------------------------------------
/*  else if (action == examine_file){
    ADOS_CMD="DECODE_TRACK_FROM_FILE";
    ADOS_ARGS="/raw/TRACK_00_SIDE_0.RAW";
    action=do_cmd;
  }*/
//------------------------------------------------------------------------------
  else if (action == toggle_debug){
    DEBUG= !DEBUG;
    if(DEBUG) Serial.print("DEBUG=ON\n"); else Serial.print("DEBUG=OFF\n");
    action=show;
  }
//------------------------------------------------------------------------------
  else if (action == showtime){
   struct tm tmstruct ;
   tmstruct.tm_year = 0;
   getLocalTime(&tmstruct);
   Serial.printf("Now is : %d-%02d-%02d %02d:%02d:%02d\n",(tmstruct.tm_year)+1900,( tmstruct.tm_mon)+1, tmstruct.tm_mday,tmstruct.tm_hour , tmstruct.tm_min, tmstruct.tm_sec);
    action=show;
  }
//------------------------------------------------------------------------------
/*  else if (action == create_dir){
    char dirName[12];
    sprintf(dirName,"/%d%d%d%2.2d%2.2d", rtc.getYear(),rtc.getMonth()+1,rtc.getDay(),rtc.getHour(),rtc.getMinute() );
    if (createDir(SD_MMC, dirName))
     {Serial.printf("directory [%s] created",dirName);
     }else{
      Serial.printf("error creating new directory [%s]",dirName);
     }
     action = show;
    }*/
  else if (action == info)
  {
//    uint16_t dirCount = ListDir("/");
    Serial.print(PROG_ID+"\n");
    Serial.printf("Memory stats: Free heap: %u\n", ESP.getFreeHeap());
    Serial.print("SD Card info:\n");
    Serial.printf("Size: %lluMB\n", SD_MMC.totalBytes() / (1024 * 1024));
    Serial.printf("Used space: %lluMB\n", SD_MMC.usedBytes() / (1024 * 1024));
    Serial.printf("Available space: %lluMB\n",  (SD_MMC.totalBytes() / (1024 * 1024)) - (SD_MMC.usedBytes() / (1024 * 1024))  );
    action = show;
  }
//------------------------------------------------------------------------------
// process commands received from FTP SERVER CLIENT (AMIGA DOPUS5.92)
//------------------------------------------------------------------------------
  else if (action == do_ftp_ados_cmd){
   //Send CMD via UART2 to MEGA2560 and Read RESULT data
   uint8_t head[3];
   uint8_t data[128];
   int len = 0;
   String msg = "";
   unsigned short msgSize=0;
   //if(DEBUG) Serial.println("[FTP_ADOS]->"+ftpSrv._ADOS_CMD);
   msgSize = ftpSrv._ADOS_CMD.length();
   uint8_t lo = msgSize&0xff;
   uint8_t hi = (msgSize>>8);
   head[0]= '|';//SIGNALS THE MEGA2560 HWSERIAL TO START PROCESS THE FOLLOWING 2 BYTES"
   head[1]= hi;
   head[2]= lo;
   uart_write_bytes(uart_num, (const char*)head , 3);
   delay(10);
   uart_write_bytes(uart_num, ftpSrv._ADOS_CMD.c_str() , msgSize);
   delay(10); //skip->ESP_ERROR_CHECK(uart_get_buffered_data_len(uart_num, (size_t*)&len));
   len = uart_read_bytes (uart_num, data, BUF_SIZE, PACKET_READ_TICS);
   msg=String((const char*)&data[0]).substring(0,len);
   Serial.println(msg);
   ftpSrv._ADOS_RESULT = "";
   ftpSrv._ADOS_CMD = "";
   action = idle;
  }
//------------------------------------------------------------------------------
// process commands results received from HWSERIAL2 (MEGA2560)
// some results are commands themselves so beware
//------------------------------------------------------------------------------
  else if (action == do_cmd)
  {
   if (ADOS_CMD=="ECHO"){
      if (ADOS_ARGS!=""){
       Serial.println(ADOS_ARGS);
      }else{
       Serial.println(ADOS_CMD);
        } 
    action = show;
   }
   else if (ADOS_CMD=="REBOOT"){
    Serial.print("restarting ESP32..\n");
    action = reboot;
   }
//------------------------------------------------------------------------------
// WRITE TRACK - Write 1 track to disk
//------------------------------------------------------------------------------
   else if (ADOS_CMD=="WRITE_TRACK"){
    bool writeMode = true;
    bool verify = false;
    //std::string filename = ADOS_ARGS;
    String filename = ADOS_ARGS;

    //PSL reading works, so lets focus on writing and remember ESP32 has not enough memory to load an ADF-file in one pass
    //so we need buffering while reading from SD and write track by track but first 1 needs to be perfect
      if (writeMode) adf2Disk(filename.c_str(), verify); else
      Serial.print("DISK READ_ONLY!\n");//disk2ADF(filename.c_str());
   action = idle;
   }  
   else if (ADOS_CMD=="ADF2DISK"){ //PSL almost there...
    //
   action = idle;
   }
//------------------------------------------------------------------------------
//THIS WORKS MUCH FASTER AND MORE ACCURATE I HOPE
//------------------------------------------------------------------------------
   else if (ADOS_CMD=="DISK2ADF"){ //PSL almost there...major breakthrough 20220102/00:45
    if (ADOS_ARGS.length()>0){  //optional args
     bool has_sectors=false;
     byte currentTrack = 0;
     byte FLOPPY_SIDE = 1; //ATTENTION CHECK MEGA2560 TO START AT SIDE 1 !!!
     //we start with the upper surface which is kinda reversed thinking and fooled me with unneeded bugs
     String cOK = "1"; String cRETRY = "!"; String cABORT = "#";
     byte retry_count=0;
     bool aborted=false;
     //Serial.printf("TRACK=[%d] SIDE[%d]\n",currentTrack,FLOPPY_SIDE);
//------------------------------------------------------------------------------
     char filName[21];
     sprintf(filName,"/adf/%d%d%d%2.2d%2.2d.ADF", rtc.getYear(),rtc.getMonth()+1,rtc.getDay(),rtc.getHour(),rtc.getMinute() );
     File file = SD_MMC.open(filName, FILE_WRITE);
     if(file){
      DecodedTrack MFM_DecodedTrack;
//------------------------------------------------------------------------------
//    LOOP 0/79 TRACKS
//------------------------------------------------------------------------------
      uint32_t start_time = millis(); uint32_t end_time = start_time; start_time = millis();
      while (currentTrack<80 && aborted==false){
//------------------------------------------------------------------------------
       MFM_DecodedTrack.validSectors.clear();
       for (unsigned int sector=0; sector<NUM_SECTORS_PER_TRACK_DD; sector++)
        MFM_DecodedTrack.invalidSectors[sector].clear();
       //if(DEBUG)Serial.println("Reset sectors list OK");
//------------------------------------------------------------------------------
       const DiskSurface surface = (FLOPPY_SIDE == LOW) ? DiskSurface::dsUpper : DiskSurface::dsLower;
       const unsigned int maxSectorsPerTrack = NUM_SECTORS_PER_TRACK_DD;
//------------------------------------------------------------------------------
//     WHILE LOOP BEGIN - recapture in a loop until all 11 decoded sectors are valid
//     Repeat until we have all 11 sectors
       retry_count=0; //reset it
       has_sectors=false; 
//------------------------------------------------------------------------------
       while (MFM_DecodedTrack.validSectors.size() < maxSectorsPerTrack && aborted==false) {
//------------------------------------------------------------------------------
        long total = 0; int len = 0; int cnt = 0;
//------------------------------------------------------------------------------
        memset(*TMP_RawTrackData,0,sizeof(RawTrackData));
        memset(*MFM_RawTrackData,0,sizeof(RawTrackData)); //IMPORTANT OR CRASH
//------------------------------------------------------------------------------
        //delay(20);
        //if(DEBUG)Serial.printf("CAPTURE TRACK %d SIDE %d\n", currentTrack, FLOPPY_SIDE);
        uart_write_bytes(uart_num, "1" , 1);//start grab_track & read data from UART
//------------------------------------------------------------------------------
//      I guess this is the most adequate uninterrupted way to capture direct to the buffer
//------------------------------------------------------------------------------
        while (total<TMP_RawTrackDataSize && cnt<1){
         len=uart_read_bytes(uart_num, *TMP_RawTrackData+total, TMP_RawTrackDataSize-total, PACKET_READ_TICS+100);
         total +=len;
         if (len>0){cnt++;}
        }
        //if(DEBUG)Serial.printf("CAPTURED BYTES: %d\n", len);
//------------------------------------------------------------------------------
//      UNPACK - RECONSTRUCT MFM DATA for processing
//------------------------------------------------------------------------------
        unpack( *TMP_RawTrackData, *MFM_RawTrackData);
        //if(DEBUG)Serial.println("UNPACK MFM DATA OK");
        //int failureTotal = 0;
        bool ignoreChecksums = false;//TEST
        //if(DEBUG)Serial.println("EXTRACT SECTORS");
        findSectors(*MFM_RawTrackData, currentTrack, surface, AMIGA_WORD_SYNC, MFM_DecodedTrack, ignoreChecksums);
        //CHECK FOR 11 SECTORS (remember with a proper drive & floppy it is normal to capture 11 sectors in one-go!)
        if (MFM_DecodedTrack.validSectors.size()<11){
         if (retry_count<3){
          uart_write_bytes(uart_num, cRETRY.c_str(),1);
          retry_count++;
         }else{
          uart_write_bytes(uart_num, cABORT.c_str(),1);
          aborted=true;
         }
        }
       }
//------------------------------------------------------------------------------
//     WHILE LOOP END all 11 sectors are valid (or aborted)
//     depending on ESP32 memory availability more functions could be implemented
//------------------------------------------------------------------------------
       if (!aborted){
        uart_write_bytes(uart_num, cOK.c_str(),1);
        if (retry_count>0)Serial.printf("Warning: OPERATION COMPLETED WITH %d RETRIES!\n", retry_count);
        has_sectors=(MFM_DecodedTrack.validSectors.size()>0);
//------------------------------------------------------------------------------
//      APPEND TRACK SECTORS to ADF FILE
//------------------------------------------------------------------------------
        if (has_sectors){
         // Sort the sectors in order
         std::sort(MFM_DecodedTrack.validSectors.begin(), MFM_DecodedTrack.validSectors.end(), [](const DecodedSector & a, const DecodedSector & b) -> bool {
          return a.sectorNumber < b.sectorNumber;
         });
         for (unsigned int sector = 0; sector < 11; sector++)
          file.write(MFM_DecodedTrack.validSectors[sector].data, 512);
         //if(DEBUG)Serial.printf("Track:%d Side:%d 11 Sectors (%u bytes) written to ADF-FILE\n", currentTrack, FLOPPY_SIDE, MFM_DecodedTrack.validSectors.size()*512);
         FLOPPY_SIDE=not FLOPPY_SIDE;  
         if (FLOPPY_SIDE==1) currentTrack++; 
        }//else file.write(*MFM_RawTrackData, RAW_TRACKDATA_LENGTH);      
       }else{Serial.print("OPERATION ABORTED\n");}//not aborted
      } //END LOOP TRACKS 0/79
//------------------------------------------------------------------------------
      file.close();
//------------------------------------------------------------------------------
      end_time = millis() - start_time;
      if (has_sectors){
       Serial.printf("ADF file (%u bytes) written for %u ms\r\n", MFM_DecodedTrack.validSectors.size()*512*160, (unsigned int)end_time);
      }else Serial.print("OPERATION FAILED\n");
     }else{Serial.print("Failed to open file for writing\n");}
    }else{ Serial.println("SYNTAX_ERROR: BAD NAME "+ADOS_ARGS);}
    action = show;
   }//DISK2ADF
//------------------------------------------------------------------------------
   else if (ADOS_CMD=="DECODE_TRACK"){ //PSL almost there...major breakthrough 20220102/00:45
//------------------------------------------------------------------------------
    if (ADOS_ARGS.length()==19){  //name matches 19 chars
     String curTrack=String(ADOS_ARGS[6])+String(ADOS_ARGS[7]);
     String curSide=String(ADOS_ARGS[14]);
     byte currentTrack = (byte) curTrack.toInt();
     byte FLOPPY_SIDE = (byte) curSide.toInt();
     String cOK = "1"; String cRETRY = "!"; String cABORT = "#";
     byte retry_count=0;  bool aborted=false;
     Serial.printf("TRACK=[%d] SIDE[%d]\n",currentTrack,FLOPPY_SIDE);
//------------------------------------------------------------------------------
     DecodedTrack MFM_DecodedTrack;
//------------------------------------------------------------------------------
     MFM_DecodedTrack.validSectors.clear();
     for (unsigned int sector=0; sector<NUM_SECTORS_PER_TRACK_DD; sector++)
      MFM_DecodedTrack.invalidSectors[sector].clear();
     Serial.println("Reset sectors list OK");
//------------------------------------------------------------------------------
     const DiskSurface surface = (FLOPPY_SIDE == LOW) ? DiskSurface::dsUpper : DiskSurface::dsLower;
     const unsigned int maxSectorsPerTrack = NUM_SECTORS_PER_TRACK_DD;
//------------------------------------------------------------------------------
//   LOOP BEGIN - recapture in a loop until all 11 decoded sectors are valid
//   Repeat until we have all 11 sectors
//------------------------------------------------------------------------------
     while (MFM_DecodedTrack.validSectors.size() < maxSectorsPerTrack && aborted==false) {
//------------------------------------------------------------------------------
      long total = 0; int len = 0; int cnt = 0;
//------------------------------------------------------------------------------
      memset(*TMP_RawTrackData,0,sizeof(RawTrackData));
      memset(*MFM_RawTrackData,0,sizeof(RawTrackData)); //IMPORTANT OR CRASH
//------------------------------------------------------------------------------
      delay(20);
      Serial.printf("CAPTURE TRACK %d SIDE %d\n", currentTrack, FLOPPY_SIDE);
      uart_write_bytes(uart_num, "1" , 1);//start grab_track & read data from UART
//------------------------------------------------------------------------------
     // I guess this is the most adequate uninterrupted way to capture direct to the buffer
//------------------------------------------------------------------------------
      while (total<TMP_RawTrackDataSize && cnt<1){
       len=uart_read_bytes(uart_num, *TMP_RawTrackData+total, TMP_RawTrackDataSize-total, PACKET_READ_TICS+100);
       total +=len;
       if (len>0){cnt++;}
      }
      Serial.printf("CAPTURED BYTES: %d\n", len);
//------------------------------------------------------------------------------
//    UNPACK - RECONSTRUCT MFM DATA for processing
//------------------------------------------------------------------------------
      unpack( *TMP_RawTrackData, *MFM_RawTrackData);
      Serial.println("UNPACK MFM DATA OK");
      //int failureTotal = 0;
      bool ignoreChecksums = false;//TEST
      Serial.println("EXTRACT SECTORS");
      findSectors(*MFM_RawTrackData, currentTrack, surface, AMIGA_WORD_SYNC, MFM_DecodedTrack, ignoreChecksums);
     //CHECK FOR 11 SECTORS
      if (MFM_DecodedTrack.validSectors.size()<11){
       if (retry_count<3){
        uart_write_bytes(uart_num, cRETRY.c_str(),1);
        retry_count++;
       }else{
        uart_write_bytes(uart_num, cABORT.c_str(),1);
        aborted=true;
       }
      }
     }
     //end while when all 11 sectors are valid (or aborted)
//   LOOP END
     if (!aborted){
      uart_write_bytes(uart_num, cOK.c_str(),1);
      Serial.printf("OPERATION COMPLETED WITH %d RETRIES!\n", retry_count);
      bool has_sectors=(MFM_DecodedTrack.validSectors.size()>0);
      // Sort the sectors in order
      if (has_sectors){
       std::sort(MFM_DecodedTrack.validSectors.begin(), MFM_DecodedTrack.validSectors.end(), [](const DecodedSector & a, const DecodedSector & b) -> bool {
        return a.sectorNumber < b.sectorNumber;
       });
      }
//------------------------------------------------------------------------------
//    SAVE SECTORS to ADF FILE
//------------------------------------------------------------------------------
      String path="/trk/"+ADOS_ARGS;
      uint32_t start_time = millis();
      uint32_t end_time = start_time;
      start_time = millis();
      File file = SD_MMC.open(path.c_str(), FILE_WRITE);
      if(file){
       if (has_sectors){
        for (unsigned int sector = 0; sector < 11; sector++)
        file.write(MFM_DecodedTrack.validSectors[sector].data, 512);
       }else file.write(*MFM_RawTrackData, RAW_TRACKDATA_LENGTH);      
       file.close();
       end_time = millis() - start_time;
       if (has_sectors){
        Serial.printf("11 sectors (%u bytes) written for %u ms\r\n", MFM_DecodedTrack.validSectors.size()*512, (unsigned int)end_time);
       }else Serial.printf("1 incomplete track (%u bytes) written for %u ms\r\n", (unsigned int)RAW_TRACKDATA_LENGTH, (unsigned int)end_time);
      }else{Serial.print("Failed to open file for writing\n");}
     }else{Serial.print("OPERATION ABORTED\n");}//not aborted
    }else{ Serial.println("SYNTAX_ERROR: BAD NAME "+ADOS_ARGS);}
    action = show;
   }
//------------------------------------------------------------------------------
   else if (ADOS_CMD=="READ_TRACK"){
//------------------------------------------------------------------------------
    if (ADOS_ARGS.length()==19){  //name matches 19 chars
     String curTrack=String(ADOS_ARGS[6])+String(ADOS_ARGS[7]);
     String curSide=String(ADOS_ARGS[14]);
     byte currentTrack = (byte) curTrack.toInt();
     byte FLOPPY_SIDE = (byte) curSide.toInt();
     int len = 0; int cnt = 0; long total = 0;
     String path="/trk/"+ADOS_ARGS;
//------------------------------------------------------------------------------
     DecodedTrack MFM_DecodedTrack;
//------------------------------------------------------------------------------
     MFM_DecodedTrack.validSectors.clear();
     for (unsigned int sector=0; sector<NUM_SECTORS_PER_TRACK_DD; sector++)
      MFM_DecodedTrack.invalidSectors[sector].clear();
     Serial.println("Reset sectors list OK");
//------------------------------------------------------------------------------
// a trivial mistake also caused a logic trap so be confused...;-)
     const DiskSurface surface = (FLOPPY_SIDE == LOW) ? DiskSurface::dsUpper : DiskSurface::dsLower;
//------------------------------------------------------------------------------
     memset(*TMP_RawTrackData,0,sizeof(RawTrackData));
     memset(*MFM_RawTrackData,0,sizeof(RawTrackData)); //IMPORTANT OR CRASH
     uart_write_bytes(uart_num, "1" , 1);
     //Read data from UART
     while (total<TMP_RawTrackDataSize && cnt<1){
      len=uart_read_bytes(uart_num, *TMP_RawTrackData+total, TMP_RawTrackDataSize-total, PACKET_READ_TICS+100);
      total +=len;
      if (len>0){cnt++;}
     }
     Serial.printf("CAPTURED BYTES: %d\n", len);
//------------------------------------------------------------------------------
//   UNPACK - RECONSTRUCT MFM DATA for processing
//------------------------------------------------------------------------------
     unpack( *TMP_RawTrackData, *MFM_RawTrackData);
     Serial.println("UNPACK MFM DATA OK");
     bool ignoreChecksums = false;//TEST
     Serial.println("EXTRACT SECTORS");
     findSectors(*MFM_RawTrackData, currentTrack, surface, AMIGA_WORD_SYNC, MFM_DecodedTrack, ignoreChecksums);
//------------------------------------------------------------------------------
     printf("validSectors size=%d\n",MFM_DecodedTrack.validSectors.size());
     bool has_sectors=(MFM_DecodedTrack.validSectors.size()>0);
      // Sort the available sectors in order
     if (has_sectors){
      std::sort(MFM_DecodedTrack.validSectors.begin(), MFM_DecodedTrack.validSectors.end(), [](const DecodedSector & a, const DecodedSector & b) -> bool {
        return a.sectorNumber < b.sectorNumber;
      });
     }
     File file = SD_MMC.open(path.c_str(), FILE_WRITE);
     if(!file){
      Serial.println("Failed to open file for writing");
      action = show;
      return;
     }
     else {
      uint32_t start_time = millis();
      uint32_t end_time = start_time;
      //Serial.printf("Saving %d valid sectors to SD-card with size of %d bytes\r\n", MFM_DecodedTrack.validSectors.capacity(), MFM_DecodedTrack.validSectors.capacity()*512);     
     if (has_sectors){
      for (unsigned int sector = 0; sector < 11; sector++)
       file.write(MFM_DecodedTrack.validSectors[sector].data, 512);
     }else file.write(*MFM_RawTrackData, RAW_TRACKDATA_LENGTH);
      file.close();
      end_time = millis() - start_time;
      if (has_sectors){
       Serial.printf("%u bytes written for %u ms\r\n", MFM_DecodedTrack.validSectors.size()*512, (unsigned int)end_time);
      }else Serial.printf("%u bytes written for %u ms\r\n", (unsigned int)RAW_TRACKDATA_LENGTH, (unsigned int)end_time);
     }
    }else{
     Serial.println("SYNTAX_ERROR: BAD NAME "+ADOS_ARGS);
    }
    action = show;
   }
   else{
    //sprintf( hexstr, "%02X ", (uint8_t)inChar);
    Serial.print("unknown command..[CMD=["+ADOS_CMD+"] ARGS=["+ADOS_ARGS+"]\n");
    action = show;
   }
   ADOS_CMD="";ADOS_ARGS="";
  }
}
//------------------------------------------------------------------------------
// the ESP has no auto serial interrupt routines set (unlike the MEGA/UNO/NANO)
// so we just call 'em directly from the main loop
//------------------------------------------------------------------------------
// SERIALEVENT - watch UART0 (USB|UART0 PINS)
//------------------------------------------------------------------------------
void serialEvent() {
//     Serial.println("CHECK SERIAL USB");
    if (Serial.available())
    {
      char c = Serial.read();
       if (c == '1'){
       ADOS_CMD="ECHO";ADOS_ARGS="MACRO_1";
        action = do_cmd;}
      else if (c == '2'){
       ADOS_CMD="ECHO";ADOS_ARGS="MACRO_2";
        action = do_cmd;}
      else if (c == '3'){
       ADOS_CMD="ECHO";ADOS_ARGS="MACRO_3";
        action = do_cmd;}
      else if (c == 'i')
        action = info;
      else if (c == 'R')
        action = reboot;
      else if (c == 't')
        action = showtime;
      else if (c == 'd')
        action = toggle_debug;
      else if (!(c == '\n' || c == '\r'))
        action = show;
    }
    else {action = checkserial2;}//checkserial2
}
//------------------------------------------------------------------------------
// SERIAL2EVENT - watch UART2 (UART2 PINS)
//------------------------------------------------------------------------------
void serial2Event() {
  inString = ""; //CLEAR IT
  //Read data from UART
  uint8_t data[128];
  int len = 0;//int length = 0;
  ESP_ERROR_CHECK(uart_get_buffered_data_len(uart_num, (size_t*)&len));
  //len = uart_read_bytes (uart_num, data, len, 100);
  //Serial.println("Read data from UART");
  len = uart_read_bytes(uart_num, data, BUF_SIZE , PACKET_READ_TICS);
  //process data
  if (len > 0) {
   for (int i = 0; i < len; i++) {
     // uart_write_bytes(uart_num, "\n", 1);
     if (data[i]!='\n'){
      if (data[i]!='\r'){
       if (data[i]!=' '){
        if (data[i]>31){
         inString += (char) data[i];
        } //debugged using://sprintf( hexstr, "%02X ", (uint8_t)inChar);//inputString += hexstr;
       }
      }
     }
     if (data[i] == ' ') {
      if (ADOS_CMD!=""){
        inString += (char)data[i];
        }else{
      ADOS_CMD = inString;
      inString = "";
      }
     }
     if (data[i] == '\n') {
      if (ADOS_CMD !=""){
        ADOS_ARGS = inString;
       }
       else {
         ADOS_CMD = inString;
        }
      ADOS_CMD.toUpperCase();  
      //stringComplete = true;
      action = do_cmd;
     }
   }
  }
   if (action != do_cmd){action = idle;}
}
//------------------------------------------------------------------------------
// FILESYSTEM FUNCTIONS
//------------------------------------------------------------------------------
bool createDir(fs::FS &fs, const char * path){
    Serial.printf("Creating Dir: %s\n", path);
    if(fs.mkdir(path)){
        //Serial.println("Dir created");
        return true;
    } else {
        return false;
        //Serial.println("mkdir failed");
    }
}
//------------------------------------------------------------------------------
/*uint16_t ListDir(const char *path)
{
  uint16_t dirCount = 0;
//  uint16_t filCount = 0;
  File root = SD_MMC.open(path);
  if (!root)
  {
    Serial.println("failed to open root");
    return 0;
  }
  if (!root.isDirectory())
  {
    Serial.println("/ not a directory");
    return 0;
  }

  File file = root.openNextFile();
  while (file)
  {
    if (file.isDirectory())
    {
     ++dirCount;
      Serial.print("     ");
      Serial.print(file.name());
      Serial.print(" (dir)\r\n");
      dirCount += ListDir(file.name());
    }
    else
    {
      Serial.print(file.name());
      //Serial.printf(" %lluMB\r\n", SD_MMC.usedBytes() / (1024 * 1024));
      Serial.printf(" %d bytes\r\n", file.size());
    }
    file = root.openNextFile();
  }
  return dirCount;
}*/
//------------------------------------------------------------------------------
