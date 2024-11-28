Experimental ESP32 Gigolo_FTPServer with MEGA2565 Amiga Floppy grabbing handler and more..

intro: A couple of years ago I got my hands on an old floppy drive and applied a hw modification
 to use it as replacement Amiga FloppyDisk-drive in case of emergency.. At that time I did not know about
 the AmigaFloppyDriveReader project but that surely would have been a much easier way to work with..
 After I learned about that great project renamed to "DrawBridge" by the author I thought it would be convenient
 to create a similar interface to Mega2560 instead of MiniPro/Nano.. this time using the Amigafied Floppy Drive..
 That was not an easy path but after a lot of experimenting I had a working prototype to build into an A600
 case I came across.. and with a BT 60% KB plus the real floppy drive it started to feel genuine.. just like
 back in the day only with more versatile options.. at that time I wanted to integrate a grabber without
 the need for a PC so then the ESP32 came to the rescue.. with its powerfull 32bit CPU.. so here it is..
 a working proof of concept for anyone to play with.. albeit using quite a bunch of dirty hacks to get things
 done my way..feel free to improve it so we can make it better even when on a tight budget..

summary: Hard & Software project to use your ESP32 as SD-Card Reader/Writer together with MEGA2560 Floppy grabber,
 all controlled from DOpus5.. create custom DOpus button docks and remote control the Server..
 grab disks to adf, download from DOpus.. and more..

todo/ideas: write ADF back to Floppy, mount ADF and access the OFS/FFS filesystem, act as replacement for gotek,
 add MP3/WAV/SVX8/MIDI player capabilities, let external emulator access the drive for direct-reading/writing..
 note: I do not know if it all fits into one package but I can always try..

original: FTP Serveur for Arduino Due and Ethernet shield.
sourcecode by Jean-Michel Gallego (https://github.com/gallegojm/Arduino-Ftp-Server/tree/master/FtpServer)

modified by nailbuster for ESP8266/ESP32. (https://github.com/nailbuster/esp8266FTPServer)

modified by Daniel Plasa (https://github.com/dplasa/FTPClientServer)

final modifications by Peter Slootbeek

limitations, highlights and extra features:
- only supports real Amiga Floppy Drive or converted PC Floppy Drive on the MEGA2560 side
- only supports class 10 FAT32 MicroSD (max 32gb)
- duplex support 2MBAUD ESP32 serial port for commands & data from/to MEGA2565 HW-Floppyinterface and CLI
- added support for Directory Opus 5.91 build-in FTPClient (listers and arexx cmds)
- ported core-routines from DrawBridge by Rob Smith (https://github.com/RobSmithDev/ArduinoFloppyDiskReader)
- support for Amiga case std. LEDs
- support for Amiga case custom Potmeter

Attention: for use with Classic Amiga flavoured computers/emulators only!

to be continued..

with kind regards,

PTz()uAH
