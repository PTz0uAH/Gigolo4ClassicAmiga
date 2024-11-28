Gigolo4ClassicAmiga: Experimental ESP32 Gigolo_FTPServer with MEGA2560 Amiga Floppy grabbing handler and more..

intro: A couple of years ago I got my hands on an old floppy drive and applied a hw modification
 to use it as replacement Amiga FloppyDisk-drive in case of emergency.. At that time I did not know about
 the "ArduinoFloppyDiskReader" project but that surely would have been a much easier way to work with..
 After I learned about that great project renamed to "DrawBridge" by the author I thought it would be convenient
 to create a similar interface to Mega2560 instead of MiniPro/Nano.. this time using the Amigafied Floppy Drive..
 That was not an easy path but after a lot of experimenting I had a working prototype to build into an A600
 case I came across.. and with a BT 60% KB plus the real floppy drive it started to feel genuine.. just like
 back in the day only with more versatile options.. at that time I wanted to integrate a grabber without
 the need for a PC so then the ESP32 came to the rescue.. with its powerfull 32bit CPU.. so here it is..
 a working proof of concept for anyone to play with.. albeit using quite a bunch of dirty hacks to get things
 done my way..feel free to improve it so we can make it better even on a tight budget using improvised parts..

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

current modified experimental version, prototyping & Gigolo4ClassicAmiga project development by Peter Slootbeek

limitations, highlights and extra features:
- only supports real Amiga Floppy Drive or converted PC Floppy Drive on the MEGA2560 side
- only supports class 10 FAT32 MicroSD (max 32gb)
- duplex support 2MBAUD ESP32 serial port for commands & data from/to MEGA2565 HW-Floppyinterface and CLI
- added support for Directory Opus 5.91 build-in FTPClient (listers and arexx cmds)
- ported core-routines from DrawBridge by Rob Smith (https://github.com/RobSmithDev/ArduinoFloppyDiskReader)
- support for Amiga case std. LEDs
- support for Amiga case custom Potmeter

mandatory parts used to create the 1st working proof-of-concept:
- Amiga floppy-drive or converted PC floppy-drive + powersupply 5v/1A
- ESP32 WROOM devkit
- 5v to 3.3v level-converter
- MEGA2560
- a couple of prototype boards
- a modified SD2MicroSD adapter as cardreader
- various wires
- see the sources to find the connections
- no schematics available yet..

![alt text](./Gigolo4ClassicAmiga.jpg?raw=true)

"luxury" parts used to showcase a functional product prototype:
- An empty Commodore Amiga 600 case
- 1 Raspberry Pi 4 (inside) with monitor and/or Android13 tablet (outside) with suitable Amiga emulator
- and more..

note: the potmeter used is standard, the final one also has a powerswitch so twisting the knob will turn "her" on..

"G4CA" is intended to be used with Classic Amiga flavoured computers/emulators only but might work after some
machine specific alterations with other "oldskool" platforms.. but that is up to you IYKWIM!

to be continued..

with kind regards,

PTz(Peter Slootbeek)uAH

p.s. artwork by "Brother G." & released on Thanksgiving Day 2024 dedicated to the Commodore Amiga usergroup @FB
