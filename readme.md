<h1 align="center">
  Fiscion Razor32
</h1>
<h3 align="center">
  A Mini Foldable OldStyle Computer
</h3>

<h3 align="center">
  Being only 14x6x5cm, Razor32 is a complete computer with Firmware, OS and Apps.
</h3>

## Features
- Powered by ESP32 S3 Chip with 8mb RAM and 16mb Flash
- Embedded TFT Screen
- 30 keys Matricial Keyboard. Each key has X and Y push up output
- 3000mah Rechargeable Battery
- Foldable Case Easy and Light to Carry
- USB C charging port
- Easy to assemble
- Custom Firmware and OS
- Runs LUA for Apps
- Easy to create your own apps in a Inside IDE and share them within a local network

## Case
Designed using Blender, the Case comes with 4 parts: The lower box, upper box, hinges and screws. Made using PLA material.
<img width="1920" height="1080" alt="showcase" src="https://github.com/user-attachments/assets/3cccf4bc-ab7a-4a01-922d-23621e8450e6" />
<img width="1920" height="1080" alt="showcase2" src="https://github.com/user-attachments/assets/1353ff8d-b118-446b-86f4-7a9aedf1dd68" />

## PCB
The PCBs were made using KiCad. One board for general purpose components and another for the keyboard.

These 2 PCBs will fit perfectly on top of each other using pins headers and pin sockets.

### General
<img width="1550" height="640" alt="image" src="https://github.com/user-attachments/assets/11f284b7-2648-4147-8021-5cf85007e2b3" />

#### Top
<img width="1187" height="488" alt="image" src="https://github.com/user-attachments/assets/12147813-6e4d-4f62-a5c5-ef456df88fc2" />
<img width="1123" height="571" alt="image" src="https://github.com/user-attachments/assets/70541a1f-8a9a-4981-b5fe-38dee026ddfa" />

#### Bottom
<img width="981" height="406" alt="image" src="https://github.com/user-attachments/assets/9f5db379-4d50-4417-a9ff-e687828f219a" />

### Keyboard
<img width="1549" height="644" alt="image" src="https://github.com/user-attachments/assets/b7b1ccdd-ffe4-4a1f-959e-4b68503ee66e" />

#### Top
<img width="893" height="366" alt="image" src="https://github.com/user-attachments/assets/1b467d28-7b5b-4ef7-b5ba-74bb3d7093e3" />
<img width="974" height="491" alt="image" src="https://github.com/user-attachments/assets/99998d80-41d5-4293-8c83-24a6404c17f1" />

#### Bottom
<img width="888" height="364" alt="image" src="https://github.com/user-attachments/assets/f4def33c-f917-460e-bfa7-b13f282e4bf4" />
<img width="977" height="496" alt="image" src="https://github.com/user-attachments/assets/ff375853-f109-4d80-bb4a-1d2707ba1d42" />

#### External Wiring Diagram
<img width="883" height="662" alt="wiring diagram" src="https://github.com/user-attachments/assets/c300e543-da08-47b7-805b-9ff1040d1792" />

## BOM
|Id |Designator                                                                                                                              |Quantity|Comment                                          |JLCPCB Part #|Link                                                                                                                                                                |ENTIRE PRICE WITH FREIGHT WITH TAXES:                                          |FIELD8|FIELD9|FIELD10              |
|---|----------------------------------------------------------------------------------------------------------------------------------------|--------|-------------------------------------------------|-------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------|------|------|---------------------|
|1  |TFT Screen 3.5" st7796 320*480 ips                                                                                                      |2       |TFT Screen 3.5" st7796 320*480 ips Touch         |~            |https://pt.aliexpress.com/item/1005005995931721.html?spm=a2g0o.order_list.order_list_main.31.288ecaa4EGynx2&gatewayAdapt=glo2bra                                    |R$ 162,63                                                                      |      |      |                     |
|2  |ESP32 S3 8mb PSRAM 16mb Flash                                                                                                           |2       |ESP32-S3-DevKitC-1                               |~            |https://pt.aliexpress.com/item/1005007215982839.html?spm=a2g0o.order_list.order_list_main.46.288ecaa4EGynx2&gatewayAdapt=glo2bra                                    |R$ 104,46                                                                      |      |      |                     |
|3  |Screw M3 x  6mm                                                                                                                         |1       |m3 preto 304 a�o inoxid�vel allen 6mm 50pcs      |~            |https://pt.aliexpress.com/item/1005005070119421.html?pdp_ext_f=%7B"sku_id"%3A"12000031519353304"%7D&sourceType=1&spm=a2g0o.wish-manage-home.0.0&gatewayAdapt=glo2bra|R$ 52,45                                                                       |      |      |                     |
|4  |ESP32PinSocket_1, ESP32PinSocket_2                                                                                                      |4       |PinSocket_1x21_P2.54mm_Vertical                  |C2883738     |https://jlcpcb.com/partdetail/XKBConnection-X6511FV_21C85D32/C2883738                                                                                               |INCLUDE IN PCBA                                                                |      |      |                     |
|6  |KeyboardSocket_1, KeyboardSocket_2                                                                                                      |4       |PinSocket_1x07_P2.54mm_Vertical                  |C124418      |https://jlcpcb.com/partdetail/125693-B_2200S07PA120/C124418                                                                                                         |INCLUDE IN PCBA                                                                |      |      |PRICE FOR 2 COMPUTERS|
|8  |FlatCable_Header                                                                                                                        |2       |PinHeader_1x14_P1.00mm_Vertical                  |C30607496    |https://jlcpcb.com/partdetail/XUNPU-PH1_0_0114PZD/C30607496                                                                                                         |INCLUDE IN PCBA                                                                |      |R$    |1352,71              |
|9  |FlatCableSocket                                                                                                                         |2       |Hirose_FH12-14S-0.5SH_1x14-1MP_P0.50mm_Horizontal|C5378715     |https://jlcpcb.com/partdetail/HRS_Hirose-FH12_14S_0_5SH_55/C5378715                                                                                                 |INCLUDE IN PCBA                                                                |      |USD   |260,13               |
|10 |a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s,t,u,v,w,x,y,z,ret,fun,enter,space                                                                 |61      |SW_PUSH_6mm                                      |C97440       |https://jlcpcb.com/partdetail/ALPSALPINE-SKHHAJA010/C97440                                                                                                          |INCLUDE IN PCBA                                                                |      |      |PRICE FOR 1 COMPUTER |
|11 |PinHeader_1, PinHeader_2                                                                                                                |1       |PinHeader_1x07_P2.54mm_Vertical 10pcs 15mm height|~            |https://pt.aliexpress.com/item/1005001514058091.html?pdp_ext_f=%7B"sku_id"%3A"12000016418976460"%7D&sourceType=1&spm=a2g0o.wish-manage-home.0.0&gatewayAdapt=glo2bra|R$ 59,28                                                                       |      |R$    |676,35               |
|12 |3D Print                                                                                                                                |2       |3D Print Case                                    |~            |~                                                                                                                                                                   |$18,00                                                                         |      |USD   |130,06               |
|13 |PCB General                                                                                                                             |2       |PCB and PCBA Manufacture Process                 |~            |~                                                                                                                                                                   |$55,16                                                                         |      |      |                     |
|14 |PCB Keyboard                                                                                                                            |2       |PCB and PCBA Manufacture Process                 |~            |~                                                                                                                                                                   |$46,29                                                                         |      |      |                     |
|15 |Pixy Mini 5000mah Power  Bank                                                                                                           |2       |Pixy Mini 5000mah Power  Bank                    |~            |https://futurizta.com/products/smallest-5000mah-usb-c-power-bank                                                                                                    |$53,00                                                                         |      |      |                     |
|   |NOTE: I'm going to make 2 computers, because I want to do a communication system between Computers to send files and messages using WIFI|        |                                                 |             |                                                                                                                                                                    |                                                                               |      |      |                     |
|   |                                                                                                                                        |        |                                                 |             |                                                                                                                                                                    |NOTE: THE PCB PRICES WERE CALCULATED: THE PRICE + FREIGHT DIVIDED BY THE 2 PCBS|      |      |                     |


## Keyboard Keys Organization
q w e r t y u i o p

a s d f g h j k l B

z x c v b S n m f E



1 2 3 4 5 6 7 8 9 0

+ - % < > ? & ! ( )

/ * = " _ ; . , f Sh



ESC F1 F2 F3 F4 F5 F6 F7 F8 F9

{    }  @ :  [  ]  

|    \  ' -  HOME


## OS and APPS
The OS is the RazorOS, made using ArduinoIDE, LovyanGFX and LUA Interpreter.

The OS itself is programmed in C++ using Arduino IDE, with own firmware. The OS has:
- Home Page with a Desktop area
- Control Panel for Rapid Settings
- Users
- Clock
- Settings
- Task Manager
- Notifications
- Watchdog
- Terminal in LUA

The apps are made in LUA. Apps will create a LUA Virtual Machine to execute the code directly in the processor using various C++ External Functions. 

Using the Text Editor, you will also be able to create your own apps to Razor32 and control everything.

Planned Apps:
- Internet Browser Based on Text (Lynx)
- File Sender

- Text Editor
- Calculator
- To-do
- Calendar
- PDF Viewer
- Excel Like

- Tetris
- Snake

- Scanner Wifi
- Terminal Serial
- Packet Logger

OS and Firmware are still being programmed. For now, RazorOS already has basic drawing and OS functions and Lua VM implemented.

## Cart
<img width="1919" height="1079" alt="aliexpress_part3" src="https://github.com/user-attachments/assets/d4868fa4-d680-48d4-adda-8c6005fb2e31" />
<img width="1918" height="1079" alt="aliexpress_part2" src="https://github.com/user-attachments/assets/72f35016-bb23-401c-8030-aee20c3b7041" />
<img width="1916" height="1079" alt="aliexpress_part1" src="https://github.com/user-attachments/assets/8302e466-119e-4487-8e66-fdd2f9b6aee8" />
<img width="970" height="467" alt="jlcpcb_keyboard_components" src="https://github.com/user-attachments/assets/31c5ffbc-fccc-45c4-9f74-5161148dc1f4" />
<img width="969" height="682" alt="jlcpcb_generalpcb_components" src="https://github.com/user-attachments/assets/94716b24-03b8-46f2-a09b-89a303be1b91" />
<img width="1919" height="1079" alt="jlcpcb" src="https://github.com/user-attachments/assets/874aa010-34b3-4348-8627-57cde340be39" />

## Getting Started
### How to flash the ESP32:
- Install ArduinoIDE
- Install the following libraries:
-- LovyanGFX, by lovyan03
-- ArduinoJSON, by BenoitBlanchon
- Install the following Board Managers:
-- esp32, by Espressif Systems
### Extract the files from /firmware/MicroSD Backup.rar
- Copy the files from /firmware/MicroSD Backup.rar to the microsd root directory
- Insert the micro sd in the MicroSD socket below the screen
