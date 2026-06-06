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

![image.png](https://cdn.hackclub.com/019e9a14-9995-791b-8da9-a8cd481a6d7d/image.png)
F.Cu
![image.png](https://cdn.hackclub.com/019e9a14-ef15-776e-ab54-2a114d270246/image.png)
In.1.Cu
![image.png](https://cdn.hackclub.com/019e9a15-2ac7-7f65-a99d-7dbcc88320c5/image.png)
In.2.Cu
![image.png](https://cdn.hackclub.com/019e9a15-5ecc-78b3-8588-41ce45a9a9ad/image.png)
B.Cu
![image.png](https://cdn.hackclub.com/019e9a15-8c56-7ef0-9d29-8406d29d2e8c/image.png)
3D render front

![image.png](https://cdn.hackclub.com/019e9a15-c1cd-702c-8423-ca17017fb9ae/image.png)
3D render back

![image.png](https://cdn.hackclub.com/019e9a16-30f5-767a-8891-511ec2bbf898/image.png)

Schematic
<img width="4961" height="3508" alt="schematic" src="https://github.com/user-attachments/assets/6e44b57b-2fc0-4e3b-9ddc-f8a348e92192" />

#### External Wiring Diagram
<img width="883" height="662" alt="wiring diagram" src="https://github.com/user-attachments/assets/c300e543-da08-47b7-805b-9ff1040d1792" />

## BOM
|2  |Designator                                                             |Quantity|Comment                                          |JLCPCB Part #|Link                                                                                                                                                                |ENTIRE PRICE WITH FREIGHT:                                                     |FIELD8|FIELD9|FIELD10                                   |
|---|-----------------------------------------------------------------------|--------|-------------------------------------------------|-------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------|------|------|------------------------------------------|
|3  |TFT Screen 3.5" st7796 320*480 ips                                     |1       |TFT Screen 3.5" st7796 320*480 ips Touch         |~            |https://pt.aliexpress.com/item/1005005995931721.html?spm=a2g0o.order_list.order_list_main.31.288ecaa4EGynx2&gatewayAdapt=glo2bra                                    |R$ 90,84 (PAID BY MY OWN)                                                      |      |      |                                          |
|4  |ESP32 S3 8mb PSRAM 16mb Flash                                          |1       |ESP32-S3-DevKitC-1                               |~            |https://pt.aliexpress.com/item/1005007215982839.html?spm=a2g0o.order_list.order_list_main.46.288ecaa4EGynx2&gatewayAdapt=glo2bra                                    |R$ 63,38 (PAID BY MY OWN)                                                      |      |      |                                          |
|5  |Screw M3 x  6mm                                                        |1       |m3 preto 304 aço inoxidável allen 6mm 50pcs      |~            |https://pt.aliexpress.com/item/1005005070119421.html?pdp_ext_f=%7B"sku_id"%3A"12000031519353304"%7D&sourceType=1&spm=a2g0o.wish-manage-home.0.0&gatewayAdapt=glo2bra|R$ 52,45                                                                       |      |      |                                          |
|6  |ESP32PinSocket_1, ESP32PinSocket_2                                     |4       |PinSocket_1x21_P2.54mm_Vertical                  |C2883738     |https://jlcpcb.com/partdetail/XKBConnection-X6511FV_21C85D32/C2883738                                                                                               |INCLUDE IN PCBA                                                                |      |      |                                          |
|7  |FlatCable_Header                                                       |2       |PinHeader_1x14_P1.00mm_Vertical                  |C30607496    |https://jlcpcb.com/partdetail/XUNPU-PH1_0_0114PZD/C30607496                                                                                                         |INCLUDE IN PCBA                                                                |      |      |PRICE FOR 1 COMPUTER                      |
|8  |FlatCableSocket                                                        |2       |Hirose_FH12-14S-0.5SH_1x14-1MP_P0.50mm_Horizontal|C5378715     |https://jlcpcb.com/partdetail/HRS_Hirose-FH12_14S_0_5SH_55/C5378715                                                                                                 |INCLUDE IN PCBA                                                                |      |R$    |594,3                                     |
|9  |a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s,t,u,v,w,x,y,z,ret,fun,enter,space|61      |SW_PUSH_6mm                                      |C97440       |https://jlcpcb.com/partdetail/ALPSALPINE-SKHHAJA010/C97440                                                                                                          |INCLUDE IN PCBA                                                                |      |USD   |117,68                                    |
|10 |PinHeader_1, PinHeader_2                                               |1       |PinHeader_1x07_P2.54mm_Vertical 10pcs 15mm height|~            |https://pt.aliexpress.com/item/1005001514058091.html?pdp_ext_f=%7B"sku_id"%3A"12000016418976460"%7D&sourceType=1&spm=a2g0o.wish-manage-home.0.0&gatewayAdapt=glo2bra|R$ 59,28                                                                       |      |      |                                          |
|11 |3D Print                                                               |2       |3D Print Case                                    |~            |~                                                                                                                                                                   |$18,00                                                                         |      |      |                                          |
|12 |PCB                                                                    |2       |PCB and PCBA Manufacture Process                 |~            |~                                                                                                                                                                   |$77,56                                                                         |      |      |                                          |
|13 |Pixy Mini 5000mah Power  Bank                                          |1       |Pixy Mini 5000mah Power  Bank                    |~            |https://futurizta.com/products/smallest-5000mah-usb-c-power-bank                                                                                                    |$25.00 (PAID BY ME)                                                            |      |      |                                          |
|   |                                                                       |        |                                                 |             |                                                                                                                                                                    |                                                                               |      |      |                                          |
|   |                                                                       |        |                                                 |             |                                                                                                                                                                    |                                                                               |      |      |                                          |
|   |                                                                       |        |                                                 |             |                                                                                                                                                                    |                                                                               |      |      |                                          |
|   |                                                                       |        |                                                 |             |                                                                                                                                                                    |NOTE: THE PCB PRICES WERE CALCULATED: THE PRICE + FREIGHT DIVIDED BY THE 2 PCBS|      |      |*PAID BY MY OWN was not added to the price|


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
<img width="1558" height="491" alt="jlc" src="https://github.com/user-attachments/assets/e4d19497-573a-46a8-b4a9-d9d5a8b76855" />
<img width="1919" height="1079" alt="aliexpress_part2" src="https://github.com/user-attachments/assets/b1e4eaaf-577a-48d2-9abe-aabe4ceeaddf" />
<img width="1918" height="1079" alt="aliexpress_part1" src="https://github.com/user-attachments/assets/d718e924-0470-451b-8e50-f4c35a468519" />

## Getting Started
### How to flash the ESP32:
- Install ArduinoIDE
- Install the following libraries:
-- LovyanGFX, by lovyan03
-- ArduinoJSON, by BenoitBlanchon
- Install the following Board Managers:
-- esp32, by Espressif Systems
-- Select the COM and board 4D Systems gen4-ESP32 Modules (ESP32-S3)
### Extract the files from /firmware/MicroSD Backup.rar
- Copy the files from /firmware/MicroSD Backup.rar to the microsd root directory
- Insert the micro sd in the MicroSD socket below the screen

Made for: Hackclub FORGE - https://forge.hackclub.com/projects/457
