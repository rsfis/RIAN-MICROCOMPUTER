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
<img width="1920" height="1080" alt="showcase" src="https://github.com/user-attachments/assets/b38d05f8-6946-4924-aa4b-648328f9abd5" />
<img width="1920" height="1080" alt="showcase2" src="https://github.com/user-attachments/assets/1353ff8d-b118-446b-86f4-7a9aedf1dd68" />

## PCB
The PCBs were made using KiCad. The board contains the sockets for esp32 and screen and a 3x10 matricial keyboard using push buttons.

<img width="1552" height="635" alt="image" src="https://github.com/user-attachments/assets/bb65db5e-8fe4-406e-8cc4-4279fa65e9ce" />
F.Cu
<img width="1554" height="642" alt="image" src="https://github.com/user-attachments/assets/8e43224d-572f-4551-ab2e-7bbd130f8db6" />
In.1.Cu
<img width="1553" height="648" alt="image" src="https://github.com/user-attachments/assets/84fe1ab5-4585-4b7d-8349-80eeb58be74b" />
In.2.Cu
<img width="1554" height="647" alt="image" src="https://github.com/user-attachments/assets/5122ed8c-1942-4f9d-a764-668effb3a2fb" />
B.Cu
<img width="1553" height="646" alt="image" src="https://github.com/user-attachments/assets/180d0790-9fb7-4b6d-b69d-22ce8d4ecffd" />
3D render front

<img width="1193" height="492" alt="image" src="https://github.com/user-attachments/assets/1f13fa46-ca99-45f8-bacd-faad630ac335" />
3D render back

<img width="1195" height="493" alt="image" src="https://github.com/user-attachments/assets/934cced1-8e1a-4696-a7b5-efd85375df8f" />

Schematic
<img width="1131" height="639" alt="image" src="https://github.com/user-attachments/assets/66fd9c85-617d-40fe-bd33-a3ecb1bf88a1" />

#### External Wiring Diagram
<img width="883" height="662" alt="wiring diagram" src="https://github.com/user-attachments/assets/c300e543-da08-47b7-805b-9ff1040d1792" />

## BOM
|FIELD1|Designator                                  |Quantity|Comment                                                                                                       |Link                                                                                                                                       |ENTIRE PRICE WITH FREIGHT AND TAXES:|FIELD7|FIELD8|FIELD9|
|------|--------------------------------------------|--------|--------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------|------|------|------|
|1     |TFT Screen 3.5" st7796 320*480 ips          |1       |TFT Screen 3.5" st7796 320*480 ips Touch                                                                      |https://pt.aliexpress.com/item/1005005995931721.html?spm=a2g0o.order_list.order_list_main.31.288ecaa4EGynx2&gatewayAdapt=glo2bra           |R$ 90,84                            |      |      |      |
|2     |ESP32 S3 8mb PSRAM 16mb Flash               |1       |ESP32-S3-DevKitC-1                                                                                            |https://pt.aliexpress.com/item/1005007215982839.html?spm=a2g0o.order_list.order_list_main.46.288ecaa4EGynx2&gatewayAdapt=glo2bra           |R$ 63,38                            |      |      |      |
|3     |Aliexpress parts                            |1       |Pin Headers 1x15 and 1x6; Flat Cable Socket Bottom Connector 14pins; Screws M3x6mm Allen; Push Buttons 6x6x5mm|(Cart image at repo: cart/aliexpress.png)                                                                                                  |R$ 129,76                           |      |      |      |
|4     |3D Print                                    |2       |3D Print Case                                                                                                 |~                                                                                                                                          |~                                   |      |      |      |
|5     |PCB                                         |5       |PCB Manufacture Process                                                                                       |~                                                                                                                                          |$36,35                              |      |      |PRICE |
|6     |Battery 3.7v 3000mah                        |1       |Battery 3000mah 3.7v                                                                                          |https://www.mercadolivre.com.br/1-unidade-505573-2-fios-3000mah-37v-5mm-x-55mm-x-73mm/p/MLB2081347995?pdp_filters=item_id:MLB1929447427    |$12,00                              |      |R$    |552,13|
|7     |Battery Charger Module and Step Up Conversor|1       |Battery Charger Module and Step Up 3.7v to 5v Conversor                                                       |https://www.mercadolivre.com.br/carregador-bateria-de-litio-5v-2a-conversor-step-up-usb-c/p/MLB2071876811?pdp_filters=item_id:MLB4418035779|$3,42                               |      |USD   |106,59|

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
<img width="1565" height="662" alt="jlc" src="https://github.com/user-attachments/assets/0c8c7bd0-e917-4651-b4a3-fb7c22d53816" />
<img width="662" height="781" alt="aliexpress" src="https://github.com/user-attachments/assets/f880e897-592a-4915-9808-7078666de3db" />

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
