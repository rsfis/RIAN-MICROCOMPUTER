[BOM.csv](https://github.com/user-attachments/files/25133573/BOM.csv)<h1 align="center">
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
- 30 keys Keyboard
- 3000mah Rechargeable Battery
- Foldable Case Easy and Light to Carry
- USB C charging port
- Easy to assemble
- Custom Firmware and OS
- Runs LUA for Apps
- Easy to create your own apps in a Inside IDE and share them within a local network

## Case
Designed using Blender, the Case comes with 5 parts: The lower box, upper box, hinges, screws and battery lid. Made using PLA material.
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

## BOM



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

OS and Firmware are still being programmed. Soon, I will have more informations about it.

