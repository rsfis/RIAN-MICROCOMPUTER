# Fiscion Razor32

Portable Foldable Computer

# 1/03/2026 - The idea

**Total time spent: 1.0h_**

_Time spent: 1.0h_  

Hi! This is the first journal entry of the Rian's MicroComputer.
This is my first hardware project, and it was in these 3 weeks that I learned how to use KiCad and 3D rendering (thats why it has 12h only for pcbs and 22h only for 3D rendering from scratch using pixels, no 3D lib behind it)

I always was addicted to computers, mainly in how they work. Recently I've been addicted to OS's and PCBs. Then, I wanted so much to do something that unified these 2 addictions. So, from 3rd January to 5th January, I had an idea: what if I made my own computer?

I started studying my idea, and tried to create the first image on my head. I wanted it to be like a Nintendo 3DS - portable, foldable, small, but instead of an analog stick, it would have a keyboard.

I knew that I would need a screen and a processor, but which?
For the processor, I thought about an Arduino micro controller, and for the screen, something that communicates with it using GPIO pins: TFT Screens! So I bought one 3.5" screen at Aliexpress. But when I tested the arduino I already had with the screen, it was SO SLOW. Then, I thought: Why not a Raspberry PI? And the answer is: I want to make the Operational System too! And it is so complicated to do with a real micro computer. So, lets simplify things: I found an Arduino, but more powerful, stronger and smaller: ESP32-S3.
With the ESP32-S3 I found, I would have the great amount of 16mb flash, 8mb RAM, and 240mhz (While arduino only had 2mb flash, a few kb of RAM and 16mhz).
I bought the ESP32 and tested it with the screen, and this is what I realised... in the next journal.

![image.png](https://cdn.hackclub.com/019e8a63-f0dd-7ce1-8837-bdf56b8ea6bc/image.png)

References: 
LovyanGFX Graphics Lib Docs I created: [docs](/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6MTI4MTgwLCJwdXIiOiJibG9iX2lkIn19--34296b57b84e30e53a206c20e1219ba0597a7385/docs.txt)
TFT Screen Pins: [pins and instructions](/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6MTI4MTgxLCJwdXIiOiJibG9iX2lkIn19--d9cf887b7b4b8194627fff10a0912c5fef83ded2/pins%20and%20instructions.txt)
TFT Screen datasheet: [3.5inch_SPI_Module_MSP3525_MSP3526_User_Manual_EN](/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6MTI4MjAwLCJwdXIiOiJibG9iX2lkIn19--e6029fc0dd42921700cebb05dfc5681f751da224/3.5inch_SPI_Module_MSP3525_MSP3526_User_Manual_EN.pdf)
ESP32 S3 datasheet: [esp32-s3_datasheet_en](/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6MTI4MTgzLCJwdXIiOiJibG9iX2lkIn19--8f73b24e2fe7c50b8681c89f8a55815ece124108/esp32-s3_datasheet_en.pdf)
KiCad tutorial: https://www.youtube.com/watch?v=mmhareh99P8
Blender tutorial: https://www.youtube.com/watch?v=peSv5IT5Ve4
LovyanGFX tutorial: https://www.youtube.com/watch?v=IPCvQ4o_WP8

# 1/03/2026 - Screen Testing

**Total time spent: 8.0h_**

_Time spent: 8.0h_  

Breaking my patience with drivers and display (5h)
During these days, I tested the 3.5" TFT screen, with some 3D on it, SD card tests and ESP32 memory using LovyanGFX.

The main problem I had with the screen, was that I was trying to follow the manufacturer manual, but it suggested the library TFT_eSPI, and it didn't matter how much I tried, it didn't work. So, I tried some different libraries, like Arduino_ST76.., and the one that finally worked: LovyanGFX!

At the start, LovyanGFX didn't work. The pins were a little off (I connected some pins wrong and also configured the library wrong. But, I made it work! And here are some tests I made:

Simple Tests: 3h
Images covering the entire screen at great 17.1fps. First, created a frame to store every pixel and then blit the entire frame directly to the screen to increase performance. Then, created a struct called Sprite that creates and loads a sprite. Its Draw function draws the sprite (With transparency) to the frame. It loads .png from the SD Card.
![20260124_194019](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6OTA0MjYsInB1ciI6ImJsb2JfaWQifX0=--a37fdfa36a7d4dab177f2287f12ff80d73dad620/20260124_194019.jpg)
3D cubes rendering using the same principle of the frame, but drawing and calculating each pixel separately. Calculates the camera perspective and each cube's face and vertices position and draw each pixel.
![20260124_195411](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6OTA0MjQsInB1ciI6ImJsb2JfaWQifX0=--ab4d0df64d6d80756e12fca480fa017a22d12070/20260124_195411.jpg)
Pre made test for LovyanGFX of its drawing functions
![20260124_193836](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6OTA0MjUsInB1ciI6ImJsb2JfaWQifX0=--97f63dd6c2c810071073146a804c88524ef17844/20260124_193836.jpg)![20260124_195241](https://raw.githubusercontent.com/rsfis/Razor32/main//user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6OTA0MjMsInB1ciI6ImJsb2JfaWQifX0=--107dadf43ba48247facb8de4e10d6ba769181fbb/20260124_195241.jpg)

Complex 3D models rendering CPU bound from scratch (22h disconsidered from counting!)
Here I tested the screen making from scratch a 3D models renderer using only the CPU and functions from LovyanGFX.
To do that, I thought I could work with the .glb. But I realised that the ESP32 wouldn't be able to read it because it would be so slow to read and interpret the glb data, and it would be a binary file (more difficult to make the interpreter), so I created a script that converts a .glb model to a cpp header with the texture and vertices. After that, I created a program in ArduinoIDE (the program I'll be using to make everything) that imports some models in RAW .h (Lists of meshes and textures with int and float values) and calculates the perspective, rotation, position, scale and the colour of each pixel in the screen. But it turned out that the colours were messed up and inverted:
![IMG-20251025-WA0013](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6MTI3OTc0LCJwdXIiOiJibG9iX2lkIn19--f193c59cf5f3eeb16e3c53aed920c6b55dee7e6d/IMG-20251025-WA0013.jpg)
Even if I inverted the LovyanGFX colours setting, it wouldn't work. After, I realised that my colour converter from the normal colours to RGB565 colours was wrong.
When I changed the orders: Perfect pixels and render.
![Screenshot_20260304_143709_WhatsApp](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6MTE1MTc5LCJwdXIiOiJibG9iX2lkIn19--0de2718384356b252dbc36896d52fab86999a1b0/Screenshot_20260304_143709_WhatsApp.jpg)
The code is avaliable on github at tests/3d_models/3d_models.ino

# 1/19/2026 - Keyboard PCB

**Total time spent: 8.0h_**

_Time spent: 8.0h_  

This week I turned everything I had tested on the protoboard into actual PCBs. I used KiCad, which I had never used before, so it took a while to figure out. I made two separate boards.


Keyboard PCB (8h | Jan 19 to Jan 22)
The biggest challenge here was that the ESP32 doesn't have that many GPIO pins, so I couldn't just dedicate one pin per key. I organized the keys in a matrix — the same idea used in screens, where every pixel has an X and Y position. Each key has its own column and row, and when both are activated at the same time, the firmware knows which key was pressed. This way I use way fewer pins.
I placed the components based on my protoboard layout and routed the tracks manually. I actually drew the schematic after finishing the PCB, copying from the tracks I had already placed — not the usual order, but it worked. This board connects to the main PCB using pin headers and sockets.

![Captura de tela 2026-01-24 215701](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6OTA0NzMsInB1ciI6ImJsb2JfaWQifX0=--4cadd019df3daf6a296604d1bb966342517b0124/Captura%20de%20tela%202026-01-24%20215701.png)
![Captura de tela 2026-01-24 215708](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6OTA0NzIsInB1ciI6ImJsb2JfaWQifX0=--ee4ffd9532ae39152e276f660a2b885854b54427/Captura%20de%20tela%202026-01-24%20215708.png)

# 1/23/2026 - Main PCB

**Total time spent: 4.0h_**

_Time spent: 4.0h_

Main PCB (4h | Jan 22 to Jan 24)
This one connects everything together: the ESP32, the screen (via a flat cable socket), and the keyboard PCB. I based the connections on what I had already tested, which made it a bit faster. The main difficulty was figuring out the placement so everything fits inside the case.

I made two separate PCBs instead of one because fitting everything on a single board would be too tight. Having them separate also makes the project more reliable — if one board gets damaged, I can just replace that one without losing the other.

Power issue
I still haven't figured out how to power the device inside a small case. AA rechargeable batteries at 3.7V are already above what the ESP32 can take directly, and the capacity would probably only last a few minutes anyway. I'm looking into either a step-down module that converts 7.6V to 3.3V, or a battery case with a USB-C output that handles the voltage for me.

![Captura de tela 2026-01-24 215628](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6OTA0NzQsInB1ciI6ImJsb2JfaWQifX0=--83743c123c276c650e272b6dc75b6033b5ef782d/Captura%20de%20tela%202026-01-24%20215628.png)
![Captura de tela 2026-01-24 215636](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6OTA0NzUsInB1ciI6ImJsb2JfaWQifX0=--9a6e509a2bff8d439121e55667efe845e7c6a250/Captura%20de%20tela%202026-01-24%20215636.png)


# 2026-05-28: 2026-05-18: 1/25/2026 - Design

**Total time spent: 4.0h_**

**Total time spent: 4.0h_**

_Time spent: 4.0h_  

Today I've made the computer design  using Blender. Since this was my first time working on Blender, it was pretty hard to get the shortcuts and become fast on it.
The most difficult part was making the upper lid, because it needs to have some space on the hinge to turn. The 3D printer makes everything more fat (3.8h)

Also discovered how I will power it up, charge and power down.

So, for powering I will use a 18650 USB, which transforms an AA battery into a Power Bank. The output is 5V  2A, which seems ok to the setup. I decided to do so, because I can connect an USB-C cable from the output directly to the ESP32, and in my tests, 5V directly from a battery didn't power up everything and the screen backlight was so weak.

To charging up, I will put a USB-C male connected to the input of the battery and connect it to a converter USB-C male to USB-A female. So, there will be a USB-A port ready to plug in and charge everything.

The system will never turn down completely, because this battery holder I ordered doesn't have a switch and I wouldn't be able to cut the powering of the usb-c cable (at least for now).

So, there will be a button in the system to turn off, which will activate the esp32 deep hybernation mode and turn off the screen. Then, when clicking any key on the keyboard's row 5 (tgv row), it will restart and exit hybernation. The only way to turn it completely down, would be letting it run out od battery.
![20260125_224601](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6OTE0ODQsInB1ciI6ImJsb2JfaWQifX0=--3ee1f17a1ca277fa3d08d09cab695fff2dc70835/20260125_224601.jpg)
![20260125_224615](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6OTE0ODUsInB1ciI6ImJsb2JfaWQifX0=--0c92161010bdf78996041a66f735ab7a132e7073/20260125_224615.jpg)
![20260125_224622](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6OTE0ODYsInB1ciI6ImJsb2JfaWQifX0=--f7c00ac296d268843dbff484d2b339fd82ba300d/20260125_224622.jpg)
![20260125_224638](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6OTE0ODcsInB1ciI6ImJsb2JfaWQifX0=--3f39b146c5413a85d09a21fd6e5c458943268ba6/20260125_224638.jpg)

I also made some modifications on the PCB (Cut a hole on the PCB to pass the USB C to charge the ESP32, removed the battery support and added the flat cable socket for screen and reconected tracks to the socket) (0.3h).
![image](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6MTE1MTI1LCJwdXIiOiJibG9iX2lkIn19--866f290b92e460524438a53ec64dbde56d7bb9ac/image.png)
![image](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6MTE1MTI2LCJwdXIiOiJibG9iX2lkIn19--a9698914f51cd54f19db0e5f263cfd2db0b42320/image.png)

# 2026-05-28: 2026-05-18: 1/26/2026 - Design Edited and More Powering Tests

**Total time spent: 2.5h**

**Total time spent: 1.0h**

Time spent: 1.0h  

Today I've made some changes on the design. Added a USB port, added the esp32 on the board, modified the hinges to be more tight to the hinge "connector).

I also tested the light sleep and tft sleep functions, reset using software and googled about power consumption. Everything now is schematized.

I made a chart about the powering:
![imagem_2026-01-26_164341373](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6OTIyMDAsInB1ciI6ImJsb2JfaWQifX0=--f1d5ed56aafea86e646d89e5facfebdd9489e9de/imagem_2026-01-26_164341373.png)
Powering looks decent and so the battery duration.

# 2026-05-28: 2026-05-18: 1/30/2026 - Tracked Prices

**Total time spent: 2.0h**

**Total time spent: 2.0h**

Time spent: 2.0h

During these 2 days of work, I've made some work on the PCB -fixed the tracks that were not straight[tortas] (idk how to say it in english) and revised all the tracks- and routed where I should manufacture them.

I started calling a lot of local factories, but they were so expensive.

So, I will manufacture everything with JLCPCB. I've added the general board to my cart, but 1 component for the keyboard were in inventory shortage, so I pre-ordered by 5ish dollars the Pin Header I need.

The problem when quoting on JLCPCB was that my BOM was really messed up, and I had to redo it using the JLCPCB pattern. This took me a lot of time.
And the other problem was that I had to search for the components, and it took so much time. The time consuming one were the pin headers and pin sockets. Some were not avaliable, so I had to make a pre-order.

Everything will cost approximately 700 reais (5 general boards and 5 keyboards. 2 assembled).

Plus the 3D print for the case will cost ~18 dollars (Two PCs) (JayG from HackClub Printing Stuff).

The screen: ~140 reais (paid by me)
The ESP32: ~90 reais (paid by me)

![imagem_2026-01-30_211424497](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6OTU0NTEsInB1ciI6ImJsb2JfaWQifX0=--8c480420b0e525c17d8782693c50ae5fa6362665/imagem_2026-01-30_211424497.png)

# 2026-05-28: 2026-05-18: 1/31/2026 - First Firmware steps and OS

**Total time spent: 6.0h**

**Total time spent: 6.0h**

Time spent: 6.0h

Today, I've researched some ways to run the Firmware, multitasking and apps.

So, the problem was about the apps: How the system would run multiple apps with open software and that wouldn't depend on just the base firmware and save states, I mean, if I put every app inside the firmware (.ino), the apps wouldn't be updateable easily. Everything would be linked in a way that: If a software breaks, all the system breaks, and I would have to compile the ENTIRE SKETCH again and again.

To solve it, I decided to run a Lua interpreter within Esp32.

This will allow me to create: The bios, firmware, OS and apps runner separately from the apps themselves.

The firmware and OS will be created apart in a .ino sketch, and the apps will have their own folder inside the SD Card and their own folder structure. Generally containing fonts, images, config json, .lua scripting files.

The OS job will only be read the lua script and interpret it.

There will be a ton of external functions which could be used to create the apps, such as: File functions, Drawing, Math, OS management (Configs, restart, shutdown, close app, etc). The definitions and instructions controlled and set by the C++ sketch (Firmware).

This is the base of the firmware. So far it has only the screen and sd init, interpreter starter and one external function: endProgram() to end the VM.

![imagem_2026-01-31_182833809](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6OTU4NjksInB1ciI6ImJsb2JfaWQifX0=--957d23f0206282a48f3ed2b585673e3b99ef78ae/imagem_2026-01-31_182833809.png)

# 2026-05-28: 2026-05-18: 2/3/2026 - Case Fixes and More PCB Quotings

**Total time spent: 2.0h**

**Total time spent: 2.0h**

Time spent: 2.0h

During these days, I've added screw holes to my case, and discovered that the JLCPCB don't have the part I need (Pinheader Pitch 2.54mm 8mm height), so I'm going to buy this part from AliExpress, and solder it by myself with a borrowed Solding Station from the theater lights guy.

I also converted the .blend into a .step file. It had a some of errors and convertion isn't precise, but ok. It is 4gb...

In addition, to the 3D prints, I found a guy that prints inside Blueprint (I forgot the program name). And he just ask for the shipping price and fillament.

![imagem_2026-02-03_190743825](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6OTc4MDQsInB1ciI6ImJsb2JfaWQifX0=--3cc36f9502937673fe28ba041904c46183192d18/imagem_2026-02-03_190743825.png)

![imagem_2026-02-03_190820493](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6OTc4MDUsInB1ciI6ImJsb2JfaWQifX0=--5e55f74c9a0d1fba02d5b3f738c5a6e715640430/imagem_2026-02-03_190820493.png)

# 2026-05-28: 2026-05-18: 2/6/2026 - README, Github, Name and Notes for the reviewer

**Total time spent: 1.0h**

**Total time spent: 1.0h**

Time spent: 1.0h

During these 2 days, I've been writing the README and configuring the Github Repo.

The README is already avaliable on Github.

I also changed the project's name, to be more, shocking...

Added some notes for the reviewer about why 2 computers and the lack of a correct .step (Current is 4gb).

![image](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6OTkzMDksInB1ciI6ImJsb2JfaWQifX0=--4e33bb4490a814440b3904b9e3803141445c06ff/image.png)

Furthermore, I made the wiring diagram for parts out of the pcb.
![wiring diagram](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6OTkzMTMsInB1ciI6ImJsb2JfaWQifX0=--f7bced60d88d1517b670a2406941892d914df3e7/wiring%20diagram.png)

# 2026-05-28: 2026-05-18: 2/7/2026 - OS Design

**Total time spent: 0.5h**

**Total time spent: 1.0h**

Time spent: 0.5h  

In an hour, I made the OS design and pages in paint.
I want the design to be a little Linux Like, kinda cozy, and easy to use. It will work with an arrow system. With WASD you control where your "cursor" is, like game consoles.

Starting:
![image](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6OTk2MjksInB1ciI6ImJsb2JfaWQifX0=--7888eb3ec57eb279e7b22be18cbe743046925937/image.png)

Login:
![image](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6OTk2MzAsInB1ciI6ImJsb2JfaWQifX0=--756ee2fe644b34610bcb12731faa80a48224071c/image.png)

Home Page:
![image](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6OTk2MjgsInB1ciI6ImJsb2JfaWQifX0=--49b55430845738d0b215d3726487313dc5c966ac/image.png)

# 2026-05-28: 2026-05-18: 2/10/2026 - Powering Issues

**Total time spent: 0.3h**

**Total time spent: 0.3h**

Time spent: 0.3h

Today I've finally recieved my stickers, and they are so cute!

Loved the processor one!!

![20260210_185206](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6MTAxODk4LCJwdXIiOiJibG9iX2lkIn19--748e92104835db2ac6bed7cd86872b72a087455a/20260210_185206.jpg)

![20260210_185324](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6MTAxODk5LCJwdXIiOiJibG9iX2lkIn19--306cd2bc400b15e44527433316792f43cfddd4b1/20260210_185324.jpg)

To frustrate a little bit the day, though, the battery holders didn't work. It just doesn't charge and neither transfers the electricity to the esp32.
To solve that, I will use a power bank that must have 2 USBS (one for charging, and one for output) and a powering on/off button.
I will use a power bank module to prioritize safety, reduce development risk, and keep the project scope focused on PCB design, firmware, and system integration. The power bank provides built-in protection and regulated output.

# 2026-05-28: 2026-05-18: 2/12/2026 - Powering

**Total time spent: 0.3h**

**Total time spent: 0.3h**

Time spent: 0.3h

Today I finally found the perfect powering system to this project: Futurizta Power Bank 5000mah Mini.

This power bank has 5000mah (Battery would last so longer than if I've bought a AAA battery), so it would last 27h. And with the WIFI on: 14.5h.

In addition, it is so tiny being only 7.5x3.5x2cm, fitting perfectly inside the computer.

This project is gonna be really fat. The lower part will be 4cm tall.

![Screenshot_20260212_091511_Opera GX](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6MTAyODc3LCJwdXIiOiJibG9iX2lkIn19--759f881e9137a489ce3d2ade971acd37526cf9fd/Screenshot_20260212_091511_Opera%20GX.jpg)

So, the powering chart is like this now:
![image](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6MTI1MTExLCJwdXIiOiJibG9iX2lkIn19--6597fe36b5fe80526b3fae7c83df5907cb1d0270/image.png)

# 2026-05-28: 2026-05-18: 2/27/2026 - System's Kernel and Tests

**Total time spent: 12.0h**

**Total time spent: 12.0h**

Time spent: 12.0h

During 4 days (From 4 days ago to today), I've been developing the OS Kernel.
I've spent great 12 hours developing the kernel main functions and folder structure.

The kernel now starts screen, starts sd module, loads boot configs, creates screen frame, loads general configs, synchronize time using wifi to obtain real life time and set it to the RTC and then turns off wifi to save energy, loads user, starts first lua vm (OS Graphic Interface).

OS functions:

Display_ClearScreen, Display_UpdateScreen, OS_GetFreeMemory, OS_GetMemorySize, OS_GetFreeHeapMemory, OS_GetHeapMemorySize, OS_GetCPUTemperature, OS_GetTime:toString("%d/%m/%Y - %H:%M:%S"), OS_GetLuaVMHeapMemoryUsage, OS_GetUserUsername, OS_GetUserPassword

Furthermore, I've made the Sprite exported structure and exported lua variable type and so with Fonts.
This is a simple example lua code of how the system is working:
```
local user_login_bg = Sprite("/sys/img/user_login_bg.png", 480, 320)
local gothambold_font = Font("/sys/fonts/Gotham Bold.bin")

-- FPS
local fps = 0
local frameCount = 0
local lastTime = os.clock()

function loop()
   while true do
      local currentTime = os.clock()
      frameCount = frameCount + 1

      -- Atualiza FPS a cada 1 segundo
      if currentTime - lastTime >= 1 then
         collectgarbage("collect")
         fps = frameCount
         frameCount = 0
         lastTime = currentTime
         --print("FPS:", fps)
         --print("Temperature: ", OS_GetCPUTemperature())
         --print("Free Memory: ", OS_GetFreeHeapMemory())
         --print("Memory Size: ", OS_GetMemorySize())
         --print("RTC: ", OS_GetTime():toString("%d/%m/%Y - %H:%M:%S"))
         --print("Lua Heap Memory Usage: ", OS_GetLuaVMHeapMemoryUsage())
      end

      Display_ClearScreen()

      user_login_bg:draw(0, 0)
      gothambold_font:drawString(OS_GetUserUsername(), 148, 150, 20, 255, 255, 255)

      Display_UpdateScreen()
   end
end

function start()
   loop()
end

start()
endProgram()

```

Images are always in .png, supported transparency, and fonts are always in .bin.
The fonts are converted from ttf to binary files using a python script avaliable on github.

The most difficult part was the Lua VM memory management: In the beginning, there was a big memory leak in LUA VMs. The solution was to call every second collectgarbage("collect").
And also: the luaHeapUsed variable was being miscounted, causing a memory leak.

Here is the OS booting and start page:
![WhatsApp Image 2026-02-27 at 21.43.36(1)](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6MTEyNTcyLCJwdXIiOiJibG9iX2lkIn19--fad7c37d7f5d2ae81a223833f3933576445f9c32/WhatsApp%20Image%202026-02-27%20at%2021.43.36(1).jpeg)
![WhatsApp Image 2026-02-27 at 21.43.36](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6MTEyNTczLCJwdXIiOiJibG9iX2lkIn19--fc565008f973b24c9f828267c535f0b0d1358455/WhatsApp%20Image%202026-02-27%20at%2021.43.36.jpeg)

# 2026-05-28: 2026-05-18: 5/18/2026 - Transference from Blueprint to Forge program

**Total time spent: 0.1 hours**

**Total time spent: 0.1 hours**

Time spent: 0.1h  

After 2 months of waiting, I decided to transfer Blueprint to Forge.

Now, Im going to wait anxiously to the final review.

hahahahahahahahahahahahahahaha

![image](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6MTYwNjU1LCJwdXIiOiJibG9iX2lkIn19--524e89be46406af7e8b07ccf29cad5bd0bef55a9/image.png)

