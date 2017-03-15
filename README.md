# Rover

In this repository I’ll share source code that I wrote to run a small rover. I started this rover as a side-project in my free time about a year after starting my studies as a mechatronics student. In the beginning it was meant to be a simple and universal hardware platform that I could use to test software ideas on and improve my coding skills but also get to know real-life challenges that my appear in process of designing & building full hardware-software solutions; and how to get both parts to play nicely together :) . But more than anything it was meant to be educational project for myself, to try and write good and reusable code.
Platform itself is built around a rather unconventional chip, TM4C1294, a versatile and powerful Cortex-M4 microcontroller from Texas Instrument with a lot of peripherals and GPIOs. This complicated the process of adding new sensors and peripherals as there were not too many example codes available for this board, but on the other side, TI’s forum, documentation and SDK are great. 

## Current state

### Hardware

In the last update of rover platform, new parts were mostly designed in CAD and 3D printed to fit the existing platform. There were several major changes:
1) Cooler wheels which are a)wider than its predecessor to provide more space on the inside of the wheel to place IR sender for optical encoders; b) bigger in diameter (7cm) to achieve greater speeds on the slow 50RPM motors; c) have slots on the outside for adding rubber rings to improve traction.
2) Plastic, 3D printed encoder disks with 40 slits per rotation. They are more coarse, providing poorer resolution than previous ones, but are more robust to problems with light that previous ones had.
3) 2D gimbal for radar. Instead of 1DOF that previous radar had, new gimbal allows IR sensor to move in both horizontal and vertical plane. This is not so important for IR sensor but I plan to attach a camera on this rover in future so this was a nice-to-have feature.
4) Addition of ESP8266 WiFi module with its own mount
5) Mount for Raspberry Pi; As I wrote before, I’d like to mount the camera on the rover in the future and do some vision-based navigation or mapping of surrounding for which having an on-board computer like RPi would be pretty good
![alt tag](https://hsr.duckdns.org/images/new.jpg)

As of last version, hardware platform includes 2 motorized back wheels equipped with 40 ppr (points-per-rotation) relative optical oncoders, front caster wheel. IR distance sensor on a 2D gimbal mounted on the front of the rover, ESP8266 wifi module and MPU9250 9DOF motion sensor.


### Software

Upon completing the autonomous driving project (description down) the amount of code stuffed into a single source file made working on this project a nightmare. Single file contained everything from low-level functions to control the motors, ISRs for motors, function for configuring peripherals, reading sensors but also all high-level codes that implemented logic for autonomous driving. And all that with very little comments and explanations, so continuing the project after a long break was a process in and of itself. At this point I wanted to turn this into a more serious project where I’d like to split code into layers, try to practice documenting the code and split everything into smaller chunks depending on each other. Ideally, I’d like to write universal set of libraries for the rover platform and all of its sensors that I could fetch and simply build into any new project. That’s how roverKernel, part of this project came to existence. It’s an attempt to create modular code, which should (ideally) be platform independent, both in terms of hardware (where rover physical specifications are easily changed to accommodate new versions on rover) and software, where a code could be run on different microcontroller just by changing hardware abstraction layer in roverKernel/HAL folder. Also, peripheral libraries are thought of as C++ singletons, instead of C-style structs to allow for member functions and in general to make the code more readable. 
Task scheduler part of the project is an attempt at creating a mechanism that can contain a queue of tasks to be executed:
a) at a particular point in time (e.g. move forward for 5cm at T+3s, do radar scan at T+10s etc.) 
b) periodically throughout the full time that the rover is operation (e.g. read orientation from IMU every 1s, or send telemetry data every 5s etc). In such case scheduler itself takes care of rescheduling the task once its completed
c) periodically; but only several time (e.g move forward for 5cm, 3 times, with a period of 2s, starting at T+8s)
Another important functionality implemented through task scheduler is a unified interface to functionalities provided by all modules of the kernel. Once tasks scheduler is globally enabled by defining the _USE_TASK_SCHEDULER_ each module registers its services to the task scheduler by calling the TS_RegCallback() function. Through this call, module specifies callback function to be invoked every time someone needs its services, and a memory location to which the arguments for callback are written. Inside the callback function, one the arguments from argument list specifies which functionality/service is being requested, while the rest are usually arguments for service call.

Both of above-described features of task scheduler are not useful so much for internal or low-level processes on the microcontroller itself, but are incredibly nice for trying to execute remote commands on the MCU. Due the universal interface for requesting any functionality (no matter the module), remote functions doesn’t require abstract command parser to link command to its function, but the module and its service are merely an arguments to a function.


Software modules implemented so far:

*Task scheduler (TS) - able to register new services for every kernel module, execute them at specific time

*ESP8266 - full implemented (sending/receiving over TCP, TCP client/server, AP connect/disconnect, automatic management of clients and opened sockets, watchdog timer…), integrated with TS

*IR radar - perform scan by rotating around vertical axis, move vertical/horizontal axis, integrated with TS

*Engines – ability to move in one direction be specified distance/angle, drive following arc , or move by specifying % of max speed for each motor (negative % drives the wheel in reverse), integrated with TS

*MPU9250 – in progress (working using integrated digital motion processor, DMP)

*Hardware abstraction layer for TM4C1294

## History

First design iteration of platform was assembled from whatever scrap material was available at the time. It was steered around by two motorized back wheels, and one front caster wheel. In front, the vehicle had and IR distance sensor (Sharp GP2Y0A21YK) rotated on a servo motor to provide a radar-like view in a horizontal plane in front of it to detect any possible obstacles Each motorized wheel had an improvised “mechanical position encoder”, which essentially consisted of two wires sliding on the side of the plastic wheel. On that side, wheel had 6 metal pieces evenly distributed over the circumference so when the sliding wires would come in contact with the metal pieces it would close the circuit producing a detectable signal. Total resolution was accordingly 6 points per revolution, or one signal every 60°. Motors on each wheel were controlled through H-bridge from where it was possible to change the direction of rotation and speed (PWM signal fed as a power-input to H-bridge).
![alt tag](https://hsr.duckdns.org/images/enc.png)

### Wi-Fi controlled rover

First project to utilize this platform was a WiFi - controlled rover which featured TI’s CC3000 chip to bring Internet to the platform. The rover was then communicating with an online server through GET requests and  delivering/receiving data through PHP scripts on the server. Scripts were also rover’s interface to a MySQL database, a permanent storage of data and settings. On the other side, controls were issued through GUI, sent directly to online database, from where they could be sent to the rover in first successive connection. GUI was implemented in VB.Net and allowed to control the movement of the vehicle (distance and direction to move), request a radar scan (and plot result of that scan) as well as move a 3 DOF robot arm that was added as a part of this project. Video of this project can be seen here: https://hsr.duckdns.org/videos/MOV_0495.mp4

Conclusion

The project provided a fun intro to what this platform could potentially become and pointed out some problems in the design. Firstly, for any applications that include Internet communication, having a free online account for hosting webpage/database is not convenient. Firstly because the speed is very slow which could partially be seen in video above, where each command sent from GUI took few seconds to propagate to rover. Secondly, most free web hostings will not be happy with hosting PHP scripts which are used for this purpose and will delete your account in short time saying that’s not the service they provide. Hardwarewise, “mechanical position encoders” were a mess to work with, due to the way they were implemented they would trigger multiple times on a single pass and required different software workarounds to function properly. Their main use here was to move the car for the distance specified in GUI application (or as close to it as possible).

### Self-balancing platform

Second project that made use of this platform was an attempt of making a self-balancing 2-wheeled robot. The platform from above was essentially placed in vertical position from where it tried to keep the balance on two motorized back wheels. Angular position of the platform was calculated from raw accelerometer and gyroscope data provided by MPU9250 module (at the time I didn’t know it supports DMP feature) and passed through the complementary filter to get roll/pitch/yaw angles. And a simple PI controller was used to minimize the error from the starting position.
Video: https://hsr.duckdns.org/videos/MOV_0584.mp4

Conclusion

This wasn’t a particularly successful project and there were many hardware & software problems along the way. Firstly, the wight distribution wasn’t optimal and in order to maintain equilibrium platform had to be at an angle instead of straight up. Secondly, motors weren’t responsive enough and had troubles reacting to controller’s signal in a decent time, and in general were very slow (max 60 RPM and 7.4V).

### Autonomous driving
Goal of this project was to utilize the IR distance sensor mounted on the servo motor in order to provide information about potential obstacles in front of the rover and and guide it around them. The test setup consisted of a small maze the robot needed to go through, relying only on the sensory feedback about distance, and then using it to calculate where to move and for how much. At this point, the mechanical encoders were not suitable any more, instead they were replaced by homemade optical encoders, where a slit pattern with 90 slits (points per revolution/ppr) was printed on a transparent foil using laser printer and glued to the back of a wheel. Small IR sender-receiver was then fitted partly inside the wheel (IR sender) and partly outside of it (IR receiver), having a printed disk moving through the space in the middle interrupting beam of light. This allowed for much better navigation bringing the minimum step the car can move down from around 5.2 cm (1 encoder point on 6ppr disk, 5cm wheel dia) to 0.35 mm (1 encoder point on 90ppr disk, 5cm wheel dia).
Video: https://hsr.duckdns.org/videos/MOV_0537.mp4
