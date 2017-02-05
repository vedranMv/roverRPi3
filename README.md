# roverRPi3
Software for a small homemade rover. Used to interface hardware peripherals, provide communication interface over ESP chip and control task scheduling (of either local or remote tasks).

Code Composer Studio project
Target chip: TM4C1294NCPDT

This project is mainly envisioned as a way for me to improve my C/C++ and documentation-keeping skills. It’s supposed to run on a Texas Instruments Tiva C board (ARM Cortex M4) and drive a home-made rover. Lowest layer of software is so called hardware abstraction layer (HAL), which is an attempt to make all the higher-level code easily portable to other platforms. It contains a set of functions through which everything else communicates with the hardware. Remaining software is supposed to follow a design of a very simple OS, at its core having a set of libraries used to directly control the hardware (“kernel modules”) and a simple task scheduler (TS). Job of TS is to handle tasks issued by higher-level modules (usually controlling the behaviour, not the individual peripherals) but also to ease up the process of running remote commands on the rover. Ideally, rover would at some point be equipped with a camera and a Raspberry Pi which would allow it to move around autonomously or manually controlled over some user interface (ergo the need for executing remote commands). So for rover is equipped with an ESP8266 wifi module, MPU9250 IMU module, 2 back motors with optical encoders (front caster wheel) and IR distance sensor mounted on a 2D gimbal.
Choice of C++ was mainly because it allows a more intuitive interface for both programming and understanding the code due to its higher abstraction through objects.

Software modules implemented:
*Task scheduler (TS) – able to register new services for every kernel module, execute them at specific time
*ESP8266 – full implemented (sending/receiving over TCP, TCP client/server, AP connect/disconnect, automatic management of clients and opened sockets, watchdog timer…), integrated with TS
*IR radar – perform scan by rotating around vertical axis, move vertical/horizontal axis, integrated with TS
*HAL library – functions for all implemented modules
