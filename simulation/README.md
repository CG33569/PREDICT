# PREDICT Drone in CoppeliaSim #

This a test-application for connecting the DE10-Nano FPGA to the CoppeliaSim simulator and try to use the flight controller on it. The idea is that the user can control the drone with the transmitter, the flight controller runs on the board but it gets the feedback and moves the drone in the simulation.


### Simulator CoppeliaSim set-up ###

Download the simulator (version EDU) from its [official website](https://www.coppeliarobotics.com/downloads).

Use the ExternalAPI to access from the FPGA-Simulator Interface the components inside the simulator.

Make sure you have following files in your directory, in order to run the various examples:

1. sim.py
2. simConst.py
3. the appropriate remote API library: "remoteApi.dll" (Windows), "remoteApi.dylib" (Mac) or "remoteApi.so" (Linux)
4. simpleTest.py (or any other example file) <- Run this script to verify that the connection between Coppelia and a Python scrip is successful.
5. Launch and start the simulation in drone_remoteAPI.ttt.


### FPGA and application set-up ###

1. Download on the FPGA the Quartus project "de10_nano_drone_simulator.jic" and re-start the board.
2. Connect the receiver and a UART-USB adapter on the uart3 of the board. See bellow the GPIO0 pins:

```
       * *
       * *
       * *
       * *
       * *
       * *
       * *
       * *
txd 26 * * 25 rxd UART3
AC3 24 * *
AC3 22 * *
AC3 20 * *
AC2 18 * *
AC1 16 * *
AC0 14 * *
       * *
       * *
       * *
       * *  
       * *  
txd  2 * *  1 rxd UART
```

Before doing the next steps, start the simulation in CoppeliaSim.

3. Download this C application as the simulation.c file as the main script.
4. At the same time, launch the main.py on the PC, which works as an interface between the simulator and the FPGA.

```
+--------------+         +--------------+         +-------------------+
|     FPGA     | <-----> |  Interface   | <-----> |     Simulator     |
| simulation.c |         |    main.py   |         |drone_remoteAPI.ttt|
+--------------+         +--------------+         +-------------------+
                        |___________________ PC ______________________|
```


##### Future works #####

* The current code only gets the receiver chanels 0-3 values on the simulation in order to spin the motors, but there is no controller avaliable yet.
* Improve the physics dynamics on the drone body so it can be controlled with a PID.
* Re-write the main multicore-flight-controller in order to get the sensor values from the simulation.
