# Battery Board Firmware

The battery board firmware needs to be flashed onto the ESP32 on the battery board in ordrer for the board to work. Its purpose is to configure the chips on the board, monitor the cells and control the charge. It also provide human readable board status trough serial output.

## Prerequistes

The firmware is developped with the Espressif framwork on the Platform.io IDE. The team recommands the Platform.io extension on Visual Studio Code, but other editors, such as Atom, can be used.

You need the CP210x USB serial driver to connect the ESP32 to the computer. It normally works out of the box for Mac and Linux, but require a separated download for Windows. You can specify the upload port in the platformio.ini file.

## Compile and upload

- Connect the ESP32 to the computer with a USB cable **before docking the ESP32 on the board**.
- Click the Platform.io upload button.
- Wait for the build and upload to finish.

## Monitor

**The board must be powered on to communicate with the ESP32 when it is docked!**

Connect the board to the computer with a USB cable and click the Platform.io serial monitor button. You can specify the monitor port in the platformio.ini file.

## Documentation

The code is documented with Doxygen. The Doxygen output is in the doc folder of the repository.

## Design Rationale

The firmware is splitted in three sections: the chip library (chip), the harware abstraction layer (hal) and the tasks (monitor and control).

The chip library are hardware specific classes to communicate with the integrated circuits on the board. They are not thread safe and require a single instance per chip.

The hardware abstraction layer are singleton pattern classes to provide board functionnality. Thet are thread safe except for the begin calls and can be referenced from anywhere without the need for an explicit instantiation. All calls to the same class are done on the same hidden instance.

The tasks are the logic of the firmware. Each task is started in the main program. The monitor task is responsible of readings board parameters and updating the state. It can notify other task when an update is avaible. The control task implement the battery charge and balance control. It suscribes to monitor updates and perform a control cycle each time the state is updated.