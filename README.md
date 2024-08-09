# EmbeddedBSP_Development

Development of a board support package meant to be generic for different embedded platform architectures. At this moment this provides support for Atmel SMART MCUs SAMD21 and SAMV71.

## Project Structure

The **xdk-asf** folder contains the cmsis headers contained in Atmel's Advanced Software Framework (ASF). Only the header files with register addresses and linker scripts from ASF were used in this project.

The **BSP** folder contains the developed support package for the selected boards. This folder contains the drivers for the peripherals used in this example (Clocks, GPIOs, UART, ADC)

The **App** folder contains the main application file, some config files that may be used to define peripherals behavior, and the makefiles to compile for the different targets.

## BSP

The BSP includes a subfolder named **interfaces** which contains the different structures that define the interfaces that each driver needs to implement. The **src** folder contains different subfolders with the target specific implementation of the interfaces.

The compiler should define a preprocessor describing the target system. Based on this variable, the mcu.h file in the app folder will include the correct drivers.

## Building

The project uses make to run the compiler and linker. This project was tested on windows with GNU make and should be compatible with Linux tools.
The project uses gcc arm toolchain for compilation and needs to be installed to build the app.

To build, go to the app folder and run one of the following commands:

**make samd21**

**make samv71**
