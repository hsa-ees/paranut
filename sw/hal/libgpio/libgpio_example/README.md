# Example Program for the use of the GPIO API

# Summary

This programm provides a way to test the GPIO Hardware Module.
For these tests the program uses the lib_gpio_api and also tests the full functionality of the library.
The software can only be run on the FPGA Hardware and not in the simulator.

# Prerequisites

   - a zynq 7000 or zynq z7 7020 fpga board
   - a PmodTPH2 adater for ease of plugin in the cables 
   - a logic probe of your choice we used the salea logic 4
   - icsc and sv2v (for running the hls synthesis), which can be found here:
        - [ICSC](https://github.com/intel/systemc-compiler/wiki/Getting-started)
        - [sv2v](https://github.com/zachjs/sv2v/releases/tag/v0.0.10)


# Setup

To run this program the following steps need to be done:

1. Configure the CFG_GPIO_BASE_ENABLE attribute in the config.mk file in the root of the project and in the config.mk for the board being used to "true" to enable the GPIO module
2. Change the address of the GPIO module with the CFG_GPIO_BASE_ADDRESS attribute in the config.mk at the root of the project and in the config.mk file of the board being used.
    <br>**Warning: When the Address the changed the GPIO the HSL with ICSC needs to be rerun. For further informations read the README in /hw/sysc/gpio**
3. Configure the CFG_GPIO_OUTAMOUNT how you need it the amount must not be higher than 8 
4. Build the hardware in the system/refdesign folder.
5. Connect a Logic Probe to the PMOD connector PC as follows:
![If no Picture is shown please open this file in doc-src/figs with the name GPIO_Port.jpeg](../../../doc-src/figs/GPIO_Port.jpeg)
Depending on the Configuration of the CFG_GPIO_OUTAMOUNT the pin setup changes. 
Starting from Pin 1 we have the Input Pins starting with Input Pin 1, upto the Amount of Input Pins determined by CFG_GPIO_AMOUNT-CFG_GPI_OUTAMOUNT
after these the Output Pins follow starting with Output Pin 1, upto the CFG_GPIO_AMOUNT of Pins
<br/>for e.g. with a Setup of CFG_GPIO_AMOUNT of 8 and a CFG_GPIO_OUTAMOUNT of 6
We have the following PIN LAYOUT: 
    ```
    | Pmod Pins | GPIO Pins    |
    |-----------|--------------|
    | 1         | Input Pin 1  |
    | 2         | Input Pin 2  |
    | 3         | Output Pin 1 |
    | 4         | Output Pin 2 |
    | 5         | Output Pin 3 |
    | 6         | Output Pin 4 |
    | 7         | Output Pin 5 |
    | 8         | Output Pin 6 |
    ```
    Connect your logic probe or GPIO device to the Input and Output Pins how you need it, it is advised to put a serial 300 Ohm resistor to use as a short protection.

    **WARNING: Be very careful and dont connect a Input device to the Output Pin because the Pmod Connector doesn't have a protection against shorting of output pins**


6. Now open the corresponding Logic Probe software to the Outputs and you can see them beeing toggled on in an intervall the program structure looks as follows: 
-GetInputs-->ToogleOutputs-->SetOutputsLow-->SetOutputsHigh
7. Run the software in the sw/lib_gpio_api/libgpio_example folder with the following command:
    ```sh
        make flash-refdesign-bit
    ```

    your console output should look like this for a CFG_GPIO_OUTAMOUNT of 6 and be repeating on the logic probe you can see the outputs beeing enabled and disabled:

    ```
    nput on Pin 1 is 0
    Input on Pin 2 is 0
    Output on Pin using toggleGPIO 1 is 1
    Register is now 0x00000001
    Output on Pin using toggleGPIO 2 is 1
    Register is now 0x00000003
    Output on Pin using toggleGPIO 3 is 1
    Register is now 0x00000007
    Output on Pin using toggleGPIO 4 is 1
    Register is now 0x0000000f
    Output on Pin using toggleGPIO 5 is 1
    Register is now 0x0000001f
    Output on Pin using toggleGPIO 6 is 1
    Register is now 0x0000003f
    Output on Pin using set GPIO_LOW 1 is 0
    Register is now 0x0000003e
    Output on Pin using set GPIO_LOW 2 is 0
    Register is now 0x0000003c
    Output on Pin using set GPIO_LOW 3 is 0
    Register is now 0x00000038
    Output on Pin using set GPIO_LOW 4 is 0
    Register is now 0x00000030
    Output on Pin using set GPIO_LOW 5 is 0
    Register is now 0x00000020
    Output on Pin using set GPIO_LOW 6 is 0
    Register is now 0x00000000
    Output on Pin using set GPIO_HIGH 1 is 1
    Register is now 0x00000001
    Output on Pin using set GPIO_HIGH 2 is 1
    Register is now 0x00000003
    Output on Pin using set GPIO_HIGH 3 is 1
    Register is now 0x00000007
    Output on Pin using set GPIO_HIGH 4 is 1
    Register is now 0x0000000f
    Output on Pin using set GPIO_HIGH 5 is 1
    Register is now 0x0000001f
    Output on Pin using set GPIO_HIGH 6 is 1
    Register is now 0x0000003f
    Register is now 0x0000003f
    ```

## For Further Infomation on the GPIO Module
- Bachelor Thesis "Entwurf eines Demonstrators für intelligente Tierimplantate auf Basis eines ParaNut/RISC-V-Prozessors"
- READMEs in the /\<Paranut_Root_Directory\>/hw/sysc/gpio and /\<Paranut_Root_Directory\>/sw/lib_gpio_api/ and /\<Paranut_Root_Directory\>/sw/lib_gpio_api/libgpio_example
