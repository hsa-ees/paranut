# Example Program for the use of the Bluetooth API

# Summary

This programm provides a way to test the Bluetooth Module BM71.
For these tests the program uses the lib_gpio_api, lib_uart_api, libparanut and the lib_bluetooth_api.
It waits for a connection with the Microchip Bluetooth Data App and if you send it a message it will respond accordingly, if the message is *Hallo* it responds with *Hallo Handy* and else it will responde with *Wer bist du*
The software can only be run on the FPGA Hardware and not in the simulator.

# Prerequisites

   - a zynq 7000 or zynq z7 7020 fpga board
   - a BM70 Bluetooth Module or the BM70 pictail board
   - the Microchip Bluetooth Data App from the Appstore
   - icsc and sv2v (for running the hls synthesis), which can be found here:
        - [ICSC](https://github.com/intel/systemc-compiler/wiki/Getting-started)
        - [sv2v](https://github.com/zachjs/sv2v/releases/tag/v0.0.10)


# Setup

To run this program the following steps need to be done:

1. Configure the CFG_GPIO_BASE_ENABLE and the CFG_UART_ENABLE attribute in the config.mk file in the root of the project and in the config.mk for the board being used to "true" to enable the GPIO module
2. Configure the CFG_GPIO_OUTAMOUNT to 5
3. Run the following command from the /\<ParaNut_Root_Directory\>/hw/sysc/gpio
```sh 
    update_gpio
``` 
4. Run the following command from the /\<Paranut_Root_Directory\>/hw/sysc/uart
```sh 
    update_uart
``` 
5. Build the hardware in the system/refdesign folder.
6. Connect a BM70 Pictail Board to the PMOD connector PC as follows:

    Connect the VCC and GND connectors of the FPGA to the VCC and GND connectors of the BM70 Pictail Board, now connect the HCI RXD Pin of the BM70 Pictail Board to the Rx Pin of the JE Pmod of the ParaNut see the picture below. Aswell as the HCI TXD Pin of the BM70 Pictail Board to the Tx Pin of the JE see the picture below
    ![If no Picture is shown please open this file in doc-src/figs with the name uart_connections.jpeg](../../../doc-src/figs/uart_connection.jpeg)
    Connect the Pin P1_1 of the BM70 to the Input Pin 1/Pin1 of the JC Pmod, this is the StatusPin1 aswell as the Pin P1_0 of the BM70 to the Input Pin 2/Pin2 of the JC Pmod, this is the StatusPin2. Also connect the Pin P2_3 of the BM70 to the Output Pin 5/Pin8 of the JC Pmod, this is the WakeUpPin. For the Pin Layouts see the two pictores after this paragraph. It is advised to put a a series resitor of 300 Ohm between each of the GPIO connections, to protect yourself of shortages. 
    ![If no Picture is shown please open this file in doc-src/figs with the name GPIO_Port.jpeg](../../../doc-src/figs/GPIO_Port.jpeg)
    ![If no Picture is shown please open this file in doc-src/figs with the name Pictail_BM70.jpeg](../../../doc-src/figs/Pictail_BM70.jpeg)

    **WARNING: Be very careful and dont connect a Input device to the Output Pin because the Pmod Connector doesn't have a protection against shorting of output pins**

7. Run the software in the sw/lib_gpio_api/libgpio_example folder with the following command:
    ```sh
        make flash-refdesign-bit
    ```

    now the BM70 should be running.
8. Connecting yourself with the Microchip Bluetooth Data open the app on android, click on th BLE UART and select the BM70.   
Now click on Scan and on Scan once again. Now all the bluetooth devices should be listed select the Device named *02-10475-2*. If connected succesfully you should see the service *Transparent Transfer data to the device*, click on it and a Input Window should open.  
Now enable the Append CR/LF Flag and you can send your commands.
*Hallo* should get the respons *Hallo Handy* and anything else a response of *Wer bist du* 

# Information
## For Further Information on the Bluetooth Module
- Bachelor Thesis "Entwurf eines Demonstrators für intelligente Tierimplantate auf Basis eines ParaNut/RISC-V-Prozessors"
- https://ww1.microchip.com/downloads/en/DeviceDoc/70005235D.pdf
- http://ww1.microchip.com/downloads/en/DeviceDoc/50002542A.pdf
- https://microchipdeveloper.com/ble:bm70-mcu-interface
## For Further Infomation on the GPIO Module
- Bachelor Thesis "Entwurf eines Demonstrators für intelligente Tierimplantate auf Basis eines ParaNut/RISC-V-Prozessors"
- READMEs in the /\<Paranut_Root_Directory\>/hw/sysc/gpio and /\<Paranut_Root_Directory\>/sw/lib_gpio_api/ and /\<Paranut_Root_Directory\>/sw/lib_gpio_api/libgpio_example

## For Further Information on the UART Module
- Bachelor Thesis "Entwurf eines Demonstrators für intelligente Tierimplantate auf Basis eines ParaNut/RISC-V-Prozessors"

- Bachelor Thesis "Weiterentwicklung der Debug-Infrastruktur des ParaNut-Prozessors"

- READMEs in the /\<Paranut_Root_Directory\>/hw/sysc/uart and /\<Paranut_Root_Directory\>/sw/lib_uart_api/ and /\<Paranut_Root_Directory\>/sw/lib_uart_api/libuart_example