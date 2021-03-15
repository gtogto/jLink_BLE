# RTLS TDOA Tag Firmware 
  This is the documentation of firmware for Sewio tags. That is usable for 
  following platforms:

- Tag Leonardo Personal
- Tag Leonardo iMU
- Tag Leonardo Asset
- Tag Leonardo Vehicle

All aforementioned platforms have the same radio/MCU module DWM1001C with MCU Nordic MCU nRF52832_xxAA.



# Build firmware  {#section_build}


The souce files are supplied as project for EmBitz v1.10.
## Software requirements {#subsection_Software_requirements}

- **IDE:** EmBitz v1.10
- **SDK:** nRF5_SDK_12.3.0 
- **Toolchain:** GNU ARM v 4.9.3
- **Flash SW:** SEGGER JLink_V631

## Build instructions {#subsection_Build_instructions}

 - Download the firmware from Sewio Partner portal.
 - Unzip content of downloaded archive. Project folder must be placed in the **same folder as SDK folder**. 
 - Open EmBitz 1.10.
 - Click on "File -> Open"
 - Open project file "TDOA_DWM1001.ebp"
 - Using the combo box on the top bar in the EmBitz select the target (according to used HW).
 - Click on "Build -> Build Target". Hex (intel hex format) file "RTLS_TDOA_HW_Leonardo_v1r3_FW_3_124_5.hex"  will be generated in "[project_path]\_build"



# Load firmware to the TAG {#section_flashMCU} 

##With EmBitz{#subsection_EmBitz_IDE}

 - Connect the programmer to the PC via USB
 - Connect the programmer to the Tag via SWD. (See Leonardo HW description for SWD pinout on the tag)
 - Click debug -> Start/stop Debug Session. (Tag must be preflashed with SoftDevice s132_nrf52_3.0.0 see next section) 

	
##With J-Flash Lite{#subsection_J-Flash_lite}

- Connect the programmer to the PC via USB
- Connect the programmer to the Tag via SWD. (See Leonardo HW description for SWD pinout on the tag)
- Open J-Flash Lite (Segger)
- Select device: nRF52832_xxAA
- Select interface: SWD and click "OK"
- Erase chip (just in case)
- For flash the device with softDevice insert: "[project_path]\..\nRF5_SDK_12.3.0_d7731ad\components\softdevice\s132\hex\s132_nrf52_3.0.0_softdevice.hexs132_nrf52_3.0.0_softdevice.hex" into "Data file" label.
- Program device
- For flash the device with Leonardo FW insert: "[project_path]\_build\RTLS_TDOA_HW_Leonardo_v1r3_FW_3_124_6.hex" into "Data file" label.
- Program device





# User functionality  {#section_userFC}

User can add their own functionality into tag FW. For example he can add sensor on tag and sent measured data in Blink message.
There is predefined special blink message with user payload, that can be sent to RTLS System, where it can be further disseminated via API.
                 
## Hardware {#subsection_sensorsHW}
          
 If user want to add some sensors or other HW to the tag, that is possible by connect it to the I2C Bus. On Tag Leonardo, the I2C wires (SDA, SCL) and supply wires (3V3, GND) are connected to the throw hole pins on the border of PCB. Also some GPIOs are connected here. See <a href="../html/d1/ddf/md__leonardo__i_o_description.html">Tag Leonardo Interfaces description</a> for furthere info.

## Firmware {#subsection_sensorsFW}
   User must of course also change the firmware. It is neccessary to add driver and handling functions into source code. Function shoud be called in main.c  at line 69. Due to this it will be called every blink period. To achieve the set Refresh Rate, the duration of the user function should not exceed (RefeshRate [ms] - 10 ms).

### <a href="https://docs.sewio.net/display/PUB/Tag+Leonardo+Firmware+Changelog">FW changelog</a>