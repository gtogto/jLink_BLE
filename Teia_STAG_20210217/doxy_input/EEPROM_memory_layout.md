# NFC EEPROM memory layout

Memory has different paging for NFC and for I2C, so the addresses of each sector are different.

| NFC address 	| I2C address   | Content    				| Note       				 			|
| :------: 		| :--------: 	| :--------: 				| :--------: 				 			|
| 0xe9			| 0x3a			| Setting registers			| See nt3h2111 datasheet	 			|
| 0xe2			| 0x38			| ^          				| ^          				 			|
| 0xe1			| 0x38			| Reserved					| Reserved for further use	 			|
| 0xe0			| 0x37			| ^          				| ^          				 			|
| 0xdf			| 0x37			| TAG info					| MAC address, platform, hw, fw, etc.	|
| 0xdc			| 0x37			| ^          				| ^          				 			|
| 0xdb			| 0x36			| Default setting			| Default setting of tag is stored here	|
| 0xbc			| 0x2F			| ^          				| ^          				 			|
| 0xbb			| 0x2E			| User setting  			| User setting of tag is stored here  	|
| 0x9c			| 0x27			| ^          				| ^          				 			|
| 0x9b			| 0x26			| New setting    			| Used for new user config	 			|
| 0x7c			| 0x1F			| ^          				| ^          				 			|
| 0x7b			| 0x1E			| Reserved					| Reserved for further use	 			|
| 0x44			| 0x11			| ^          				| ^          				 			|
| 0x43			| 0x10			| User data   				| Can be freely used by user 			|
| 0x04			| 0x01			| ^          				| ^          				 			|
| 0x03			| 0x00			| NFC configuration area 	| See nt3h2111 datasheet 	 			|
| 0x00			| 0x00			| ^         			 	| ^         			 	 			|

<br>
<br>

## Detailed desctription of some sectors
### New setting
This sector is used for configuration of tag parameters using Android application: Sewio Leonardo Configurator.
The configuration must be stored here as structure defined by Sewio. If someone store here something with different structure, it will be automaticaly erased after 5 seconds (if the tag is powered on).
This section of memory is unlocked on original Sewio tags.
See definition of data type *NFC_config_msg_t* in header file *non_volatile_memory.h* to know what structure of data is used.    


### User setting
This memory section stores settings defined by user. The firmware read this section after start and applies it, after CRC verification.
If this sector is empty or contains incorrect settings, it will be automatically replaced with Default setting.
This setting can be change using Tag Wireless Configuration in RTLS Manager or using Sewio Leonardo Configurator (Android app).
It can't be changed directly by storing data via NFC . This section is locked (NFC) on Sewio tags during manufacturing. Still I2C can read/write to this memory segment.
See definition of data type *userData_t* in header file *sewio_var.h* to get complete data structure.  


### Default setting
Here factory default setting is stored.
This section can't be overwrite via NFC because it is locked (from NFC perspective).
The settings which are stored here have the same structure as User setting.  


### TAG info
In this section are stored fundamental informations about the Tag.
Following information are stored here:
* Platform - Tag Leonardo have following platform:
    * Tag Leonardo iMU (2)
    * Tag Leonardo Asset (3)
    * Tag Leonardo Personal (4)
    * Tag Leonardo Vehicle (5)
* Hardware version
* Hardware revision
* Firmware version
* Firmware subversion
* Firmware revision
* MAC address - unique identifer of the tag thah is used for identification of UWB messages.
* Mounted sensors - same strucure as in InfoBlink - see AN12 for more info.



		
