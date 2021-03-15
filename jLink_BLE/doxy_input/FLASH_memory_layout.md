# MCU flash memory layout 



| Address  | Content    |
| ------: | :--------: |
| 0x007e000| Bootloader |
| 0x0075000| ^          | 
| 0x0074fff| ...        |
| 0x0074000| ^          |
| 0x0073fff| MAC_address|
| 0x0073000| ^          | 
| 0x0072fff‬| Firmware   |
| 0x001f000| ^          |
| 0x001efff‬| SoftDevice |
| 0x0000000| ^          |

