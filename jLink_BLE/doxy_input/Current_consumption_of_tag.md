# Current cunsumption of the Tag Leonardo
The tag has different current consumption in different states. Some indicative values are listed below:
| TAG state 				| TAG variant				| Current consumption [mA] | Note    |
| :------: 					| :------: 					| :-------- 			   | :------ | 
| DW sleep + MCU normal mode| iMU	                    | 20.58					   |Vin = 4.2V                           |
| ^                     	| Personal                  | 20.46					   |Vin = 4.2V                           |
| ^                     	| Asset                     | TBD 					   |Vin = 3V                             |
| ^                     	| Vehicle                   | TBD 					   |Vin = 24V                            |
| DW idle + MCU normal mode	| iMU	                    | 7.48					   |Vin = 4.2V                           |
| ^                     	| Personal                  | 7.39					   |Vin = 4.2V 							 |
| ^                     	| Asset                     | TBD 					   |Vin = 3V   							 |
| ^                     	| Vehicle                   | TBD 					   |Vin = 24V 							 |
| DW TX + MCU normal mode	| iMU	                    | 120.62				   |CH5,RF4,TX gain = 19.5dB, Vin = 4.2V |
| ^                     	| Personal                  | 120.48				   |CH5,RF4,TX gain = 19.5dB, Vin = 4.2V |
| ^                     	| Asset                     | TBD 					   |CH5,RF4,TX gain = 19.5dB, Vin = 3V   |
| ^                     	| Vehicle                   | TBD 					   |CH5,RF4,TX gain = 19.5dB, Vin = 24V  |
| DW sleep + MCU sleep mode	| iMU	                    | 0.0325				   |Vin = 4.2V                           |
| ^                     	| Personal                  | 0.0245				   |Vin = 4.2V                           |
| ^                     	| Asset                     | 0.0090 				   |Vin = 3V                             |
| ^                     	| Vehicle                   | 0.040					   |Vin = 24V                            |

 
*All values shown are for reference only, as they may vary depending on different configurations, input voltage and temperature.