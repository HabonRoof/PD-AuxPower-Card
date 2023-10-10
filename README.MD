# Type-C PD Auxiliary power supply
### _Light, small, isolated digial power_
Tiny-Aux is a small protable auxiliary digital power supply for electrical engineer using everyday.
As a power electronic engineer, I'm tired moving heavy linear bench power for only less then 1W power.
So I develop this Tiny-Aux for four different voltage as auxiliary power source.
It sould support USB type-C connector and PD protocol fand four different output voltage, but now can only draw 5V input.
Most importantly, it's isolated from primary and secondary, ground path is no more concerned.
## Appearence
![Appearence of Tiny-Aux](/Image/Tiny-Aux-45deg.jpg "Tiny-Aux side view")
The two ER9.5 transformer provide physical isolation side by side.
There are two output channel provide 3.3V/5V and 12V/24V saperately. Use two switches to set the output voltage of each channel.
The black/yellow wire and ground pin is for debugging.
## Features
- PWM limit protection
- UART real-time power monitoring

## Fearures to be develop
- Over current/ Over voltage protection
- FUSB302B driver library
- EEPROM for power event recording

## Specification
- Input voltage: 5V-24V (USB PD is in development)
- Channel 1 output voltage: 3.3V/ 5V
- Channel 1 output current: 1000 mA
- Channel 2 output voltage: 12V/ 24V
- Channel 2 output current: 500 mA
- Total putput wattage: 2W

## Technical detail
All of the component can be placed or soldered by machine automatically, **but no transformer**.
This project using flyback topology as isolated power, so the transformer is the key component and need to satisfy following charactisic.
The bobbin size used is ER9.5, core material is PC44, diameter of conductor is 0.5 mm.
The transformer need grind air-gap manualy. The distant is about 0.5mm. (Need to be confirm).
For **Channel 1** 3.3V/5V rail, the transformer should have this charactisic:
- Primary inductance: 330uH, roughtly 10 turns
- Secondary inductance: 30uH, roughtly 30 turns
 -Leakage inductance: 10uH

For **Channel 2** 12V/24V rail, the transformer should have this charactisic:
- Primary inductance: 30uH, roughtly 3 turns
- Secondary inductance: 330uH, roughtly 30 turns
- Leakage inductance: 10uH

The **dot point** of transformer is marked on PCB schematic file, **DOUBLE CHECK the polarity**.

The switching frequency is set to 200kHz, using voltage control for stable output voltage.
PID parameter can be tune to best for dynamic load.

## Important Notice
The communication pin (UART, Debug interface) is grounded with primary side of Tiny-Aux module, please DO NOT connect to the output channel incase burn your computer.
Over current protection and over voltage protection is not implement yet!
USB PD ptorocol is not implement yet!

## Known Issue
- PD protocol can not read PDO message.
- 5V transient will go into unstable state when applying 100 mA 1000mA dynamic load.
- TPS62177 EN pin connect to GND (fixed in schematic)

## Contribute
If you want to join the development, please feel free to send me e-mail, and I'll send you the hardware for free.