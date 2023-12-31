# Miyoo Mini Plus Bluetooth Mod firmware

This repository contains the code that runs on the ATmega32u4 microcontroller.
This software essentially acts as a bridge between the KCX_BT_Emitter and the Miyoo Mini Plus.

The KCX_BT_Emitter and the microcontroller are fed by the battery directly, this is needed because the KCX_BT_Emitter won't power on with a lower voltage (3v or 3v3).

This means that both are essentially always powered. To avoid excessive power drain the lower power features of the KCX_BT_Emitter and the ATmega32u4 are used when Bluetooth isn't used or when the system is turned off.

## KCX_BT_Emitter communication

Communication with the KCX_BT_Emitter is achieved by using:
- AT UART interface.
- CON pin (wake from sleep).


## Miyoo Mini Plus communication

Communication with the Miyoo Mini Plus is not trivial due to missing software drivers (kernel and device tree overlay).
This limits the communication to I2C as this is already used for the PMIC (AXP223).

So it uses:
- I2C as the main communication method.
- A 3v input originating from DCDC1 (LX1) pin on the AXP223.
    - This is used to detect when the system is powered.
- Headphone control (Enable headphone output when Bluetooth device is connected).


![](./assets/mmp_back.jpg)