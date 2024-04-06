# Iridium-tracker
Repository for code that runs on the iridium tracker for the MIDAS avionics system. 

## Getting Started
As there is no current platformio build target for the SparkFun Artemis board, we will be using the Arduino IDE (all rejoice).

In the arduino IDE, using the board manager, Install an "Aditional Core", and enter this [link](https://raw.githubusercontent.com/sparkfun/Arduino_Apollo3/master/package_sparkfun_apollo3_index.json).

Once this is installed, you will need to isntall some libraries using the library manager. These are all outlined in the big comment block at the top of ```Software/Iridium/main/main.ino``` and ```Software/Iridium/main_no_tx/main_no_tx.ino```. The libraries you need to install are:
- [Iridium SBD I2C](https://github.com/sparkfun/SparkFun_IridiumSBD_I2C_Arduino_Library), avalible by searching `IridiumSBDi2c` in the arduino library
- [Qwiic_PHT_MS8607](https://github.com/sparkfun/SparkFun_PHT_MS8607_Arduino_Library), avalible by searching `SparkFun MS8607`
- [SparkFun u-blox](https://github.com/sparkfun/SparkFun_u-blox_GNSS_Arduino_Library), avalible by searching `SparkFun u-blox GNSS`

Once you have all these installed, you should be good to start flashing code.

## File Structure
```
.
└── Iridium-Tracker/
    ├── binaries/
    ├── Documentation/
    ├── Hardware/
    ├── img/
    ├── Processing/
    ├── Software/
    │   ├── examples/
    │   └── Iridium/
    │       ├── main
    │       └── main_no_tx
    ├── Tools/
    ├── .gitignore
    └── README.md
```
`binaries` Provided binaries from SparkFun

`Documentation` Provided documentation for the Artemis boards from SparkFun

`Hardware` Schematics for the hardware of the Artemis board from SparkFun

`Processing` Data and scripts for processing GPS outputs

`Software` Provided and written code to flash

`examples` Provided code examples from SparkFun

`Iridium` Written code to flash

`main` Main code with transmitting to Iridium service

`main_no_tx` Main code without transmitting

`Tools` Provided tools from SparkFun

## Running Code
Most of the code in this comes from the starter guide for the [Artemis Global Tracke](https://learn.sparkfun.com/tutorials/artemis-global-tracker-hookup-guide/artemis-global-tracker-arduino-examples). To find the code that we (ISS) updated, which is the code you should be flashing, go to ```Software/Iridium```. Here you should see two folders, `main` and `mmain_no_tx`. Due to the price of using the Iridum Tracker, if you are not directly testing/using the iridium uplink, **DO NOT FLASH** `main`.

You can also reference the examples in `Software/examples` if you want to see how to do other things with the module.
