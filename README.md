# Iridium-tracker
Repository for code that runs on the iridium tracker for the MIDAS avionics system. 

## Getting Started
As there is no current platformio build target for the SparkFun Artemis board, we will be using the Arduino IDE (all rejoice).

In the arduino IDE, using the board manager, install an "Aditional Core", and enter this [link](https://raw.githubusercontent.com/sparkfun/Arduino_Apollo3/master/package_sparkfun_apollo3_index.json).

Once this is installed, you will need to install some libraries using the library manager. These are all outlined in the big comment block at the top of ```Software/Iridium/main/main.ino```. The libraries you need to install are:
- [Iridium SBD I2C](https://github.com/sparkfun/SparkFun_IridiumSBD_I2C_Arduino_Library), available by searching `IridiumSBDi2c` in the arduino library
- [Qwiic_PHT_MS8607](https://github.com/sparkfun/SparkFun_PHT_MS8607_Arduino_Library), available by searching `SparkFun MS8607`
- [SparkFun u-blox](https://github.com/sparkfun/SparkFun_u-blox_GNSS_Arduino_Library), available by searching `SparkFun u-blox GNSS`

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
    |   ├── decoding/
    │   ├── examples/
    │   └── Iridium/
    │       └──  main
    ├── Tools/
    ├── .gitignore
    ├── Makefile
    └── README.md
```
`binaries` Provided binaries from SparkFun

`Documentation` Provided documentation for the Artemis boards from SparkFun

`Hardware` Schematics for the hardware of the Artemis board from SparkFun

`Processing` Data and scripts for processing GPS outputs

`Software` Provided and written code to flash

`decoding` A Script to decode compressed data from the iridium service

`examples` Provided code examples from SparkFun

`Iridium` Written code to flash

`main` Main code with transmitting to Iridium service

`Tools` Provided tools from SparkFun

## Running Code
Most of the code in this comes from the starter guide for the [Artemis Global Tracker](https://learn.sparkfun.com/tutorials/artemis-global-tracker-hookup-guide/artemis-global-tracker-arduino-examples). To find the code that we (ISS) updated, which is the code you should be flashing, go to ```Software/Iridium/main```. 

There is a macro in this code, called `noTX`, located around line 100 under a comment which says "this is important". Defining this macro will prevent transmission to the iridium service, which is useful to save credits. If you are not directly testing the iridium uplink, ensure this macro is not commented. If you require the iridium service, ensure it is commented out.

You can also reference the examples in `Software/examples` if you want to see how to do other things with the module.

## Decoding
In `Software/decoding` there is a script to decode the encoded message sent to the iridium service. You can compile, and then run, this script with `make` and `./a.out`, or do it all in one command with `make run`. Once the script is running, copy in the text from the iridium service into the terminal, and hit enter to get the decoded packet.
