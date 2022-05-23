PSA CANBus adapter FW

This is VERY much personal notes, etc. at the moment.

TODO: Make this presentable!
TODO: Open up the adapter schematic/board files.

- Written/Tested against a Peugeot 407
- Most options are currently hardcoded.
- Extended info and debug info currently piped through the Bluetooth 
Serial.

- Notable things:
	- BT Serial is enabled/disabled with the HU being 
enabled/disabled.
	- BT Serial is RO currently.
	- WiFi not used.
	- No power saving effort.
	- Now should power down HU if the door opened and engine off.
	- TODO: If door not opened, potentially should add a timer to 
power down HU after ~10 mins?
	- Write Android app to parse the BT Serial data.
