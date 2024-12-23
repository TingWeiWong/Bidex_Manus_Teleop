# MANUS SDK Client - Windows

The MANUS SDK Client is an example client that demonstrates most of the functionality of the MANUS SDK. It handles setting up a connection to the gloves (either directly or via a MANUS Core instance) and implements all major features available in the MANUS SDK.  
For a full guide on how to get started, please refer to the guide on our Knowledge Center: [https://docs.manus-meta.com/latest/Plugins/SDK/Windows/SDK%20Client/](https://docs.manus-meta.com/latest/Plugins/SDK/Windows/SDK%20Client/)

## VirtualBox Setup (Ubuntu Computer as Host)

1. Install VirtualBox on Ubuntu.  
2. Select Windows 10/11 as the guest operating system.  
3. Install the VirtualBox Extension Pack.

### USB Setting

1. Open VirtualBox.  
2. Go to the top menu and select `Devices` -> `USB` -> `USB Settings` -> `USB Device Filters`.  
3. Add a new USB filter with the following details:  
`Name: manus, Vendor ID: 3325`


### Network Setting

1. Go to `Network Settings` -> `Adapter 1` -> `Attached to`.  
2. Select `Bridged Adapter` and connect it to the Ethernet of your host PC (e.g., `enp5s0`).

### Missing Files from .gitignore

Some essential source and header files are ignored (e.g., `ManusSDK`, `ClientLogging.hpp`, etc.).  
You will need to download them manually from ManusCore.
