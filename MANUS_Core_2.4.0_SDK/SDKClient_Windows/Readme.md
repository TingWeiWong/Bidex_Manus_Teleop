# MANUS SDK Client - Windows
Some essential source and header files are ignored (e.g., `ManusSDK`, `ClientLogging.hpp`, etc.).  
You will need to download them manually from ManusCore.

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
3. Also set network inbound/outbound of 8000 port to allow all connections.

### MISC
1. Make sure to allocate enough RAM for VirtualBox, otherwise there might be memory exception errors (from libZMQ).
2. Make sure the ManusCore is running in the background
