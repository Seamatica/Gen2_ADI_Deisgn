## How to Setup Project

### Run these Commands in CMD prompt one by one within this directory:
- "C:\Xilinx\Vivado\2019.1\settings64.bat" && vivado -mode tcl
    - Find your specific path for settings64.bat   
- source Gen2_ADI_Design.tcl

Resolve lwip compatibility
- Right click bsp folder, select Board Support Package Settings, tick lwip211 (Version 1.5).
- After ticking lwip211, click lwip211 under Overview/standalone/lwip211, expand dhcp_options, change dhcp_does_arp_check and lwip_dhcp from false to true. Expand pbuf_options, change pbuf_pool_size from 256 to 16384. Click OK
