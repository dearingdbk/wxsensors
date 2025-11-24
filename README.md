# wxsensors
Weather Sensor Emulation scripts and programs


If standard assignment of USB serial devices is required to assigned symbolic links follow the below steps:
1. Find the Vendor ID and Product ID of the USB Serial device using lsusb | awk '{print $6}' this gives each line of USB devices VendorID:ProductID
2. Create a udev rule: Create a new rule file in /etc/udev/rules.d/ (e.g. usb-serial.rules).
3. Write the rule: Add a line to this file that links the device to a static name. A common pattern is: 
	SUBSYSTEM=="tty", ATTRS{idVendor}=="<VendorID>", ATTRS{idProduct}=="<ProductID>", SYMLINK+="ttyUSB3"
 	Replace <VendorID> and <ProductID> with the actual IDs you found from step 2.
4. Reload udev rules: Run 
	sudo udevadm control --reload-rules
   and then 
	sudo udevadm trigger 
   to apply the changes.
5. Check the symlink: The device should now consistently be available as /dev/ttyUSB3. 
