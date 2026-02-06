# wxsensors

Weather Sensor Emulation Scripts and Programs

## Overview

wxsensors is a collection of C programs designed to emulate various meteorological sensors over RS-485/RS-422/RS-232/SDI-12 serial connections. These emulators read pre-recorded sensor data from files and transmit it over serial ports, mimicking the behavior and protocols of real weather instrumentation.

This project is useful for:
- Testing data acquisition systems without physical sensors
- Developing and debugging weather station software
- Training and educational purposes
- Validating communication protocols before field deployment

## Architecture

Each sensor emulator follows a common architecture:
- **Receiver thread**: Listens for and parses incoming serial commands
- **Sender thread**: Transmits sensor data continuously or on-demand (polled mode)
- **Protocol handling**: Implements sensor-specific data framing with STX/ETX and checksums as required.

## Supported Sensors

| Sensor | Manufacturer | Type | Description |
|--------|--------------|------|-------------|
| WindObserver 75 | Gill Instruments | Wind | Ultrasonic anemometer measuring wind speed (0-75 m/s) and direction (0-359°) |
| HC2A-S3 | Rotronic | Temperature/Humidity | HygroClip2 T/RH probe (-50°C to +100°C, 0-100% RH) housed in Apogee TS-100 aspirated shield |
| AtmosVue30 | Campbell Scientific | Present Weather | Multi-variable sensor providing visibility (10 m to 30 km), precipitation type/intensity, and WMO weather codes |
| DSP8100 | GE Druck  | Pressure | Barometric pressure sensor emulator |
| PTB-330 | Vaisala   | Pressure | Barometric pressure sensor emulator |
| SkyVUE8 | Campbell Scientific | Cloud Height | Cloud base height and sky condition sensor emulator |
| BTD-300 | Biral | Lightning | Lightning detection sensor emulator |
| 0872F1  | Goodrich | Ice Detector | Ultrasonic axially vibrating tube ice accumulation detector | 
| CS700H  | Campbell Scientific | Rain | Tipping Bucket Rain Gauge | 

## Supported Commands

All sensor emulators support a number of different command sets:
```
TO BE POPULATED WITH COMMAND SETS PER SENSOR
```
<details>

<summary><h3> Sensor HC2A-S3 </h3></summary>

### Sensor [HC2A-S3](https://s.campbellsci.com/documents/us/manuals/hc2s3.pdf)
| Command | Description |
| :--- | :--- |
|`{F00RDD}`| `<{> <ID> <Address> <RDD> <}>` |


### {F00RDD} Response
`{F00rdd 001;4.45;%RH;000;=;20.07;°C;000;=;nc;---.-;°C;000; ;001;V1.7;0060568338;HC2-S3;000}`

</details>

<details>

<summary><h3> Sensor BTD-300 </h3></summary>

### Sensor [BTD-300](https://www.biral.com/wp-content/uploads/2023/04/106543.04F.pdf)
| Command | Description |
|---------|-------------|
| `RUN` | Enable continuous data output mode |
| `STOP` | Disable continuous mode  |
| `SITE?` | Request site characterisation values |
| `R?` | Request self-test/status message |
| `DIST?` | Request distance limits |
| `DISTDEF?` | Reset the flash distance limits to FAA Defaults 5,10,20,30 | 
| `DISTx,yyyy` | Set Distance Limits x == 0-OH, 1-V, 2-ND, 3-FD. yyyy == decametres | 
| `SN?` | Request sensor serial number | 

</details>

<details>

<summary><h3> Sensor PTB-330 </h3></summary>

### Sensor [PTB-330](https://docs.vaisala.com/viewer/book-attachment/kZW4tXhynko07HLEqbcq3Q/xy6s8h~KZ_iCT2U9pW5CMw-kZW4tXhynko07HLEqbcq3Q)
| Command | Description |
| :--- | :--- |
| `R`        		| Start continuous output (RUN mode)
| `S`        		| Stop continuous output (STOP mode)
| `SEND`    	  	| Request a single measurement message
| `INTV`      		| Set/Query continuous output interval (0-255 s/min/h/d)
| `?`     	    	| Show device information (SN, SW version, etc.)
| `VERS`   	   		| Display product name and software version
| `SNUM`    	  	| Show serial numbers for device and modules
| `ERRS`      		| List and clear unacknowledged error flags
| `HELP`      		| Display list of available serial commands
| `UNIT <u/q>` 		| Set units (hPa, mbar, kPa, Pa, inHg, mmH2O, mmHg, torr, psi)
| `FORM <str>` 		| Set custom output format string
| `SMODE <m>`  		| Set start mode (STOP, POLL, RUN, SEND)
| `ADDR <a>`   		| Set device address for RS-485 networking (0-255)
| `SERI <b p d s>`	| Set baud (up to 115200), parity, data bits, stop bits
| `SDELAY <n>` 		| Set response delay (0-2540 ms)
| `ECHO <on/off>` 	| Enable/Disable serial character echo
| `HHCP <m>`   		| Set altitude for Height Corrected Pressure (HCP)
| `HQNH <m>`   		| Set altitude for QNH corrected pressure
| `HQFE <m>`   		| Set altitude for QFE corrected pressure
| `TQFE <deg>` 		| Set temperature for QFE calculation
| `PSTAB <hPa>` 	| Set pressure stability indicator limit
| `AVRG <s>`   		| Set measurement averaging time (0-600 seconds)
| `...`				| ...
</details>


## Output Format

Sensor data is transmitted using standard framing:
```
<STX>data<ETX>checksum<CR><LF>
data checksum <CR><LF>
data <CR><LF>
```
- **STX**: Start of text (0x02)
- **ETX**: End of text (0x03)
- **Checksum**: XOR of data bytes (2 hex digits)

## Dependencies

```bash
# GTK-3 Libraries required for only the GUI build.
sudo apt install libgtk-3-dev

# C build essentials package. 
sudo apt install build-essential

# add the user to the dialout group, enabling access to serial devices.
sudo usermod -aG dialout $USER && logout 
```

## Building
```bash
# Build only the sensors (default)
make

# Build only the GUI
make gui

# Build everything (sensors + GUI)
make full

# Clean all
make clean
```

Executables are output to the `bin/` directory, organized by sensor type.

## Usage
```bash
# General usage pattern
<sensor> <data_file> [serial_port] [baud_rate] [RS232|RS422|RS485|SDI-12]

# Examples
bin/wind/wind_listen /path/to/wind_data.txt
bin/wind/wind_listen /path/to/wind_data.txt /dev/ttyUSB0 9600 RS485
bin/rh_temp/rh_temp_listen /path/to/rh_temp_data.txt /dev/ttyUSB1 9600 RS422
```

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| data_file | (required) | Path to file containing sensor data to transmit |
| serial_port | /dev/ttyUSB0 | Serial device (must match `/dev/tty(S\|USB\|ACM)[0-9]+`) |
| baud_rate | 9600 | Serial baud rate |
| mode | RS485 | Serial mode: RS422 or RS485 |

## Data Files

Each emulator reads line-by-line from a data file, cycling back to the beginning when EOF is reached. Data files should contain one sensor reading per line in the appropriate format for that sensor type.

Example data files are provided in the `data_files/` directory.

## Serial Port Configuration

For consistent USB serial device naming, create a udev rule:

1. Find your device's Vendor ID and Product ID:
```bash
   lsusb | awk '{print $6}'
```

2. Create a udev rule in `/etc/udev/rules.d/usb-serial.rules`:
```
   SUBSYSTEM=="tty", ATTRS{idVendor}=="<VENDOR_ID>", ATTRS{idProduct}=="<PRODUCT_ID>", SYMLINK+="ttyUSB_wind"
```

3. Reload udev rules:
```bash
   sudo udevadm control --reload-rules
   sudo udevadm trigger
```

## Project Structure
```
wxsensors/
├── include/              # Shared header files
│   ├── atmosvue30_utils.h
│   ├── console_utils.h
│   ├── crc_utils.h
│   ├── dsp8100_utils.h
│   ├── file_utils.h
│   ├── ptb330_utils.h
│   ├── q131.h
│   ├── sensor_utils.h
│   └── serial_utils.h
├── common/               # Shared source files
│   ├── atmosvue30_utils.c
│   ├── console_utils.c
│   ├── crc_utils.c
│   ├── dsp8100_utils.c
│   ├── file_utils.c
│   ├── ptb330_utils.h
│   ├── sensor_utils.c
│   └── serial_utils.c
├── wind/                 # Gill WindObserver 75 emulator
│   └── wind_listen.c
├── rh_temp/              # Rotronic HC2A-S3 emulator
│   └── tmp_rh_listen.c
├── pres_weather/         # Campbell Scientific AtmosVue30 emulator
│   └── pres_weather.c
├── dsp8100/           	  # Druck DSP-8100 barometric pressure emulator
│   └── dsp8100.c
├── ptb330/               # Vaisala PTB-330 barometric pressure emulator
│   └── ptb330.c
├── ceilometer/           # Ceilometer emulator
│   └── ceilometer.c
├── ice/        	      # Ice Accumulation Detection emulator
│   └── ice.c
├── flash/                # Lightning sensor emulator
│   └── flash.c
├── rain/                 # Rain tipping bucket sensor emulator
│   └── rain.c
├── sensor_control/       # Graphical User Interface Program
│   └── sensor_control.c
├── data_files/           # Sample sensor data files
├── bin/                  # Compiled executables (generated)
├── obj/                  # Object files (generated)
├── Makefile
├── LICENSE.md
└── README.md
```

## Dependencies

- GCC compiler
- POSIX threads (pthread)
- Linux serial port support

## License

Copyright (C) 2025

This project is licensed under the GNU General Public License v3.0 - see the [LICENSE](LICENSE.md) file for details.

This project is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the [GNU General Public License]

## Author

Bruce Dearing

## Contributing

[Add contribution guidelines here]
