# wxsensors

Weather Sensor Emulation Scripts and Programs

## Overview

wxsensors is a collection of C programs designed to emulate various meteorological sensors over RS-485/RS-422 serial connections. These emulators read pre-recorded sensor data from files and transmit it over serial ports, mimicking the behavior and protocols of real weather instrumentation.

This project is useful for:
- Testing data acquisition systems without physical sensors
- Developing and debugging weather station software
- Training and educational purposes
- Validating communication protocols before field deployment

## Architecture

Each sensor emulator follows a common architecture:
- **Receiver thread**: Listens for and parses incoming serial commands
- **Sender thread**: Transmits sensor data continuously or on-demand (polled mode)
- **Protocol handling**: Implements sensor-specific data framing with STX/ETX and checksums

## Supported Sensors

| Sensor | Manufacturer | Type | Description |
|--------|--------------|------|-------------|
| WindObserver 75 | Gill Instruments | Wind | Ultrasonic anemometer measuring wind speed (0-75 m/s) and direction (0-359°) |
| HC2A-S3 | Rotronic | Temperature/Humidity | HygroClip2 T/RH probe (-50°C to +100°C, 0-100% RH) housed in Apogee TS-100 aspirated shield |
| AtmosVue30 | Campbell Scientific | Present Weather | Multi-variable sensor providing visibility (10 m to 30 km), precipitation type/intensity, and WMO weather codes |
| DSP8100 | GE Druck  | Pressure | Barometric pressure sensor emulator |
| SkyVUE8 | Campbell Scientific | Cloud Height | Cloud base height and sky condition sensor emulator |
| BTD-300 | Biral | Lightning | Lightning detection sensor emulator |

## Supported Commands

All sensor emulators support a number of different command sets:
```
TO BE POPULATED WITH COMMAND SETS PER SENSOR
```
### Sensor HC2A-S3
| Command | Description |
| :--- | :--- |
|`{F00RDD}`| <{> <ID> <Address> <RDD> <}> |

### {F00RDD} Response
`{F00rdd 001;4.45;%RH;000;=;20.07;°C;000;=;nc;---.-;°C;000; ;001;V1.7;0060568338;HC2-S3;000}`
### Sensor B
| Command | Description |
|---------|-------------|
| `!` | Enable continuous data output mode |
| `?` | Disable continuous mode (switch to polled mode) |
| `<A-Z>` | Poll sensor at specified unit address |
| `*<A-Z>` | Enter configuration mode for specified unit |
| `&` | Request unit identifier |

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

## Building
```bash
# Build all sensor emulators
make

# Clean build artifacts
make clean
```

Executables are output to the `bin/` directory, organized by sensor type.

## Usage
```bash
# General usage pattern
<sensor> <data_file> [serial_port] [baud_rate] [RS422|RS485]

# Examples
bin/wind/wind_listen /path/to/wind_data.txt
bin/wind/wind_listen /path/to/wind_data.txt /dev/ttyUSB0 9600 RS485
bin/rh_temp/rh_temp_listen /path/to/rh_temp_data.txt /dev/ttyUSB1 9600 RS422
```

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| data_file | (required) | Path to file containing sensor data to transmit |
| serial_port | /dev/ttyUSB0 | Serial device (must match `/dev/tty(S\|USB)[0-9]+`) |
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
├── include/           # Shared header files
│   ├── crc_utils.h
│   ├── sensor_utils.h
│   └── serial_utils.h
├── common/            # Shared source files
│   ├── crc_utils.c
│   ├── sensor_utils.c
│   └── serial_utils.c
├── wind/              # Gill WindObserver 75 emulator
│   └── wind_listen.c
├── rh_temp/           # Rotronic HC2A-S3 emulator
│   └── tmp_rh_listen.c
├── pres_weather/      # Campbell Scientific AtmosVue30 emulator
│   └── pres_weather.c
├── barometric/        # Barometric pressure emulator
│   └── barometric.c
├── ceilometer/        # Ceilometer emulator
│   └── ceilometer.c
├── flash/             # Lightning sensor emulator
│   └── lightning.sh
├── data_files/        # Sample sensor data files
├── bin/               # Compiled executables (generated)
├── obj/               # Object files (generated)
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
