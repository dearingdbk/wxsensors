#!/bin/bash
# read stdin into a temp file, or chosen file
# File:     lightning.sh
# Author:   Bruce Dearing
# Date:     17/11/2025
# Version:  1.4

# Purpose:  Weather Sensor script which emulates a BTD-300 lightning sensors output.
#           Initially it creates one temp file 'tmpfile' which it will cat in either a
#           file from a known location, 'knownfile', standard input, or a new files location
#           if provided in arguments to the script. It will then open a serial connection and print
#	    each line of that file to the the serial connection, pausing for 2 seconds each time.
#	    The script traps exit signals, and cleans up the file descriptor, and serial connection on close.
#
#           use case ' lightning.sh <location_of_file_to_be_transmitted>'
#           use case ' cat <file_to_be_transmitted> | lightning.sh'
#	    use case ' ligthning.sh < <file_to_be_transmitted>'
#	    use case ' lightning.sh' this will use the known file location 'knownfile'


#identify a known lightning data file to fall back on when testing,
#if data is not provided by pipe, or redirect.

knownfile="/home/raspy/Documents/lightning_data.txt"
tmpfile=$(mktemp) || { printf '%s\n' "mktemp failed" >&2; exit 1; } #create a safe tmp file using mktemp to put our data.

trap '{
	# Remove temporary file if it exists
    	if [ -n "$tmpfile" ] && [ -e "$tmpfile" ]; then
		rm -f "$tmpfile"
	fi

    	# Restore terminal settings and close FD 3 if it is open
	if exec 3>&3 2>/dev/null; then
        	stty sane <&3 2>/dev/null
        	exec 3>&-
    	fi

    	exit 0

}' EXIT SIGINT SIGQUIT SIGTERM #trap any exit to clean up our tmp file, and close fd.

if [ ! -t 0 ]; then  # check stdin for any data
	cat >"$tmpfile"

elif [ -n "$1" ] && [ -f "$1" ]; then # check if there is an argument for file location
	cat -- "$1" > "$tmpfile"

elif [ -f "$knownfile" ]; then # does our known file exist
	cat -- "$knownfile" > "$tmpfile" # if so copy it to our temp file

else
	echo "error: file not found" >&2
	exit 1
fi

port="/dev/ttyUSB0" #set port to the location of the usb to serial device
baud=9600 # set the baud rate

# If we need to set the data to rs485 not posix
#if command -v setserial &>/dev/null; then
#	setserial "$port" rs485 on 2>/dev/null
#fi

# open $port for writing on File Descriptor 3
exec 3<> "$port" || {
    printf '%s\n' "Failed to open $port for writing" >&2
    exit 1
}

# Configure the serial port
# NON-POSIX stty command
# -F is gnu, use only if known.
# stty -F "$port" $baud cs8 -cstopb -parenb -ixon -ixoff -crtscts

# Configure the serial port
# POSIX stty command
stty "$baud" cs8 -cstopb -parenb -ixon -ixoff -crtscts <&3 || {
    printf '%s\n' "Failed to configure $port" >&2
    exit 1
}

# Loop through the file, sending one line at a time every 2 seconds.
while true; do
	while IFS= read -r line || [ -n "$line" ]; do
		if [ -n "$line" ]; then
			printf '%s\r\n' "$line" >&3
		fi
		sleep 2
	done < "$tmpfile"
done
