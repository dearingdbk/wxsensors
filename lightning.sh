#!/bin/bash
# read stdin into a temp file, or chosen file

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

	#[ -n "$tmpfile" ] && rm -f "$tmpfile"
	#[ -e /proc/$$/fd/3 ] && stty -icrnl <&3 2>/dev/null && stty flush <&3 2>/dev/null && exec 3>&-
	#exit 0
}' EXIT SIGINT SIGQUIT SIGTERM #trap any exit to clean up our tmp file, and close fd.

if [ ! -t 0 ]; then  # check stdin for any data
	cat >"$tmpfile"

elif [ -n "$1" ] && [ -f "$1" ]; then # check if there is an argument for file location
	cat -- "$1" > "$tmpfile"

elif [ -f "$knownfile" ]; then # does our known file exist
	cat -- "$knownfile" > "$tmpfile" # if so copy it to our temp file

else
	echo "error: no file not found" >&2
	exit 1
fi

port="/dev/ttyUSB0" #set port to the location of the usb to serial device
baud=9600 # set the baud rate

# If we need to set the data to rs485 not posix
#if command -v setserial &>/dev/null; then
#	setserial "$port" rs485 on 2>/dev/null
#fi

# open $port for writing on FD 3
exec 3<> "$port" || {
    printf '%s\n' "Failed to open $port for writing" >&2
    exit 1
}

# -F is gnu, use only if known.
#stty -F "$port" $baud cs8 -cstopb -parenb -ixon -ixoff -crtscts
# Configure the serial port
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
