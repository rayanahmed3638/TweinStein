#!/usr/bin/env bash
# Capture UART0 output from MSPM0 LaunchPad to a file.
# Usage:  ./log_serial.sh [output_filename]
# Default output: robot_log.csv
# Stop with Ctrl+C.

OUT="${1:-robot_log.csv}"

# Find the LaunchPad serial device (usually ttyACM0 or ttyACM1)
DEV=""
for d in /dev/ttyACM0 /dev/ttyACM1 /dev/ttyUSB0 /dev/ttyUSB1; do
  if [ -e "$d" ]; then
    DEV="$d"
    break
  fi
done

if [ -z "$DEV" ]; then
  echo "No serial device found. Is the LaunchPad plugged in?"
  echo "Checked: /dev/ttyACM{0,1} /dev/ttyUSB{0,1}"
  exit 1
fi

echo "Using device: $DEV"
echo "Baud: 115200"
echo "Output: $OUT"
echo "Press Ctrl+C to stop."
echo "---"

# Configure the serial port: 115200 baud, 8N1, raw mode (no line processing)
stty -F "$DEV" 115200 cs8 -cstopb -parenb raw -echo -icanon -isig -iexten -opost

# Stream bytes straight to the output file
cat "$DEV" | tee "$OUT"
