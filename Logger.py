import argparse
import serial
import time
import csv

parser = argparse.ArgumentParser(description="Log serial lines to a CSV file.")
parser.add_argument("csvfile", nargs="?", default="experiment2.csv", help="output CSV filename")
parser.add_argument("--port", default="COM7", help="serial port (e.g. COM13 or /dev/ttyUSB0)")
parser.add_argument("--baud", type=int, default=115200, help="baud rate")
args = parser.parse_args()

ser = serial.Serial(args.port, args.baud, timeout=1)
time.sleep(2)  # let Arduino reset


try:
    with open(args.csvfile, "w", newline="") as f:
        writer = csv.writer(f)
        while True:
            try:
                line = ser.readline().decode("utf-8").strip()
                if line:
                    writer.writerow(line.split(","))
                    print(line)
            except KeyboardInterrupt:
                break
            except Exception:
                # ignore decode/split errors and continue
                continue
finally:
    ser.close()
