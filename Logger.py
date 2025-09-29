import serial, time, csv

port = "COM13"   # or /dev/ttyUSB0 on Linux / Mac
baud = 115200

ser = serial.Serial(port, baud, timeout=1)
time.sleep(2)  # let Arduino reset

with open("experiment2.csv", "w", newline="") as f:
    writer = csv.writer(f)
    while True:
        try:
            line = ser.readline().decode("utf-8").strip()
            if line:
                writer.writerow(line.split(","))
                print(line)  # optional: show live
        except KeyboardInterrupt:
            break
