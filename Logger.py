import serial, time, csv

port = "COM11"   # or /dev/ttyUSB0 on Linux / Mac
baud = 115200

ser = serial.Serial(port, baud, timeout=1)
time.sleep(2)  # let Arduino reset

with open("log.csv", "w", newline="") as f:
    writer = csv.writer(f)
    while True:
        try:
            line = ser.readline().decode("utf-8").strip()
            if line:
                writer.writerow(line.split(","))
                print(line)  # optional: show live
        except KeyboardInterrupt:
            break
