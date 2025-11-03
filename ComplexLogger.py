import argparse
import serial
import time
import csv
from typing import List, Optional

def to_float(tok: str) -> Optional[float]:
    tok = tok.strip()
    if tok == "" or tok.lower() in ("nan", "inf", "+inf", "-inf"):
        return None
    try:
        return float(tok)
    except ValueError:
        return None

def expand(lst: List, n: int, fill):
    if len(lst) < n:
        lst.extend([fill] * (n - len(lst)))

parser = argparse.ArgumentParser(description="Log serial lines to a CSV file.")
parser.add_argument("csvfile", nargs="?", default="experiment2.csv", help="output CSV filename")
parser.add_argument("--port", default="COM7", help="serial port (e.g. COM13 or /dev/ttyUSB0)")
parser.add_argument("--baud", type=int, default=115200, help="baud rate")
parser.add_argument("--stop-token", default="STOP", help='line that stops logging (default "STOP")')
parser.add_argument("--append", action="store_true", help="append to CSV instead of overwrite")
parser.add_argument("--delimiter", default=",", help='CSV delimiter (default ",")')
args = parser.parse_args()

ser = serial.Serial(args.port, args.baud, timeout=1)
time.sleep(2)

mode = "a" if args.append else "w"
header: Optional[List[str]] = None
best_vals: List[Optional[float]] = []  # value with largest |value| per column
saw_data_row = False

def print_extrema():
    if not best_vals:
        print("No data logged before STOP.")
        return
    labels = header if header and len(header) >= len(best_vals) else [f"col{i+1}" for i in range(len(best_vals))]
    print("\nMax magnitude per column (signed):")
    for i, v in enumerate(best_vals):
        out = "n/a" if v is None else f"{v:.6g}"
        print(f"{labels[i]}: {out}")

try:
    with open(args.csvfile, mode, newline="") as f:
        writer = csv.writer(f, delimiter=args.delimiter)
        while True:
            try:
                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="ignore").strip()
                if not line:
                    continue

                if line.strip() == args.stop_token:
                    print("[STOP] token received. Halting logging.")
                    break

                fields = [t.strip() for t in line.split(args.delimiter)]

                if not saw_data_row:
                    nums = [to_float(t) for t in fields]
                    if any(v is None for v in nums):
                        header = fields[:]
                        if not args.append:
                            writer.writerow(header)
                        print(f"[header] {args.delimiter.join(header)}")
                        continue
                    else:
                        saw_data_row = True

                expand(best_vals, len(fields), None)

                for i, tok in enumerate(fields):
                    v = to_float(tok)
                    if v is None:
                        continue
                    if best_vals[i] is None or abs(v) > abs(best_vals[i]):
                        best_vals[i] = v

                writer.writerow(fields)
                f.flush()
                print(line)

            except KeyboardInterrupt:
                print("\n[INTERRUPT] Halting logging.")
                break
            except Exception as e:
                print(f"[warn] skipped line due to error: {e}")
                continue
finally:
    try:
        ser.close()
    except Exception:
        pass
    print_extrema()
