# file: pendulum_cli.py
import sys, os, time, csv, threading, argparse
from datetime import datetime

try:
    import serial
except ImportError:
    print("Install pyserial: pip install pyserial")
    sys.exit(1)

DEFAULT_PORT = "COM5"
DEFAULT_BAUD = 115200

HELP_TEXT = """
Commands:
  help                           Show this help
  port <name>                    Set port (COM5, /dev/ttyACM0, etc.)
  baud <rate>                    Set baud (default 115200)
  connect                        Open serial
  disconnect                     Close serial

  send <raw line>                Send raw line to device
  status                         STATUS
  home                           HOME
  zeroenc                        ZEROENC

  set vel <steps_s>              SET VEL
  set accel <steps_s2>           SET ACCEL
  set spmm <steps_per_mm>        SET STEPSPERMM
  set sample <ms>                SET SAMPLE

  autorec on|off                 AUTOREC ON|OFF
  reset on|off                   RESET ON/OFF

  capture on|off                 Manual CAPTURE ON/OFF (rarely needed)
  move rel <mm>                  Trapezoid relative move
  qmove rel <mm> <T_ms|AUTO>     Quintic relative move

  dump                           DUMP now
  clear                          CLEAR buffer
  waitdump [timeout_s]           Wait until a dump finishes

  quit / exit                    Leave program

Notes:
  - Firmware emits '#MOVE TYPE=... DIST_MM=... T_MS=...' before a move.
  - CSV file name: <type>_<distance_mm>_<YYYYMMDD_HHMMSS>.csv
  - The file includes data from just before the move (capture starts at command)
    to 10 s after motion stops (post-hold), then it is dumped.
"""

class SessionNamer:
    """Tracks the next dump's filename based on move metadata."""
    def __init__(self):
        self.pending_type = None
        self.pending_dist = None

    def set(self, mtype: str, dist_str: str):
        self.pending_type = mtype.lower()
        self.pending_dist = dist_str

    def next_filename(self):
        tstamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        mt = self.pending_type or "unknown"
        dd = self.pending_dist or "NA"
        # sanitize distance string for filename
        dd = dd.replace(" ", "").replace("+", "").replace("/", "_")
        name = f"{mt}_{dd}_{tstamp}.csv"
        # clear after use
        self.pending_type = None
        self.pending_dist = None
        return os.path.abspath(name)

class SerialCLI:
    def __init__(self, port, baud):
        self.port = port
        self.baud = baud
        self.ser = None
        self.stop_reader = threading.Event()
        self.reader_thread = None
        self.dump_event = threading.Event()
        self._in_dump = False
        self._dump_lines = []
        self._session = SessionNamer()

    # ---------- serial ----------
    def connect(self):
        if self.ser and self.ser.is_open:
            return "already connected"
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            time.sleep(1.5)  # allow reset
        except Exception as e:
            return f"connect error: {e}"
        self.stop_reader.clear()
        self.reader_thread = threading.Thread(target=self._reader, daemon=True)
        self.reader_thread.start()
        return f"connected {self.port} @ {self.baud}"

    def disconnect(self):
        self.stop_reader.set()
        if self.reader_thread:
            self.reader_thread.join(timeout=1.0)
        if self.ser:
            try: self.ser.close()
            except: pass
        self.ser = None
        return "disconnected"

    def sendline(self, s):
        if not self.ser or not self.ser.is_open:
            return "not connected"
        try:
            self.ser.write((s.strip() + "\n").encode())
            return None
        except Exception as e:
            return f"send error: {e}"

    # ---------- reader ----------
    def _reader(self):
        while not self.stop_reader.is_set():
            try:
                line = self.ser.readline().decode(errors="ignore").strip()
            except Exception:
                break
            if not line:
                continue
            self._handle_line(line)

    def _handle_line(self, line):
        # Move metadata
        if line.startswith("#MOVE"):
            # format: #MOVE TYPE=<TRAP|QUINTIC|RESET> DIST_MM=<val> T_MS=<...>
            parts = line.split()
            mtype = None
            dist  = None
            for p in parts:
                if p.startswith("TYPE="):
                    mtype = p.split("=",1)[1]
                elif p.startswith("DIST_MM="):
                    dist = p.split("=",1)[1]
            if mtype and dist:
                self._session.set(mtype, dist)
            print(line)
            return

        # Dump block
        if line.startswith("#BEGIN DUMP"):
            self._in_dump = True
            self._dump_lines = []
            self._dump_lines.append("time_ms,pos_step,angle_deg,target_step")  # ensure header
            # choose filename now
            self.current_csv = self._session.next_filename()
            return
        if self._in_dump:
            if line.startswith("#END DUMP"):
                # write CSV
                rows = list(self._dump_lines)
                n = self._write_csv(self.current_csv, rows)
                print(f"[dump] {n} rows -> {self.current_csv}")
                self._in_dump = False
                self._dump_lines = []
                self.dump_event.set()
            else:
                # either header or data row
                if "," in line:
                    self._dump_lines.append(line)
            return

        # Normal console echo
        print(line)

    def _write_csv(self, path, rows):
        n = 0
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            for i, line in enumerate(rows):
                parts = line.split(",")
                if i == 0:
                    w.writerow(parts)
                    continue
                if len(parts) == 4:
                    w.writerow(parts)
                    n += 1
        return n

    # ---------- REPL ----------
    def repl(self):
        print(HELP_TEXT)
        while True:
            try:
                cmd = input("> ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                cmd = "quit"
            if not cmd:
                continue
            parts = cmd.split()
            op = parts[0].lower()

            if op in ("quit","exit"):
                print(self.disconnect())
                break

            elif op == "help":
                print(HELP_TEXT)

            elif op == "port" and len(parts) >= 2:
                self.port = parts[1]
                print(f"port -> {self.port}")

            elif op == "baud" and len(parts) >= 2:
                try: self.baud = int(parts[1]); print(f"baud -> {self.baud}")
                except: print("bad baud")

            elif op == "connect":
                print(self.connect())

            elif op == "disconnect":
                print(self.disconnect())

            elif op == "send" and len(parts) >= 2:
                line = cmd[len("send "):]
                err = self.sendline(line)
                if err: print(err)

            elif op == "status":
                err = self.sendline("STATUS");  print(err) if err else None

            elif op == "home":
                err = self.sendline("HOME");    print(err) if err else None

            elif op == "zeroenc":
                err = self.sendline("ZEROENC"); print(err) if err else None

            elif op == "set" and len(parts) >= 3:
                key = parts[1].lower(); val = parts[2]
                if   key == "vel":    err = self.sendline(f"SET VEL {val}")
                elif key == "accel":  err = self.sendline(f"SET ACCEL {val}")
                elif key in ("spmm","stepspermm"): err = self.sendline(f"SET STEPSPERMM {val}")
                elif key == "sample": err = self.sendline(f"SET SAMPLE {val}")
                else: print("unknown set key"); continue
                if err: print(err)

            elif op == "autorec" and len(parts) >= 2:
                onoff = parts[1].upper()
                if onoff not in ("ON","OFF"): print("usage: autorec on|off")
                else:
                    err = self.sendline(f"AUTOREC {onoff}")
                    if err: print(err)

            elif op == "reset" and len(parts) >= 2:
                onoff = parts[1].upper()
                if onoff not in ("ON","OFF"): print("usage: reset on|off")
                else:
                    err = self.sendline(f"RESET {onoff}")
                    if err: print(err)

            elif op == "capture" and len(parts) >= 2:
                onoff = parts[1].upper()
                if onoff not in ("ON","OFF"): print("usage: capture on|off")
                else:
                    self.dump_event.clear()
                    err = self.sendline(f"CAPTURE {onoff}")
                    if err: print(err)

            elif op == "move":
                if len(parts) < 3 or parts[1].lower() != "rel":
                    print("usage: move rel <mm>")
                else:
                    mm = parts[2]
                    # Set filename hint in case firmware banner is missed
                    self._session.set("TRAP", mm)
                    err = self.sendline(f"MOVE REL {mm}")
                    if err: print(err)

            elif op == "qmove":
                # allow: qmove <mm> <T> or qmove rel <mm> <T>
                if len(parts) == 3:
                    mm, T = parts[1], parts[2].upper()
                elif len(parts) >= 4 and parts[1].lower() == "rel":
                    mm, T = parts[2], parts[3].upper()
                else:
                    print("usage: qmove rel <mm> <T_ms|AUTO>"); continue
                if T != "AUTO":
                    try: int(T)
                    except: print("T must be integer ms or AUTO"); continue
                self._session.set("QUINTIC", mm)
                err = self.sendline(f"QMOVE REL {mm} {T}")
                if err: print(err)

            elif op == "dump":
                err = self.sendline("DUMP")
                if err: print(err)

            elif op == "clear":
                err = self.sendline("CLEAR")
                if err: print(err)

            elif op == "waitdump":
                timeout = float(parts[1]) if len(parts) >= 2 else None
                print("waiting for dump...")
                ok = self.dump_event.wait(timeout=timeout)
                print("dump complete" if ok else "timeout")

            else:
                print("unknown command. type 'help'.")

def main():
    ap = argparse.ArgumentParser(add_help=False)
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    args, _ = ap.parse_known_args()

    cli = SerialCLI(args.port, args.baud)
    print(f"Default port: {cli.port} @ {cli.baud}")
    cli.repl()

if __name__ == "__main__":
    main()
