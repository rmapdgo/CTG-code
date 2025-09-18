#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Philips Avalon FM20/30 RS232 Data Extractor
-------------------------------------------
Reads maternal & fetal numerics via RS232 (SLIP framing + IntelliVue MIB).

Requirements:
    pip install pyserial
Usage:
    python avalon_rs232.py --port COM5 --baud 115200 --info
    python avalon_rs232.py --port /dev/ttyUSB0 --baud 19200
"""

import sys, time, struct, datetime, argparse, threading
import serial

# =========================
# SLIP utilities
# =========================
SLIP_END = 0xC0
SLIP_ESC = 0xDB
SLIP_ESC_END = 0xDC
SLIP_ESC_ESC = 0xDD

def slip_encode(payload: bytes) -> bytes:
    out = bytearray([SLIP_END])
    for b in payload:
        if b == SLIP_END:
            out.extend([SLIP_ESC, SLIP_ESC_END])
        elif b == SLIP_ESC:
            out.extend([SLIP_ESC, SLIP_ESC_ESC])
        else:
            out.append(b)
    out.append(SLIP_END)
    return bytes(out)

def slip_read_frame(ser: serial.Serial, timeout=5.0) -> bytes:
    ser.timeout = 0.1
    start = time.time()
    buf, in_frame = bytearray(), False

    while True:
        if timeout and (time.time() - start) > timeout:
            raise TimeoutError("Timed out waiting for SLIP frame")
        b = ser.read(1)
        if not b:
            continue
        val = b[0]
        if val == SLIP_END:
            if not in_frame:
                in_frame, buf = True, bytearray()
                continue
            else:
                return bytes(buf)
        elif not in_frame:
            continue
        elif val == SLIP_ESC:
            esc = ser.read(1)
            if not esc: continue
            if esc[0] == SLIP_ESC_END: buf.append(SLIP_END)
            elif esc[0] == SLIP_ESC_ESC: buf.append(SLIP_ESC)
        else:
            buf.append(val)

# =========================
# Avalon RS232 Data Source
# =========================
class AvalonRS232:
    def __init__(self, port, baud=115200, debug_info=False, debug_error=True):
        self.port, self.baud = port, baud
        self.debug_info, self.debug_error = debug_info, debug_error

        # serial
        self.ser = None
        self.session_id = None

        # state
        self.run_loop, self.is_active = False, False

        # numerics
        self.p_id = ""
        self.p_name = ""
        self.p_age = None
        self.p_gender = ""

        self.p_nbp_sys = self.p_nbp_dias = self.p_nbp_mean = 0
        self.p_nbp_pulse = 0
        self.p_spo2 = self.p_spo2_pulse = 0
        self.p_ecg_pulse = 0
        self.p_temp = None

        # Avalon fetal/maternal
        self.p_fhr1 = self.p_fhr2 = self.p_fhr3 = None
        self.p_toco = None
        self.p_mhr = None
        self.p_fmp = None
        self.p_iup = None

    # ========== Transport ==========
    def open_serial(self):
        try:
            self.ser = serial.Serial(
                self.port, self.baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.2
            )
            if self.debug_info: print(f"[OK] Serial opened {self.port}@{self.baud}")
            return True
        except Exception as e:
            if self.debug_error: print(f"[Err] Serial open failed: {e}")
            return False

    def send_apdu(self, payload: bytes):
        self.ser.write(slip_encode(payload))
        self.ser.flush()

    def recv_apdu(self, timeout=5.0) -> bytes:
        return slip_read_frame(self.ser, timeout)

    # ========== APDU builders ==========
    def assoc_request(self):
        return bytearray(b'\x0d\xff\x01\x28\x05\x08\x13\x01\x00\x16\x01\x02\x80\x00'
                         b'\x14\x02\x00\x02\xc1\xff\x01\x16\x31\x80\xa0\x80\x80\x01'
                         b'\x01\x00\x00\xa2\x80\xa0\x03\x00\x00\x01\xa4\x80\x30\x80'
                         b'\x02\x01\x01\x06\x04\x52\x01\x00\x01\x30\x80\x06\x02\x51'
                         b'\x01\x00\x00\x00\x00\x30\x80\x02\x01\x02\x06\x0c\x2a\x86'
                         b'\x48\xce\x14\x02\x01\x00\x00\x00\x01\x01\x30\x80\x06\x0c'
                         b'\x2a\x86\x48\xce\x14\x02\x01\x00\x00\x00\x02\x01\x00\x00'
                         b'\x00\x00\x00\x00\x61\x80\x30\x80\x02\x01\x01\xa0\x80\x60'
                         b'\x80\xa1\x80\x06\x0c\x2a\x86\x48\xce\x14\x02\x01\x00\x00'
                         b'\x00\x03\x01\x00\x00\xbe\x80\x28\x80\x06\x0c\x2a\x86\x48'
                         b'\xce\x14\x02\x01\x00\x00\x00\x01\x01\x02\x01\x02\x81\x82'
                         b'\x00\x80\x80\x00\x00\x00\x40\x00\x00\x00\x00\x00\x00\x00'
                         b'\x80\x00\x00\x00\x20\x00\x00\x00\x00\x00\x00\x00\x00\x02'
                         b'\x00\x64\x00\x01\x00\x28\x80\x00\x00\x00\x00\x00\x0f\xa0'
                         b'\x00\x00\x05\xb0\x00\x00\x05\xb0\xff\xff\xff\xff\x60\x00'
                         b'\x00\x00\x00\x01\x00\x0c\xf0\x01\x00\x08\x8e\x00\x00\x00'
                         b'\x00\x00\x00\x00\x01\x02\x00\x34\x00\x06\x00\x30\x00\x01'
                         b'\x00\x21\x00\x00\x00\x01\x00\x01\x00\x06\x00\x00\x00\xc9'
                         b'\x00\x01\x00\x09\x00\x00\x00\x3c\x00\x01\x00\x05\x00\x00'
                         b'\x00\x10\x00\x01\x00\x2a\x00\x00\x00\x01\x00\x01\x00\x36'
                         b'\x00\x00\x00\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00'
                         b'\x00\x00\x00\x00\x00\x00')

    def release_request(self):
        return bytearray(b'\x09\x18\xC1\x16\x61\x80\x30\x80\x02\x01\x01'
                         b'\xA0\x80\x62\x80\x80\x01\x00\x00\x00\x00\x00'
                         b'\x00\x00\x00\x00')

    # ========== Decoders ==========
    def decode_float(self, b):
        exponent = struct.unpack('!b', b[:1])[0]
        if (b[1] >> 7) == 0:
            mantissa = struct.unpack('!i', b'\x00' + b[1:4])[0]
        else:
            j = struct.unpack('!i', b'\x80' + b[1:4])[0]
            k = ~struct.unpack('!i', b'\x00\x80\x00\x00')[0]
            mantissa = j & k
        return mantissa * (10 ** exponent)

    # map Physio IDs → attributes
    def extract_physio(self, pid, val):
        standard = {
            18949: ("NBP_sys", "p_nbp_sys"),
            18950: ("NBP_dias", "p_nbp_dias"),
            18951: ("NBP_mean", "p_nbp_mean"),
            61669: ("NBP_pulse", "p_nbp_pulse"),
            19384: ("SpO2", "p_spo2"),
            18466: ("SpO2_pulse", "p_spo2_pulse"),
            16770: ("ECG_pulse", "p_ecg_pulse"),
            19272: ("Temp", "p_temp"),
        }
        avalon = {
            150020: ("FHR1", "p_fhr1"),
            150021: ("FHR2", "p_fhr2"),
            150022: ("FHR3", "p_fhr3"),
            150023: ("TOCO", "p_toco"),
            150024: ("MHR", "p_mhr"),
            150025: ("FMP", "p_fmp"),
            150026: ("IUP", "p_iup"),
        }
        if pid in standard:
            setattr(self, standard[pid][1], val)
        elif pid in avalon:
            setattr(self, avalon[pid][1], val)
        else:
            if self.debug_info:
                print(f"[Info] Unknown phys id {pid} -> {val}")

    # ========== Main loop ==========
    def run(self):
        if not self.open_serial(): return
        self.run_loop, self.is_active = True, True

        # Associate
        self.send_apdu(self.assoc_request())
        data = self.recv_apdu(5)
        if not data or data[0] != 0x0E:
            print("[Err] Assoc failed"); return

        # Receive MDS event + ack
        mds = self.recv_apdu(5)
        self.session_id = mds[:4]
        ack = self.session_id + b'\x00\x02\x00\x14' + b'\x00\x01\x00\x01\x00\x0e' \
              + b'\x00\x21\x00\x00\x00\x00' + b'\x00\x00\x00\x32' + b'\x0d\x06\x00\x00'
        self.send_apdu(ack)

        pollcount = 1
        while self.run_loop:
            # Poll request
            req = self.session_id + b'\x00\x01\x00\x20\x00\x01\x00\x07\x00\x1a' \
                  + b'\x00\x21\x00\x00\x00\x00\x00\x00\x00\x00\x0C\x16' \
                  + b'\x00\x0C' + struct.pack('!H', pollcount) + b'\x00\x01' \
                  + b'\x00\x2A\x00\x00\x00\x00\x00\x00'
            self.send_apdu(req)

            try:
                resp = self.recv_apdu(5)
            except Exception:
                continue

            if not resp: continue
            # quick parse for simple numerics in compound obj (0x2379 / 0x2384)
            if b'\x09\x6b' in resp or b'\x09\x50' in resp:
                for off in range(0, len(resp)-12):
                    try:
                        objid = struct.unpack('!H', resp[off:off+2])[0]
                        if objid in (2384, 2379):  # observation
                            pid = struct.unpack('!H', resp[off+4:off+6])[0]
                            val = self.decode_float(resp[off+10:off+14])
                            self.extract_physio(pid, val)
                    except Exception:
                        continue

            # Print snapshot
            now = datetime.datetime.now().strftime("%H:%M:%S")
            print(f"[{now}] "
                  f"FHR1={self.p_fhr1} FHR2={self.p_fhr2} "
                  f"TOCO={self.p_toco} MHR={self.p_mhr} "
                  f"NBP={self.p_nbp_sys}/{self.p_nbp_dias} "
                  f"SpO2={self.p_spo2} HR={self.p_ecg_pulse}")

            pollcount += 1
            time.sleep(2)

        # Release
        self.send_apdu(self.release_request())
        self.ser.close()
        self.is_active = False

# =========================
# Main entry
# =========================
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--info", action="store_true")
    args = ap.parse_args()

    avalon = AvalonRS232(args.port, args.baud, debug_info=args.info)
    try:
        avalon.run()
    except KeyboardInterrupt:
        print("Stopping…")
        avalon.run_loop = False

if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Philips Avalon FM20/30 RS232 Data Extractor
-------------------------------------------
Reads maternal & fetal numerics via RS232 (SLIP framing + IntelliVue MIB).

Requirements:
    pip install pyserial
Usage:
    python avalon_rs232.py --port COM5 --baud 115200 --info
    python avalon_rs232.py --port /dev/ttyUSB0 --baud 19200
"""

import sys, time, struct, datetime, argparse, threading
import serial

# =========================
# SLIP utilities
# =========================
SLIP_END = 0xC0
SLIP_ESC = 0xDB
SLIP_ESC_END = 0xDC
SLIP_ESC_ESC = 0xDD

def slip_encode(payload: bytes) -> bytes:
    out = bytearray([SLIP_END])
    for b in payload:
        if b == SLIP_END:
            out.extend([SLIP_ESC, SLIP_ESC_END])
        elif b == SLIP_ESC:
            out.extend([SLIP_ESC, SLIP_ESC_ESC])
        else:
            out.append(b)
    out.append(SLIP_END)
    return bytes(out)

def slip_read_frame(ser: serial.Serial, timeout=30.0) -> bytes:
    ser.timeout = 0.1
    start = time.time()
    buf, in_frame = bytearray(), False

    while True:
        if timeout and (time.time() - start) > timeout:
            raise TimeoutError("Timed out waiting for SLIP frame")
        b = ser.read(1)
        if not b:
            continue
        val = b[0]
        if val == SLIP_END:
            if not in_frame:
                in_frame, buf = True, bytearray()
                continue
            else:
                return bytes(buf)
        elif not in_frame:
            continue
        elif val == SLIP_ESC:
            esc = ser.read(1)
            if not esc: continue
            if esc[0] == SLIP_ESC_END: buf.append(SLIP_END)
            elif esc[0] == SLIP_ESC_ESC: buf.append(SLIP_ESC)
        else:
            buf.append(val)

# =========================
# Avalon RS232 Data Source
# =========================
class AvalonRS232:
    def __init__(self, port, baud=115200, debug_info=False, debug_error=True):
        self.port, self.baud = port, baud
        self.debug_info, self.debug_error = debug_info, debug_error

        # serial
        self.ser = None
        self.session_id = None

        # state
        self.run_loop, self.is_active = False, False

        # numerics
        self.p_id = ""
        self.p_name = ""
        self.p_age = None
        self.p_gender = ""

        self.p_nbp_sys = self.p_nbp_dias = self.p_nbp_mean = 0
        self.p_nbp_pulse = 0
        self.p_spo2 = self.p_spo2_pulse = 0
        self.p_ecg_pulse = 0
        self.p_temp = None

        # Avalon fetal/maternal
        self.p_fhr1 = self.p_fhr2 = self.p_fhr3 = None
        self.p_toco = None
        self.p_mhr = None
        self.p_fmp = None
        self.p_iup = None

    # ========== Transport ==========
    def open_serial(self):
        try:
            self.ser = serial.Serial(
                self.port, self.baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.2
            )
            if self.debug_info: print(f"[OK] Serial opened {self.port}@{self.baud}")
            return True
        except Exception as e:
            if self.debug_error: print(f"[Err] Serial open failed: {e}")
            return False

    def send_apdu(self, payload: bytes):
        self.ser.write(slip_encode(payload))
        self.ser.flush()

    def recv_apdu(self, timeout=5.0) -> bytes:
        return slip_read_frame(self.ser, timeout)

    # ========== APDU builders ==========
    def assoc_request(self):
        return bytearray(b'\x0d\xff\x01\x28\x05\x08\x13\x01\x00\x16\x01\x02\x80\x00'
                         b'\x14\x02\x00\x02\xc1\xff\x01\x16\x31\x80\xa0\x80\x80\x01'
                         b'\x01\x00\x00\xa2\x80\xa0\x03\x00\x00\x01\xa4\x80\x30\x80'
                         b'\x02\x01\x01\x06\x04\x52\x01\x00\x01\x30\x80\x06\x02\x51'
                         b'\x01\x00\x00\x00\x00\x30\x80\x02\x01\x02\x06\x0c\x2a\x86'
                         b'\x48\xce\x14\x02\x01\x00\x00\x00\x01\x01\x30\x80\x06\x0c'
                         b'\x2a\x86\x48\xce\x14\x02\x01\x00\x00\x00\x02\x01\x00\x00'
                         b'\x00\x00\x00\x00\x61\x80\x30\x80\x02\x01\x01\xa0\x80\x60'
                         b'\x80\xa1\x80\x06\x0c\x2a\x86\x48\xce\x14\x02\x01\x00\x00'
                         b'\x00\x03\x01\x00\x00\xbe\x80\x28\x80\x06\x0c\x2a\x86\x48'
                         b'\xce\x14\x02\x01\x00\x00\x00\x01\x01\x02\x01\x02\x81\x82'
                         b'\x00\x80\x80\x00\x00\x00\x40\x00\x00\x00\x00\x00\x00\x00'
                         b'\x80\x00\x00\x00\x20\x00\x00\x00\x00\x00\x00\x00\x00\x02'
                         b'\x00\x64\x00\x01\x00\x28\x80\x00\x00\x00\x00\x00\x0f\xa0'
                         b'\x00\x00\x05\xb0\x00\x00\x05\xb0\xff\xff\xff\xff\x60\x00'
                         b'\x00\x00\x00\x01\x00\x0c\xf0\x01\x00\x08\x8e\x00\x00\x00'
                         b'\x00\x00\x00\x00\x01\x02\x00\x34\x00\x06\x00\x30\x00\x01'
                         b'\x00\x21\x00\x00\x00\x01\x00\x01\x00\x06\x00\x00\x00\xc9'
                         b'\x00\x01\x00\x09\x00\x00\x00\x3c\x00\x01\x00\x05\x00\x00'
                         b'\x00\x10\x00\x01\x00\x2a\x00\x00\x00\x01\x00\x01\x00\x36'
                         b'\x00\x00\x00\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00'
                         b'\x00\x00\x00\x00\x00\x00')

    def release_request(self):
        return bytearray(b'\x09\x18\xC1\x16\x61\x80\x30\x80\x02\x01\x01'
                         b'\xA0\x80\x62\x80\x80\x01\x00\x00\x00\x00\x00'
                         b'\x00\x00\x00\x00')

    # ========== Decoders ==========
    def decode_float(self, b):
        exponent = struct.unpack('!b', b[:1])[0]
        if (b[1] >> 7) == 0:
            mantissa = struct.unpack('!i', b'\x00' + b[1:4])[0]
        else:
            j = struct.unpack('!i', b'\x80' + b[1:4])[0]
            k = ~struct.unpack('!i', b'\x00\x80\x00\x00')[0]
            mantissa = j & k
        return mantissa * (10 ** exponent)

    # map Physio IDs → attributes
    def extract_physio(self, pid, val):
        standard = {
            18949: ("NBP_sys", "p_nbp_sys"),
            18950: ("NBP_dias", "p_nbp_dias"),
            18951: ("NBP_mean", "p_nbp_mean"),
            61669: ("NBP_pulse", "p_nbp_pulse"),
            19384: ("SpO2", "p_spo2"),
            18466: ("SpO2_pulse", "p_spo2_pulse"),
            16770: ("ECG_pulse", "p_ecg_pulse"),
            19272: ("Temp", "p_temp"),
        }
        avalon = {
            150020: ("FHR1", "p_fhr1"),
            150021: ("FHR2", "p_fhr2"),
            150022: ("FHR3", "p_fhr3"),
            150023: ("TOCO", "p_toco"),
            150024: ("MHR", "p_mhr"),
            150025: ("FMP", "p_fmp"),
            150026: ("IUP", "p_iup"),
        }
        if pid in standard:
            setattr(self, standard[pid][1], val)
        elif pid in avalon:
            setattr(self, avalon[pid][1], val)
        else:
            if self.debug_info:
                print(f"[Info] Unknown phys id {pid} -> {val}")

    # ========== Main loop ==========
    def run(self):
        if not self.open_serial(): return
        self.run_loop, self.is_active = True, True

        # Associate
        self.send_apdu(self.assoc_request())
        data = self.recv_apdu(5)
        if not data or data[0] != 0x0E:
            print("[Err] Assoc failed"); return

        # Receive MDS event + ack
        mds = self.recv_apdu(5)
        self.session_id = mds[:4]
        ack = self.session_id + b'\x00\x02\x00\x14' + b'\x00\x01\x00\x01\x00\x0e' \
              + b'\x00\x21\x00\x00\x00\x00' + b'\x00\x00\x00\x32' + b'\x0d\x06\x00\x00'
        self.send_apdu(ack)

        pollcount = 1
        while self.run_loop:
            # Poll request
            req = self.session_id + b'\x00\x01\x00\x20\x00\x01\x00\x07\x00\x1a' \
                  + b'\x00\x21\x00\x00\x00\x00\x00\x00\x00\x00\x0C\x16' \
                  + b'\x00\x0C' + struct.pack('!H', pollcount) + b'\x00\x01' \
                  + b'\x00\x2A\x00\x00\x00\x00\x00\x00'
            self.send_apdu(req)

            try:
                resp = self.recv_apdu(5)
            except Exception:
                continue

            if not resp: continue
            # quick parse for simple numerics in compound obj (0x2379 / 0x2384)
            if b'\x09\x6b' in resp or b'\x09\x50' in resp:
                for off in range(0, len(resp)-12):
                    try:
                        objid = struct.unpack('!H', resp[off:off+2])[0]
                        if objid in (2384, 2379):  # observation
                            pid = struct.unpack('!H', resp[off+4:off+6])[0]
                            val = self.decode_float(resp[off+10:off+14])
                            self.extract_physio(pid, val)
                    except Exception:
                        continue

            # Print snapshot
            now = datetime.datetime.now().strftime("%H:%M:%S")
            print(f"[{now}] "
                  f"FHR1={self.p_fhr1} FHR2={self.p_fhr2} "
                  f"TOCO={self.p_toco} MHR={self.p_mhr} "
                  f"NBP={self.p_nbp_sys}/{self.p_nbp_dias} "
                  f"SpO2={self.p_spo2} HR={self.p_ecg_pulse}")

            pollcount += 1
            time.sleep(2)

        # Release
        self.send_apdu(self.release_request())
        self.ser.close()
        self.is_active = False

# =========================
# Main entry
# =========================
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--info", action="store_true")
    args = ap.parse_args()

    avalon = AvalonRS232(args.port, args.baud, debug_info=args.info)
    try:
        avalon.run()
    except KeyboardInterrupt:
        print("Stopping…")
        avalon.run_loop = False

if __name__ == "__main__":
    main()
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Philips Avalon FM20/30 RS232 Data Extractor
-------------------------------------------
Reads maternal & fetal numerics via RS232 (SLIP framing + IntelliVue MIB).

Requirements:
    pip install pyserial
Usage:
    python avalon_rs232.py --port COM5 --baud 115200 --info
    python avalon_rs232.py --port /dev/ttyUSB0 --baud 19200
"""

import sys, time, struct, datetime, argparse, threading
import serial

# =========================
# SLIP utilities
# =========================
SLIP_END = 0xC0
SLIP_ESC = 0xDB
SLIP_ESC_END = 0xDC
SLIP_ESC_ESC = 0xDD

def slip_encode(payload: bytes) -> bytes:
    out = bytearray([SLIP_END])
    for b in payload:
        if b == SLIP_END:
            out.extend([SLIP_ESC, SLIP_ESC_END])
        elif b == SLIP_ESC:
            out.extend([SLIP_ESC, SLIP_ESC_ESC])
        else:
            out.append(b)
    out.append(SLIP_END)
    return bytes(out)

def slip_read_frame(ser: serial.Serial, timeout=5.0) -> bytes:
    ser.timeout = 0.1
    start = time.time()
    buf, in_frame = bytearray(), False

    while True:
        if timeout and (time.time() - start) > timeout:
            raise TimeoutError("Timed out waiting for SLIP frame")
        b = ser.read(1)
        if not b:
            continue
        val = b[0]
        if val == SLIP_END:
            if not in_frame:
                in_frame, buf = True, bytearray()
                continue
            else:
                return bytes(buf)
        elif not in_frame:
            continue
        elif val == SLIP_ESC:
            esc = ser.read(1)
            if not esc: continue
            if esc[0] == SLIP_ESC_END: buf.append(SLIP_END)
            elif esc[0] == SLIP_ESC_ESC: buf.append(SLIP_ESC)
        else:
            buf.append(val)

# =========================
# Avalon RS232 Data Source
# =========================
class AvalonRS232:
    def __init__(self, port, baud=115200, debug_info=False, debug_error=True):
        self.port, self.baud = port, baud
        self.debug_info, self.debug_error = debug_info, debug_error

        # serial
        self.ser = None
        self.session_id = None

        # state
        self.run_loop, self.is_active = False, False

        # numerics
        self.p_id = ""
        self.p_name = ""
        self.p_age = None
        self.p_gender = ""

        self.p_nbp_sys = self.p_nbp_dias = self.p_nbp_mean = 0
        self.p_nbp_pulse = 0
        self.p_spo2 = self.p_spo2_pulse = 0
        self.p_ecg_pulse = 0
        self.p_temp = None

        # Avalon fetal/maternal
        self.p_fhr1 = self.p_fhr2 = self.p_fhr3 = None
        self.p_toco = None
        self.p_mhr = None
        self.p_fmp = None
        self.p_iup = None

    # ========== Transport ==========
    def open_serial(self):
        try:
            self.ser = serial.Serial(
                self.port, self.baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.2
            )
            if self.debug_info: print(f"[OK] Serial opened {self.port}@{self.baud}")
            return True
        except Exception as e:
            if self.debug_error: print(f"[Err] Serial open failed: {e}")
            return False

    def send_apdu(self, payload: bytes):
        self.ser.write(slip_encode(payload))
        self.ser.flush()

    def recv_apdu(self, timeout=5.0) -> bytes:
        return slip_read_frame(self.ser, timeout)

    # ========== APDU builders ==========
    def assoc_request(self):
        return bytearray(b'\x0d\xff\x01\x28\x05\x08\x13\x01\x00\x16\x01\x02\x80\x00'
                         b'\x14\x02\x00\x02\xc1\xff\x01\x16\x31\x80\xa0\x80\x80\x01'
                         b'\x01\x00\x00\xa2\x80\xa0\x03\x00\x00\x01\xa4\x80\x30\x80'
                         b'\x02\x01\x01\x06\x04\x52\x01\x00\x01\x30\x80\x06\x02\x51'
                         b'\x01\x00\x00\x00\x00\x30\x80\x02\x01\x02\x06\x0c\x2a\x86'
                         b'\x48\xce\x14\x02\x01\x00\x00\x00\x01\x01\x30\x80\x06\x0c'
                         b'\x2a\x86\x48\xce\x14\x02\x01\x00\x00\x00\x02\x01\x00\x00'
                         b'\x00\x00\x00\x00\x61\x80\x30\x80\x02\x01\x01\xa0\x80\x60'
                         b'\x80\xa1\x80\x06\x0c\x2a\x86\x48\xce\x14\x02\x01\x00\x00'
                         b'\x00\x03\x01\x00\x00\xbe\x80\x28\x80\x06\x0c\x2a\x86\x48'
                         b'\xce\x14\x02\x01\x00\x00\x00\x01\x01\x02\x01\x02\x81\x82'
                         b'\x00\x80\x80\x00\x00\x00\x40\x00\x00\x00\x00\x00\x00\x00'
                         b'\x80\x00\x00\x00\x20\x00\x00\x00\x00\x00\x00\x00\x00\x02'
                         b'\x00\x64\x00\x01\x00\x28\x80\x00\x00\x00\x00\x00\x0f\xa0'
                         b'\x00\x00\x05\xb0\x00\x00\x05\xb0\xff\xff\xff\xff\x60\x00'
                         b'\x00\x00\x00\x01\x00\x0c\xf0\x01\x00\x08\x8e\x00\x00\x00'
                         b'\x00\x00\x00\x00\x01\x02\x00\x34\x00\x06\x00\x30\x00\x01'
                         b'\x00\x21\x00\x00\x00\x01\x00\x01\x00\x06\x00\x00\x00\xc9'
                         b'\x00\x01\x00\x09\x00\x00\x00\x3c\x00\x01\x00\x05\x00\x00'
                         b'\x00\x10\x00\x01\x00\x2a\x00\x00\x00\x01\x00\x01\x00\x36'
                         b'\x00\x00\x00\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00'
                         b'\x00\x00\x00\x00\x00\x00')

    def release_request(self):
        return bytearray(b'\x09\x18\xC1\x16\x61\x80\x30\x80\x02\x01\x01'
                         b'\xA0\x80\x62\x80\x80\x01\x00\x00\x00\x00\x00'
                         b'\x00\x00\x00\x00')

    # ========== Decoders ==========
    def decode_float(self, b):
        exponent = struct.unpack('!b', b[:1])[0]
        if (b[1] >> 7) == 0:
            mantissa = struct.unpack('!i', b'\x00' + b[1:4])[0]
        else:
            j = struct.unpack('!i', b'\x80' + b[1:4])[0]
            k = ~struct.unpack('!i', b'\x00\x80\x00\x00')[0]
            mantissa = j & k
        return mantissa * (10 ** exponent)

    # map Physio IDs → attributes
    def extract_physio(self, pid, val):
        standard = {
            18949: ("NBP_sys", "p_nbp_sys"),
            18950: ("NBP_dias", "p_nbp_dias"),
            18951: ("NBP_mean", "p_nbp_mean"),
            61669: ("NBP_pulse", "p_nbp_pulse"),
            19384: ("SpO2", "p_spo2"),
            18466: ("SpO2_pulse", "p_spo2_pulse"),
            16770: ("ECG_pulse", "p_ecg_pulse"),
            19272: ("Temp", "p_temp"),
        }
        avalon = {
            150020: ("FHR1", "p_fhr1"),
            150021: ("FHR2", "p_fhr2"),
            150022: ("FHR3", "p_fhr3"),
            150023: ("TOCO", "p_toco"),
            150024: ("MHR", "p_mhr"),
            150025: ("FMP", "p_fmp"),
            150026: ("IUP", "p_iup"),
        }
        if pid in standard:
            setattr(self, standard[pid][1], val)
        elif pid in avalon:
            setattr(self, avalon[pid][1], val)
        else:
            if self.debug_info:
                print(f"[Info] Unknown phys id {pid} -> {val}")

    # ========== Main loop ==========
    def run(self):
        if not self.open_serial(): return
        self.run_loop, self.is_active = True, True

        # Associate
        self.send_apdu(self.assoc_request())
        data = self.recv_apdu(5)
        if not data or data[0] != 0x0E:
            print("[Err] Assoc failed"); return

        # Receive MDS event + ack
        mds = self.recv_apdu(5)
        self.session_id = mds[:4]
        ack = self.session_id + b'\x00\x02\x00\x14' + b'\x00\x01\x00\x01\x00\x0e' \
              + b'\x00\x21\x00\x00\x00\x00' + b'\x00\x00\x00\x32' + b'\x0d\x06\x00\x00'
        self.send_apdu(ack)

        pollcount = 1
        while self.run_loop:
            # Poll request
            req = self.session_id + b'\x00\x01\x00\x20\x00\x01\x00\x07\x00\x1a' \
                  + b'\x00\x21\x00\x00\x00\x00\x00\x00\x00\x00\x0C\x16' \
                  + b'\x00\x0C' + struct.pack('!H', pollcount) + b'\x00\x01' \
                  + b'\x00\x2A\x00\x00\x00\x00\x00\x00'
            self.send_apdu(req)

            try:
                resp = self.recv_apdu(5)
            except Exception:
                continue

            if not resp: continue
            # quick parse for simple numerics in compound obj (0x2379 / 0x2384)
            if b'\x09\x6b' in resp or b'\x09\x50' in resp:
                for off in range(0, len(resp)-12):
                    try:
                        objid = struct.unpack('!H', resp[off:off+2])[0]
                        if objid in (2384, 2379):  # observation
                            pid = struct.unpack('!H', resp[off+4:off+6])[0]
                            val = self.decode_float(resp[off+10:off+14])
                            self.extract_physio(pid, val)
                    except Exception:
                        continue

            # Print snapshot
            now = datetime.datetime.now().strftime("%H:%M:%S")
            print(f"[{now}] "
                  f"FHR1={self.p_fhr1} FHR2={self.p_fhr2} "
                  f"TOCO={self.p_toco} MHR={self.p_mhr} "
                  f"NBP={self.p_nbp_sys}/{self.p_nbp_dias} "
                  f"SpO2={self.p_spo2} HR={self.p_ecg_pulse}")

            pollcount += 1
            time.sleep(2)

        # Release
        self.send_apdu(self.release_request())
        self.ser.close()
        self.is_active = False

# =========================
# Main entry
# =========================
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="Serial port, e.g. COM7 or /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200, help="Baud rate, usually 115200")
    ap.add_argument("--info", action="store_true", help="Enable debug logging of unknown IDs")
    args = ap.parse_args()

    avalon = AvalonRS232(args.port, args.baud, debug_info=args.info)
    try:
        avalon.run()
    except KeyboardInterrupt:
        print("Stopping…")
        avalon.run_loop = False


if __name__ == "__main__":
    main()
