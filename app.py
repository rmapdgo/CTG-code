import serial
import serial.tools.list_ports
import time
from datetime import datetime
from typing import Optional, Dict
import threading
import pandas as pd
from dash import Dash, html, dcc, dash_table, Input, Output, State, callback_context
import dash

# ---------- Constants (packet state) ----------
AVALONFM30_PACKETSTATE_IDLE = 0
AVALONFM30_PACKETSTATE_PROTOCOLID = 1
AVALONFM30_PACKETSTATE_MESSAGETYPE = 2
AVALONFM30_PACKETSTATE_MESSAGELENGTH_MSB = 3
AVALONFM30_PACKETSTATE_MESSAGELENGTH_LSB = 4
AVALONFM30_PACKETSTATE_MESSAGEPAYLOAD_FIRSTBYTE = 5
AVALONFM30_PACKETSTATE_MESSAGEPAYLOAD_NORMAL = 6
AVALONFM30_PACKETSTATE_MESSAGEPAYLOAD_POSTESC = 7
AVALONFM30_PACKETSTATE_CRC_LSB = 8
AVALONFM30_PACKETSTATE_CRC_MSB = 9
AVALONFM30_PACKETSTATE_EOF = 10

# Global variables
ser = None
collecting = False
serial_thread = None
status_message = "Not connected"
data_records = []
latest_measurements: Dict[str, 'MeasurementData'] = {}

# ---------- CRC helpers ----------
def reflect_byte(value: int) -> int:
    result = 0
    for i in range(8):
        if value & (1 << i):
            result |= (1 << (7 - i))
    return result

def reflect_word(value: int) -> int:
    result = 0
    for i in range(16):
        if value & (1 << i):
            result |= (1 << (15 - i))
    return result

def calculate_crc16_ccitt(data_byte: int, previous_crc: int) -> int:
    polynomial = 0x1021
    data_byte = reflect_byte(data_byte)
    data_shifted = (data_byte << 8) & 0xFFFF
    crc = previous_crc ^ data_shifted
    for _ in range(8):
        if crc & 0x8000:
            crc = ((crc << 1) ^ polynomial) & 0xFFFF
        else:
            crc = (crc << 1) & 0xFFFF
    return crc

def byte_to_hex_str(b: int) -> str:
    return f"{b:02X}"

def word_to_hex_str(w: int) -> str:
    return f"{w:04X}"

def hex_str_to_word(s: str) -> int:
    return int(s, 16)

def hex_str_to_byte(s: str) -> int:
    return int(s, 16)

# ---------- Packet state holder ----------
class AvalonPacketState:
    def __init__(self):
        self.protocol_id = ""
        self.message_type = ""
        self.packet_length = 0
        self.packet_length_string = ""
        self.packet_payload_string = ""
        self.crc = 0xFFFF
        self.crc_string = ""
        self.received_e0_packet = False
        self.packet_state = AVALONFM30_PACKETSTATE_IDLE

    def init(self):
        self.protocol_id = ""
        self.message_type = ""
        self.packet_length = 0
        self.packet_length_string = ""
        self.packet_payload_string = ""
        self.crc = 0xFFFF
        self.crc_string = ""
        self.received_e0_packet = False
        self.packet_state = AVALONFM30_PACKETSTATE_IDLE

# ---------- TX helpers ----------
def hex_payload_to_bytes(hex_text: str) -> bytes:
    parts = hex_text.strip().split()
    return bytes(hex_str_to_byte(p) for p in parts)

def calculate_payload_length_hex(payload_hex_text: str) -> str:
    parts = [p for p in payload_hex_text.strip().split() if p]
    length = len(parts)
    return word_to_hex_str(length)

def calculate_payload_crc_hex(proto_id_hex: str, cmd_hex: str, length_hex: str,
                              payload_hex_text: str) -> str:
    crc = 0xFFFF
    crc = calculate_crc16_ccitt(hex_str_to_byte(proto_id_hex), crc)
    crc = calculate_crc16_ccitt(hex_str_to_byte(cmd_hex), crc)
    crc = calculate_crc16_ccitt(hex_str_to_byte(length_hex[0:2]), crc)
    crc = calculate_crc16_ccitt(hex_str_to_byte(length_hex[2:4]), crc)
    for b in hex_payload_to_bytes(payload_hex_text):
        crc = calculate_crc16_ccitt(b, crc)
    crc = reflect_word(crc)
    return word_to_hex_str(crc)

def build_and_escape_message(proto_id_hex: str, cmd_hex: str,
                             payload_hex_text: str) -> bytes:
    length_hex = calculate_payload_length_hex(payload_hex_text)
    crc_hex = calculate_payload_crc_hex(proto_id_hex, cmd_hex, length_hex,
                                        payload_hex_text)

    msg = bytearray()
    msg.append(0xC0)
    msg.append(hex_str_to_byte(proto_id_hex))
    msg.append(hex_str_to_byte(cmd_hex))
    msg.append(hex_str_to_byte(length_hex[0:2]))
    msg.append(hex_str_to_byte(length_hex[2:4]))

    for b in hex_payload_to_bytes(payload_hex_text):
        if b in (0xC0, 0xC1, 0x7D):
            msg.append(0x7D)
            b ^= 0x20
        msg.append(b)

    crc_lsb = hex_str_to_byte(crc_hex[2:4]) ^ 0xFF
    crc_msb = hex_str_to_byte(crc_hex[0:2]) ^ 0xFF
    msg.append(crc_lsb)
    msg.append(crc_msb)
    msg.append(0xC1)
    return bytes(msg)

# ---------- Payloads ----------
ASSOC_REQ_PAYLOAD = (
    " 0D EC 05 08 13 01 00 16 01 02 80 00 14 02 00 02 C1 DC 31 80 A0 80 80 01 01 00 00 A2 80 A0 03 00 00 01 A4 80 30 80 02 01 01 06"
    " 04 52 01 00 01 30 80 06 02 51 01 00 00 00 00 30 80 02 01 02 06 0C 2A 86 48 CE 14 02 01 00 00 00 01 01 30 80 06 0C 2A 86 48 CE"
    " 14 02 01 00 00 00 02 01 00 00 00 00 00 00 61 80 30 80 02 01 01 A0 80 60 80 A1 80 06 0C 2A 86 48 CE 14 02 01 00 00 00 03 01 00"
    " 00 BE 80 28 80 06 0C 2A 86 48 CE 14 02 01 00 00 00 01 01 02 01 02 81 48 80 00 00 00 40 00 00 00 00 00 00 00 80 00 00 00 20 00"
    " 00 00 00 00 00 00 00 01 00 2C 00 01 00 28 80 00 00 00 00 04 E2 00 00 00 30 00 00 00 30 00 FF FF FF FF 60 00 00 00 00 01 00 0C"
    " F0 01 00 08 8C 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00"
)

RELEASE_REQ_PAYLOAD = (
    " 09 18 C1 16 61 80 30 80 02 01 01 A0 80 62 80 80 01 00 00 00 00 00 00 00 00 00"
)

MDS_CREATE_EVENT_RSP_PAYLOAD = (
    " E1 00 00 02 00 02 00 14 00 01 00 01 00 0E 00 21 00 00 00 00 00 48 47 00 0D 06 00 00"
)

POLL_REQ_PAYLOAD = (
    " E1 00 00 02"
    " 00 01 00 1C 00 01 00 07 00 16"
    " 00 21 00 00 00 00 00 00 00 00 0C 16 00 08"
    " 00 01 00 01 00 06 00 00"
)

# ---------- Measurement data ----------
class MeasurementData:
    def __init__(self):
        self.label = ""
        self.pid = ""
        self.state = ""
        self.units = ""
        self.value = ""
        self.timestamp: Optional[datetime] = None

# ---------- Field decoding ----------
def extract_physiological_string(pid_hex: str) -> str:
    pid = int(pid_hex, 16)
    if pid == 0x4BB0: return "Perf"
    if pid == 0x4822: return "Pulse"  # Changed from "Pulse Plethysmogram"
    if pid == 0x4BB8: return "SpO2"
    if pid == 0xF9A4: return "Pulse from Toco"
    if pid == 0xF0D4: return "Toco"
    if pid == 0xF0CB: return "FHR1"
    return "PID undefined"

def extract_units_string(units_hex: str) -> str:
    v = int(units_hex, 16)
    if v == 0x0AA0: return "Beats per minute"
    if v == 0x0F20: return "mmHg"
    if v == 0x0F03: return "kPa"
    if v == 0x0220: return "perCent"
    if v == 0x0200: return "Dimensionless"
    return "Units undefined"

def extract_state_string(state_hex: str) -> str:
    v = int(state_hex, 16)
    if v == 0x8000: return "INVALID"
    if v == 0x4000: return "QUESTIONABLE"
    if v == 0x2000: return "UNAVAILABLE"
    if v == 0x1000: return "CALIBRATION_ONGOING"
    if v == 0x0800: return "TEST_DATA"
    if v == 0x0400: return "DEMO_DATA"
    if v == 0x0080: return "VALIDATED_DATA"
    if v == 0x0040: return "EARLY_INDICATION"
    if v == 0x0020: return "MSMT_ONGOING"
    if v == 0x0002: return "MSMT_STATE_IN_ALARM"
    if v == 0x0001: return "MSMT_STATE_AL_INHIBITED"
    return "State undefined"

def extract_float_string(float_hex: str) -> str:
    if len(float_hex) != 8:
        return "0E0"
    exponent_byte = int(float_hex[0:2], 16)
    if exponent_byte >= 128:
        exponent_byte = exponent_byte ^ 0xFF
        exponent_byte = exponent_byte + 1
        exponent = -1 * exponent_byte
    else:
        exponent = exponent_byte
    mantissa = int(float_hex[2:], 16)
    return f"{mantissa}E{exponent}"

def parse_measurement_field_string(field_hex: str) -> Optional[MeasurementData]:
    field = field_hex.replace(" ", "")
    
    idx = field.find("0927")
    if idx == -1:
        return None

    label = ""
    i = idx + 4
    i += 8
    
    for _ in range(14):
        if i + 4 <= len(field):
            hex_byte = field[i+2:i+4]
            try:
                char_code = int(hex_byte, 16)
                if 33 <= char_code < 128:
                    label += chr(char_code)
                else:
                    label += " "
            except:
                label += " "
            i += 4
        else:
            break
    label = label.strip()

    idx50 = field.find("0950")
    if idx50 == -1:
        return None
    
    i = idx50 + 4
    
    if i + 4 > len(field):
        return None
    i += 4

    if i + 4 > len(field):
        return None
    pid_hex = field[i:i+4]
    pid_name = extract_physiological_string(pid_hex)
    i += 4

    if i + 4 > len(field):
        return None
    state_hex = field[i:i+4]
    state_str = extract_state_string(state_hex)
    i += 4

    if i + 4 > len(field):
        return None
    units_hex = field[i:i+4]
    units_str = extract_units_string(units_hex)
    i += 4

    if i + 8 > len(field):
        return None
    float_hex = field[i:i+8]
    float_str = extract_float_string(float_hex)

    m = MeasurementData()
    m.label = label
    m.pid = pid_name
    m.state = state_str
    m.units = units_str
    m.value = float_str
    m.timestamp = datetime.now()
    return m

def parse_measurements_packet(payload: bytes):
    try:
        parsed = " ".join(f"{b:02X}" for b in payload)
        
        pos = parsed.find("09 2F")
        if pos == -1:
            return
        
        parsed = parsed[pos:]
        
        while True:
            next_pos = parsed.find("09 2F", 5)
            
            if next_pos != -1:
                field_str = parsed[:next_pos]
                m = parse_measurement_field_string(field_str)
                if m and m.pid != "PID undefined":
                    latest_measurements[m.pid] = m
                parsed = parsed[next_pos:]
            else:
                if parsed.startswith("09 2F"):
                    field_str = parsed
                    m = parse_measurement_field_string(field_str)
                    if m and m.pid != "PID undefined":
                        latest_measurements[m.pid] = m
                break
    except Exception:
        pass

def _value_from_scientific_str(s: str) -> Optional[float]:
    if not s or "E" not in s:
        return None
    try:
        mantissa_str, exp_str = s.split("E", 1)
        mantissa = int(mantissa_str)
        exponent = int(exp_str)
        if mantissa == 0x7FFFFF or mantissa == 0x7FFF or mantissa == 0x7F:
            return None
        return mantissa * (10 ** exponent)
    except Exception:
        return None

def store_measurements():
    """Store current measurements to data records"""
    global data_records
    
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    
    fhr = toco = pulse = spo2 = None
    
    if "FHR1" in latest_measurements:
        fhr = _value_from_scientific_str(latest_measurements["FHR1"].value)
    if "Toco" in latest_measurements:
        toco = _value_from_scientific_str(latest_measurements["Toco"].value)
    
    # Check for both pulse types
    if "Pulse from Toco" in latest_measurements:
        pulse = _value_from_scientific_str(latest_measurements["Pulse from Toco"].value)
    elif "Pulse" in latest_measurements:
        pulse = _value_from_scientific_str(latest_measurements["Pulse"].value)
    
    if "SpO2" in latest_measurements:
        spo2 = _value_from_scientific_str(latest_measurements["SpO2"].value)
    
    record = {
        "Timestamp": timestamp,
        "FHR1 (bpm)": int(fhr) if fhr is not None else None,
        "TOCO": int(toco) if toco is not None else None,
        "Pulse (bpm)": int(pulse) if pulse is not None else None,
        "SpO2 (%)": int(spo2) if spo2 is not None else None
    }
    
    data_records.append(record)

# ---------- RX: decode packets ----------
def handle_payload(payload_hex: str):
    if not payload_hex.startswith("E1"):
        return
    
    payload_bytes = bytes(int(b, 16) for b in payload_hex.split())
    
    if len(payload_bytes) < 10:
        return
    
    if payload_bytes[0:4] == b"\xE1\x00\x00\x02":
        body = payload_bytes[4:]
    else:
        body = payload_bytes

    parse_measurements_packet(body)
    store_measurements()

def avalon_packet_check(ser: serial.Serial, state: AvalonPacketState) -> bool:
    packet_available = False
    
    while ser.in_waiting > 0:
        data_from_port = ser.read(1)
        if not data_from_port:
            break
        data_byte = data_from_port[0]

        if state.packet_state == AVALONFM30_PACKETSTATE_IDLE:
            if data_byte == 0xC0:
                state.init()
                state.packet_state = AVALONFM30_PACKETSTATE_PROTOCOLID

        elif state.packet_state == AVALONFM30_PACKETSTATE_PROTOCOLID:
            state.protocol_id = byte_to_hex_str(data_byte)
            state.crc = calculate_crc16_ccitt(data_byte, state.crc)
            state.packet_state = AVALONFM30_PACKETSTATE_MESSAGETYPE

        elif state.packet_state == AVALONFM30_PACKETSTATE_MESSAGETYPE:
            state.message_type = byte_to_hex_str(data_byte)
            state.crc = calculate_crc16_ccitt(data_byte, state.crc)
            state.packet_state = AVALONFM30_PACKETSTATE_MESSAGELENGTH_MSB

        elif state.packet_state == AVALONFM30_PACKETSTATE_MESSAGELENGTH_MSB:
            state.packet_length_string = byte_to_hex_str(data_byte)
            state.crc = calculate_crc16_ccitt(data_byte, state.crc)
            state.packet_state = AVALONFM30_PACKETSTATE_MESSAGELENGTH_LSB

        elif state.packet_state == AVALONFM30_PACKETSTATE_MESSAGELENGTH_LSB:
            state.packet_length_string += byte_to_hex_str(data_byte)
            state.crc = calculate_crc16_ccitt(data_byte, state.crc)
            state.packet_length = hex_str_to_word(state.packet_length_string)
            state.packet_state = AVALONFM30_PACKETSTATE_MESSAGEPAYLOAD_FIRSTBYTE

        elif state.packet_state == AVALONFM30_PACKETSTATE_MESSAGEPAYLOAD_FIRSTBYTE:
            if data_byte == 0xE0:
                state.received_e0_packet = True
            elif data_byte == 0x7D:
                state.packet_state = AVALONFM30_PACKETSTATE_MESSAGEPAYLOAD_POSTESC
            else:
                state.packet_payload_string += " " + byte_to_hex_str(data_byte)
                state.crc = calculate_crc16_ccitt(data_byte, state.crc)
                state.packet_length -= 1
                if state.packet_length == 0:
                    state.packet_state = AVALONFM30_PACKETSTATE_CRC_LSB
                else:
                    state.packet_state = AVALONFM30_PACKETSTATE_MESSAGEPAYLOAD_NORMAL

        elif state.packet_state == AVALONFM30_PACKETSTATE_MESSAGEPAYLOAD_NORMAL:
            if data_byte != 0x7D:
                state.packet_payload_string += " " + byte_to_hex_str(data_byte)
                state.crc = calculate_crc16_ccitt(data_byte, state.crc)
                state.packet_length -= 1
                if state.packet_length == 0:
                    state.packet_state = AVALONFM30_PACKETSTATE_CRC_LSB
            else:
                state.packet_state = AVALONFM30_PACKETSTATE_MESSAGEPAYLOAD_POSTESC

        elif state.packet_state == AVALONFM30_PACKETSTATE_MESSAGEPAYLOAD_POSTESC:
            data_byte ^= 0x20
            state.packet_payload_string += " " + byte_to_hex_str(data_byte)
            state.crc = calculate_crc16_ccitt(data_byte, state.crc)
            state.packet_length -= 1
            if state.packet_length == 0:
                state.packet_state = AVALONFM30_PACKETSTATE_CRC_LSB
            else:
                state.packet_state = AVALONFM30_PACKETSTATE_MESSAGEPAYLOAD_NORMAL

        elif state.packet_state == AVALONFM30_PACKETSTATE_CRC_LSB:
            lsb = data_byte ^ 0xFF
            state.crc_string = byte_to_hex_str(lsb)
            state.packet_state = AVALONFM30_PACKETSTATE_CRC_MSB

        elif state.packet_state == AVALONFM30_PACKETSTATE_CRC_MSB:
            msb = data_byte ^ 0xFF
            state.crc_string = byte_to_hex_str(msb) + state.crc_string
            state.packet_state = AVALONFM30_PACKETSTATE_EOF

        elif state.packet_state == AVALONFM30_PACKETSTATE_EOF:
            if data_byte == 0xC1:
                calc_crc_reflected = reflect_word(state.crc)
                if word_to_hex_str(calc_crc_reflected) == state.crc_string:
                    packet_available = True
                    handle_payload(state.packet_payload_string.strip())
            elif state.received_e0_packet:
                packet_available = True
                handle_payload(state.packet_payload_string.strip())
            
            state.packet_state = AVALONFM30_PACKETSTATE_IDLE

    return packet_available

# ---------- Serial Reader Thread ----------
def serial_reader():
    global ser, collecting, status_message
    
    state = AvalonPacketState()
    state.init()
    
    # Send release requests
    rel_msg = build_and_escape_message("11", "01", RELEASE_REQ_PAYLOAD)
    ser.write(rel_msg)
    ser.flush()
    time.sleep(1.0)
    ser.write(rel_msg)
    ser.flush()
    time.sleep(1.0)
    
    # Send association request
    assoc_msg = build_and_escape_message("11", "01", ASSOC_REQ_PAYLOAD)
    ser.write(assoc_msg)
    ser.flush()
    
    mds_sent = False
    last_poll_time = 0.0
    poll_interval = 1.0
    
    while collecting:
        try:
            avalon_packet_check(ser, state)
            
            payload = state.packet_payload_string.strip()
            
            if payload.startswith("E1") and not mds_sent:
                mds_msg = build_and_escape_message("11", "01", MDS_CREATE_EVENT_RSP_PAYLOAD)
                ser.write(mds_msg)
                ser.flush()
                mds_sent = True
            
            now = time.time()
            if mds_sent and (now - last_poll_time) >= poll_interval:
                poll_msg = build_and_escape_message("11", "01", POLL_REQ_PAYLOAD)
                ser.write(poll_msg)
                ser.flush()
                last_poll_time = now
            
            time.sleep(0.005)
        except Exception as e:
            status_message = f"Error in data collection: {e}"
            break

# =========================
# Dash App
# =========================
app = Dash(__name__)

app.layout = html.Div([
    html.H2("Avalon FM30 CTG Data Collector", style={"textAlign": "center", "marginBottom": "20px"}),
    
    html.Div([
        html.Label("Select COM Port", style={"fontWeight": "bold", "marginRight": "10px"}),
        dcc.Dropdown(
            id="port-dropdown", 
            options=[], 
            placeholder="Waiting for COM ports...",
            style={"width": "300px"}
        ),
    ], style={"display": "flex", "alignItems": "center", "marginBottom": "20px"}),
    
    dcc.Interval(id="port-refresh", interval=1500, n_intervals=0),
    
    html.Div([
        html.Button(
            "Connect", 
            id="btn-connect", 
            style={
                "fontSize": "18px", 
                "padding": "15px 30px", 
                "marginRight": "10px",
                "backgroundColor": "#4CAF50",
                "color": "white",
                "border": "none",
                "borderRadius": "5px",
                "cursor": "pointer"
            }
        ),
        html.Button(
            "Start", 
            id="btn-start", 
            disabled=True, 
            style={
                "fontSize": "18px", 
                "padding": "15px 30px", 
                "marginRight": "10px",
                "backgroundColor": "#008CBA",
                "color": "white",
                "border": "none",
                "borderRadius": "5px",
                "cursor": "pointer"
            }
        ),
        html.Button(
            "Stop", 
            id="btn-stop", 
            disabled=True, 
            style={
                "fontSize": "18px", 
                "padding": "15px 30px",
                "backgroundColor": "#f44336",
                "color": "white",
                "border": "none",
                "borderRadius": "5px",
                "cursor": "pointer"
            }
        ),
    ], style={"marginTop": "10px", "marginBottom": "20px"}),
    
    html.Div(
        id="status", 
        style={
            "marginTop": "10px", 
            "marginBottom": "20px",
            "padding": "10px",
            "fontSize": "16px",
            "fontWeight": "bold",
            "backgroundColor": "#f0f0f0",
            "borderRadius": "5px"
        }
    ),
    
    dcc.Interval(id="ui-update", interval=1000),
    
    html.H3("Collected Data", style={"marginTop": "30px", "marginBottom": "10px"}),
    
    dash_table.DataTable(
        id="data-table",
        columns=[
            {"name": "Timestamp", "id": "Timestamp"},
            {"name": "FHR1 (bpm)", "id": "FHR1 (bpm)"},
            {"name": "TOCO", "id": "TOCO"},
            {"name": "Pulse (bpm)", "id": "Pulse (bpm)"},
            {"name": "SpO2 (%)", "id": "SpO2 (%)"}
        ],
        data=[],
        page_size=20,
        style_table={"overflowX": "auto", "marginBottom": "20px"},
        style_cell={
            "textAlign": "center", 
            "fontSize": "14px", 
            "padding": "10px"
        },
        style_header={
            "backgroundColor": "#4CAF50",
            "color": "white",
            "fontWeight": "bold"
        },
        style_data_conditional=[
            {
                "if": {"row_index": "odd"},
                "backgroundColor": "#f9f9f9"
            }
        ]
    ),
    
    html.Button(
        "Download Excel", 
        id="btn-download", 
        style={
            "fontSize": "18px", 
            "padding": "15px 30px", 
            "marginTop": "20px",
            "backgroundColor": "#FF9800",
            "color": "white",
            "border": "none",
            "borderRadius": "5px",
            "cursor": "pointer"
        }
    ),
    dcc.Download(id="download")
], style={"padding": "40px", "maxWidth": "1200px", "margin": "0 auto"})

# =========================
# COM Port Refresh Callback
# =========================
@app.callback(
    Output("port-dropdown", "options"),
    Input("port-refresh", "n_intervals")
)
def refresh_ports(_):
    ports = serial.tools.list_ports.comports()
    options = [{"label": f"{p.device} - {p.description}", "value": p.device} for p in ports]
    if not options:
        options = [{"label": "No COM ports detected", "value": ""}]
    return options

# =========================
# Connect / Start / Stop Callback
# =========================
@app.callback(
    Output("status", "children"),
    Output("status", "style"),
    Output("btn-start", "disabled"),
    Output("btn-stop", "disabled"),
    Input("btn-connect", "n_clicks"),
    Input("btn-start", "n_clicks"),
    Input("btn-stop", "n_clicks"),
    State("port-dropdown", "value"),
    prevent_initial_call=True
)
def handle_actions(connect_click, start_click, stop_click, port):
    global ser, collecting, serial_thread, status_message
    ctx = callback_context
    
    if not ctx.triggered:
        raise dash.exceptions.PreventUpdate
    
    button_id = ctx.triggered[0]['prop_id'].split('.')[0]
    
    default_style = {
        "marginTop": "10px", 
        "marginBottom": "20px",
        "padding": "10px",
        "fontSize": "16px",
        "fontWeight": "bold",
        "borderRadius": "5px"
    }
    
    if button_id == "btn-connect":
        if not port:
            status_message = "⚠️ Please select a valid COM port"
            error_style = {**default_style, "backgroundColor": "#ffebee", "color": "#c62828"}
            return status_message, error_style, True, True
        
        try:
            if ser is not None and ser.is_open:
                ser.close()
            
            ser = serial.Serial(port, baudrate=115200, timeout=0.01)
            status_message = f"✅ Connected to {port}"
            success_style = {**default_style, "backgroundColor": "#e8f5e9", "color": "#2e7d32"}
            return status_message, success_style, False, True
        except Exception as e:
            status_message = f"❌ Connection failed: {e}"
            error_style = {**default_style, "backgroundColor": "#ffebee", "color": "#c62828"}
            return status_message, error_style, True, True
    
    elif button_id == "btn-start":
        if ser is None or not ser.is_open:
            status_message = "⚠️ Not connected to any COM port"
            error_style = {**default_style, "backgroundColor": "#ffebee", "color": "#c62828"}
            return status_message, error_style, True, True
        
        collecting = True
        data_records.clear()  # Clear previous data
        latest_measurements.clear()
        status_message = "🔄 Collecting data..."
        collecting_style = {**default_style, "backgroundColor": "#e3f2fd", "color": "#1565c0"}
        
        serial_thread = threading.Thread(target=serial_reader, daemon=True)
        serial_thread.start()
        
        return status_message, collecting_style, True, False
    
    elif button_id == "btn-stop":
        collecting = False
        status_message = "⏸️ Data collection stopped"
        stopped_style = {**default_style, "backgroundColor": "#fff3e0", "color": "#e65100"}
        return status_message, stopped_style, False, True
    
    return dash.no_update

# =========================
# Update Table Callback
# =========================
@app.callback(
    Output("data-table", "data"),
    Input("ui-update", "n_intervals")
)
def update_table(_):
    if len(data_records) == 0:
        return []
    
    # Return last 20 records (most recent first)
    return data_records[-20:][::-1]

# =========================
# Download Excel Callback
# =========================
@app.callback(
    Output("download", "data"),
    Input("btn-download", "n_clicks"),
    prevent_initial_call=True
)
def download_excel(n_clicks):
    if len(data_records) == 0:
        return dash.no_update
    
    df = pd.DataFrame(data_records)
    
    # Generate filename with timestamp
    filename = f"Avalon_FM30_Data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.xlsx"
    
    return dcc.send_data_frame(df.to_excel, filename, index=False)

if __name__ == "__main__":
    app.run(debug=True, port=8050)
    
