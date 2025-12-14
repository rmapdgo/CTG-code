import threading
import datetime
import serial
import serial.tools.list_ports
import pandas as pd
import dash
from dash import Dash, dcc, html, Input, Output, State, dash_table

# =========================
# Global state
# =========================
ser = None
serial_thread = None
collecting = False

data_lock = threading.Lock()
raw_df = pd.DataFrame(columns=[
    "datetime",
    "pid",
    "value"
])

status_message = "Not connected"

# =========================
# CRC + packet parsing
# =========================
def reflect_byte(b):
    r = 0
    for i in range(8):
        if b & (1 << i):
            r |= 1 << (7 - i)
    return r

def reflect_word(w):
    r = 0
    for i in range(16):
        if w & (1 << i):
            r |= 1 << (15 - i)
    return r

def crc16_ccitt(data_byte, crc):
    crc ^= (data_byte << 8)
    for _ in range(8):
        if crc & 0x8000:
            crc = (crc << 1) ^ 0x1021
        else:
            crc <<= 1
        crc &= 0xFFFF
    return crc

# =========================
# Measurement decoding
# =========================
PID_MAP = {
    0xF0CB: "FHR",
    0xF0D4: "TOCO",
    0xF9A4: "Pulse",
}

def extract_float(hexstr):
    exponent = int(hexstr[0:2], 16)
    if exponent & 0x80:
        exponent = -((exponent ^ 0xFF) + 1)
    mantissa = int(hexstr[2:], 16)
    return mantissa * (10 ** exponent)

def parse_measurement_payload(payload_hex):
    results = []
    idx = 0
    while idx < len(payload_hex) - 24:
        if payload_hex[idx:idx+4] == "0950":
            pid = int(payload_hex[idx+4:idx+8], 16)
            value = extract_float(payload_hex[idx+16:idx+24])
            name = PID_MAP.get(pid, f"PID_{pid:04X}")
            results.append((name, value))
            idx += 24
        else:
            idx += 2
    return results

# =========================
# Serial reader thread
# =========================
def serial_reader():
    global collecting, status_message, raw_df
    payload = []
    crc = 0xFFFF

    while collecting:
        try:
            b = ser.read(1)
            if not b:
                continue
            byte = b[0]

            if byte == 0xC0:  # Start
                payload = []
                crc = 0xFFFF
                continue

            if byte == 0xC1:  # End
                crc = reflect_word(crc)
                hex_payload = "".join(f"{x:02X}" for x in payload)
                decoded = parse_measurement_payload(hex_payload)
                now = datetime.datetime.now()

                with data_lock:
                    for name, value in decoded:
                        raw_df.loc[len(raw_df)] = [now, name, value]
                continue

            crc = crc16_ccitt(byte, crc)
            payload.append(byte)

        except Exception as e:
            status_message = f"Error: {e}"
            collecting = False

    status_message = "Data collection has stopped"

# =========================
# Dash App
# =========================
app = Dash(__name__)

app.layout = html.Div([
    html.H2("Avalon FM30 CTG Data Collector"),

    html.Label("Select COM Port"),
    dcc.Dropdown(id="port-dropdown", options=[], placeholder="Waiting for COM ports..."),
    dcc.Interval(id="port-refresh", interval=1500, n_intervals=0),  # Refresh ports every 1.5s

    html.Div([
        html.Button("Connect", id="btn-connect", style={"fontSize": "18px", "padding": "15px 30px", "marginRight": "10px"}),
        html.Button("Start", id="btn-start", disabled=True, style={"fontSize": "18px", "padding": "15px 30px", "marginRight": "10px"}),
        html.Button("Stop", id="btn-stop", disabled=True, style={"fontSize": "18px", "padding": "15px 30px"}),
    ], style={"marginTop": "10px", "marginBottom": "20px"}),

    html.Div(id="status", style={"marginTop": "10px", "color": "red"}),

    dcc.Interval(id="ui-update", interval=1000),

    dash_table.DataTable(
        id="data-table",
        page_size=20,  # Show only recent 20 rows
        style_table={"overflowX": "auto"},
        style_cell={"textAlign": "center", "fontSize": "14px", "padding": "5px"}
    ),

    html.Button("Download Excel", id="btn-download", style={"fontSize": "18px", "padding": "15px 30px", "marginTop": "20px"}),
    dcc.Download(id="download")
])

# =========================
# COM Port Refresh Callback
# =========================
@app.callback(
    Output("port-dropdown", "options"),
    Input("port-refresh", "n_intervals")
)
def refresh_ports(_):
    ports = serial.tools.list_ports.comports()
    options = [{"label": p.device + (" - " + p.description if p.description else ""), "value": p.device} for p in ports]
    if not options:
        options = [{"label": "No COM ports detected", "value": ""}]
    return options

# =========================
# Unified Connect / Start / Stop Callback
# =========================
@app.callback(
    Output("status", "children"),
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
    ctx = dash.callback_context
    if not ctx.triggered:
        raise dash.exceptions.PreventUpdate
    button_id = ctx.triggered[0]['prop_id'].split('.')[0]

    if button_id == "btn-connect":
        if not port:
            status_message = "Please select a valid COM port"
            return status_message, True, True
        try:
            ser = serial.Serial(port, baudrate=115200, timeout=0.1)
            status_message = f"Connected to {port}"
            return status_message, False, True
        except Exception as e:
            status_message = f"Connection failed: {e}"
            return status_message, True, True

    elif button_id == "btn-start":
        if ser is None or not ser.is_open:
            status_message = "Not connected to any COM port"
            return status_message, True, True
        collecting = True
        status_message = "Collecting data"
        serial_thread = threading.Thread(target=serial_reader, daemon=True)
        serial_thread.start()
        return status_message, True, False

    elif button_id == "btn-stop":
        collecting = False
        status_message = "Data collection stopped"
        return status_message, False, True

    return dash.no_update

# =========================
# Update Table Callback
# =========================
@app.callback(
    Output("data-table", "data"),
    Output("data-table", "columns"),
    Input("ui-update", "n_intervals")
)
def update_table(_):
    with data_lock:
        if raw_df.empty:
            return [], []
        df_pivot = raw_df.copy()
        df_pivot["date"] = df_pivot["datetime"].dt.date
        df_pivot["time"] = df_pivot["datetime"].dt.strftime("%H:%M:%S:%f").str[:-4]  # HH:MM:SS:MS two digits
        pivot = df_pivot.pivot_table(index=["date", "time"], columns="pid", values="value", aggfunc="first").reset_index()
        columns = [{"name": c, "id": c} for c in pivot.columns]
        data = pivot.tail(20).to_dict("records")  # Show only recent 20 rows
        return data, columns

# =========================
# Excel Download Callback
# =========================
@app.callback(
    Output("download", "data"),
    Input("btn-download", "n_clicks"),
    prevent_initial_call=True
)
def download_excel(_):
    with data_lock:
        if raw_df.empty:
            return None
        df_export = raw_df.copy()
        df_export["date"] = df_export["datetime"].dt.date
        df_export["time"] = df_export["datetime"].dt.strftime("%H:%M:%S:%f").str[:-4]
        pivot = df_export.pivot_table(index=["date", "time"], columns="pid", values="value", aggfunc="first").reset_index()
        return dcc.send_data_frame(pivot.to_excel, "ctg_data.xlsx", index=False)

# =========================
# Run
# =========================
if __name__ == "__main__":
    app.run(debug=True)
