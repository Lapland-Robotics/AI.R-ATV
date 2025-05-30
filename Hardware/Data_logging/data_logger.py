from flask import Flask, request
import pandas as pd
import os

app = Flask(__name__)
DATA_FILE = "measurements_full.xlsx"

# translate INA addresses → human labels
ADDR_MAP = {
    "0x40": "24V",
    "0x41": "19V",
    "0x44": "12V_1",
    "0x45": "5V",
    "0x46": "16V",
    "0x4F": "12V_2"
}
METRICS = ["bus_V", "shunt_mV", "current_mA", "power_mW"]

# build header: timestamp + raw voltage + current + each rail’s metrics
COLUMNS = (
    ["timestamp_ms", "system_voltage_V", "system_current_A"]
    + [f"{label}_{m}" for label in ADDR_MAP.values() for m in METRICS]
)

# on first run, create an empty sheet with our headers
if not os.path.exists(DATA_FILE):
    pd.DataFrame(columns=COLUMNS).to_excel(DATA_FILE, index=False)

@app.route("/data", methods=["POST"])
def receive():
    payload = request.get_json()
    ts = payload["timestamp"]

    # start our single “wide” row
    row = {
        "timestamp_ms":       ts,
        "system_voltage_V":   payload.get("system_voltage_V"),
        "system_current_A":   payload.get("system_current_A")
    }

    # fill in each INA219’s metrics
    for m in payload["readings"]:
        label = ADDR_MAP.get(m["addr"], m["addr"])
        row[f"{label}_bus_V"]      = m["V"]
        row[f"{label}_shunt_mV"]   = m["mV"]
        row[f"{label}_current_mA"] = m["mA"]
        row[f"{label}_power_mW"]   = m["mW"]

    # append + save back to Excel
    df_old = pd.read_excel(DATA_FILE)
    df_new = pd.DataFrame([row], columns=COLUMNS)
    df = pd.concat([df_old, df_new], ignore_index=True)
    df.to_excel(DATA_FILE, index=False)

    return "OK", 200

if __name__ == "__main__":
    # requires openpyxl: pip install openpyxl
    app.run(host="0.0.0.0", port=5000)
