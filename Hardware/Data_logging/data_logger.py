from flask import Flask, request
import pandas as pd
import os

app = Flask(__name__)
DATA_FILE = "measurements.xlsx"

# map INA addresses → labels
ADDR_MAP = {
    "0x40": "24V",
    "0x41": "19V",
    "0x44": "12V_1",
    "0x45": "5V",
    "0x46": "16V",
    "0x4F": "12V_2"
}
METRICS = ["bus_V", "shunt_mV", "current_mA", "power_mW"]

# build header: timestamp, system_current, then each label+metric
COLUMNS = ["timestamp_ms", "system_current_A"] + [
    f"{label}_{metric}"
    for label in ADDR_MAP.values()
    for metric in METRICS
]

# first‐run: make empty sheet with headers
if not os.path.exists(DATA_FILE):
    pd.DataFrame(columns=COLUMNS).to_excel(DATA_FILE, index=False)

@app.route("/data", methods=["POST"])
def receive():
    payload = request.get_json()
    ts = payload["timestamp"]

    # start our wide‐row
    row = {
        "timestamp_ms":      ts,
        "system_current_A":  payload.get("system_current_A", None)
    }

    # unpack each INA219 reading into its wide column
    for m in payload["readings"]:
        label = ADDR_MAP.get(m["addr"], m["addr"])
        row[f"{label}_bus_V"]      = m["V"]
        row[f"{label}_shunt_mV"]   = m["mV"]
        row[f"{label}_current_mA"] = m["mA"]
        row[f"{label}_power_mW"]   = m["mW"]

    # append + save
    df_old = pd.read_excel(DATA_FILE)
    df_new = pd.DataFrame([row], columns=COLUMNS)
    df = pd.concat([df_old, df_new], ignore_index=True)
    df.to_excel(DATA_FILE, index=False)

    return "OK", 200

if __name__ == "__main__":
    # ensure you have openpyxl installed: pip install openpyxl
    app.run(host="0.0.0.0", port=5000)
