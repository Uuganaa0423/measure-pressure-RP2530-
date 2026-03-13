# measure-pressure-RP2530-

## Project Description

Created two Python scripts to capture and graph your data. The Pico doesn't create files directly; it sends data over USB serial, and the scripts save it to a CSV file.

## How to Use - Step by Step Guide

### Prerequisites Checklist
Before starting, make sure you have:
- [ ] Python 3.6 or higher installed (`python3 --version` to check)
- [ ] Raspberry Pi Pico connected via USB cable
- [ ] Pico firmware flashed and running (should be sending data over serial)

---

### Step 1: Install Python Dependencies

**1.1. Open Terminal/Command Prompt**
- Navigate to the project directory:
  ```bash
  cd /path/to/measure-pressure-RP2530-
  ```

**1.2. (Optional) Create Virtual Environment**
- Recommended to keep dependencies isolated:
  ```bash
  python3 -m venv .venv
  ```
- Activate it:
  ```bash
  # On macOS/Linux:
  source .venv/bin/activate
  
  # On Windows:
  .venv\Scripts\activate
  ```

**1.3. Install Required Packages**
```bash
pip3 install -r requirements.txt
```

**1.4. Verify Installation**
- You should see packages installed: `pyserial`, `pandas`, `matplotlib`
- If you see errors, check your Python version and internet connection

---

### Step 2: Prepare Your Pico

**2.1. Connect Pico to Computer**
- Plug USB cable into Pico and your computer
- Wait a few seconds for the computer to recognize the device

**2.2. Verify Pico is Running**
- The Pico should be powered (LED may be on)
- The firmware should be sending data over serial

**2.3. (Optional) Find Serial Port Manually**
- If auto-detection fails, find the port:
  ```bash
  python3 -m serial.tools.list_ports
  ```
- Look for ports like:
  - macOS: `/dev/cu.usbmodem*` or `/dev/tty.usbmodem*`
  - Linux: `/dev/ttyACM*` or `/dev/ttyUSB*`
  - Windows: `COM3`, `COM4`, etc.

---

### Step 3: Capture Data from Pico

**3.1. Navigate to Project Directory**
```bash
cd /path/to/measure-pressure-RP2530-
```

**3.2. Run the Capture Script**

**Option A: Auto-detect Port (Recommended)**
```bash
python3 capture_csv.py
```

**Option B: Specify Output Filename**
```bash
python3 capture_csv.py my_data.csv
```

**Option C: Manual Port Selection**
```bash
# On macOS/Linux:
python3 capture_csv.py /dev/ttyACM0 my_data.csv

# On Windows:
python3 capture_csv.py COM3 my_data.csv
```

**3.3. What You Should See**
```
Connecting to /dev/ttyACM0...
Output file: pressure_data.csv
Press Ctrl+C to stop capturing

Capturing data... (saving to CSV file)
------------------------------------------------------------
Captured 10 data points (1.2s elapsed)
Captured 20 data points (2.4s elapsed)
Captured 30 data points (3.5s elapsed)
...
```

**3.4. Let It Run**
- The script will continuously capture data
- Progress updates appear every 10 data points
- Data is saved to CSV file in real-time

**3.5. Stop Capturing**
- Press `Ctrl+C` (or `Cmd+C` on Mac) to stop
- You should see:
  ```
  Stopping capture...
  
  Capture complete!
  Total data points: 154
  Duration: 14.3 seconds
  CSV file saved: /full/path/to/pressure_data.csv
  ```

**3.6. Verify CSV File Created**
```bash
# On macOS/Linux:
ls -lh pressure_data.csv

# On Windows:
dir pressure_data.csv
```
- File should exist and have a size > 0 bytes

---

### Step 4: Plot the Data

**4.1. Run the Plot Script**
```bash
python3 plot_pressure.py pressure_data.csv
```

**4.2. What Happens**
1. Script reads the CSV file
2. Processes the data (converts timestamps, calculates statistics)
3. Creates two graphs:
   - **Top graph**: Pressure vs Time (with target range lines at -52000 Pa and -40000 Pa)
   - **Bottom graph**: System Status (pump and vent valve states over time)
4. Displays plot window (may appear behind terminal)
5. Saves PNG image automatically

**4.3. Expected Output**
```
Plot saved to: /full/path/to/pressure_data_plot.png
============================================================
Data Statistics:
============================================================
Total data points: 154
Duration: 16.28 seconds
Pressure - Min: -52123.45 Pa
Pressure - Max: -38987.12 Pa
Pressure - Mean: -45234.56 Pa
Pressure - Std Dev: 1234.56 Pa
Pump ON time: 12.3 seconds
Vent Valve ON time: 2.1 seconds
```

**4.4. View the Plot**
- A matplotlib window should open showing the graphs
- Close the window when done viewing
- The PNG file is saved in the same directory as the CSV file

**4.5. Verify Plot File**
```bash
# On macOS/Linux:
ls -lh pressure_data_plot.png

# On Windows:
dir pressure_data_plot.png
```
- File should exist and be viewable as an image

## Troubleshooting

**Problem: "Could not auto-detect Pico serial port"**
- Solution: List available ports manually:
  ```bash
  python3 -m serial.tools.list_ports
  ```
  Then use the port name: `python3 capture_csv.py /dev/ttyACM0`

**Problem: "No such file or directory: '/dev/ttyACM0'"**
- Solution: Check if Pico is connected and the port name is correct
- On macOS: Check `/dev/cu.usbmodem*` or `/dev/tty.usbmodem*`
- On Linux: Check `/dev/ttyACM*` or `/dev/ttyUSB*`

**Problem: "Error opening serial port: Permission denied"**
- Solution: Add your user to the dialout group (Linux):
  ```bash
  sudo usermod -a -G dialout $USER
  # Then log out and back in
  ```

**Problem: CSV file doesn't create in folder**
- Solution: The CSV file is created in the current working directory where you run the script
- Check: `ls -la` or `dir` to see the file
- Use absolute path: `python3 capture_csv.py /dev/ttyACM0 /full/path/to/my_data.csv`

**Problem: No data being captured**
- Check: Pico is powered on and firmware is running
- Check: Pico is sending CSV format data (should start with "CSV,")
- Check: Serial baud rate matches (115200)