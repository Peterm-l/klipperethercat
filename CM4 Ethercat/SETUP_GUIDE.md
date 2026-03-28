# CM4 EtherCAT Servo Setup — Step-by-Step Guide

## Phase 1: Prepare the CM4

### Step 1 — Update the CM4's Pi OS
```
ssh pi@<cm4-ip>
sudo apt update && sudo apt upgrade -y
sudo reboot
```
Wait for reboot, reconnect via SSH.

### Step 2 — Transfer project files from your Windows machine to CM4

From your Windows terminal (PowerShell or Git Bash):
```
scp -r "C:\Users\peter\Documents\CM4 Ethercat" pi@<cm4-ip>:~/klipper-ethercat
```
Replace `<cm4-ip>` with your CM4's WiFi IP address. You can find it with `ping raspberrypi.local` or check your router.

### Step 3 — SSH into the CM4 and verify files arrived
```
ssh pi@<cm4-ip>
ls ~/klipper-ethercat/
```
You should see: `CMakeLists.txt  Makefile  config  klippy  profiles  scripts  src  tests` etc.

---

## Phase 2: Build the PREEMPT_RT Kernel

### Step 4 — Make scripts executable
```
chmod +x ~/klipper-ethercat/scripts/*.sh
```

### Step 5 — Build and install the RT kernel

This takes **2-4 hours** on the CM4. Run it in a `screen` session so it survives SSH disconnection:
```
sudo apt install screen
screen -S rtbuild
sudo ~/klipper-ethercat/scripts/setup_rt_kernel.sh
```
If SSH disconnects, reconnect and run `screen -r rtbuild` to reattach.

### Step 6 — Reboot into the new kernel
```
sudo reboot
```

### Step 7 — Verify the RT kernel is running
```
ssh pi@<cm4-ip>
uname -v
```
You should see `PREEMPT_RT` in the output. If not, the kernel build failed — check `/boot/firmware/` for the kernel image.

---

## Phase 3: Install IgH EtherLab Master

### Step 8 — Install EtherLab
```
sudo ~/klipper-ethercat/scripts/setup_etherlab.sh
```
This clones, builds, and installs the IgH EtherCAT master. Takes ~10-15 minutes.

### Step 9 — Start the EtherCAT master

Make sure your LC10E drives are:
- Powered on
- Connected via Ethernet daisy chain to the CM4's RJ45 port

Then:
```
sudo systemctl start ethercat
```

### Step 10 — Verify slaves are detected
```
ethercat slaves
```
Expected output (3 drives):
```
0  0:0  PREOP  +  LC10E_V1.04
1  0:1  PREOP  +  LC10E_V1.04
2  0:2  PREOP  +  LC10E_V1.04
```
If you see 0 slaves: check cables, drive power, and `dmesg | grep -i ethercat`.

---

## Phase 4: Configure CPU Isolation & RT Settings

### Step 11 — Run the RT configuration script
```
sudo ~/klipper-ethercat/scripts/configure_rt.sh
```

### Step 12 — Reboot (required for `isolcpus` kernel parameter)
```
sudo reboot
```

### Step 13 — Verify everything with the verification script
```
ssh pi@<cm4-ip>
sudo ~/klipper-ethercat/scripts/verify_rt_setup.sh
```
Fix any **FAIL** items before proceeding. **WARN** items are okay for initial testing.

---

## Phase 5: Build the RT Process

### Step 14 — Install build tools and build
```
sudo apt install -y build-essential cmake
cd ~/klipper-ethercat
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j3
```

### Step 15 — Run the tests
```
ctest --output-on-failure
```
All 4 tests should pass (shared_mem, pid, cia402, trajectory).

### Step 16 — Install the binary
```
sudo make install
```
This puts `ethercat_rt` in `/usr/local/bin/`.

---

## Phase 6: Install Klipper Modules

### Step 17 — Find your Klipper installation directory
```
ls ~/klipper/klippy/extras/
```
If Klipper is somewhere else, note the path.

### Step 18 — Copy the Python modules into Klipper
```
cp ~/klipper-ethercat/klippy/extras/ethercat_*.py ~/klipper/klippy/extras/
```

### Step 19 — Copy the drive profile
```
mkdir -p ~/klipper/klippy/extras/ethercat_profiles
cp ~/klipper-ethercat/profiles/lichuan_lc10e.py ~/klipper/klippy/extras/ethercat_profiles/
```

### Step 20 — Copy the ESI XML
```
sudo mkdir -p /etc/klipper-ethercat/esi
sudo cp ~/klipper-ethercat/profiles/lichuan_lc10e.xml /etc/klipper-ethercat/esi/
```

---

## Phase 7: Configure printer.cfg

### Step 21 — Edit your printer.cfg
```
nano ~/printer_data/config/printer.cfg
```

Add these sections (adjust `rotation_distance` and other values for your machine).
See `~/klipper-ethercat/config/example_printer.cfg` for a full reference.

**Minimum additions:**
```ini
[ethercat]
cycle_time: 0.001

[ethercat_stepper stepper_x]
slave_position: 0
drive_type: lichuan_lc10e
encoder_resolution: 8388608
rotation_distance: 40
max_velocity: 500
max_accel: 30000
max_following_error: 2.0
pid_kp: 50.0
pid_ki: 5.0
pid_kd: 0.5
ff_velocity: 1.0
autotune_travel: 30

[ethercat_stepper stepper_y]
slave_position: 1
drive_type: lichuan_lc10e
encoder_resolution: 8388608
rotation_distance: 40
max_velocity: 500
max_accel: 30000
max_following_error: 2.0
pid_kp: 50.0
pid_ki: 5.0
pid_kd: 0.5
ff_velocity: 1.0
autotune_travel: 30

[ethercat_stepper stepper_z]
slave_position: 2
drive_type: lichuan_lc10e
encoder_resolution: 8388608
rotation_distance: 8
max_velocity: 20
max_accel: 500
max_following_error: 1.0
pid_kp: 50.0
pid_ki: 5.0
pid_kd: 0.5
ff_velocity: 1.0
autotune_travel: 10

[ethercat_homing]

[ethercat_autotune]
```

Keep your existing `[stepper_a]` (NEMA 17), `[extruder]`, `[mcu EBBCan]`, and other sections.

### Step 22 — Restart Klipper
```
sudo systemctl restart klipper
```

Check for errors:
```
journalctl -u klipper -f
```

---

## Phase 8: First Motion Test

### Step 23 — Open Mainsail/Fluidd
In your browser, go to `http://<cm4-ip>`

### Step 24 — Enable the drives
In the console:
```
SERVO_ENABLE AXIS=ALL
```
Wait 2 seconds, then check status:
```
ETHERCAT_STATUS
```
All axes should show `OPERATION_ENABLED`.

### Step 25 — Home the axes (absolute encoder, no motion)
```
SERVO_HOME AXIS=X
SERVO_HOME AXIS=Y
SERVO_HOME AXIS=Z
```

### Step 26 — Check positions
```
SERVO_STATUS AXIS=X
SERVO_STATUS AXIS=Y
SERVO_STATUS AXIS=Z
```

### Step 27 — Run auto-tune (axes WILL move — ensure clear travel!)
```
SERVO_AUTOTUNE AXIS=X
SERVO_AUTOTUNE AXIS=Y
SERVO_AUTOTUNE AXIS=Z
SAVE_CONFIG
```

---

## Troubleshooting Quick Reference

| Problem | Check |
|---------|-------|
| `ethercat slaves` shows 0 | Cables, drive power, MAC in `/etc/ethercat.conf` |
| RT process won't start | `journalctl -u ethercat-rt`, check shared memory permissions |
| Klipper errors on startup | `journalctl -u klipper -f`, check printer.cfg syntax |
| High latency / overruns | Run `verify_rt_setup.sh`, check `isolcpus=3` in `/proc/cmdline` |
| Drive goes to FAULT | `SERVO_STATUS AXIS=X` to read error code, power cycle drive |
| No EtherCAT device node | Check `/etc/udev/rules.d/99-ethercat.rules`, run `sudo udevadm control --reload-rules` |
| WiFi not working | Make sure `wpa_supplicant` is not disabled in `configure_rt.sh` |
| Build fails — ecrt.h not found | EtherLab not installed, run `setup_etherlab.sh` first |
| Permission denied on RT | Check user is in `realtime` group: `groups` command, may need logout/login |

## GCode Command Reference

| Command | Description |
|---------|-------------|
| `ETHERCAT_STATUS` | Show bus status, all axes |
| `SERVO_ENABLE AXIS=X` | Enable one axis (or `AXIS=ALL`) |
| `SERVO_DISABLE AXIS=X` | Disable one axis (or `AXIS=ALL`) |
| `SERVO_HOME AXIS=X` | Home axis via absolute encoder |
| `SERVO_HOME AXIS=X METHOD=torque_limit` | Home by hitting hard stop |
| `SERVO_STATUS AXIS=X` | Detailed status with following error stats |
| `SERVO_AUTOTUNE AXIS=X` | Full auto-tune (~60-90 seconds, axis moves) |
| `SERVO_AUTOTUNE AXIS=X AGGRESSION=conservative` | Gentler auto-tune |
| `SERVO_AUTOTUNE AXIS=X VALIDATE_ONLY=1` | Retest current gains only |
| `SERVO_PLOT AXIS=X DURATION=2` | Record 2s of position data to CSV |
| `SERVO_POSITION AXIS=X` | Quick position readout |
