#!/bin/bash
# =============================================================================
# CPU Isolation and Real-Time Configuration Script
# =============================================================================
# CRITICAL: Without proper core isolation, the ethercat_rt process will
# experience multi-millisecond latency spikes from Linux kernel tasks,
# making servo control unstable.
#
# This script:
#   a) Verifies/adds kernel command line parameters for CPU isolation
#   b) Creates a systemd service to move IRQs off the isolated core at boot
#   c) Disables unnecessary services that cause RT latency
#   d) Sets CPU governor to performance
#   e) Configures RT scheduling privileges
#   f) Configures display settings for minimal jitter
#   g) Creates the ethercat-rt systemd service
#
# Usage:
#   chmod +x configure_rt.sh
#   sudo ./configure_rt.sh
# =============================================================================

set -euo pipefail

RT_CPU_CORE=3
RT_CPU_MASK=7  # Bitmask for cores 0,1,2 (binary: 0111)

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

info()  { echo -e "${GREEN}[INFO]${NC} $1"; }
warn()  { echo -e "${YELLOW}[WARN]${NC} $1"; }
error() { echo -e "${RED}[ERROR]${NC} $1"; exit 1; }

if [ "$EUID" -ne 0 ]; then
    error "This script must be run as root (sudo ./configure_rt.sh)"
fi

info "=== CPU Isolation and RT Configuration ==="
info "Isolating core $RT_CPU_CORE for real-time EtherCAT process"

# =========================================================================
# (a) Kernel command line — isolate core 3 from the Linux scheduler
# =========================================================================
info ""
info "--- (a) Kernel Command Line ---"

BOOT_DIR="/boot/firmware"
[ -d "$BOOT_DIR" ] || BOOT_DIR="/boot"
CMDLINE_FILE="$BOOT_DIR/cmdline.txt"

if [ ! -f "$CMDLINE_FILE" ]; then
    error "Cannot find $CMDLINE_FILE"
fi

# Backup
cp "$CMDLINE_FILE" "${CMDLINE_FILE}.bak.$(date +%Y%m%d%H%M%S)"

CURRENT_CMDLINE=$(cat "$CMDLINE_FILE")
MODIFIED=0

for param in "isolcpus=$RT_CPU_CORE" "nohz_full=$RT_CPU_CORE" "rcu_nocbs=$RT_CPU_CORE" "rcu_nocb_poll"; do
    if ! echo "$CURRENT_CMDLINE" | grep -q "$param"; then
        CURRENT_CMDLINE="$CURRENT_CMDLINE $param"
        MODIFIED=1
        info "Adding: $param"
    else
        info "Already set: $param"
    fi
done

if [ "$MODIFIED" -eq 1 ]; then
    echo "$CURRENT_CMDLINE" > "$CMDLINE_FILE"
    info "Updated $CMDLINE_FILE"
    warn "Reboot required for kernel parameter changes to take effect"
else
    info "All kernel parameters already present"
fi

# =========================================================================
# (b) Move all IRQ handlers off the isolated core
# =========================================================================
info ""
info "--- (b) IRQ Affinity Service ---"

cat > /usr/local/bin/isolate-rt-core.sh << IRQSCRIPT
#!/bin/bash
# Move all IRQ handlers off core $RT_CPU_CORE
# Run at boot before ethercat_rt starts

RT_CORE=$RT_CPU_CORE
OTHER_CORES="0-2"
IRQ_MASK=$RT_CPU_MASK

# Move all existing IRQs to cores 0-2
for irq in /proc/irq/*/smp_affinity_list; do
    echo "\$OTHER_CORES" > "\$irq" 2>/dev/null || true
done

# Set default IRQ affinity for any new IRQs
echo \$IRQ_MASK > /proc/irq/default_smp_affinity 2>/dev/null || true

# Move kernel threads off the isolated core
for pid in \$(ps -eo pid,psr | awk '\$2 == '\$RT_CORE' {print \$1}'); do
    taskset -cp \$OTHER_CORES \$pid 2>/dev/null || true
done

echo "IRQs and kernel threads moved off core \$RT_CORE"
IRQSCRIPT
chmod +x /usr/local/bin/isolate-rt-core.sh

cat > /etc/systemd/system/isolate-rt-core.service << 'EOF'
[Unit]
Description=Isolate CPU core for real-time EtherCAT process
After=sysinit.target
Before=ethercat.service

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/usr/local/bin/isolate-rt-core.sh

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable isolate-rt-core.service
info "Created and enabled isolate-rt-core.service"

# =========================================================================
# (c) Disable unnecessary services
# =========================================================================
info ""
info "--- (c) Disabling Unnecessary Services ---"

# Services to disable (non-essential for a 3D printer controller)
SERVICES_TO_DISABLE=(
    "bluetooth"
    "avahi-daemon"
    "multipathd"          # Known to cause RT latency spikes
    "ModemManager"
    "triggerhappy"
    "pigpiod"
)

# Services to mask (prevent starting entirely)
SERVICES_TO_MASK=(
    "apt-daily.timer"
    "apt-daily-upgrade.timer"
    "man-db.timer"
)

for svc in "${SERVICES_TO_DISABLE[@]}"; do
    if systemctl is-enabled "$svc" &>/dev/null; then
        systemctl disable "$svc"
        systemctl stop "$svc" 2>/dev/null || true
        info "Disabled: $svc"
    else
        info "Already disabled or not installed: $svc"
    fi
done

for svc in "${SERVICES_TO_MASK[@]}"; do
    systemctl mask "$svc" 2>/dev/null || true
    info "Masked: $svc"
done

# NOTE: We do NOT disable wpa_supplicant because CM4 WiFi is needed
# for web interface access
info "Keeping wpa_supplicant enabled (needed for WiFi web interface)"

# =========================================================================
# (d) Set CPU governor to performance
# =========================================================================
info ""
info "--- (d) CPU Governor ---"

# Set immediately if possible
for cpu in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
    echo "performance" > "$cpu" 2>/dev/null || true
done

# Make persistent via systemd service
cat > /etc/systemd/system/cpu-performance.service << 'EOF'
[Unit]
Description=Set CPU governor to performance mode
After=sysinit.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/bin/bash -c 'for cpu in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do echo performance > $cpu; done'

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable cpu-performance.service
info "CPU governor set to performance (persistent)"

# =========================================================================
# (e) Configure RT scheduling privileges
# =========================================================================
info ""
info "--- (e) RT Scheduling Privileges ---"

# Create realtime group if it doesn't exist
groupadd -f realtime

# Add common users to realtime group
for user in pi klipper root; do
    if id "$user" &>/dev/null; then
        usermod -aG realtime "$user"
        info "Added user '$user' to realtime group"
    fi
done

# Configure security limits for RT scheduling
cat > /etc/security/limits.d/99-realtime.conf << 'EOF'
# Real-time scheduling limits for EtherCAT servo control
# Members of the 'realtime' group can:
#   - Use SCHED_FIFO/SCHED_RR with max priority
#   - Lock unlimited memory (mlockall)
#   - Set maximum nice value

@realtime   -   rtprio      99
@realtime   -   memlock     unlimited
@realtime   -   nice        -20
EOF

info "RT limits configured in /etc/security/limits.d/99-realtime.conf"

# Also configure PAM to apply limits
if ! grep -q "pam_limits.so" /etc/pam.d/common-session 2>/dev/null; then
    echo "session required pam_limits.so" >> /etc/pam.d/common-session
    info "Added pam_limits.so to common-session"
fi

# =========================================================================
# (f) Display configuration (minimize jitter)
# =========================================================================
info ""
info "--- (f) Display Configuration ---"

# For a headless 3D printer controller, disable the display manager entirely
# This eliminates all GPU-related jitter
if systemctl is-enabled lightdm &>/dev/null || \
   systemctl is-enabled gdm3 &>/dev/null || \
   systemctl is-enabled sddm &>/dev/null; then

    warn "Display manager detected. For best RT performance, run headless."
    warn "Switching to multi-user (CLI) target..."
    systemctl set-default multi-user.target
    info "Set default target to multi-user.target (headless)"
    info "Access the printer via WiFi web interface (Mainsail/Fluidd)"
else
    info "No display manager active — already headless (good)"
fi

# If X11/Wayland is needed for some reason, ensure X11 is used
if command -v raspi-config &>/dev/null; then
    raspi-config nonint do_wayland W1 2>/dev/null || true
fi

# =========================================================================
# (g) Create ethercat-rt systemd service
# =========================================================================
info ""
info "--- (g) EtherCAT RT Process Service ---"

# Create config directory
mkdir -p /etc/klipper-ethercat

# Create default config file
cat > /etc/klipper-ethercat/config.ini << 'EOF'
[general]
# Number of EtherCAT servo axes
num_axes = 3
# EtherCAT cycle time in microseconds
cycle_time_us = 1000
# Isolated CPU core for RT thread
rt_cpu_core = 3
# RT scheduling priority (below watchdog at 99)
rt_priority = 95

[axis_0]
# X axis
name = X
slave_position = 0
encoder_resolution = 8388608
rotation_distance = 40.0
max_velocity = 500.0
max_accel = 30000.0
max_following_error = 2.0

[axis_1]
# Y axis
name = Y
slave_position = 1
encoder_resolution = 8388608
rotation_distance = 40.0
max_velocity = 500.0
max_accel = 30000.0
max_following_error = 2.0

[axis_2]
# Z axis
name = Z
slave_position = 2
encoder_resolution = 8388608
rotation_distance = 8.0
max_velocity = 20.0
max_accel = 500.0
max_following_error = 1.0
EOF

# Create the systemd service
cat > /etc/systemd/system/ethercat-rt.service << 'EOF'
[Unit]
Description=Klipper EtherCAT Real-Time Servo Controller
After=ethercat.service isolate-rt-core.service
Requires=ethercat.service
Wants=isolate-rt-core.service

[Service]
Type=simple
ExecStart=/usr/local/bin/ethercat_rt --config /etc/klipper-ethercat/config.ini

# Run as root for RT privileges (or use capabilities)
User=root

# Do NOT set CPUAffinity here — the process pins itself in code
# This avoids a race condition where systemd pins before mlockall
LimitMEMLOCK=infinity
LimitRTPRIO=99
LimitNICE=-20

# Restart on failure with a short delay
Restart=on-failure
RestartSec=2

# Logging
StandardOutput=journal
StandardError=journal
SyslogIdentifier=ethercat-rt

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
# Don't enable yet — the binary doesn't exist until we build it
info "Created ethercat-rt.service (not enabled until binary is built)"

info ""
info "=== RT Configuration Complete ==="
info ""
info "Summary of changes:"
info "  - Kernel cmdline: isolcpus=3 nohz_full=3 rcu_nocbs=3 rcu_nocb_poll"
info "  - IRQ isolation service: isolate-rt-core.service (enabled)"
info "  - Disabled services: bluetooth, avahi, multipathd, ModemManager"
info "  - CPU governor: performance (persistent)"
info "  - RT limits: rtprio=99, memlock=unlimited for @realtime group"
info "  - Display: headless (multi-user.target)"
info "  - ethercat-rt.service: created (enable after building ethercat_rt)"
info "  - Config: /etc/klipper-ethercat/config.ini"
info ""
info "REBOOT REQUIRED for kernel parameters to take effect."
info "After reboot, run scripts/verify_rt_setup.sh to confirm."
info ""
