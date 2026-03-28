#!/bin/bash
# =============================================================================
# IgH EtherLab Master Installation Script for Raspberry Pi CM4
# =============================================================================
# Installs the IgH EtherCAT Master (EtherLab) as a kernel module.
# The CM4's onboard Ethernet (bcmgenet) is used as the EtherCAT interface.
#
# Prerequisites:
#   - PREEMPT_RT kernel installed and running (run setup_rt_kernel.sh first)
#   - Internet connection
#   - Kernel headers installed
#
# Usage:
#   chmod +x setup_etherlab.sh
#   sudo ./setup_etherlab.sh
# =============================================================================

set -euo pipefail

# Configuration
ETHERLAB_VERSION="1.5.2"
ETHERLAB_REPO="https://gitlab.com/etherlab.org/ethercat.git"
ETHERLAB_SRC_DIR="/usr/src/ethercat-${ETHERLAB_VERSION}"
INSTALL_PREFIX="/usr/local"
ETHERNET_IFACE="eth0"       # CM4 onboard Ethernet (bcmgenet)
ETHERNET_MAC=""              # Auto-detected below

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

info()  { echo -e "${GREEN}[INFO]${NC} $1"; }
warn()  { echo -e "${YELLOW}[WARN]${NC} $1"; }
error() { echo -e "${RED}[ERROR]${NC} $1"; exit 1; }

if [ "$EUID" -ne 0 ]; then
    error "This script must be run as root (sudo ./setup_etherlab.sh)"
fi

# Verify PREEMPT_RT kernel is running
if ! uname -v | grep -q "PREEMPT_RT"; then
    warn "Kernel is NOT PREEMPT_RT. EtherCAT will work but RT performance will be degraded."
    warn "Run setup_rt_kernel.sh first for production use."
    read -p "Continue anyway? (y/N) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Auto-detect Ethernet interface and MAC
info "Detecting Ethernet interface..."
if ip link show "$ETHERNET_IFACE" &>/dev/null; then
    ETHERNET_MAC=$(ip link show "$ETHERNET_IFACE" | grep ether | awk '{print $2}')
    info "Found $ETHERNET_IFACE with MAC: $ETHERNET_MAC"
else
    # Try to find the bcmgenet interface
    for iface in $(ls /sys/class/net/); do
        if [ -d "/sys/class/net/$iface/device" ]; then
            DRIVER=$(basename "$(readlink /sys/class/net/$iface/device/driver)" 2>/dev/null || echo "")
            if [ "$DRIVER" = "bcmgenet" ] || [ "$DRIVER" = "genet" ]; then
                ETHERNET_IFACE="$iface"
                ETHERNET_MAC=$(ip link show "$iface" | grep ether | awk '{print $2}')
                info "Found bcmgenet interface: $ETHERNET_IFACE ($ETHERNET_MAC)"
                break
            fi
        fi
    done

    if [ -z "$ETHERNET_MAC" ]; then
        error "Could not find Ethernet interface. Check network configuration."
    fi
fi

info "=== IgH EtherLab Master Installation ==="
info "EtherLab version: $ETHERLAB_VERSION"
info "Ethernet interface: $ETHERNET_IFACE ($ETHERNET_MAC)"

# Step 1: Install build dependencies
info "Step 1/8: Installing build dependencies..."
apt-get update
apt-get install -y \
    build-essential autoconf automake libtool pkg-config \
    linux-headers-$(uname -r) \
    dkms \
    2>/dev/null || {
    warn "Some packages may not be available. Installing kernel headers manually..."
    apt-get install -y raspberrypi-kernel-headers 2>/dev/null || true
}

# Step 2: Clone EtherLab source
info "Step 2/8: Cloning EtherLab source..."
if [ -d "$ETHERLAB_SRC_DIR" ]; then
    warn "Source directory exists, removing..."
    rm -rf "$ETHERLAB_SRC_DIR"
fi

git clone "$ETHERLAB_REPO" "$ETHERLAB_SRC_DIR"
cd "$ETHERLAB_SRC_DIR"

# Checkout stable version if available
git tag -l | grep -q "$ETHERLAB_VERSION" && git checkout "v${ETHERLAB_VERSION}" 2>/dev/null || {
    info "Using latest master branch"
}

# Step 3: Generate build system
info "Step 3/8: Generating build system..."
./bootstrap || {
    # If bootstrap fails, try autoreconf
    autoreconf -i
}

# Step 4: Configure
info "Step 4/8: Configuring EtherLab..."

# The CM4 uses bcmgenet Ethernet driver. IgH doesn't have a native driver
# for bcmgenet, so we use the generic Ethernet driver which works by
# replacing the standard network stack with raw Ethernet frame handling.
./configure \
    --prefix="$INSTALL_PREFIX" \
    --sysconfdir=/etc \
    --with-linux-dir="/lib/modules/$(uname -r)/build" \
    --enable-generic \
    --disable-8139too \
    --disable-e100 \
    --disable-e1000 \
    --disable-e1000e \
    --disable-r8169 \
    --enable-cycles \
    --enable-hrtimer

# Step 5: Build
info "Step 5/8: Building EtherLab (kernel modules + userspace tools)..."
make -j$(nproc)

# Step 6: Install
info "Step 6/8: Installing EtherLab..."
make install

# Install kernel modules
make modules_install 2>/dev/null || {
    # Manual module install if make target doesn't exist
    MODULES_DIR="/lib/modules/$(uname -r)/extra"
    mkdir -p "$MODULES_DIR"
    find . -name "*.ko" -exec cp {} "$MODULES_DIR/" \;
}

# Update module dependencies
depmod -a

# Step 7: Configure EtherCAT master
info "Step 7/8: Configuring EtherCAT master..."

# Create the EtherCAT configuration file
cat > /etc/ethercat.conf << EOF
# IgH EtherCAT Master Configuration
# Generated by setup_etherlab.sh

# MAC address of the Ethernet interface used for EtherCAT
# This is the CM4 onboard Ethernet (bcmgenet)
MASTER0_DEVICE="${ETHERNET_MAC}"

# Use the generic Ethernet driver (bcmgenet doesn't have a native IgH driver)
DEVICE_MODULES="generic"
EOF

info "EtherCAT config written to /etc/ethercat.conf"

# Create udev rules for EtherCAT device node
cat > /etc/udev/rules.d/99-ethercat.rules << 'EOF'
# IgH EtherCAT Master device node
KERNEL=="EtherCAT[0-9]*", MODE="0664", GROUP="realtime"
EOF

# Reload udev rules
udevadm control --reload-rules

# Create systemd service for EtherCAT master
cat > /etc/systemd/system/ethercat.service << 'EOF'
[Unit]
Description=IgH EtherCAT Master
After=network-pre.target
Before=network.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStartPre=/sbin/modprobe ec_master
ExecStartPre=/sbin/modprobe ec_generic
ExecStart=/usr/local/sbin/ethercatctl start
ExecStop=/usr/local/sbin/ethercatctl stop
ExecStopPost=/sbin/modprobe -r ec_generic
ExecStopPost=/sbin/modprobe -r ec_master

[Install]
WantedBy=multi-user.target
EOF

# If ethercatctl doesn't exist, create a simple start/stop script
if [ ! -f "$INSTALL_PREFIX/sbin/ethercatctl" ]; then
    cat > "$INSTALL_PREFIX/sbin/ethercatctl" << 'CTRL'
#!/bin/bash
case "$1" in
    start)
        # The modules are already loaded by ExecStartPre
        echo "EtherCAT master started"
        ;;
    stop)
        echo "EtherCAT master stopped"
        ;;
    *)
        echo "Usage: ethercatctl {start|stop}"
        exit 1
        ;;
esac
CTRL
    chmod +x "$INSTALL_PREFIX/sbin/ethercatctl"
fi

# Create init script as alternative to systemd
cat > /etc/init.d/ethercat << 'INITSCRIPT'
#!/bin/bash
### BEGIN INIT INFO
# Provides:          ethercat
# Required-Start:    $network
# Required-Stop:     $network
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Description:       IgH EtherCAT Master
### END INIT INFO

MODPROBE=/sbin/modprobe
ETHERCAT=/usr/local/bin/ethercat

case "$1" in
    start)
        echo "Starting EtherCAT master..."
        $MODPROBE ec_master
        $MODPROBE ec_generic
        echo "EtherCAT master started"
        ;;
    stop)
        echo "Stopping EtherCAT master..."
        $MODPROBE -r ec_generic 2>/dev/null
        $MODPROBE -r ec_master 2>/dev/null
        echo "EtherCAT master stopped"
        ;;
    restart)
        $0 stop
        sleep 1
        $0 start
        ;;
    status)
        if lsmod | grep -q ec_master; then
            echo "EtherCAT master is running"
            $ETHERCAT master 2>/dev/null || true
        else
            echo "EtherCAT master is not running"
        fi
        ;;
    *)
        echo "Usage: $0 {start|stop|restart|status}"
        exit 1
        ;;
esac
INITSCRIPT
chmod +x /etc/init.d/ethercat

# Step 8: Enable and set up library path
info "Step 8/8: Final configuration..."

# Add library path
echo "$INSTALL_PREFIX/lib" > /etc/ld.so.conf.d/ethercat.conf
ldconfig

# Enable the systemd service
systemctl daemon-reload
systemctl enable ethercat.service

# Add ethercat command to PATH if not already there
if ! grep -q "$INSTALL_PREFIX/bin" /etc/environment 2>/dev/null; then
    echo "PATH=\"$INSTALL_PREFIX/bin:$INSTALL_PREFIX/sbin:\$PATH\"" >> /etc/environment
fi

info ""
info "=== IgH EtherLab Master Installation Complete ==="
info ""
info "To start the EtherCAT master now:"
info "  sudo systemctl start ethercat"
info ""
info "To verify slaves are detected:"
info "  ethercat slaves"
info ""
info "Expected output with 3 LC10E drives:"
info "  0  0:0  PREOP  +  LC10E_V1.04"
info "  1  0:1  PREOP  +  LC10E_V1.04"
info "  2  0:2  PREOP  +  LC10E_V1.04"
info ""
info "To check master status:"
info "  ethercat master"
info ""
info "If no slaves are detected:"
info "  1. Check Ethernet cable connections (daisy chain)"
info "  2. Check LC10E drive power"
info "  3. Verify MAC address in /etc/ethercat.conf"
info "  4. Check: dmesg | grep -i ethercat"
info ""
info "IMPORTANT: The onboard Ethernet is now dedicated to EtherCAT."
info "Use WiFi for network access (CM4 WiFi is independent)."
info ""
