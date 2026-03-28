#!/bin/bash
# =============================================================================
# Klipper EtherCAT Servo Integration — Installation Script
# =============================================================================
# Run this on the Raspberry Pi CM4 to install all components.
#
# Prerequisites:
#   - Raspberry Pi OS with Klipper already installed
#   - Internet connection
#   - This repository cloned to the CM4
#
# Usage:
#   cd klipper-ethercat
#   chmod +x scripts/install.sh
#   sudo scripts/install.sh
# =============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
KLIPPER_DIR="${KLIPPER_DIR:-$HOME/klipper}"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

info()  { echo -e "${GREEN}[INFO]${NC} $1"; }
warn()  { echo -e "${YELLOW}[WARN]${NC} $1"; }
error() { echo -e "${RED}[ERROR]${NC} $1"; exit 1; }
step()  { echo -e "\n${CYAN}=== $1 ===${NC}"; }

if [ "$EUID" -ne 0 ]; then
    error "This script must be run as root (sudo scripts/install.sh)"
fi

echo ""
echo "============================================"
echo "  Klipper EtherCAT Servo Integration"
echo "  Installation Script"
echo "============================================"
echo ""
echo "Project directory: $PROJECT_DIR"
echo "Klipper directory: $KLIPPER_DIR"
echo ""

# Verify Klipper is installed
if [ ! -d "$KLIPPER_DIR" ]; then
    error "Klipper not found at $KLIPPER_DIR. Set KLIPPER_DIR env variable."
fi

# =========================================================================
step "Step 1/7: Install build dependencies"
# =========================================================================
apt-get update
apt-get install -y \
    build-essential cmake \
    python3-dev python3-pip \
    rt-tests \
    2>/dev/null || true

# =========================================================================
step "Step 2/7: Build PREEMPT_RT kernel (if needed)"
# =========================================================================
if uname -v | grep -q "PREEMPT_RT"; then
    info "PREEMPT_RT kernel already running: $(uname -r)"
else
    warn "PREEMPT_RT kernel not detected."
    read -p "Build and install RT kernel now? This takes 2-4 hours. (y/N) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        bash "$SCRIPT_DIR/setup_rt_kernel.sh"
        warn "Reboot required after kernel install. Run this script again after reboot."
        exit 0
    else
        warn "Skipping RT kernel build. EtherCAT will work but RT performance may be degraded."
    fi
fi

# =========================================================================
step "Step 3/7: Install IgH EtherLab Master (if needed)"
# =========================================================================
if lsmod | grep -q ec_master 2>/dev/null || [ -f /usr/local/lib/libethercat.so ]; then
    info "IgH EtherLab appears to be installed"
else
    info "Installing IgH EtherLab Master..."
    bash "$SCRIPT_DIR/setup_etherlab.sh"
fi

# =========================================================================
step "Step 4/7: Configure CPU isolation and RT settings"
# =========================================================================
bash "$SCRIPT_DIR/configure_rt.sh"

# =========================================================================
step "Step 5/7: Build ethercat_rt C process"
# =========================================================================
BUILD_DIR="$PROJECT_DIR/build"
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

cmake "$PROJECT_DIR" -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

# Install the binary
if [ -f ethercat_rt ]; then
    cp ethercat_rt /usr/local/bin/
    chmod +x /usr/local/bin/ethercat_rt
    info "Installed ethercat_rt to /usr/local/bin/"
else
    warn "ethercat_rt binary not built (EtherLab may not be installed)"
fi

cd "$PROJECT_DIR"

# =========================================================================
step "Step 6/7: Install Klipper modules"
# =========================================================================
EXTRAS_DIR="$KLIPPER_DIR/klippy/extras"

if [ ! -d "$EXTRAS_DIR" ]; then
    error "Klipper extras directory not found: $EXTRAS_DIR"
fi

# Copy Python modules
for module in ethercat_shm ethercat_servo ethercat_stepper ethercat_homing ethercat_autotune; do
    src="$PROJECT_DIR/klippy/extras/${module}.py"
    dst="$EXTRAS_DIR/${module}.py"
    if [ -f "$src" ]; then
        cp "$src" "$dst"
        info "Installed $module.py to $EXTRAS_DIR/"
    else
        warn "Module not found: $src"
    fi
done

# Copy drive profiles
PROFILE_DIR="$KLIPPER_DIR/klippy/extras/ethercat_profiles"
mkdir -p "$PROFILE_DIR"
cp "$PROJECT_DIR/profiles/"*.py "$PROFILE_DIR/" 2>/dev/null || true
info "Installed drive profiles"

# Copy ESI XML files
ESI_DIR="/etc/klipper-ethercat/esi"
mkdir -p "$ESI_DIR"
cp "$PROJECT_DIR/"*.xml "$ESI_DIR/" 2>/dev/null || true
info "Installed ESI XML files to $ESI_DIR/"

# =========================================================================
step "Step 7/7: Enable systemd services"
# =========================================================================
systemctl daemon-reload

# Enable EtherCAT master service
systemctl enable ethercat.service 2>/dev/null || true

# Enable RT core isolation service
systemctl enable isolate-rt-core.service 2>/dev/null || true

# Enable CPU performance governor service
systemctl enable cpu-performance.service 2>/dev/null || true

# Enable ethercat-rt service (but don't start yet)
if [ -f /usr/local/bin/ethercat_rt ]; then
    systemctl enable ethercat-rt.service 2>/dev/null || true
    info "Enabled ethercat-rt.service"
fi

echo ""
echo "============================================"
echo "  Installation Complete"
echo "============================================"
echo ""
echo "Next steps:"
echo ""
echo "  1. If you haven't rebooted after RT kernel install:"
echo "     sudo reboot"
echo ""
echo "  2. After reboot, verify the setup:"
echo "     sudo bash $SCRIPT_DIR/verify_rt_setup.sh"
echo ""
echo "  3. Connect LC10E drives and start EtherCAT:"
echo "     sudo systemctl start ethercat"
echo "     ethercat slaves"
echo ""
echo "  4. Add EtherCAT sections to your printer.cfg:"
echo "     See config/example_printer.cfg for reference"
echo ""
echo "  5. Restart Klipper:"
echo "     sudo systemctl restart klipper"
echo ""
echo "  6. Enable and home axes:"
echo "     SERVO_ENABLE AXIS=ALL"
echo "     SERVO_HOME AXIS=X"
echo "     SERVO_HOME AXIS=Y"
echo "     SERVO_HOME AXIS=Z"
echo ""
echo "  7. Run auto-tune:"
echo "     SERVO_AUTOTUNE AXIS=X"
echo "     SERVO_AUTOTUNE AXIS=Y"
echo "     SERVO_AUTOTUNE AXIS=Z"
echo "     SAVE_CONFIG"
echo ""
