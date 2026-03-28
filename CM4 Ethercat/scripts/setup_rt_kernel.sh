#!/bin/bash
# =============================================================================
# PREEMPT_RT Kernel Build Script for Raspberry Pi CM4
# =============================================================================
# This script downloads, patches, configures, and builds a PREEMPT_RT kernel
# for the Raspberry Pi CM4 (BCM2711). Run this ON the CM4 itself.
#
# Prerequisites:
#   - Raspberry Pi OS installed on CM4 (eMMC or SD)
#   - Internet connection
#   - ~10GB free disk space
#   - ~2-4 hours build time on CM4 (or cross-compile on a faster machine)
#
# Usage:
#   chmod +x setup_rt_kernel.sh
#   sudo ./setup_rt_kernel.sh
# =============================================================================

set -euo pipefail

# Configuration
KERNEL_BRANCH="rpi-6.6.y"
RT_PATCH_VERSION="6.6.87-rt44"   # Match to kernel branch - check https://cdn.kernel.org/pub/linux/kernel/projects/rt/
KERNEL_SRC_DIR="/usr/src/linux-rt"
NUM_CPUS=$(nproc)

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

info()  { echo -e "${GREEN}[INFO]${NC} $1"; }
warn()  { echo -e "${YELLOW}[WARN]${NC} $1"; }
error() { echo -e "${RED}[ERROR]${NC} $1"; exit 1; }

# Must run as root
if [ "$EUID" -ne 0 ]; then
    error "This script must be run as root (sudo ./setup_rt_kernel.sh)"
fi

# Detect architecture
ARCH=$(uname -m)
if [ "$ARCH" = "aarch64" ]; then
    KERNEL_IMG="Image"
    KERNEL_CONFIG="bcm2711_defconfig"
    ARCH_FLAG="arm64"
    CROSS_COMPILE=""
    info "Detected 64-bit ARM (aarch64) — building natively on CM4"
elif [ "$ARCH" = "armv7l" ]; then
    KERNEL_IMG="zImage"
    KERNEL_CONFIG="bcm2711_defconfig"
    ARCH_FLAG="arm"
    CROSS_COMPILE=""
    info "Detected 32-bit ARM — building natively on CM4"
else
    error "Unsupported architecture: $ARCH. This script is for Raspberry Pi CM4."
fi

info "=== PREEMPT_RT Kernel Build for Raspberry Pi CM4 ==="
info "Kernel branch: $KERNEL_BRANCH"
info "RT patch: $RT_PATCH_VERSION"
info "Build CPUs: $NUM_CPUS"

# Step 1: Install build dependencies
info "Step 1/7: Installing build dependencies..."
apt-get update
apt-get install -y \
    git bc bison flex libssl-dev make \
    libncurses5-dev libncurses-dev \
    crossbuild-essential-arm64 \
    wget xz-utils kmod cpio \
    2>/dev/null || true

# Step 2: Clone Raspberry Pi kernel source
info "Step 2/7: Cloning Raspberry Pi kernel source ($KERNEL_BRANCH)..."
if [ -d "$KERNEL_SRC_DIR" ]; then
    warn "Kernel source directory already exists at $KERNEL_SRC_DIR"
    warn "Removing and re-cloning..."
    rm -rf "$KERNEL_SRC_DIR"
fi

git clone --depth=1 --branch "$KERNEL_BRANCH" \
    https://github.com/raspberrypi/linux.git "$KERNEL_SRC_DIR"

cd "$KERNEL_SRC_DIR"

# Determine exact kernel version from Makefile
KERNEL_VERSION=$(make kernelversion)
info "Kernel version from source: $KERNEL_VERSION"

# Step 3: Download and apply PREEMPT_RT patch
info "Step 3/7: Downloading PREEMPT_RT patch..."

# Extract major.minor for the RT patch URL path
KERNEL_MAJOR_MINOR=$(echo "$KERNEL_VERSION" | grep -oP '^\d+\.\d+')
RT_PATCH_URL="https://cdn.kernel.org/pub/linux/kernel/projects/rt/${KERNEL_MAJOR_MINOR}/patch-${RT_PATCH_VERSION}.patch.xz"

info "Downloading from: $RT_PATCH_URL"
wget -q "$RT_PATCH_URL" -O /tmp/rt-patch.patch.xz || {
    warn "Exact patch version not found. Listing available RT patches for ${KERNEL_MAJOR_MINOR}..."
    wget -q "https://cdn.kernel.org/pub/linux/kernel/projects/rt/${KERNEL_MAJOR_MINOR}/" -O /tmp/rt-listing.html 2>/dev/null
    grep -oP 'patch-\d+\.\d+\.\d+-rt\d+' /tmp/rt-listing.html | sort -V | tail -5
    error "Download the correct RT patch version and update RT_PATCH_VERSION in this script"
}

info "Applying PREEMPT_RT patch..."
xzcat /tmp/rt-patch.patch.xz | patch -p1 --forward || {
    warn "Some patches may have already been applied (this is OK if re-running)"
}

# Step 4: Configure the kernel
info "Step 4/7: Configuring kernel for PREEMPT_RT..."

# Start with the default CM4 config
make ARCH=$ARCH_FLAG $KERNEL_CONFIG

# Apply RT-specific configuration changes
# These are written to .config and then validated with olddefconfig
cat >> .config << 'RTCONFIG'
# === PREEMPT_RT Configuration ===
# Enable full PREEMPT_RT (real-time) preemption
CONFIG_PREEMPT_RT=y
CONFIG_PREEMPT=y

# Use periodic timer ticks at 1000Hz for consistent RT timing
# Do NOT use NO_HZ_FULL globally — it's enabled per-core via kernel cmdline
CONFIG_HZ_1000=y
CONFIG_HZ=1000

# Disable kernel debugging that adds latency
# CONFIG_DEBUG_PREEMPT is not set
# CONFIG_DEBUG_RT_MUTEXES is not set
# CONFIG_DEBUG_SPINLOCK is not set
# CONFIG_DEBUG_MUTEXES is not set
# CONFIG_DEBUG_LOCK_ALLOC is not set
# CONFIG_PROVE_LOCKING is not set
# CONFIG_LOCKDEP is not set
# CONFIG_LOCK_STAT is not set
# CONFIG_DEBUG_OBJECTS is not set

# Disable kernel profiling/tracing (adds latency)
# CONFIG_PROFILING is not set
# CONFIG_KPROBES is not set

# Enable high-resolution timers (required for clock_nanosleep precision)
CONFIG_HIGH_RES_TIMERS=y

# Memory locking support (for mlockall in RT process)
CONFIG_MEMLOCK=y

# CPU isolation support (for isolcpus= kernel parameter)
CONFIG_CPU_ISOLATION=y

# Kernel module support (needed for IgH EtherCAT master)
CONFIG_MODULES=y
CONFIG_MODULE_UNLOAD=y
RTCONFIG

# Validate and fill in defaults for any new config options
make ARCH=$ARCH_FLAG olddefconfig

# Verify PREEMPT_RT was actually enabled
if grep -q "CONFIG_PREEMPT_RT=y" .config; then
    info "PREEMPT_RT enabled in kernel config"
else
    error "Failed to enable PREEMPT_RT in kernel config. Check patch compatibility."
fi

# Step 5: Build the kernel
info "Step 5/7: Building kernel (this will take a while on CM4)..."
info "Using $NUM_CPUS parallel jobs"

make ARCH=$ARCH_FLAG -j"$NUM_CPUS" $KERNEL_IMG modules dtbs 2>&1 | \
    tail -1  # Show only the final line to avoid flooding the terminal

# Step 6: Install the kernel
info "Step 6/7: Installing kernel and modules..."

# Install modules
make ARCH=$ARCH_FLAG modules_install

# Backup the current kernel
BOOT_DIR="/boot/firmware"
if [ ! -d "$BOOT_DIR" ]; then
    BOOT_DIR="/boot"
fi

cp "$BOOT_DIR/kernel8.img" "$BOOT_DIR/kernel8.img.bak" 2>/dev/null || true

# Install the new kernel image
if [ "$ARCH_FLAG" = "arm64" ]; then
    cp arch/arm64/boot/Image "$BOOT_DIR/kernel8.img"
else
    cp arch/arm/boot/zImage "$BOOT_DIR/kernel7l.img"
fi

# Install device tree blobs
cp arch/$ARCH_FLAG/boot/dts/broadcom/*.dtb "$BOOT_DIR/" 2>/dev/null || true
cp arch/$ARCH_FLAG/boot/dts/overlays/*.dtb* "$BOOT_DIR/overlays/" 2>/dev/null || true

# Step 7: Configure boot parameters
info "Step 7/7: Configuring boot parameters..."

# Add RT-specific kernel command line parameters
CMDLINE_FILE="$BOOT_DIR/cmdline.txt"
if [ -f "$CMDLINE_FILE" ]; then
    # Backup
    cp "$CMDLINE_FILE" "${CMDLINE_FILE}.bak"

    # Add isolcpus and RT parameters if not already present
    CURRENT_CMDLINE=$(cat "$CMDLINE_FILE")
    PARAMS_TO_ADD=""

    for param in "isolcpus=3" "nohz_full=3" "rcu_nocbs=3" "rcu_nocb_poll"; do
        if ! echo "$CURRENT_CMDLINE" | grep -q "$param"; then
            PARAMS_TO_ADD="$PARAMS_TO_ADD $param"
        fi
    done

    if [ -n "$PARAMS_TO_ADD" ]; then
        # cmdline.txt must be a single line
        echo "${CURRENT_CMDLINE}${PARAMS_TO_ADD}" > "$CMDLINE_FILE"
        info "Added kernel parameters:${PARAMS_TO_ADD}"
    else
        info "RT kernel parameters already present in cmdline.txt"
    fi
else
    warn "Could not find $CMDLINE_FILE — manually add: isolcpus=3 nohz_full=3 rcu_nocbs=3 rcu_nocb_poll"
fi

info ""
info "=== PREEMPT_RT Kernel Build Complete ==="
info ""
info "Kernel version: $(make kernelversion)"
info "Installed to: $BOOT_DIR"
info ""
info "IMPORTANT: You must reboot for the new kernel to take effect:"
info "  sudo reboot"
info ""
info "After reboot, verify with:"
info "  uname -v   # Should show PREEMPT_RT"
info "  cat /proc/cmdline   # Should show isolcpus=3"
info ""
info "If the new kernel fails to boot, recover by:"
info "  1. Mount the SD/eMMC on another machine"
info "  2. Copy kernel8.img.bak back to kernel8.img"
info ""
