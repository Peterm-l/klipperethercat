#!/bin/bash
# =============================================================================
# RT Setup Verification Script
# =============================================================================
# Run this after setup_rt_kernel.sh, setup_etherlab.sh, and configure_rt.sh
# to confirm everything is properly configured.
#
# Usage:
#   chmod +x verify_rt_setup.sh
#   sudo ./verify_rt_setup.sh
# =============================================================================

PASS=0
FAIL=0
WARN=0

pass() { echo -e "\033[0;32m[PASS]\033[0m $1"; PASS=$((PASS+1)); }
fail() { echo -e "\033[0;31m[FAIL]\033[0m $1"; FAIL=$((FAIL+1)); }
warn() { echo -e "\033[1;33m[WARN]\033[0m $1"; WARN=$((WARN+1)); }
info() { echo -e "\033[0;36m[INFO]\033[0m $1"; }

echo "========================================="
echo "  RT Setup Verification"
echo "  $(date)"
echo "========================================="
echo ""

# --- Kernel ---
echo "--- Kernel ---"

if uname -v | grep -q "PREEMPT_RT"; then
    pass "PREEMPT_RT kernel active: $(uname -r)"
else
    fail "Kernel is NOT PREEMPT_RT: $(uname -v)"
    echo "     Run scripts/setup_rt_kernel.sh and reboot"
fi

# --- CPU Isolation ---
echo ""
echo "--- CPU Isolation ---"

CMDLINE=$(cat /proc/cmdline)

if echo "$CMDLINE" | grep -q "isolcpus=3"; then
    pass "Core 3 isolated (isolcpus=3)"
else
    fail "Core 3 NOT isolated — add isolcpus=3 to /boot/firmware/cmdline.txt and reboot"
fi

if echo "$CMDLINE" | grep -q "nohz_full=3"; then
    pass "nohz_full=3 set (scheduler tick disabled on core 3)"
else
    warn "nohz_full=3 not set — scheduler tick still active on core 3"
fi

if echo "$CMDLINE" | grep -q "rcu_nocbs=3"; then
    pass "rcu_nocbs=3 set (RCU callbacks moved off core 3)"
else
    warn "rcu_nocbs=3 not set — RCU callbacks may cause latency spikes on core 3"
fi

if echo "$CMDLINE" | grep -q "rcu_nocb_poll"; then
    pass "rcu_nocb_poll set"
else
    warn "rcu_nocb_poll not set — RCU still using interrupts on isolated cores"
fi

# --- Processes on core 3 ---
echo ""
echo "--- Core 3 Process Count ---"

procs_on_3=$(ps -eo pid,psr,comm | awk '$2 == 3 {print}' | wc -l)
if [ "$procs_on_3" -le 2 ]; then
    pass "Core 3 has $procs_on_3 processes (expected: 0-2 kernel threads)"
elif [ "$procs_on_3" -le 5 ]; then
    warn "Core 3 has $procs_on_3 processes (should be nearly empty)"
else
    fail "Core 3 has $procs_on_3 processes — isolation not working"
    echo "     Processes on core 3:"
    ps -eo pid,psr,comm | awk '$2 == 3 {print "     " $0}'
fi

# --- CPU Governor ---
echo ""
echo "--- CPU Governor ---"

for cpu in 0 1 2 3; do
    gov_file="/sys/devices/system/cpu/cpu${cpu}/cpufreq/scaling_governor"
    if [ -f "$gov_file" ]; then
        gov=$(cat "$gov_file")
        if [ "$gov" = "performance" ]; then
            pass "CPU${cpu} governor: performance"
        else
            warn "CPU${cpu} governor: $gov (should be 'performance')"
        fi
    fi
done

# --- RT Scheduling Limits ---
echo ""
echo "--- RT Scheduling Limits ---"

if getent group realtime &>/dev/null; then
    pass "Group 'realtime' exists"
    members=$(getent group realtime | cut -d: -f4)
    info "  Members: $members"
else
    fail "Group 'realtime' does not exist"
fi

if [ -f /etc/security/limits.d/99-realtime.conf ]; then
    pass "RT limits config exists (/etc/security/limits.d/99-realtime.conf)"
else
    fail "RT limits config missing"
fi

# --- EtherCAT Master ---
echo ""
echo "--- EtherCAT Master ---"

if lsmod | grep -q ec_master; then
    pass "EtherCAT master module loaded (ec_master)"
else
    fail "EtherCAT master module NOT loaded"
    echo "     Run: sudo systemctl start ethercat"
fi

if lsmod | grep -q ec_generic; then
    pass "EtherCAT generic driver loaded (ec_generic)"
else
    warn "EtherCAT generic driver not loaded"
fi

if [ -c /dev/EtherCAT0 ] || [ -e /dev/EtherCAT0 ]; then
    pass "EtherCAT device node exists (/dev/EtherCAT0)"
else
    warn "EtherCAT device node not found (/dev/EtherCAT0)"
fi

# --- EtherCAT Slaves ---
echo ""
echo "--- EtherCAT Slaves ---"

if command -v ethercat &>/dev/null; then
    slave_output=$(ethercat slaves 2>/dev/null || echo "ERROR")
    if [ "$slave_output" = "ERROR" ]; then
        warn "Could not query EtherCAT slaves (master may not be running)"
    else
        slave_count=$(echo "$slave_output" | grep -c "." 2>/dev/null || echo "0")
        if [ "$slave_count" -ge 3 ]; then
            pass "EtherCAT slaves detected: $slave_count"
            echo "$slave_output" | while read -r line; do
                info "  $line"
            done
        elif [ "$slave_count" -ge 1 ]; then
            warn "Only $slave_count EtherCAT slave(s) detected (expected 3 LC10E drives)"
            echo "$slave_output" | while read -r line; do
                info "  $line"
            done
        else
            warn "No EtherCAT slaves detected — check cables and drive power"
        fi
    fi
else
    warn "'ethercat' command not found — EtherLab may not be installed"
fi

# --- Systemd Services ---
echo ""
echo "--- Systemd Services ---"

for svc in ethercat isolate-rt-core cpu-performance; do
    if systemctl is-enabled "$svc" &>/dev/null; then
        pass "Service enabled: $svc"
    else
        warn "Service not enabled: $svc"
    fi
done

if systemctl is-active ethercat &>/dev/null; then
    pass "EtherCAT master service is running"
else
    warn "EtherCAT master service is not running"
fi

# --- Disabled Services ---
echo ""
echo "--- Jitter-Causing Services ---"

for svc in bluetooth avahi-daemon multipathd ModemManager; do
    if systemctl is-active "$svc" &>/dev/null; then
        warn "Service still active: $svc (should be disabled for RT)"
    else
        pass "Service inactive: $svc"
    fi
done

# --- Shared Memory ---
echo ""
echo "--- Shared Memory ---"

shm_max=$(cat /proc/sys/kernel/shmmax 2>/dev/null || echo "0")
info "Max shared memory segment: $shm_max bytes"

# --- Quick Latency Test ---
echo ""
echo "--- Latency Test ---"

if command -v cyclictest &>/dev/null; then
    info "Running 10-second latency test on core 3..."
    echo "  (This tests worst-case scheduling latency)"
    result=$(sudo cyclictest --smp --priority=95 --affinity=3 \
                  --interval=1000 --duration=10 --mlockall --quiet 2>/dev/null)
    if [ -n "$result" ]; then
        echo "$result" | while read -r line; do
            info "  $line"
        done
        max_latency=$(echo "$result" | grep -oP 'Max:\s+\K\d+' | head -1)
        if [ -n "$max_latency" ]; then
            if [ "$max_latency" -lt 50 ]; then
                pass "Max latency: ${max_latency}us (excellent for servo control)"
            elif [ "$max_latency" -lt 100 ]; then
                pass "Max latency: ${max_latency}us (good for 1ms cycle time)"
            elif [ "$max_latency" -lt 500 ]; then
                warn "Max latency: ${max_latency}us (marginal — check isolation)"
            else
                fail "Max latency: ${max_latency}us (too high for servo control)"
            fi
        fi
    else
        warn "cyclictest produced no output"
    fi
    echo ""
    info "For a thorough test, run:"
    info "  sudo cyclictest -a3 -p95 -i1000 -D 300 -m -q"
    info "  (5-minute test — max latency should be <100us)"
else
    warn "cyclictest not installed — cannot measure RT latency"
    echo "     Install with: sudo apt install rt-tests"
fi

# --- ethercat_rt Binary ---
echo ""
echo "--- EtherCAT RT Binary ---"

if [ -x /usr/local/bin/ethercat_rt ]; then
    pass "ethercat_rt binary exists at /usr/local/bin/ethercat_rt"
else
    info "ethercat_rt binary not yet installed (build with: cd src && mkdir build && cd build && cmake .. && make)"
fi

if [ -f /etc/klipper-ethercat/config.ini ]; then
    pass "RT config exists at /etc/klipper-ethercat/config.ini"
else
    warn "RT config not found at /etc/klipper-ethercat/config.ini"
fi

# --- Summary ---
echo ""
echo "========================================="
echo "  Summary"
echo "========================================="
echo -e "  \033[0;32mPASS: $PASS\033[0m"
echo -e "  \033[1;33mWARN: $WARN\033[0m"
echo -e "  \033[0;31mFAIL: $FAIL\033[0m"
echo ""

if [ "$FAIL" -eq 0 ]; then
    echo -e "\033[0;32mSystem is ready for EtherCAT servo control.\033[0m"
else
    echo -e "\033[0;31mFix the FAIL items above before running EtherCAT servo control.\033[0m"
fi
echo ""
