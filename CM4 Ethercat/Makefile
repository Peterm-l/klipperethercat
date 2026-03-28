# =============================================================================
# Klipper EtherCAT Servo Integration — Top-Level Makefile
# =============================================================================

BUILD_DIR = build
INSTALL_PREFIX = /usr/local

.PHONY: all build clean install test setup-rt setup-etherlab configure-rt verify

all: build

# Build the ethercat_rt binary
build:
	mkdir -p $(BUILD_DIR)
	cd $(BUILD_DIR) && cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$$(nproc)

# Clean build artifacts
clean:
	rm -rf $(BUILD_DIR)

# Install everything (run as root)
install: build
	sudo cp $(BUILD_DIR)/ethercat_rt $(INSTALL_PREFIX)/bin/ 2>/dev/null || true
	sudo chmod +x $(INSTALL_PREFIX)/bin/ethercat_rt 2>/dev/null || true
	@echo "Binary installed to $(INSTALL_PREFIX)/bin/ethercat_rt"
	@echo ""
	@echo "To install Klipper modules and full setup, run:"
	@echo "  sudo scripts/install.sh"

# Run tests
test: build
	cd $(BUILD_DIR) && ctest --output-on-failure

# Setup steps (run individually as needed)
setup-rt:
	sudo bash scripts/setup_rt_kernel.sh

setup-etherlab:
	sudo bash scripts/setup_etherlab.sh

configure-rt:
	sudo bash scripts/configure_rt.sh

verify:
	sudo bash scripts/verify_rt_setup.sh

# Full installation
full-install:
	sudo bash scripts/install.sh
