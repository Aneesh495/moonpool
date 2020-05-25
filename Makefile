# Moonpool: Subsea Autonomous Intervention and Acoustic Navigation Executive
# Build and Verification Makefile

CC ?= clang
CFLAGS ?= -std=c17 -O3 -Wall -Wextra -Werror -pedantic -fPIC
LDFLAGS ?= -shared

PYTHON ?= python3
RUNNER := tools/eu_runner.py
LINTER := tools/lint_no_em_dash.py

BUILD_DIR := build
BIN_DIR := bin
FASTPATH_SRC := src/fastpath/fastpath.c
FASTPATH_LIB := $(BIN_DIR)/libfastpath.dylib

.PHONY: all bootstrap test test-unit test-fastpath test-integration bench sim lint clean help

all: bootstrap $(FASTPATH_LIB)
	@echo "Build complete: Moonpool executive ready."

bootstrap:
	@tools/bootstrap.sh

$(FASTPATH_LIB): $(FASTPATH_SRC)
	@mkdir -p $(BIN_DIR) $(BUILD_DIR)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $<
	@echo "Compiled C17 fastpath shared library: $@"

lint:
	@$(PYTHON) $(LINTER)

test-unit: all
	@echo "Executing Euphoria unit test matrix..."
	@$(PYTHON) tools/test_runner.py --unit

test-fastpath: all
	@echo "Executing C17 fastpath test suite..."
	@$(PYTHON) tools/test_runner.py --fastpath

test-integration: all
	@echo "Executing subsea mission scenario integration tests..."
	@$(PYTHON) tools/test_runner.py --integration

test: lint test-unit test-fastpath test-integration
	@echo "All Moonpool test suites passed with 100% success."

bench: all
	@echo "Running Moonpool performance benchmarks..."
	@$(PYTHON) tools/test_runner.py --bench

sim: all
	@echo "Running subsea pipeline intervention simulation..."
	@$(PYTHON) $(RUNNER) src/mission/sim_driver.ex

clean:
	rm -rf $(BUILD_DIR) $(BIN_DIR)/*.dylib $(BIN_DIR)/*.so *.log test_output
	@echo "Cleaned build artifacts."

help:
	@echo "Moonpool Build Targets:"
	@echo "  make all              Compile libraries and bootstrap environment"
	@echo "  make test             Execute complete test matrix and linters"
	@echo "  make test-unit        Run Euphoria core unit tests"
	@echo "  make test-fastpath    Run C17 fastpath engine tests"
	@echo "  make test-integration Run end-to-end mission simulation tests"
	@echo "  make bench            Run numerical and throughput benchmarks"
	@echo "  make sim              Execute full subsea pipeline survey simulation"
	@echo "  make lint             Verify zero em-dash typography invariant"
	@echo "  make clean            Remove build artifacts"
