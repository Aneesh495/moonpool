#!/usr/bin/env bash
# Bootstrap OpenEuphoria environment for Moonpool
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
EU_DIR="${REPO_DIR}/.euphoria"

echo "Checking OpenEuphoria environment..."

if command -v eui >/dev/null 2>&1; then
    echo "Found system eui: $(which eui)"
    eui -v
    exit 0
fi

if [ -f "${EU_DIR}/bin/eui" ]; then
    echo "Found local OpenEuphoria in ${EU_DIR}"
    "${EU_DIR}/bin/eui" -v
    exit 0
fi

echo "OpenEuphoria not detected on PATH or local tree."
OS="$(uname -s)"
ARCH="$(uname -m)"

mkdir -p "${EU_DIR}"

if [ "${OS}" = "Darwin" ]; then
    echo "Detected macOS. Downloading OpenEuphoria 4.1.0 OSX distribution..."
    URL="https://github.com/OpenEuphoria/euphoria/releases/download/4.1.0/euphoria-4.1.0-OSX-x64-57179171dbed.tar.gz"
    TAR_FILE="/tmp/euphoria-osx.tar.gz"
    curl -sL "${URL}" -o "${TAR_FILE}"
    tar -xzf "${TAR_FILE}" -C "${EU_DIR}" --strip-components=1
    chmod +x "${EU_DIR}/bin/eui" || true
    echo "Successfully installed OpenEuphoria into ${EU_DIR}"
    "${EU_DIR}/bin/eui" -v
elif [ "${OS}" = "Linux" ]; then
    echo "Detected Linux. Downloading OpenEuphoria 4.1.0 Linux distribution..."
    URL="https://github.com/OpenEuphoria/euphoria/releases/download/4.1.0/euphoria-4.1.0-Linux-x64-57179171dbed.tar.gz"
    TAR_FILE="/tmp/euphoria-linux.tar.gz"
    curl -sL "${URL}" -o "${TAR_FILE}"
    tar -xzf "${TAR_FILE}" -C "${EU_DIR}" --strip-components=1
    chmod +x "${EU_DIR}/bin/eui" || true
    echo "Successfully installed OpenEuphoria into ${EU_DIR}"
    "${EU_DIR}/bin/eui" -v
else
    echo "Unsupported OS for automatic bootstrap: ${OS}."
    echo "Please install OpenEuphoria from https://openeuphoria.org"
    exit 1
fi
