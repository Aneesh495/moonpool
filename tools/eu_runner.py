#!/usr/bin/env python3
"""
Euphoria Execution Runner for Moonpool.
Locates the OpenEuphoria interpreter (system eui or local .euphoria/bin/eui),
configures standard and project include paths, and executes target scripts.
"""

import os
import sys
import subprocess

REPO_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
LOCAL_EUI = os.path.join(REPO_DIR, ".euphoria", "bin", "eui")
LOCAL_INC = os.path.join(REPO_DIR, ".euphoria", "include")
SRC_INC = os.path.join(REPO_DIR, "src")

def get_eui_path():
    # 1. Check local bootstrap directory
    if os.path.isfile(LOCAL_EUI) and os.access(LOCAL_EUI, os.X_OK):
        return LOCAL_EUI
    # 2. Check system PATH
    system_eui = subprocess.run(["which", "eui"], capture_output=True, text=True).stdout.strip()
    if system_eui and os.path.isfile(system_eui):
        return system_eui
    # 3. Try to run bootstrap
    bootstrap_sh = os.path.join(REPO_DIR, "tools", "bootstrap.sh")
    if os.path.isfile(bootstrap_sh):
        print("[eu_runner] eui not found, running bootstrap.sh...")
        subprocess.run([bootstrap_sh], check=True)
        if os.path.isfile(LOCAL_EUI):
            return LOCAL_EUI
    raise RuntimeError("Could not locate or bootstrap OpenEuphoria eui interpreter.")

def run_script(script_path, args=None):
    eui_bin = get_eui_path()
    if args is None:
        args = []
    
    cmd = [eui_bin]
    if os.path.isdir(LOCAL_INC):
        cmd.extend(["-I", LOCAL_INC])
    cmd.extend(["-I", SRC_INC])
    cmd.extend(["-I", REPO_DIR])
    cmd.append(script_path)
    cmd.extend(args)
    
    env = os.environ.copy()
    if os.path.isdir(LOCAL_INC):
        env["EUDIR"] = os.path.join(REPO_DIR, ".euphoria")
    
    return subprocess.run(cmd, env=env)

def main():
    if len(sys.argv) < 2:
        print("Usage: eu_runner.py <script.ex> [args...]")
        sys.exit(1)
    
    script = sys.argv[1]
    extra_args = sys.argv[2:]
    res = run_script(script, extra_args)
    sys.exit(res.returncode)

if __name__ == "__main__":
    main()
