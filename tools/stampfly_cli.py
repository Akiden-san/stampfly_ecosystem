#!/usr/bin/env python3
"""
StampFly CLI Helper - Send commands to StampFly via WiFi CLI

Usage:
    python tools/stampfly_cli.py [--ip IP] COMMAND [ARGS...]

Commands:
    jump [altitude] [hover_duration]
    takeoff [altitude]
    land
    hover [altitude] [duration]
    flight status
    flight cancel

Options:
    --ip IP    StampFly IP address (default: 192.168.4.1 for AP mode)
               Use STAMPFLY_IP environment variable or --ip for STA mode

Examples:
    # AP mode (default)
    python tools/stampfly_cli.py jump 0.15 0.5

    # STA mode (specify IP)
    python tools/stampfly_cli.py --ip 192.168.1.100 jump 0.15 0.5

    # STA mode (environment variable)
    export STAMPFLY_IP=192.168.1.100
    python tools/stampfly_cli.py jump 0.15 0.5
"""

import socket
import sys
import time
import os

# StampFly default settings
DEFAULT_IP = "192.168.4.1"  # AP mode default
STAMPFLY_PORT = 23  # Telnet port for WiFi CLI
TIMEOUT = 2.0

def send_command(cmd, ip, silent=False):
    """Send a command to StampFly and return response"""
    try:
        if not silent:
            print(f"🚁 Connecting to StampFly at {ip}:{STAMPFLY_PORT}...")

        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(TIMEOUT)
        sock.connect((ip, STAMPFLY_PORT))

        if not silent:
            print("✅ Connected.")

        # Read welcome message
        welcome = sock.recv(1024).decode('utf-8', errors='ignore')

        # Send command
        if not silent:
            print(f"📤 Sending command: {cmd}")
        sock.send(f"{cmd}\n".encode())

        # Wait a bit for response
        time.sleep(0.1)

        # Read response
        response = sock.recv(2048).decode('utf-8', errors='ignore')

        sock.close()

        if not silent:
            print(f"📥 Response: {response.strip()}")

        return True, response

    except socket.timeout:
        if not silent:
            print("❌ Timeout: StampFly not responding")
        return False, "Timeout"
    except ConnectionRefusedError:
        if not silent:
            print("❌ Connection refused: Is StampFly powered on and WiFi enabled?")
        return False, "Connection refused"
    except Exception as e:
        if not silent:
            print(f"❌ Error: {e}")
        return False, str(e)

def main():
    # Parse command line arguments
    args = sys.argv[1:]

    # Get IP address (priority: --ip flag > STAMPFLY_IP env > default)
    ip = os.getenv('STAMPFLY_IP', DEFAULT_IP)

    if '--ip' in args:
        ip_idx = args.index('--ip')
        if ip_idx + 1 >= len(args):
            print("❌ Error: --ip requires an IP address")
            sys.exit(1)
        ip = args[ip_idx + 1]
        # Remove --ip and its value from args
        args = args[:ip_idx] + args[ip_idx+2:]

    if len(args) < 1:
        print("Usage:")
        print("  python tools/stampfly_cli.py [--ip IP] COMMAND [ARGS...]")
        print("")
        print("Commands:")
        print("  jump [altitude] [hover_duration]")
        print("  takeoff [altitude]")
        print("  land")
        print("  hover [altitude] [duration]")
        print("  flight status")
        print("  flight cancel")
        print("")
        print("Options:")
        print("  --ip IP    StampFly IP address (default: 192.168.4.1 for AP mode)")
        print("")
        print("Examples:")
        print("  # AP mode (default)")
        print("  python tools/stampfly_cli.py jump 0.15 0.5")
        print("")
        print("  # STA mode (specify IP)")
        print("  python tools/stampfly_cli.py --ip 192.168.1.100 jump 0.15 0.5")
        print("")
        print("  # STA mode (environment variable)")
        print("  export STAMPFLY_IP=192.168.1.100")
        print("  python tools/stampfly_cli.py jump 0.15 0.5")
        sys.exit(1)

    # Build command from remaining arguments
    cmd = " ".join(args)

    success, response = send_command(cmd, ip)

    if success:
        sys.exit(0)
    else:
        sys.exit(1)

if __name__ == "__main__":
    main()
