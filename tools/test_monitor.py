#!/usr/bin/env python3
"""
Phase 3 Test Monitor
Filters and highlights important state management logs during hardware testing.
"""

import sys
import re
from datetime import datetime

# ANSI color codes
class Colors:
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    MAGENTA = '\033[95m'
    RESET = '\033[0m'
    BOLD = '\033[1m'

# Log patterns to highlight
PATTERNS = {
    'state_transition': {
        'regex': r'LED updated for state transition: (\d+) -> (\d+)',
        'color': Colors.GREEN,
        'label': 'STATE'
    },
    'command_enqueue': {
        'regex': r'Command enqueued: ID=(\d+), type=(\d+), alt=([\d.]+)',
        'color': Colors.BLUE,
        'label': 'QUEUE'
    },
    'command_start': {
        'regex': r'Starting command: ID=(\d+)',
        'color': Colors.CYAN,
        'label': 'START'
    },
    'command_complete': {
        'regex': r'Command completed: ID=(\d+)',
        'color': Colors.GREEN,
        'label': 'DONE'
    },
    'calibration': {
        'regex': r'(Level calibration complete|Calibration)',
        'color': Colors.YELLOW,
        'label': 'CALIB'
    },
    'timeout': {
        'regex': r'timeout|Timeout',
        'color': Colors.RED,
        'label': 'TIMEOUT'
    },
    'queue_full': {
        'regex': r'queue full|Queue full',
        'color': Colors.RED,
        'label': 'ERROR'
    },
    'ready': {
        'regex': r'(initialized|ready|Ready)',
        'color': Colors.GREEN,
        'label': 'INIT'
    }
}

def colorize(text, color):
    """Add color to text."""
    return f"{color}{text}{Colors.RESET}"

def process_line(line):
    """Process and highlight log line."""
    timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]

    # Check each pattern
    for name, pattern_info in PATTERNS.items():
        if re.search(pattern_info['regex'], line, re.IGNORECASE):
            label = colorize(f"[{pattern_info['label']}]", Colors.BOLD + pattern_info['color'])
            colored_line = colorize(line.strip(), pattern_info['color'])
            print(f"{Colors.BOLD}[{timestamp}]{Colors.RESET} {label} {colored_line}")
            return

    # Default: print uncolored
    if 'SystemStateManager' in line or 'CommandQueue' in line or 'FlightCommandService' in line:
        print(f"[{timestamp}] {line.strip()}")

def main():
    """Main monitor loop."""
    print(colorize("=== Phase 3 Test Monitor ===", Colors.BOLD + Colors.CYAN))
    print(colorize("Monitoring state management logs...", Colors.CYAN))
    print(colorize("Press Ctrl+C to exit\n", Colors.CYAN))

    try:
        for line in sys.stdin:
            process_line(line)
    except KeyboardInterrupt:
        print(colorize("\nMonitor stopped.", Colors.YELLOW))

if __name__ == '__main__':
    main()
