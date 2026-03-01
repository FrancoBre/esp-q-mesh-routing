"""
Middleware: reads Serial from receiver ESP, extracts DELIVERY_DATA JSON,
forwards to visualization server. For forward-only Q-routing (Boyan & Littman 1993).
"""
import argparse
import json
import time

import requests
import serial

serial_port = '/dev/ttyUSB0'
baud_rate = 9600
server_url = 'http://localhost:5000/data'
log_file_path = 'learning_results_log.txt'
DELIVERY_MARKER = 'DELIVERY_DATA:'


def parse_args():
    parser = argparse.ArgumentParser(
        description="Q-routing ESP middleware: Serial → visualization server"
    )
    parser.add_argument(
        '--port', default=serial_port,
        help=f"Serial port (default: {serial_port})"
    )
    parser.add_argument(
        '--server', default=server_url,
        help=f"Server URL (default: {server_url})"
    )
    parser.add_argument(
        '--verbose', action='store_true',
        help="Enable detailed logging"
    )
    return parser.parse_args()


def open_serial_port(port, baud):
    while True:
        try:
            ser = serial.Serial(port, baud)
            print(f"Connected to {port}")
            return ser
        except serial.SerialException:
            print(f"Failed to connect to {port}. Retrying...")
            time.sleep(5)


def read_line(ser):
    try:
        line = ser.readline().decode('utf-8').strip()
        return line
    except (UnicodeDecodeError, serial.SerialException) as e:
        print(f"Read error: {e}")
        return None


def extract_json(line):
    """Extract JSON from line after DELIVERY_MARKER."""
    if DELIVERY_MARKER not in line:
        return None
    start = line.find(DELIVERY_MARKER) + len(DELIVERY_MARKER)
    json_str = line[start:].strip()
    try:
        return json.loads(json_str)
    except json.JSONDecodeError:
        return None


def send_to_server(url, log_data):
    if not log_data:
        return False
    try:
        response = requests.post(url, json=log_data, timeout=5)
        if response.status_code == 200:
            return True
        print(f"Server returned {response.status_code}")
        return False
    except requests.exceptions.RequestException as e:
        print(f"Failed to send: {e}")
        return False


def log_to_file(line):
    with open(log_file_path, 'a') as f:
        f.write(line + '\n')


def main():
    args = parse_args()
    verbose = args.verbose

    ser = open_serial_port(args.port, baud_rate)

    while True:
        if ser.in_waiting > 0:
            line = read_line(ser)
            if line:
                print(line)
                if DELIVERY_MARKER in line:
                    log_data = extract_json(line)
                    if log_data:
                        if verbose:
                            print("Extracted JSON, forwarding to server")
                        log_to_file(line)
                        if send_to_server(args.server, log_data):
                            if verbose:
                                print("Sent to server")
                        else:
                            print("Failed to send to server")

        if not ser.is_open:
            print("Serial disconnected. Reconnecting...")
            ser = open_serial_port(args.port, baud_rate)


if __name__ == '__main__':
    main()
