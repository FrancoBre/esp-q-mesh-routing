"""
Middleware: reads Serial from the receiver (destination) ESP only — not the sender.
When a packet is delivered, the receiver prints DELIVERY_DATA: + JSON; this script
extracts that JSON and POSTs it to the visualization server. For forward-only
Q-routing (Boyan & Littman 1993).
"""
import argparse
import json
import time
from pathlib import Path

import requests
import serial
from serial.tools import list_ports

serial_port = '/dev/ttyUSB0'
baud_rate = 115200
server_url = 'http://localhost:5000/data'
log_file_path = Path(__file__).resolve().parent / 'learning_results_log.txt'
DELIVERY_MARKER = 'DELIVERY_DATA:'


def parse_args():
    parser = argparse.ArgumentParser(
        description="Q-routing ESP middleware: Serial → visualization server"
    )
    parser.add_argument(
        '--port',
        help="Serial port. If omitted, prompt at startup."
    )
    parser.add_argument(
        '--server', default=server_url,
        help=f"Server URL (default: {server_url})"
    )
    parser.add_argument(
        '--verbose', action='store_true',
        help="Enable detailed logging"
    )
    parser.add_argument(
        '--diagnostics', action='store_true',
        help="Request one distributed diagnostic snapshot after connecting"
    )
    return parser.parse_args()


def choose_serial_port():
    """Prompt the user to select a detected serial port or enter one manually."""
    detected_ports = sorted(port.device for port in list_ports.comports())

    if not detected_ports:
        try:
            selected = input(
                f"No serial ports detected. Enter a port [{serial_port}]: "
            ).strip()
        except EOFError:
            selected = ''
        return selected or serial_port

    print("Available serial ports:")
    for index, port in enumerate(detected_ports, start=1):
        print(f"  {index}) {port}")
    print(f"  {len(detected_ports) + 1}) Enter a port manually")

    while True:
        try:
            choice = input("Select a port [1]: ").strip()
        except EOFError:
            choice = ''

        if not choice:
            return detected_ports[0]

        try:
            selected_index = int(choice) - 1
        except ValueError:
            print("Please enter one of the listed numbers.")
            continue

        if 0 <= selected_index < len(detected_ports):
            return detected_ports[selected_index]
        if selected_index == len(detected_ports):
            manual_port = input("Serial port: ").strip()
            if manual_port:
                return manual_port
            print("Port cannot be empty.")
            continue

        print("Please enter one of the listed numbers.")


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
    port = args.port or choose_serial_port()

    ser = open_serial_port(port, baud_rate)
    if args.diagnostics:
        ser.write(b'D\n')
        ser.flush()

    while True:
        if ser.in_waiting > 0:
            line = read_line(ser)
            if line:
                print(line)
                if DELIVERY_MARKER in line:
                    log_data = extract_json(line)
                    if log_data:
                        log_data['received_at'] = time.time()
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
            ser = open_serial_port(port, baud_rate)


if __name__ == '__main__':
    main()
