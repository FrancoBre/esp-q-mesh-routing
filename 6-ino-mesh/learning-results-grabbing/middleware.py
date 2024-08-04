import serial
import requests
import json
import time

serial_port = '/dev/ttyUSB0'
baud_rate = 115200
server_url = 'http://localhost:5000/data'
log_file_path = 'learning_results_log.txt'

def open_serial_port():
    while True:
        try:
            ser = serial.Serial(serial_port, baud_rate)
            print(f"Successfully connected to {serial_port}")
            return ser
        except serial.SerialException:
            print(f"Failed to connect to {serial_port}. Retrying...")
            time.sleep(5)  # Espera 5 segundos antes de intentar nuevamente

def read_from_serial(ser):
    try:
        line = ser.readline().decode('utf-8').strip()
        return line
    except UnicodeDecodeError as e:
        print(f"Failed to decode line: {e}")
        return None
    except serial.SerialException as e:
        print(f"Serial error: {e}")
        return None

def extract_log_data(line):
    # Ejemplo simple de extracción de datos
    try:
        start = line.find('{')
        end = line.rfind('}') + 1
        log_data = line[start:end]
        return json.loads(log_data)
    except (ValueError, json.JSONDecodeError):
        return None

def send_to_server(log_data):
    if log_data:
        response = requests.post(server_url, json=log_data)
        if response.status_code == 200:
            print("Log sent successfully")
        else:
            print("Failed to send log", response.status_code)

def log_to_file(line):
    with open(log_file_path, 'a') as log_file:
        log_file.write(line + '\n')

ser = open_serial_port()

while True:
    if ser.in_waiting > 0:
        line = read_from_serial(ser)
        if line and 'Log message with structure' in line:
            log_data = extract_log_data(line)  # Función para extraer los datos del log
            if log_data:
                log_to_file(line)  # Loguea solo los resultados del aprendizaje
                send_to_server(log_data)  # Función para enviar los datos al servidor

    # Reintentar abrir el puerto serial si se desconecta
    if not ser.is_open:
        print("Serial port disconnected. Retrying connection...")
        ser = open_serial_port()
