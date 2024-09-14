import serial
import requests
import json
import time
import re
import random
import argparse

serial_port = '/dev/ttyUSB0'
baud_rate = 9600
server_url = 'http://localhost:5000/data'
log_file_path = 'learning_results_log.txt'
ESP_ENDPOINT = None  # This will be set dynamically
max_episodes = 100  # Maximum number of episodes for LEARNING phase
convergence_threshold = 0.01  # Threshold for detecting reward convergence
previous_q_table = None
best_parameters = None
best_hops = float('inf')
internal_episode_storage = []

parser = argparse.ArgumentParser(description="Q-learning ESP Node Middleware")
parser.add_argument('--feedback', action='store_true', help="Enable sending learning data back to the master node")
parser.add_argument('--verbose', action='store_true', help="Enable detailed logging")
args = parser.parse_args()

send_feedback_to_master = args.feedback
verbose_logging = args.verbose

def open_serial_port():
    while True:
        try:
            ser = serial.Serial(serial_port, baud_rate)
            print(f"Successfully connected to {serial_port}")
            return ser
        except serial.SerialException:
            print(f"Failed to connect to {serial_port}. Retrying...")
            time.sleep(5)  # Wait 5 seconds before retrying

def read_from_serial(ser):
    try:
        line = ser.readline().decode('utf-8').strip()
        return line
    except (UnicodeDecodeError, serial.SerialException) as e:
        print(f"Failed to read line: {e}")
        return None

def extract_log_data(line):
    try:
        start = line.find('{')
        end = line.rfind('}') + 1
        log_data = line[start:end]
        return json.loads(log_data)
    except (ValueError, json.JSONDecodeError):
        return None

def send_to_server(log_data):
    if log_data:
        try:
            response = requests.post(server_url, json=log_data)
            if response.status_code == 200:
                print("Log sent successfully")
            else:
                print("Failed to send log", response.status_code)
        except requests.exceptions.RequestException as e:
            print(f"Failed to send log: {e}")

def log_to_file(line):
    with open(log_file_path, 'a') as log_file:
        log_file.write(line + '\n')

def has_converged(current_q_table, previous_q_table, threshold):
    if previous_q_table is None:
        return False
    for state in current_q_table:
        for action in current_q_table[state]:
            if abs(current_q_table[state][action] - previous_q_table[state][action]) > threshold:
                return False
    return True

def send_parameters_to_master(parameters):
    if ESP_ENDPOINT:
        try:
            print("About to send analyzed q-parameters back to master node")
            response = requests.post(ESP_ENDPOINT, data=parameters)
            if response.status_code == 200:
                print("Parameters sent successfully")
            else:
                print("Failed to send parameters", response.status_code)
        except requests.exceptions.RequestException as e:
            print(f"Failed to send parameters: {e}")
    else:
        print("ESP_ENDPOINT is not set")

def verbose_print(message):
    if verbose_logging:
        print(message)

def get_episode_count(json_data):
    return len(json_data.get("episodes", []))

def save_episodes(json_data):
    global internal_episode_storage
    episodes = json_data.get("episodes", [])
    internal_episode_storage.extend(episodes)

def add_episodes_to_line(json_data):
    global internal_episode_storage
    json_data["episodes"].extend(internal_episode_storage)
    internal_episode_storage = []

def genetic_algorithm(data, generations=10, population_size=10):
    def initialize_population(size):
        population = []
        for _ in range(size):
            individual = {
                'alpha': random.uniform(0, 1),
                'gamma': random.uniform(0, 1),
                'epsilon': random.uniform(0, 1)
            }
            population.append(individual)
        return population

    def evaluate_fitness(individual, data):
        fitness = sum(float(ep['reward']) for ep in data[-1]['episodes'])
        return fitness

    def select_parents(population, fitnesses):
        parents = []
        sample_size = min(3, len(population))  # Ensure sample size is not larger than population
        for _ in range(len(population) // 2):
            tournament = random.sample(list(zip(population, fitnesses)), k=sample_size)
            parents.append(max(tournament, key=lambda x: x[1])[0])
        return parents

    def crossover(parent1, parent2):
        child1 = {
            'alpha': (parent1['alpha'] + parent2['alpha']) / 2,
            'gamma': (parent1['gamma'] + parent2['gamma']) / 2,
            'epsilon': (parent1['epsilon'] + parent2['epsilon']) / 2
        }
        child2 = {
            'alpha': (parent1['alpha'] + parent2['alpha']) / 2,
            'gamma': (parent1['gamma'] + parent2['gamma']) / 2,
            'epsilon': (parent1['epsilon'] + parent2['epsilon']) / 2
        }
        return child1, child2

    def mutate(individual, mutation_rate=0.1):
        if random.random() < mutation_rate:
            individual['alpha'] = random.uniform(0, 1)
        if random.random() < mutation_rate:
            individual['gamma'] = random.uniform(0, 1)
        if random.random() < mutation_rate:
            individual['epsilon'] = random.uniform(0, 1)
        return individual

    population = initialize_population(population_size)
    if not population:
        raise ValueError("Population initialization failed, population is empty")

    for generation in range(generations):
        fitnesses = [evaluate_fitness(individual, data) for individual in population]
        parents = select_parents(population, fitnesses)
        
        # Ensure the number of parents is even
        if len(parents) % 2 != 0:
            parents = parents[:-1]

        next_population = []
        for i in range(0, len(parents), 2):
            child1, child2 = crossover(parents[i], parents[i+1])
            next_population.append(mutate(child1))
            next_population.append(mutate(child2))
        population = next_population
        if not population:
            raise ValueError("Population is empty after generation {}".format(generation))

    best_individual = max(population, key=lambda ind: evaluate_fitness(ind, data))
    return best_individual

# Initialize data
data = []

ser = open_serial_port()

while True:
    if ser.in_waiting > 0:
        line = read_from_serial(ser)
        if line:
            print(line)
            if send_feedback_to_master:
                ip_match = re.search(r'IP address: (\d+\.\d+\.\d+\.\d+)', line)
                if ip_match:
                    ESP_ENDPOINT = f"http://{ip_match.group(1)}/update-q-parameters"
                    verbose_print(f"ESP_ENDPOINT set to {ESP_ENDPOINT}")

            if 'Episode results:' in line:
                verbose_print("Extracting data from log")
                log_data = extract_log_data(line)  # Extract log data
                if log_data:
                    verbose_print(f"Log data extracted successfully {log_data}")

                    episode_count = get_episode_count(log_data)

                    if episode_count > 10:
                        save_episodes(log_data)
                        log_data["episodes"] = []

                    if episode_count <= 10 and internal_episode_storage:
                        add_episodes_to_line(log_data)

                    log_to_file(line)
                    send_to_server(log_data)

                    data.append(log_data)

                    best_parameters = genetic_algorithm(data)
                    send_parameters_to_master(best_parameters)

                    current_q_table = log_data.get('q_table', {})
                    if has_converged(current_q_table, previous_q_table, convergence_threshold):
                        verbose_print("Learning phase has converged.")

                    previous_q_table = current_q_table

    if not ser.is_open:
        print("Serial port disconnected. Retrying connection...")
        ser = open_serial_port()
