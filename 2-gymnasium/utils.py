import pandas as pd

def print_q_table(agent, neighbors_dict):
    q_table = {}
    for state in range(len(neighbors_dict)):
        q_table[state] = agent.q_values[state]

    df = pd.DataFrame(q_table)
    print()
    print(df.T)
