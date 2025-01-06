import numpy as np
import pandas as pd
import ast
import os

def load_u_ilc_from_csv(file_path, n_samples):
    # Ensure the file path is fully expanded
    
    file_path = 'u_ilc_initial_values.csv'
    # Load the CSV file
    data = pd.read_csv(file_path)

    # Assuming the CSV file contains two columns: 'u_ilc_x' and 'u_ilc_y'
    # Evaluate the string representation of the lists to actual lists
    u_ilc_x = ast.literal_eval(data.iloc[0]['u_ilc_x'])
    u_ilc_y = ast.literal_eval(data.iloc[0]['u_ilc_y'])

    if len(u_ilc_x) != n_samples or len(u_ilc_y) != n_samples:
        raise ValueError("CSV file does not contain the correct number of samples")

    # Convert lists to numpy arrays and stack them
    u_ilc = np.vstack([u_ilc_x, u_ilc_y])

    return u_ilc


n_samples = 1000
u_ilc = load_u_ilc_from_csv(file_path, n_samples)

print(u_ilc)
