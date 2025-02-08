
import os

ROOT_DIR = os.path.dirname(__file__)

import csv

def read_aeroshell_data(file_name):
    with open(file_name, mode='r', encoding='utf-8-sig') as file:  # Handling BOM
        reader = csv.DictReader(file)
        # Convert dict_keys to a list to make them subscriptable
        keys = list(reader.fieldnames)
        data = list(reader)
        return data, keys

def find_aeroshell(min_diameter, min_mass, aeroshell_data, keys):
    # Extracting keys directly from the list of keys
    diameter_key = next(key for key in keys if 'Diameter' in key)
    mass_key = next(key for key in keys if 'Payload Mass' in key)
    cost_key = next(key for key in keys if 'Cost' in key)
    aeroshell_name_key = next(key for key in keys if 'Aeroshell Name' in key)

    filtered_data = [
        d for d in aeroshell_data
        if float(d[diameter_key]) >= min_diameter and float(d[mass_key]) >= min_mass and d[cost_key] != 'Varies'
    ]
    if not filtered_data:
        return None, None
    min_cost_data = min(filtered_data, key=lambda x: float(x[cost_key]))

    return min_cost_data, aeroshell_name_key

# # Example usage
if __name__ == "__main__":
    file_name = ROOT_DIR + "\Aeroshells.csv"  # Replace with the actual path to your CSV file
    min_diameter = 2.0  # minimum diameter in meters
    min_mass = 300  # minimum mass in kilograms

    aeroshell_data, keys = read_aeroshell_data(file_name)
    selected_aeroshell, aeroshell_name_key = find_aeroshell(min_diameter, min_mass, aeroshell_data, keys)

    if selected_aeroshell:
        # Using the correct key for accessing aeroshell name
        print(f"Selected Aeroshell: {selected_aeroshell[aeroshell_name_key]}")
        print(f"Launch Vehicle: {selected_aeroshell['Launch Vehicle']}")
        print(f"Mission Cost ($ billion): {selected_aeroshell['Cost ($ billion)']}")
    else:
        print("No suitable aeroshell found for the given criteria.")



