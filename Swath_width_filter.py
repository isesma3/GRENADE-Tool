import pandas as pd
import os
ROOT_DIR = os.path.dirname(__file__)
# Load the CSV file
data = pd.read_csv(os.path.join(ROOT_DIR, "Failed_runs/1000_run_Combined.csv"))

# Filter the rows where Sensor_Swath_width is 10 and 100
filtered_data_10 = data[data['Sensor_Swath_width'] == 10]
filtered_data_30 = data[data['Sensor_Swath_width'] == 30]
filtered_data_100 = data[data['Sensor_Swath_width'] == 100]

# Save the filtered data to new CSV files
filtered_data_10.to_csv('Failed_runs/filtered_Sensor_Swath_width_10.csv', index=False)
filtered_data_30.to_csv('Failed_runs/filtered_Sensor_Swath_width_30.csv', index=False)
filtered_data_100.to_csv('Failed_runs/filtered_Sensor_Swath_width_100.csv', index=False)
