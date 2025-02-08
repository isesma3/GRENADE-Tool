# Loads multiple csv files of the succeeded runs, merges the data together into one dataframe, then runs TOPSIS and generatest the 3D view of the aircraft
import os
ROOT_DIR = os.path.dirname(__file__)

import pandas as pd
from TOPSIS import *
from AeroshellSelector import *
from VSP_tests.BWB_sections import *
from datetime import datetime

# Read in the input file to obtain TOPSIS weightings
input = pd.read_csv(os.path.join(ROOT_DIR, "Inp/Input.csv"))

# Read in data from Failed_runs folder
# Note: Can put as many dataframes here as you want but it should be the TOPSIS_LOG files as these are the cases that suceeded

df1 = pd.read_csv(os.path.join(ROOT_DIR, "Failed_runs/180_mass_10_SW_4m_AD.csv"))
# df2 = pd.read_csv(os.path.join(ROOT_DIR, "Failed_runs/TOPSIS_LOG_17-35_Titan_07-07.csv"))
# df3 = pd.read_csv(os.path.join(ROOT_DIR, "Failed_runs/TOPSIS_LOG_37-94_Titan_07-07.csv"))
# df4 = pd.read_csv(os.path.join(ROOT_DIR, "Failed_runs/TOPSIS_LOG_96-97_Titan_07-07.csv"))
# df5 = pd.read_csv(os.path.join(ROOT_DIR, "Failed_runs/TOPSIS_LOG_104-1000_Titan_07-08.csv"))
# df6 = pd.read_csv(os.path.join(ROOT_DIR, "Failed_runs/TOPSIS_LOG_479-566_Titan_07-07.csv"))
# df7 = pd.read_csv(os.path.join(ROOT_DIR, "Failed_runs/TOPSIS_LOG_568-698_Titan_07-07.csv"))
# df8 = pd.read_csv(os.path.join(ROOT_DIR, "Failed_runs/TOPSIS_LOG_700-875_Titan_07-07.csv"))
# df9 = pd.read_csv(os.path.join(ROOT_DIR, "Failed_runs/TOPSIS_LOG_877-887_Titan_07-07.csv"))
# df10 = pd.read_csv(os.path.join(ROOT_DIR, "Failed_runs/TOPSIS_LOG_889-1000_Titan_07-07.csv"))

# Merge data
merged_dfs = pd.concat([df1]) # Just put as many dataframes in here as needed (seperated by commas)

# Reindex the merged DataFrame
merged_dfs.reset_index(drop=True, inplace=True)

merged_dfs.to_csv(
        os.path.join(
            ROOT_DIR, "Failed_runs/RECON_TOPSIS_Compiled_" + str(1) + "-" + str(merged_dfs.shape[0]) + "_" + str(merged_dfs.Planet[0]) + "_" + datetime.now().strftime("%m-%d") + ".csv"
        )
    )

# Case selection
if merged_dfs.shape[0] == 1:
    print('Only case', merged_dfs.Case[0], 'met requirements')
elif merged_dfs.empty:
    print('No cases met requirements')
else:
    # Perform TOPSIS
    TdF,Case_num = TOPSIS(input,merged_dfs,merged_dfs.shape[0])
    # Print the case number selected
    print(TdF)
    print('Case number',TdF.Case[0], 'was selected')

# Create .csv of the best case
TdF.to_csv(
    os.path.join(
            ROOT_DIR, "Out/Best_Design_Point_for_SW" + "_" + str(TdF.Sensor_Swath_width[0]) + "_" + str(merged_dfs.Planet[0]) + "_" + datetime.now().strftime("%m-%d") + "_" + TdF.Tail_type[0] + ".csv"
        )
    )

# Find Aeroshell and Launch vehicle 
file_name = ROOT_DIR + "\Aeroshells.csv"  # Replace with the actual path to your CSV file
if TdF.Wing_folding[0] == 'yes':
    min_diameter = 0.97*TdF.Aeroshell_Diameter[0]  # If wing folding is allowed, the folded wingspan will be about 97% the aeroshell diameter
else:
    min_diameter = TdF.Span[0]  # minimum diameter in meters
min_mass = TdF.Mass[0]  # minimum mass in kilograms

aeroshell_data, keys = read_aeroshell_data(file_name)
selected_aeroshell, aeroshell_name_key = find_aeroshell(min_diameter, min_mass, aeroshell_data, keys)

if selected_aeroshell:
    # Using the correct key for accessing aeroshell name
    print(f"Selected Aeroshell: {selected_aeroshell[aeroshell_name_key]}")
    print(f"Launch Vehicle: {selected_aeroshell['Launch Vehicle']}")
    print(f"Mission Cost ($ billion): {selected_aeroshell['Cost ($ billion)']}")
else:
    print("No suitable aeroshell found for the given criteria.")

# Generate a .vsp3 file for the chosen case
print('Generating VSP file...')
if TdF.Tail_type[0] == 'Boom-mounted_inverted_V':
    generate_BWB_VSPfile(TdF)
else:
    generate_Beluga_VSPfile(TdF)
print('VSP file generation done')