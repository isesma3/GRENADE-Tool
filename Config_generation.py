# Code to down select and create a full-factorial DoE of aircraft configurations

import pandas as pd
import numpy as np
from itertools import product

import os
ROOT_DIR = os.path.dirname(__file__)

def config_gen():
    # Read input file from Input.csv
    config = pd.read_csv(os.path.join(ROOT_DIR, "Inp/Configuration_Input.csv"))
    miss_prof = pd.read_csv(os.path.join(ROOT_DIR, "Inp/Mission_Profile.csv"))
    input = pd.read_csv(os.path.join(ROOT_DIR, "Inp/Input.csv"))
    seg_type = miss_prof.Type

    # Down select process using heuristics
    # If 'Turn' is present in seg_type, filter out 'No_tail' from tail_types
    if 'Turn' in seg_type.values:
        tail_types = config.Tail_type[config.Tail_type != 'No_tail'].array
    else:
        tail_types = config.Tail_type.array
    if 'Climb' in seg_type.values and (input.RoC_min_alt[0] > 17 or input.RoC_max_alt[0] > 17): # Refs from Raymer Ch. 4.5.2 
        tail_types = config.Tail_type[config.Tail_type != 'T-tail'].array
    else:
        tail_types = config.Tail_type.array
    if input.tmission[0] >= 1:
        config.loc[config['Prop_num'] == 1.0, 'Prop_num'] = np.nan
        config.loc[config['Prop_num'] == 2.0, 'Prop_num'] = np.nan
        prop_num = config.Prop_num.transpose().array.dropna()
    else:
        prop_num = config.Prop_num.transpose().array.dropna()
    if float(input.LD_weight[0]) < 0.2:
        winglet = config.Winglet[config.Winglet != 'yes'].array
        winglet = winglet.dropna()
    else:
        winglet = config.Winglet.transpose().array.dropna()
    # if float(input.b_weight[0]) < 0.3: #TODO Add a better heuristic for wingspan
    #     wing_fold = config.Wing_folding[config.Wing_folding != 'yes'].array
    #     wing_fold = wing_fold.dropna()
    # else:
    wing_fold = config.Wing_folding.transpose().array.dropna()

    x = list(product(tail_types,prop_num,winglet,wing_fold))

    dfx = pd.DataFrame(x,columns=['Tail type', 'Num prop', 'Winglet', 'Wing folding'])
    dfx.to_csv('Inp/ConfigDoE.csv')
    return dfx

#config_gen()