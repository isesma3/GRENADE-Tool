import os

ROOT_DIR = os.path.dirname(__file__)

import numpy as np
import streamlit as st
import sys
import pandas as pd
from math import pi as PI

sys.path.insert(0, ROOT_DIR + "/Environment")
sys.path.insert(0, ROOT_DIR + "/Aero/Airfoils")
from Environment.MarsAtm import *

# Given parameters for the BWB
total_span = 2.8125 / 2  # meters, total wingspan halved for symmetry
desired_volume = 1  # m^3, significantly larger desired volume for the first 4 sections

# Original span length ratios
original_span_lengths = [0.05843, 0.05843, 0.11686, 0.23371, 0.23371, 0.23371, 0.46742, 0.23371, 0.11686, 0.02921, 0.01461]
adjusted_span_lengths = [x / sum(original_span_lengths) * total_span for x in original_span_lengths]
span_starts = np.cumsum([0] + adjusted_span_lengths[:-1])

# Calculate linear interpolation for chord lengths from root to tip
root_chord = 0.392857
tip_chord = 0.1375
chord_lengths = np.linspace(root_chord, tip_chord, len(adjusted_span_lengths))

# Increase chord lengths for the first four sections
scale_factor = 2  # Example scale factor for demonstration
for i in range(4):  # Adjust only the first four sections
    chord_lengths[i] *= scale_factor

# Adjust the rest of the chord lengths to ensure a smooth transition
# This might involve recalculating the taper ratio or adjusting chords directly

# Generate sections data with adjusted chords for the first four sections
sections = []
for i in range(len(adjusted_span_lengths)):
    span_end = span_starts[i] + adjusted_span_lengths[i]
    current_root_chord = chord_lengths[i]
    current_tip_chord = chord_lengths[i + 1] if i + 1 < len(chord_lengths) else chord_lengths[i]

    sections.append({
        'Span Start': span_starts[i],
        'Span End': span_end,
        'Span Length': adjusted_span_lengths[i],
        'Root C': current_root_chord,
        'Tip C': current_tip_chord,
        'Sweep': 0,  # Sweep angle
        'Twist': 0,  # Twist angle
        'Dihedral': 2.15538  # Dihedral angle
    })

# Convert to DataFrame for display
df_sections = pd.DataFrame(sections)

df_sections

print(df_sections.to_string(index=False))