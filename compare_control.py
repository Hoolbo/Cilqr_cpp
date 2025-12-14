import json
import os
import matplotlib.pyplot as plt
import numpy as np
import argparse
import sys
import re

def read_solver_data(data_folder, solver_type):
    """
    Read solver JSON files from specified folder.
    Matches pattern: {solver_type}_data_*.json
    """
    velocities = []
    gammas = []
    time_steps = []
    
    if not os.path.exists(data_folder):
        print(f"Warning: Folder not found: {data_folder}")
        return [], [], []

    # Get all {solver}_data_*.json files
    json_files = []
    prefix = f"{solver_type}_data_"
    
    for filename in os.listdir(data_folder):
        if filename.startswith(prefix) and filename.endswith('.json'):
            try:
                # Extract file number using regex for robustness
                match = re.search(r'_data_(\d+)\.json', filename)
                if match:
                    file_num = int(match.group(1))
                    json_files.append((file_num, filename))
            except ValueError:
                continue
    
    # Sort by file number
    json_files.sort(key=lambda x: x[0])
    print(f"[{solver_type}] Found {len(json_files)} data files in {data_folder}")
    
    for file_num, filename in json_files:
        file_path = os.path.join(data_folder, filename)
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
            
            control_seq = data.get('control_sequence', {})
            velocity_array = control_seq.get('velocity', [])
            gamma_array = control_seq.get('gamma', [])
            
            if velocity_array and gamma_array:
                velocities.append(velocity_array[0])  # First velocity
                gammas.append(gamma_array[0])         # First articulation rate
                time_steps.append(file_num)
                
        except (json.JSONDecodeError, FileNotFoundError, KeyError) as e:
            print(f"Error reading {filename}: {e}")
            continue
            
    return time_steps, velocities, gammas

def plot_comparison(map_name, data_dict):
    """
    Plot velocity and articulation rate comparison with IEEE Modern style.
    """
    # IEEE Style Configuration - Modern/Clean
    plt.rcParams.update(plt.rcParamsDefault)
    plt.rcParams['font.family'] = 'serif'
    plt.rcParams['font.serif'] = ['Times New Roman']
    plt.rcParams['axes.labelsize'] = 14
    plt.rcParams['font.size'] = 14
    plt.rcParams['legend.fontsize'] = 12
    plt.rcParams['xtick.labelsize'] = 12
    plt.rcParams['ytick.labelsize'] = 12
    plt.rcParams['axes.grid'] = True
    plt.rcParams['grid.alpha'] = 0.3
    plt.rcParams['grid.linestyle'] = ':'
    plt.rcParams['grid.linewidth'] = 0.8
    plt.rcParams['axes.spines.top'] = False
    plt.rcParams['axes.spines.right'] = False
    plt.rcParams['xtick.direction'] = 'in'
    plt.rcParams['ytick.direction'] = 'in'
    
    # Create figure with 2 subplots (Velocity, Articulation Rate)
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    
    colors = {'cilqr': '#004c6d', 'alilqr': '#c7382d'}
    linestyles = {'cilqr': '-', 'alilqr': '--'}
    markers = {'cilqr': 'o', 'alilqr': '^'}
    
    # Plot Velocity on ax1
    for solver, (steps, vels, _) in data_dict.items():
        if not steps: continue
        ax1.plot(steps, vels, 
                 label=solver.upper(),
                 color=colors.get(solver, 'black'),
                 linestyle=linestyles.get(solver, '-'),
                 linewidth=1.5,
                 marker=markers.get(solver, None),
                 markevery=20, markersize=5, alpha=0.9)
                 
    ax1.set_ylabel('Velocity (m/s)')
    ax1.set_title(f'Control Output Comparison - Map: {map_name}', fontsize=16, pad=15)
    ax1.legend(loc='lower right', frameon=True) # Legend in a clearer spot
    
    # Plot Articulation Rate on ax2
    for solver, (steps, _, gammas) in data_dict.items():
        if not steps: continue
        ax2.plot(steps, gammas, 
                 label=solver.upper(),
                 color=colors.get(solver, 'black'),
                 linestyle=linestyles.get(solver, '-'),
                 linewidth=1.5,
                 marker=markers.get(solver, None),
                 markevery=20, markersize=5, alpha=0.9)
                 
    ax2.set_xlabel('Frame Index')
    ax2.set_ylabel('Articulation Rate (rad/s)')
    # ax2.legend() # Optional: legend on bottom plot too? Maybe redundant.
    
    plt.tight_layout()
    return fig

def main():
    parser = argparse.ArgumentParser(description='Compare Control Outputs (Velocity/Gamma)')
    parser.add_argument('--map', type=str, required=True, help='Map name (e.g. B301)')
    args = parser.parse_args()
    
    script_dir = os.path.dirname(__file__)
    solvers = ['cilqr', 'alilqr']
    data_dict = {}
    
    has_data = False
    for solver in solvers:
        data_folder = os.path.join(script_dir, "outputs", args.map, solver, "data")
        print(f"Reading data for {solver}...")
        steps, vels, gammas = read_solver_data(data_folder, solver)
        if steps:
            data_dict[solver] = (steps, vels, gammas)
            has_data = True
        else:
            print(f"Warning: No valid data found for {solver}")

    if not has_data:
        print("Error: No data found for any solver. Exiting.")
        return

    # Plot comparison
    fig = plot_comparison(args.map, data_dict)
    
    # Save output
    output_dir = os.path.join(script_dir, "outputs", args.map)
    output_path = os.path.join(output_dir, f"{args.map}_control_comparison.png")
    fig.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"\nComparison chart saved to: {output_path}")

if __name__ == "__main__":
    main()
