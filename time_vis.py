import argparse
import os
import json
import matplotlib.pyplot as plt
import numpy as np
import glob
import re

def get_data_files(data_dir, solver_type):
    """Get all JSON files sorted by frame number."""
    if not os.path.exists(data_dir):
        print(f"Warning: Directory not found: {data_dir}")
        return []
    
    files = glob.glob(os.path.join(data_dir, f"{solver_type}_data_*.json"))
    
    # Sort files by frame number extracted from filename
    def get_frame_num(filepath):
        match = re.search(r'_data_(\d+)\.json', filepath)
        return int(match.group(1)) if match else -1
        
    return sorted(files, key=get_frame_num)

def extract_solve_times(files):
    """Extract solve_time_ms from a list of JSON files."""
    times = []
    frames = []
    
    for f in files:
        try:
            with open(f, 'r') as file:
                data = json.load(file)
                # Check for convergence_info and solve_time_ms
                if "convergence_info" in data and "solve_time_ms" in data["convergence_info"]:
                    times.append(data["convergence_info"]["solve_time_ms"])
                    
                    # Extract frame number for plotting x-axis
                    match = re.search(r'_data_(\d+)\.json', f)
                    frames.append(int(match.group(1)) if match else len(frames))
        except Exception as e:
            print(f"Error reading {f}: {e}")
            
    return frames, times

def main():
    parser = argparse.ArgumentParser(description='Compare solve times of CILQR and ALILQR.')
    parser.add_argument('--map', type=str, required=True, help='Name of the map (e.g., B301)')
    
    args = parser.parse_args()
    map_name = args.map
    
    base_dir = os.path.join(os.getcwd(), 'outputs', map_name)
    output_plot_path = os.path.join(base_dir, f'{map_name}_solve_time_comparison.png')
    
    solvers = ['cilqr', 'alilqr']
    # IEEE Style Configuration - Modern/Clean
    plt.rcParams.update(plt.rcParamsDefault) # Reset defaults
    
    plt.rcParams['font.family'] = 'serif'
    plt.rcParams['font.serif'] = ['Times New Roman']
    plt.rcParams['axes.labelsize'] = 14
    plt.rcParams['font.size'] = 14
    plt.rcParams['legend.fontsize'] = 12
    plt.rcParams['xtick.labelsize'] = 12
    plt.rcParams['ytick.labelsize'] = 12
    
    # Grid settings
    plt.rcParams['axes.grid'] = True
    plt.rcParams['grid.alpha'] = 0.3
    plt.rcParams['grid.linestyle'] = ':'
    plt.rcParams['grid.linewidth'] = 0.8
    
    # Clean spines
    plt.rcParams['axes.spines.top'] = False
    plt.rcParams['axes.spines.right'] = False
    
    # Inward ticks
    plt.rcParams['xtick.direction'] = 'in'
    plt.rcParams['ytick.direction'] = 'in'
    
    # Figure size (width 8 inches, height 5 inches)
    plt.figure(figsize=(8, 5))
    
    has_data = False
    
    # Distinct, high-contrast colors (Dark Blue vs Brick Red)
    markers = {'cilqr': 'o', 'alilqr': '^'} 
    linestyles = {'cilqr': '-', 'alilqr': '--'}
    colors = {'cilqr': '#004c6d', 'alilqr': '#c7382d'} 
    data = {}
    
    for solver in solvers:
        data_dir = os.path.join(base_dir, solver, 'data')
        files = get_data_files(data_dir, solver)
        
        if not files:
            print(f"No data files found for {solver} in {data_dir}")
            continue
            
        frames, times = extract_solve_times(files)
        
        if not times:
            print(f"No solve times found in files for {solver}")
            continue
            
        has_data = True
        avg_time = np.mean(times)
        data[solver] = {'frames': frames, 'times': times, 'avg': avg_time}
        
        # Plot with IEEE style attributes
        plt.plot(frames, times, 
                 label=f'{solver.upper()} (Avg: {avg_time:.2f} ms)', 
                 color=colors[solver], 
                 linestyle=linestyles[solver], 
                 linewidth=1.5,
                 marker=markers[solver], 
                 markersize=4,
                 markevery=10,  # Markers every 10 points to avoid clutter
                 alpha=0.9)

    if not has_data:
        print("No valid data found for any solver. Exiting.")
        return

    plt.title(f'Solve Time Comparison - Map: {map_name}', fontsize=14)
    plt.xlabel('Frame Index', fontsize=12)
    plt.ylabel('Computation Time (ms)', fontsize=12)
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.legend(frameon=True, loc='best')
    
    plt.tight_layout()
    plt.savefig(output_plot_path, dpi=300, bbox_inches='tight')
    print(f"Comparison plot saved to: {output_plot_path}")
    # plt.show() # Uncomment if running in an environment with display

if __name__ == "__main__":
    main()
