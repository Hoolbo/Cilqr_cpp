import json
import os
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

def read_alilqr_data(data_folder):
    """
    Read all ALILQR JSON files from specified folder
    
    Args:
        data_folder (str): Folder path containing JSON files
    
    Returns:
        tuple: (time step list, velocity list, articulation rate list)
    """
    velocities = []
    gammas = []
    time_steps = []
    
    # Get all alilqr_data_*.json files
    json_files = []
    for filename in os.listdir(data_folder):
        if filename.startswith('alilqr_data_') and filename.endswith('.json'):
            # Extract file number for sorting
            try:
                file_num = int(filename.replace('alilqr_data_', '').replace('.json', ''))
                json_files.append((file_num, filename))
            except ValueError:
                continue
    
    # Sort by file number
    json_files.sort(key=lambda x: x[0])
    
    print(f"Found {len(json_files)} ALILQR data files")
    
    # Read each file and extract data
    for file_num, filename in json_files:
        file_path = os.path.join(data_folder, filename)
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
            
            # Extract first values from velocity and gamma arrays in control_sequence
            control_seq = data.get('control_sequence', {})
            velocity_array = control_seq.get('velocity', [])
            gamma_array = control_seq.get('gamma', [])
            
            if velocity_array and gamma_array:
                velocities.append(velocity_array[0])  # First velocity value
                gammas.append(gamma_array[0])         # First articulation rate value
                time_steps.append(file_num)           # Use file number as time step
                
        except (json.JSONDecodeError, FileNotFoundError, KeyError) as e:
            print(f"Error reading file {filename}: {e}")
            continue
    
    return time_steps, velocities, gammas

def check_data_continuity(time_steps, velocities, gammas):
    """
    Check for data continuity and display statistics
    
    Args:
        time_steps (list): Time step list
        velocities (list): Velocity list
        gammas (list): Articulation rate list
    """
    # Check for data continuity
    missing_steps = []
    for i in range(1, len(time_steps)):
        if time_steps[i] - time_steps[i-1] > 1:
            missing_range = list(range(time_steps[i-1] + 1, time_steps[i]))
            missing_steps.extend(missing_range)
    
    if missing_steps:
        print(f"Warning: Missing data files for time steps: {missing_steps}")
    
    # Display statistics
    vel_mean = np.mean(velocities)
    vel_std = np.std(velocities)
    gamma_mean = np.mean(gammas)
    gamma_std = np.std(gammas)
    
    print(f"\nData Statistics:")
    print(f"Total data points: {len(time_steps)}")
    print(f"Velocity - Mean: {vel_mean:.3f} m/s, Std: {vel_std:.3f} m/s")
    print(f"Velocity - Min: {min(velocities):.3f} m/s, Max: {max(velocities):.3f} m/s")
    print(f"Articulation Rate - Mean: {gamma_mean:.6f} rad/s, Std: {gamma_std:.6f} rad/s")
    print(f"Articulation Rate - Min: {min(gammas):.6f} rad/s, Max: {max(gammas):.6f} rad/s")

def plot_alilqr_data(time_steps, velocities, gammas):
    """
    Plot velocity and articulation rate data
    
    Args:
        time_steps (list): Time step list
        velocities (list): Velocity list
        gammas (list): Articulation rate list
    
    Returns:
        matplotlib.figure.Figure: Generated figure object
    """
    # Set font to support English display
    plt.rcParams['font.family'] = ['Arial', 'DejaVu Sans', 'sans-serif']
    plt.rcParams['axes.unicode_minus'] = False
    
    # Create figure and subplots
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
    
    # Calculate statistics
    vel_mean = np.mean(velocities)
    gamma_mean = np.mean(gammas)
    
    # Plot velocity
    ax1.plot(time_steps, velocities, 'b-', linewidth=1.0, label='Velocity')
    ax1.axhline(y=vel_mean, color='r', linestyle='--', alpha=0.7, linewidth=0.8, 
                label=f'Mean: {vel_mean:.3f} m/s')
    ax1.set_ylabel('Velocity (m/s)')
    ax1.set_title('ALILQR Data Analysis - Velocity and Articulation Rate Over Time')
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    
    # Plot articulation rate
    ax2.plot(time_steps, gammas, 'g-', linewidth=1.0, label='Articulation Rate')
    ax2.axhline(y=gamma_mean, color='r', linestyle='--', alpha=0.7, linewidth=0.8,
                label=f'Mean: {gamma_mean:.5f} rad/s')
    ax2.set_xlabel('Time Step')
    ax2.set_ylabel('Articulation Rate (rad/s)')
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    
    # Adjust layout
    plt.tight_layout()
    
    # Check data continuity
    check_data_continuity(time_steps, velocities, gammas)
    
    return fig

def main():
    """Main function"""
    # Data folder path
    script_dir = os.path.dirname(__file__)
    data_folder = os.path.join(script_dir, "data")
    
    # Check if folder exists
    if not os.path.exists(data_folder):
        print(f"Error: Data folder does not exist: {data_folder}")
        return
    
    print("Starting to read ALILQR data...")
    
    # Read data
    time_steps, velocities, gammas = read_alilqr_data(data_folder)
    
    if not time_steps:
        print("Error: No valid data files found")
        return
    
    print(f"Successfully read {len(time_steps)} data points")
    
    # Plot charts
    fig = plot_alilqr_data(time_steps, velocities, gammas)
    
    # Save chart
    output_path = os.path.join(script_dir, "alilqr_analysis.png")
    fig.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"\nChart saved to: {output_path}")
    
    # Show chart
    plt.show()

if __name__ == "__main__":
    main()