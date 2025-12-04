import subprocess
import re
import os
import sys

def run_solver(solver_type, executable_path):
    """
    Runs the C++ executable with the specified solver type.
    Returns a list of dictionaries containing iteration data (cpu_time, cmax).
    """
    cmd = [executable_path, "--solver", solver_type]
    print(f"Running {solver_type} solver...")
    
    try:
        # Run the command and capture output
        result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True, encoding='utf-8')
        
        if result.returncode != 0:
            print(f"Error running {solver_type}: {result.stderr}")
            return None

        output = result.stdout
        # Parse output
        data = []
        
        # Regex patterns
        # CPU time used: 12.34 ms
        # CMax: 0.5678
        cpu_pattern = re.compile(r"CPU time used:\s+([\d\.]+)\s+ms")
        cmax_pattern = re.compile(r"CMax:\s+([\d\.]+)")
        
        lines = output.split('\n')
        current_cpu = None
        current_cmax = None
        
        for line in lines:
            cpu_match = cpu_pattern.search(line)
            if cpu_match:
                current_cpu = float(cpu_match.group(1))
            
            cmax_match = cmax_pattern.search(line)
            if cmax_match:
                current_cmax = float(cmax_match.group(1))
                
            # Assuming CMax comes after CPU time in the loop, we collect when we have both (or just check per iteration logic)
            # Based on main.cpp:
            # print CPU time
            # print CMax
            if current_cpu is not None and current_cmax is not None:
                data.append({'cpu_time': current_cpu, 'cmax': current_cmax})
                current_cpu = None
                current_cmax = None
                
        return data

    except FileNotFoundError:
        print(f"Executable not found at: {executable_path}")
        return None
    except Exception as e:
        print(f"An error occurred: {e}")
        return None

def main():
    # Determine executable path
    # Assuming the script is in the project root and build is in ./build/bin/main.exe
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Try common paths
    possible_paths = [
        os.path.join(script_dir, "build", "bin", "main.exe"),
        os.path.join(script_dir, "build", "Debug", "main.exe"),
        os.path.join(script_dir, "build", "Release", "main.exe"),
        os.path.join(script_dir, "bin", "main.exe") # Fallback
    ]
    
    executable_path = None
    for path in possible_paths:
        if os.path.exists(path):
            executable_path = path
            break
            
    if not executable_path:
        print("Could not find 'main.exe'. Please build the project first.")
        # List checked paths for debugging
        print("Checked paths:")
        for path in possible_paths:
            print(f"  {path}")
        return

    print(f"Using executable: {executable_path}")

    # Run solvers
    cilqr_data = run_solver("cilqr", executable_path)
    alilqr_data = run_solver("alilqr", executable_path)

    if not cilqr_data or not alilqr_data:
        print("Failed to get data from one or both solvers.")
        return

    # Calculate averages
    def calculate_stats(data):
        if not data:
            return 0.0, 0.0
        avg_cpu = sum(d['cpu_time'] for d in data) / len(data)
        avg_cmax = sum(d['cmax'] for d in data) / len(data)
        return avg_cpu, avg_cmax

    cilqr_avg_cpu, cilqr_avg_cmax = calculate_stats(cilqr_data)
    alilqr_avg_cpu, alilqr_avg_cmax = calculate_stats(alilqr_data)

    # Print comparison
    print("\n" + "="*60)
    print(f"{'Metric':<20} | {'CILQR':<15} | {'ALILQR':<15}")
    print("-" * 60)
    print(f"{'Avg CPU Time (ms)':<20} | {cilqr_avg_cpu:<15.4f} | {alilqr_avg_cpu:<15.4f}")
    print(f"{'Avg CMax':<20} | {cilqr_avg_cmax:<15.4f} | {alilqr_avg_cmax:<15.4f}")
    print("="*60)
    
    # Optional: Print total iterations
    print(f"Total Iterations: CILQR={len(cilqr_data)}, ALILQR={len(alilqr_data)}")

if __name__ == "__main__":
    main()
