#!/usr/bin/env python3
import os
import sys
import subprocess
import argparse


def main():
    parser = argparse.ArgumentParser(
        description='Setup CACC Visualizer environment and run visualizer')
    parser.add_argument('log_file', nargs='?',
                        help='Path to the CACC log CSV file')
    parser.add_argument('--static', action='store_true',
                        help='Generate static plots only')
    parser.add_argument('--animation', action='store_true',
                        help='Generate animation plot only')
    parser.add_argument('--setup-only', action='store_true',
                        help='Just set up the environment without running')
    parser.add_argument('--verbose', '-v', action='store_true',
                        help='Enable verbose output')

    args = parser.parse_args()
    verbose = args.verbose

    print("CACC Visualizer Setup/Run Utility")
    print("---------------------------------")

    venv_dir = os.path.join(os.path.dirname(
        os.path.abspath(__file__)), 'cacc_venv')
    print(f"Using virtual environment path: {venv_dir}")

    # Check if virtual environment exists
    if not os.path.exists(venv_dir):
        print(f"Creating virtual environment at {venv_dir}...")
        try:
            subprocess.run(
                [sys.executable, '-m', 'venv', venv_dir], check=True)
        except subprocess.CalledProcessError as e:
            print(f"ERROR: Failed to create virtual environment: {e}")
            return 1

        # Install required packages
        print("Installing required packages...")
        try:
            pip_cmd = [f"{venv_dir}/bin/pip", "install",
                       "-U", "pip", "wheel", "setuptools"]
            if verbose:
                print(f"Running: {' '.join(pip_cmd)}")
            subprocess.run(pip_cmd, check=True)

            pip_cmd = [f"{venv_dir}/bin/pip", "install",
                       "numpy==1.24.3", "matplotlib==3.7.1", "pandas==2.0.3"]
            if verbose:
                print(f"Running: {' '.join(pip_cmd)}")
            subprocess.run(pip_cmd, check=True)
            print("✅ Environment setup complete!")
        except subprocess.CalledProcessError as e:
            print(f"ERROR: Failed to install packages: {e}")
            return 1
    else:
        print(f"✅ Using existing virtual environment at {venv_dir}")

    if args.setup_only:
        print(f"Environment is ready at {venv_dir}")
        print(
            f"To run visualizer: {venv_dir}/bin/python /home/vscode/workspace/cacc_visualizer.py LOG_FILE_PATH")
        return 0

    # Run the visualizer if a log file was provided
    if args.log_file:
        log_path = args.log_file

        # Check if the log file exists
        if not os.path.exists(log_path):
            print(f"ERROR: Log file not found: {log_path}")
            print(
                "Please check that you have created log files by running CACC with logging enabled")
            print(
                "Example: ros2 launch auna_cacc cacc_controller.launch.py enable_logging:=true")
            return 1

        cmd = [f"{venv_dir}/bin/python",
               "/home/vscode/workspace/cacc_visualizer.py", log_path]

        if args.static:
            cmd.append("--static")
        if args.animation:
            cmd.append("--animation")

        print(f"Running visualizer with command: {' '.join(cmd)}")
        try:
            result = subprocess.run(cmd, check=True)
            if result.returncode != 0:
                print(
                    f"WARNING: Visualizer exited with code {result.returncode}")
            return result.returncode
        except subprocess.CalledProcessError as e:
            print(f"ERROR: Failed to run visualizer: {e}")
            return 1
    else:
        print("ERROR: No log file provided. Please specify a log file path.")
        print(
            f"Usage: {sys.argv[0]} /path/to/cacc_log_file.csv [--static] [--animation]")
        print("\nAvailable log files in the current directory:")
        # List CSV files in the workspace
        csv_files = [f for f in os.listdir(
            '/home/vscode/workspace') if f.endswith('.csv')]
        if csv_files:
            for csv_file in csv_files:
                print(f"  - /home/vscode/workspace/{csv_file}")
        else:
            print("  No CSV files found in /home/vscode/workspace")
        return 1


if __name__ == "__main__":
    sys.exit(main())
