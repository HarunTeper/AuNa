#!/usr/bin/env python3
import os
import sys
import subprocess
import argparse
import glob
import platform


def main():
    parser = argparse.ArgumentParser(
        description='Setup and run the Enhanced CACC Visualizer')
    parser.add_argument('log_file', nargs='?',
                        help='Path to the CACC log CSV file')
    parser.add_argument('--cam-log', help='Path to the CAM message log file')
    parser.add_argument('--static', action='store_true',
                        help='Generate static plots only')
    parser.add_argument('--animation', action='store_true',
                        help='Generate animation plot only')
    parser.add_argument('--setup-only', action='store_true',
                        help='Just set up the environment without running')
    parser.add_argument('--verbose', '-v', action='store_true',
                        help='Enable verbose output')
    parser.add_argument('--upgrade', action='store_true',
                        help='Force upgrade of all dependencies')

    args = parser.parse_args()
    verbose = args.verbose

    print("\n🚗 Enhanced CACC Visualizer Setup/Run Utility 🚗")
    print("==============================================")

    # Ensure we're running with Python 3.6+
    python_version = sys.version_info
    if python_version.major < 3 or (python_version.major == 3 and python_version.minor < 6):
        print("❌ Error: Python 3.6 or higher is required.")
        print(f"   Current version: {platform.python_version()}")
        return 1

    if verbose:
        print(f"✓ Python version: {platform.python_version()}")
        print(f"✓ Platform: {platform.platform()}")

    workspace_dir = os.path.dirname(os.path.abspath(__file__))
    venv_dir = os.path.join(workspace_dir, 'cacc_venv')

    print(f"🔍 Using workspace directory: {workspace_dir}")
    print(f"🔧 Virtual environment path: {venv_dir}")

    # Check if virtual environment exists
    venv_exists = os.path.exists(venv_dir) and os.path.exists(
        os.path.join(venv_dir, 'bin', 'python'))

    if not venv_exists:
        print("\n📦 Creating virtual environment...")
        try:
            subprocess.run(
                [sys.executable, '-m', 'venv', venv_dir],
                check=True,
                stdout=subprocess.PIPE if not verbose else None,
                stderr=subprocess.PIPE if not verbose else None
            )
            print("✅ Virtual environment created successfully")
        except subprocess.CalledProcessError as e:
            print(f"❌ Error: Failed to create virtual environment:")
            print(f"   {e}")
            if verbose and e.stderr:
                print(f"   Details: {e.stderr.decode('utf-8').strip()}")
            return 1

        # Install required packages
        print("\n📚 Installing required packages...")
        need_install = True
    else:
        print("✅ Using existing virtual environment")
        need_install = args.upgrade
        if args.upgrade:
            print("🔄 Upgrading dependencies (--upgrade specified)...")

    if need_install:
        try:
            # Update pip, wheel, and setuptools first
            print("   Updating pip, wheel, and setuptools...")
            pip_cmd = [f"{venv_dir}/bin/pip", "install",
                       "-U", "pip", "wheel", "setuptools"]
            if verbose:
                print(f"   Running: {' '.join(pip_cmd)}")
            subprocess.run(
                pip_cmd,
                check=True,
                stdout=subprocess.PIPE if not verbose else None,
                stderr=subprocess.PIPE if not verbose else None
            )

            # Install all required packages for the enhanced visualizer
            print("   Installing visualizer dependencies...")
            packages = [
                "numpy==1.24.3",
                "matplotlib==3.7.1",
                "pandas==2.0.3",
                "seaborn==0.12.2",  # Added for correlation heatmaps
                "scipy==1.10.1"     # Added for statistical functions
            ]

            pip_cmd = [f"{venv_dir}/bin/pip", "install"] + packages
            if verbose:
                print(f"   Running: {' '.join(pip_cmd)}")
            result = subprocess.run(
                pip_cmd,
                check=True,
                stdout=subprocess.PIPE if not verbose else None,
                stderr=subprocess.PIPE if not verbose else None
            )
            print("✅ All packages installed successfully")
        except subprocess.CalledProcessError as e:
            print(f"❌ Error: Failed to install packages:")
            print(f"   {e}")
            if verbose and e.stderr:
                print(f"   Details: {e.stderr.decode('utf-8').strip()}")
            return 1

    if args.setup_only:
        print("\n🎉 Environment setup complete!")
        print("\n🚀 To run the visualizer:")
        print(
            f"   {venv_dir}/bin/python {workspace_dir}/cacc_visualizer.py /path/to/cacc_log_file.csv [options]")
        print("\n📋 Available options:")
        print("   --cam-log PATH    Path to the CAM message log file")
        print("   --static          Generate static plots only")
        print("   --animation       Generate animation plot only")
        return 0

    # Look for log files if none specified
    if not args.log_file:
        print("\n🔍 Searching for log files...")
        cacc_logs = glob.glob(os.path.join(workspace_dir, "*cacc*.csv"))
        cam_logs = glob.glob(os.path.join(workspace_dir, "*cam*.log"))

        if cacc_logs:
            # Use the most recent log file by modification time
            cacc_logs.sort(key=lambda x: os.path.getmtime(x), reverse=True)
            args.log_file = cacc_logs[0]
            print(f"   Found CACC log: {os.path.basename(args.log_file)}")

        if not args.cam_log and cam_logs:
            cam_logs.sort(key=lambda x: os.path.getmtime(x), reverse=True)
            args.cam_log = cam_logs[0]
            print(f"   Found CAM log: {os.path.basename(args.cam_log)}")

    # Run the visualizer if a log file was provided
    if args.log_file:
        log_path = args.log_file

        # Check if the log file exists
        if not os.path.exists(log_path):
            print(f"\n❌ Error: Log file not found: {log_path}")
            print(
                "   Please check that you have created log files by running CACC with logging enabled")
            print(
                "   Example: ros2 launch auna_cacc cacc_controller.launch.py enable_logging:=true")
            return 1

        cmd = [f"{venv_dir}/bin/python",
               os.path.join(workspace_dir, "cacc_visualizer.py"), log_path]

        if args.cam_log:
            if not os.path.exists(args.cam_log):
                print(f"\n⚠️  Warning: CAM log file not found: {args.cam_log}")
                print("   Continuing without CAM data")
            else:
                cmd.extend(["--cam-log", args.cam_log])

        if args.static:
            cmd.append("--static")
        if args.animation:
            cmd.append("--animation")

        print(f"\n🚀 Running visualizer with command:")
        print(f"   {' '.join(cmd)}\n")

        try:
            result = subprocess.run(cmd)
            if result.returncode != 0:
                print(
                    f"\n⚠️  Warning: Visualizer exited with code {result.returncode}")
            else:
                print("\n✅ Visualization completed successfully")
            return result.returncode
        except subprocess.CalledProcessError as e:
            print(f"\n❌ Error: Failed to run visualizer: {e}")
            return 1
        except KeyboardInterrupt:
            print("\n⏹️  Visualization stopped by user")
            return 0
    else:
        print("\n❌ Error: No log file provided or found.")
        print("   Please specify a log file path or generate logs by running CACC with logging enabled")

        print("\n📋 Available CSV files in the workspace:")
        # List CSV files in the workspace
        csv_files = glob.glob(os.path.join(workspace_dir, "*.csv"))
        if csv_files:
            for csv_file in sorted(csv_files):
                size_mb = os.path.getsize(csv_file) / (1024 * 1024)
                mod_time = os.path.getmtime(csv_file)
                import datetime
                mod_time_str = datetime.datetime.fromtimestamp(
                    mod_time).strftime('%Y-%m-%d %H:%M:%S')
                print(
                    f"   - {os.path.basename(csv_file)} ({size_mb:.2f} MB, modified: {mod_time_str})")
        else:
            print("   No CSV files found in the workspace")

        print("\n📋 Usage examples:")
        print(f"   {sys.argv[0]} /path/to/cacc_log.csv")
        print(
            f"   {sys.argv[0]} /path/to/cacc_log.csv --cam-log /path/to/cam_messages.log")
        print(f"   {sys.argv[0]} /path/to/cacc_log.csv --static")
        return 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n⏹️  Setup stopped by user")
        sys.exit(1)
