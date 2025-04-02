#!/bin/bash
set -e

echo "CACC Visualizer Launcher"
echo "------------------------"

# Make script executable (just in case)
chmod +x /home/vscode/workspace/setup_cacc_visualizer.py

# Check if any arguments were provided
if [ $# -eq 0 ]; then
    echo "ERROR: No arguments provided. You must specify a log file path."
    echo "Usage: $0 /path/to/cacc_log_file.csv [--static] [--animation]"
    
    # List available log files
    echo -e "\nAvailable log files in workspace:"
    CSV_FILES=$(find /home/vscode/workspace -name "*.csv" -type f | sort)
    if [ -n "$CSV_FILES" ]; then
        echo "$CSV_FILES" | sed 's/^/  - /'
    else
        echo "  No CSV files found in /home/vscode/workspace"
    fi
    exit 1
fi

# Check if the log file exists if it's the first argument and doesn't start with --
if [[ $1 != --* ]] && [ ! -f "$1" ]; then
    echo "ERROR: Log file does not exist: $1"
    exit 1
fi

# Run the setup script with all arguments passed through and with verbose output
python3 /home/vscode/workspace/setup_cacc_visualizer.py --verbose "$@"
