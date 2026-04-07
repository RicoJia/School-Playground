#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd "$(dirname "$0")" && pwd)"

script="$script_dir/izci.py"

chmod +x "$script"

# Prefer the virtualenv python if present, otherwise fall back to system python
if [ -x "$script_dir/.venv/bin/python3" ]; then
	python_exec="$script_dir/.venv/bin/python3"
else
	python_exec="$(command -v python3 || echo /usr/bin/env python3)"
fi

tmpfile=$(mktemp)
crontab -l 2>/dev/null | grep -v -F "$script" > "$tmpfile" || true
# Add the job: run at 10:30 (hour 10, minute 30) in America/Chicago timezone
# Use the venv python to run the script and log output to the user's home
echo "55 10 * * * TZ=America/Chicago cd \"$script_dir\" && $python_exec \"$script\" >> \"$HOME/izci.log\" 2>&1" >> "$tmpfile"
crontab "$tmpfile"
rm -f "$tmpfile"

echo "Installed cron job to run: $python_exec $script at 10:55 AM America/Chicago every day."

echo "Log file: $HOME/izci.log"