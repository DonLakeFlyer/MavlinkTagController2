#!/usr/bin/env bash
#
# Create and configure the Python virtual environment for the simulator
# and detector pipeline.
#
# Usage:  ./setup_venv.sh
# Then:   source .venv/bin/activate

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
VENV_DIR="$SCRIPT_DIR/.venv"
REQUIREMENTS="$SCRIPT_DIR/simulator/requirements.txt"

# --- Check python3 available ---
if ! command -v python3 &>/dev/null; then
    echo "Error: python3 not found." >&2
    exit 1
fi

PYTHON_VERSION=$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')
echo "Using Python $PYTHON_VERSION"

# --- Check venv module available ---
if ! python3 -m venv --help &>/dev/null; then
    echo "Error: python3-venv is not installed." >&2
    echo "  Ubuntu/Debian: sudo apt-get install -y python${PYTHON_VERSION}-venv" >&2
    echo "  macOS:         (included with Homebrew Python)" >&2
    exit 1
fi

# --- Create or update venv ---
if [ -d "$VENV_DIR" ]; then
    echo "Virtual environment already exists at $VENV_DIR"
    echo "  Updating packages..."
else
    echo "Creating virtual environment at $VENV_DIR ..."
    python3 -m venv "$VENV_DIR"
fi

# shellcheck disable=SC1091
source "$VENV_DIR/bin/activate"

pip install --upgrade pip --quiet
pip install -r "$REQUIREMENTS"

echo ""
echo "Done. Activate with:"
echo "  source .venv/bin/activate"
