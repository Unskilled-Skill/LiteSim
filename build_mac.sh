#!/usr/bin/env bash
set -euo pipefail

# Build LiteSim macOS app bundle with PyInstaller.

if [[ "$(uname)" != "Darwin" ]]; then
  echo "This script is for macOS. Detected: $(uname)"
  exit 1
fi

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT_DIR"

if ! command -v python3 >/dev/null 2>&1; then
  echo "python3 not found. Install Python 3.10+."
  exit 1
fi

python3 - <<'PY'
import sys
if sys.version_info < (3, 10):
    sys.exit("Python 3.10+ required.")
PY

VENV_DIR="${ROOT_DIR}/.venv"
if [[ ! -d "$VENV_DIR" ]]; then
  echo "Creating venv at $VENV_DIR"
  python3 -m venv "$VENV_DIR"
fi

# shellcheck source=/dev/null
source "$VENV_DIR/bin/activate"

python -m pip install --upgrade pip
python -m pip install -r requirements.txt
python -m pip install pyinstaller

python build.py

echo "Built app at dist/LiteSim.app"
