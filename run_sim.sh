#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"
VENV_PY=".venv/bin/python"
PY=""

if [ -x "$VENV_PY" ] && "$VENV_PY" -c "import sys; sys.exit(0 if sys.version_info >= (3, 10) else 1)" >/dev/null 2>&1; then
    PY="$VENV_PY"
else
    for cand in python3 python; do
        if command -v "$cand" >/dev/null 2>&1 && "$cand" -c "import sys; sys.exit(0 if sys.version_info >= (3, 10) else 1)" >/dev/null 2>&1; then
            PY="$cand"
            break
        fi
    done
fi

if [ -z "$PY" ]; then
    echo "Python 3.10+ not found. Install Python 3.10+ and re-run." >&2
    exit 1
fi

if ! "$PY" -c "import mujoco, numpy, scipy, matplotlib, plotly, PIL" >/dev/null 2>&1; then
    if [ "$PY" != "$VENV_PY" ]; then
        echo "Creating virtual environment in .venv..."
        "$PY" -m venv .venv
        PY="$VENV_PY"
    fi
    echo "Installing requirements..."
    "$PY" -m pip install -r requirements.txt
fi

MJPY="$PY"
if [ "$(uname -s)" = "Darwin" ]; then
    PY_DIR="$(cd "$(dirname "$PY")" 2>/dev/null && pwd || true)"
    if [ -x ".venv/bin/mjpython" ]; then
        MJPY=".venv/bin/mjpython"
    elif [ -n "$PY_DIR" ] && [ -x "$PY_DIR/mjpython" ]; then
        MJPY="$PY_DIR/mjpython"
    elif command -v mjpython >/dev/null 2>&1; then
        MJPY="$(command -v mjpython)"
    fi
fi

echo ""
echo "Hydrofoil Sim Launcher (Python: $PY)"
echo "  [1] Web Dashboard + Boat Editor + CAD Boot-Up (default)"
echo "  [2] CAD Boat 3D Viewer - Auto Combined + Supervisor + Waves + Noisy Sensors"
echo "  [3] Manned 275 kg 3D Viewer - Auto Combined + Supervisor + Calm"
echo "  [4] Model 25 kg 3D Viewer   - Raw LQR/PID + Calm"
echo "  [5] Model Consistency Check"
echo "  [6] Run Pytest Suite"
echo ""
read -r -p "Select option [1-6] (Enter for 1): " CHOICE
CHOICE="${CHOICE:-1}"

case "$CHOICE" in
    1) echo "Starting web dashboard on http://127.0.0.1:8765 (Ctrl+C to stop)..."
       exec "$PY" -m hydrofoil.sim_server ;;
    2) exec "$MJPY" -m hydrofoil.viewer --preset cad --controller auto --supervisor --sensors noisy --waves ;;
    3) exec "$MJPY" -m hydrofoil.viewer --preset manned --controller auto --supervisor ;;
    4) exec "$MJPY" -m hydrofoil.viewer --preset model ;;
    5) exec "$PY" -m hydrofoil.consistency ;;
    6) exec "$PY" -m pytest -v ;;
    *) echo "Invalid choice: $CHOICE" >&2; exit 1 ;;
esac
