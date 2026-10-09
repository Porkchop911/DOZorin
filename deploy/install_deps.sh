#!/bin/bash
# One-time user-local dependencies for the gateway/CLI-facing runtime.
set -euo pipefail
python3 -m pip install --user 'fastapi<1' 'uvicorn[standard]<1' requests websockets
echo "gateway dependencies installed for $(whoami)"
