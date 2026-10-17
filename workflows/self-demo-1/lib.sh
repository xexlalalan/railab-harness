# shared by the self-demo-1 step scripts: every step is a chain of `harness ...` calls that
# stops at the first failure (non-zero exit).
set -euo pipefail
export PATH="$HOME/bin:$PATH"
step() { echo ">>> harness $*"; harness "$@"; }
