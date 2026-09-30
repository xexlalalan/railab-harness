# shared by the neuromorphic step scripts: every step is a chain of `harness ...` calls that
# stops at the first failure (non-zero exit) and ends with the module where the next hand move expects it.
set -euo pipefail
export PATH="$HOME/bin:$PATH"
step() { echo ">>> harness $*"; harness "$@"; }
