#!/usr/bin/env bash
# run_ablation.sh — HeteroPool memory-pool ablation experiment runner.
#
# Two calling conventions:
#   (a) Full matrix mode (no args):
#         ./run_ablation.sh [-- python args ...]
#       Delegates to run_ablation.py to run 4 modes × 3 scenarios × 10 reps.
#
#   (b) Single-run mode (mode + yaml args):
#         ./run_ablation.sh <full|pageable|nofastpath|noreuse> <yaml>
#       Runs one dataflow with the given ablation settings.  Useful for
#       ad-hoc testing during development.
#
# Examples:
#   ./run_ablation.sh                            # full matrix
#   ./run_ablation.sh -n 2                       # quick smoke test
#   ./run_ablation.sh --dry-run                  # preview plan
#   ./run_ablation.sh pageable cpu2cuda.yml      # single ad-hoc run
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# ------------------------------------------------------------------
# Full-matrix mode (no args, or args starting with '-')
# ------------------------------------------------------------------
if [ $# -eq 0 ] || [ "${1:0:1}" = "-" ]; then
    exec python3 "$SCRIPT_DIR/run_ablation.py" "$@"
fi

# ------------------------------------------------------------------
# Single-run mode (mode + yaml)
# ------------------------------------------------------------------
MODE="${1:?Usage: $0 <auto|pinned|pageable|nofastpath|noreuse> <yaml>}"
YAML="${2:?Usage: $0 <auto|pinned|pageable|nofastpath|noreuse> <yaml>}"

case "$MODE" in
  auto)       ;;
  pinned)     export HETEROPOOL_MODE=pinned ;;
  pageable)   export HETEROPOOL_MODE=pageable ;;
  nofastpath) export HETEROPOOL_NO_FASTPATH=1 ;;
  noreuse)    export HETEROPOOL_NO_REUSE=1 ;;
  *)
    echo "Unknown mode '$MODE'. Valid: auto, pinned, pageable, nofastpath, noreuse"
    exit 1 ;;
esac

echo "=== Ablation single run ==="
echo "Mode:  $MODE"
echo "YAML:  $YAML"
echo "Env:"
env | grep -E 'HETEROPOOL_' || echo "  (none)"
echo "==========================="

dora run "$YAML" --stop-after 100s
