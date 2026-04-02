#!/bin/bash
# Run multiple simulation configs sequentially
# Usage: ./run_batch.sh config1.json config2.json ...

set -e

if [ $# -eq 0 ]; then
  echo "Usage: $0 <config1.json> [config2.json ...]"
  exit 1
fi

for f in "$@"; do
  echo "=== Running: $f ==="
  ./mantaray_core "$f" 2>&1 | tee "$(basename "$f" .json).log"
  echo "=== Completed: $f ==="
done
