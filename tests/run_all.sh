#!/usr/bin/env bash
# Runs every test_*.py in this directory, stopping at the first failure.
set -e
cd "$(dirname "${BASH_SOURCE[0]}")"
for f in test_*.py; do
  echo "=== $f ==="
  python3 "$f"
done
echo "ALL TEST FILES PASSED"
