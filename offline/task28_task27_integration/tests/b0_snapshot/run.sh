#!/usr/bin/env bash
set -euo pipefail
repo=$(git rev-parse --show-toplevel)
test_dir="$repo/offline/task28_task27_integration/tests/b0_snapshot"
tmp_dir=$(mktemp -d)
trap 'rm -rf "$tmp_dir"' EXIT
python3 - "$repo" "$tmp_dir" <<'PY'
import json
from pathlib import Path
import sys
path = Path(sys.argv[1]) / 'offline/task28_task27_integration/evidence/task28_b0_poststep_probe_v2/poststep.jsonl'
with path.open() as stream:
    sample = json.loads(next(stream))['snapshot']
(Path(sys.argv[2]) / 'sample.json').write_text(json.dumps(sample))
PY
g++ -std=c++17 -I "$repo/ros_ws/src/fr3_dual_palletize/src" "$test_dir/test_snapshot.cpp" -lyaml-cpp -o "$tmp_dir/test_snapshot"
"$tmp_dir/test_snapshot" "$tmp_dir/sample.json"
cmake -S "$test_dir" -B "$tmp_dir/build" -DCMAKE_BUILD_TYPE=Release >/dev/null
cmake --build "$tmp_dir/build" -j2 >/dev/null
"$tmp_dir/build/ros_buffer_test"
