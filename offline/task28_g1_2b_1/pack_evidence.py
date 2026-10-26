#!/usr/bin/env python3
"""Verify and losslessly package only the new G1-2B-1 trajectory traces.

Run after run_validation.py has finished writing summary.json.  Scientific JSON
bytes are preserved exactly inside deterministic gzip members.  Receipts are
written before matching raw traces are removed; rerunning verifies existing
members and repairs an interrupted raw-file cleanup without changing results.
"""
from __future__ import annotations

from copy import deepcopy
from fractions import Fraction
import gzip
import hashlib
import json
import os
from pathlib import Path
import resource
import tempfile
import time


HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
EVIDENCE = HERE / "evidence"
SUMMARY = EVIDENCE / "summary.json"
PACKAGING = EVIDENCE / "packaging.json"
PAIR_COUNT = 414
CATEGORIES = ("controls", "a_stage")
PACKAGING_SCHEMA = "task28.g1_2b_1.lossless_trace_packaging.v1"


def digest(data):
    return hashlib.sha256(data).hexdigest()


def relative(path):
    return str(path.relative_to(REPO))


def encoded(value, sort_keys=False):
    return (json.dumps(value, ensure_ascii=False, indent=2, sort_keys=sort_keys,
                       allow_nan=False) + "\n").encode("utf-8")


def atomic_write(path, data):
    if path.is_symlink():
        raise AssertionError("SYMLINK_OUTPUT_FORBIDDEN:" + str(path))
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(prefix=path.name + ".", suffix=".pack-tmp",
                                         dir=path.parent, delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        temporary.replace(path)
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()


def receipt(path, data):
    return {"path": relative(path), "sha256": digest(data), "bytes": len(data)}


def verify_receipt(data, expected):
    assert type(expected["bytes"]) is int and len(data) == expected["bytes"], "RECEIPT_SIZE_MISMATCH"
    assert digest(data) == expected["sha256"], "RECEIPT_HASH_MISMATCH"


def integer(value):
    assert type(value) is int, "EXPECTED_EXACT_INTEGER"
    return value


def validate_row(row, pairs, shape_count):
    """Check complete pair coverage and every claimed exact separation gap."""
    boxes = row["shape_world_aabbs"]
    assert len(boxes) == shape_count, "AABB_SHAPE_COVERAGE_MISMATCH"
    for box in boxes:
        assert len(box) == 3, "INVALID_AABB_AXIS_COUNT"
        for bounds in box:
            assert len(bounds) == 2, "INVALID_AABB_BOUNDS"
            assert integer(bounds[0]) <= integer(bounds[1]), "REVERSED_AABB_BOUNDS"
    proven = []
    for proof in row["pair_separation_proofs"]:
        index = integer(proof["pair"])
        axis = integer(proof["axis"])
        direction = integer(proof["direction"])
        assert 0 <= index < PAIR_COUNT and 0 <= axis < 3, "INVALID_SEPARATION_INDEX"
        assert direction in (-1, 1), "INVALID_SEPARATION_DIRECTION"
        left, right = pairs[index]
        a, b = boxes[left][axis], boxes[right][axis]
        gap = b[0] - a[1] if direction == 1 else a[0] - b[1]
        assert integer(proof["gap_lower_scaled"]) == gap and gap > 0, "INVALID_SEPARATION_GAP"
        proven.append(index)
    unresolved = [integer(index) for index in row["unresolved_pairs"]]
    assert len(proven) == len(set(proven)), "DUPLICATE_PAIR_PROOF"
    assert len(unresolved) == len(set(unresolved)), "DUPLICATE_UNRESOLVED_PAIR"
    assert sorted(proven + unresolved) == list(range(PAIR_COUNT)), "PAIR_COVERAGE_GAP_OR_DUPLICATION"
    return len(proven)


def validate_trace(data):
    """Replay stored integer gap arithmetic and exact span coverage only."""
    trace = json.loads(data)
    trajectory, result = trace["trajectory"], trace["continuous_result"]
    assert result["collision"]["checked_pairs"] == PAIR_COUNT, "CHECKED_PAIR_COUNT_CHANGED"
    assert result["collision"]["skipped_acm_pairs"] == 40, "ACM_PAIR_COUNT_CHANGED"
    assert integer(result["interval_scale"]) == 1 << 80, "INTERVAL_SCALE_CHANGED"
    shapes, pairs = result["shape_catalog"], result["pair_catalog"]
    assert [shape["index"] for shape in shapes] == list(range(len(shapes))), "SHAPE_CATALOG_INDICES_INVALID"
    assert len(pairs) == PAIR_COUNT, "PAIR_CATALOG_SIZE_CHANGED"
    normalized_pairs = []
    for pair in pairs:
        assert len(pair) == 2, "INVALID_PAIR_CATALOG_ENTRY"
        left, right = map(integer, pair)
        assert 0 <= left < right < len(shapes), "INVALID_PAIR_SHAPE_INDEX"
        normalized_pairs.append((left, right))
    assert len(set(normalized_pairs)) == PAIR_COUNT, "DUPLICATE_CATALOG_PAIR"
    waypoint_count = len(trajectory["waypoints"])
    assert waypoint_count >= 2, "INVALID_WAYPOINT_COUNT"
    leaves = {span: [] for span in range(waypoint_count - 1)}
    endpoints, proof_count, leaf_count = [], 0, 0
    for row in result["intervals"]:
        proof_count += validate_row(row, normalized_pairs, len(shapes))
        span = integer(row["span"])
        assert span in leaves, "INVALID_SPAN_INDEX"
        assert len(row["u"]) == 2, "INVALID_PARAMETER_INTERVAL"
        lo, hi = map(Fraction, row["u"])
        if row.get("kind") == "explicit_endpoint":
            endpoint = integer(row["waypoint_index"])
            assert 0 <= endpoint < waypoint_count, "INVALID_EXPLICIT_ENDPOINT_INDEX"
            endpoints.append(endpoint)
        else:
            assert 0 <= lo < hi <= 1, "INVALID_LEAF_PARAMETER_RANGE"
            leaves[span].append((lo, hi))
            leaf_count += 1
    assert sorted(endpoints) == list(range(waypoint_count)), "EXPLICIT_ENDPOINT_COVERAGE_GAP_OR_DUPLICATION"
    for span, ranges in leaves.items():
        assert ranges, "MISSING_SPAN_COVERAGE:" + str(span)
        cursor = Fraction(0)
        for lo, hi in sorted(ranges):
            assert lo == cursor, "SPAN_PARTITION_GAP_OR_OVERLAP:" + str(span)
            cursor = hi
        assert cursor == 1, "SPAN_PARTITION_INCOMPLETE:" + str(span)
    for witness in result["witnesses"]:
        row = witness["point_enclosure"]
        proof_count += validate_row(row, normalized_pairs, len(shapes))
        assert integer(row["span"]) == integer(witness["span"]) in leaves, "WITNESS_SPAN_MISMATCH"
        assert type(witness["u"]) in (float, int), "INVALID_WITNESS_PARAMETER"
        parameter = Fraction.from_float(float(witness["u"]))
        assert 0 <= parameter <= 1, "WITNESS_PARAMETER_OUT_OF_RANGE"
        assert tuple(map(Fraction, row["u"])) == (parameter, parameter), "WITNESS_POINT_ENCLOSURE_MISMATCH"
    return {
        "checked_pairs_per_row": PAIR_COUNT,
        "span_count": len(leaves),
        "leaf_interval_count": leaf_count,
        "explicit_endpoint_count": len(endpoints),
        "interval_row_count": len(result["intervals"]),
        "witness_point_row_count": len(result["witnesses"]),
        "verified_separation_gap_count": proof_count,
        "span_partitions": "exact_0_to_1_without_gaps_or_overlap",
        "pair_coverage": "every_row_proven_plus_unresolved_exactly_0_to_413",
    }


def main():
    started = time.perf_counter()
    assert SUMMARY.is_file() and not SUMMARY.is_symlink(), "COMPLETED_SUMMARY_REQUIRED"
    original_summary_bytes = SUMMARY.read_bytes()
    original_summary = json.loads(original_summary_bytes)
    assert original_summary["schema"] == "task28.g1_2b_1.summary.v1", "UNEXPECTED_SUMMARY_SCHEMA"
    summary = deepcopy(original_summary)
    previous = None
    if PACKAGING.exists():
        assert not PACKAGING.is_symlink(), "SYMLINK_PACKAGING_FORBIDDEN"
        previous = json.loads(PACKAGING.read_bytes())
        assert previous["schema"] == PACKAGING_SCHEMA, "UNEXPECTED_PACKAGING_SCHEMA"
    if "trace_packaging" in summary:
        assert previous is not None, "MISSING_PACKAGING_RECEIPT"
        assert summary["trace_packaging"]["path"] == relative(PACKAGING), "PACKAGING_PATH_MISMATCH"
        verify_receipt(PACKAGING.read_bytes(), summary["trace_packaging"])
    conversions, raw_cleanup = [], []
    for category in CATEGORIES:
        for name, entry in sorted(summary[category].items()):
            assert type(name) is str and name and Path(name).name == name and name not in (".", ".."), "INVALID_TRACE_NAME"
            raw_path = EVIDENCE / category / (name + ".json")
            packed_path = raw_path.with_suffix(".json.gz")
            assert not raw_path.is_symlink() and not packed_path.is_symlink(), "SYMLINK_TRACE_FORBIDDEN"
            current = entry["evidence"]
            if current["path"] == relative(raw_path):
                raw = raw_path.read_bytes()
                verify_receipt(raw, current)
                original_receipt = {key: current[key] for key in ("path", "sha256", "bytes")}
            else:
                assert current["path"] == relative(packed_path), "UNEXPECTED_TRACE_RECEIPT_PATH"
                packed_existing = packed_path.read_bytes()
                verify_receipt(packed_existing, current)
                raw = gzip.decompress(packed_existing)
                original_receipt = current["packaging"]["original"]
                assert original_receipt["path"] == relative(raw_path), "ORIGINAL_TRACE_PATH_MISMATCH"
                verify_receipt(raw, original_receipt)
            validation = validate_trace(raw)
            packed = gzip.compress(raw, compresslevel=9, mtime=0)
            assert gzip.decompress(packed) == raw, "GZIP_ROUNDTRIP_MISMATCH"
            if packed_path.exists():
                existing = packed_path.read_bytes()
                assert existing == packed, "EXISTING_GZIP_NOT_DETERMINISTIC_MATCH"
                assert gzip.decompress(existing) == raw, "EXISTING_GZIP_ROUNDTRIP_MISMATCH"
            else:
                atomic_write(packed_path, packed)
                assert packed_path.read_bytes() == packed, "GZIP_WRITE_MISMATCH"
            compressed_receipt = receipt(packed_path, packed)
            metadata = {
                "format": "gzip", "compression_level": 9, "mtime": 0,
                "original": original_receipt,
                "compressed": compressed_receipt,
                "roundtrip_bytes_identical": True,
            }
            entry["evidence"] = {**compressed_receipt, "packaging": metadata}
            conversions.append({"category": category, "name": name,
                                "packaging": metadata, "trace_validation": validation})
            raw_cleanup.append((raw_path, original_receipt))
    packaging = {
        "schema": PACKAGING_SCHEMA,
        "method": "gzip.compress(raw,compresslevel=9,mtime=0)",
        "script": receipt(Path(__file__).resolve(), Path(__file__).read_bytes()),
        "source_summary_sha256_before_packaging": (
            previous["source_summary_sha256_before_packaging"] if previous else digest(original_summary_bytes)),
        "scientific_trace_json_modified": False,
        "trace_count": len(conversions),
        "original_trace_bytes": sum(row["packaging"]["original"]["bytes"] for row in conversions),
        "compressed_trace_bytes": sum(row["packaging"]["compressed"]["bytes"] for row in conversions),
        "conversions": conversions,
    }
    if (previous is not None and previous.get("conversions") == conversions
            and previous.get("script") == packaging["script"]):
        packaging["packaging_runtime_s"] = previous["packaging_runtime_s"]
        packaging["peak_RSS_KiB"] = previous["peak_RSS_KiB"]
    else:
        packaging["packaging_runtime_s"] = time.perf_counter() - started
        packaging["peak_RSS_KiB"] = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    packaging_bytes = encoded(packaging, sort_keys=True)
    atomic_write(PACKAGING, packaging_bytes)
    summary["trace_packaging"] = receipt(PACKAGING, packaging_bytes)
    assert summary["schema"] == original_summary["schema"] and summary["status"] == original_summary["status"], "SCIENTIFIC_SUMMARY_IDENTITY_CHANGED"
    for key in ("new_tests", "regression"):
        assert summary[key] == original_summary[key], "TEST_RECEIPT_CHANGED"
    # Predict the post-cleanup inventory and include summary.json's own bytes.
    # Its decimal byte-count field reaches a fixed point in at most a few steps.
    excluded = {path for path, _ in raw_cleanup} | {SUMMARY}
    other_bytes = sum(path.stat().st_size for path in EVIDENCE.rglob("*")
                      if path.is_file() and path not in excluded)
    for _ in range(16):
        summary_bytes = encoded(summary)
        total = other_bytes + len(summary_bytes)
        if summary.get("evidence_file_bytes") == total:
            break
        summary["evidence_file_bytes"] = total
    else:
        raise AssertionError("EVIDENCE_BYTE_COUNT_DID_NOT_STABILIZE")
    atomic_write(SUMMARY, summary_bytes)
    # Both durable receipts now identify every original and compressed byte.
    # Delete only the exact new raw trace named by its verified old receipt.
    removed = 0
    for raw_path, original_receipt in raw_cleanup:
        if raw_path.exists():
            verify_receipt(raw_path.read_bytes(), original_receipt)
            raw_path.unlink()
            removed += 1
    actual_bytes = sum(path.stat().st_size for path in EVIDENCE.rglob("*") if path.is_file())
    assert actual_bytes == summary["evidence_file_bytes"], "FINAL_EVIDENCE_BYTE_COUNT_MISMATCH"
    print("Lossless traces:", packaging["trace_count"],
          "raw bytes:", packaging["original_trace_bytes"],
          "gzip bytes:", packaging["compressed_trace_bytes"],
          "raw files removed:", removed,
          "evidence bytes:", actual_bytes,
          "packaging seconds:", packaging["packaging_runtime_s"],
          "peak RSS KiB:", packaging["peak_RSS_KiB"], flush=True)


if __name__ == "__main__":
    main()
