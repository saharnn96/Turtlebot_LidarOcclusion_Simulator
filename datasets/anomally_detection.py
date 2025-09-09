#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Read LiDAR scans from a CSV (timestep,lidar_0,...,lidar_N) and detect occlusion vs out-of-range.
Outputs an annotated CSV with per-timestep occlusion classification and angle ranges.

Usage:
  python lidar_occlusion_from_csv.py \
      --input scans.csv \
      --output occlusions.csv \
      --history-size 10 \
      --min-segment-beams 5 \
      --gap-merge-beams 2 \
      --drift-tolerance-beams 3 \
      --persistence-threshold 0.7 \
      --min-occlusion-width-deg 5.0 \
      --angle-min-deg -180.0 \
      --angle-span-deg 360.0
"""

import csv
import math
import argparse
from collections import deque
from dataclasses import dataclass
from typing import List, Optional, Tuple

# ----------------- Occlusion Detector (from previous message, trimmed where possible) -----------------

@dataclass
class OcclusionSegment:
    start_idx: int
    end_idx:   int
    start_angle: float   # radians
    end_angle:   float   # radians
    width_deg:   float
    stability:   float   # 0..1

class LidarOcclusionDetector:
    def __init__(
        self,
        history_size: int = 10,
        min_segment_beams: int = 5,
        gap_merge_beams: int = 2,
        drift_tolerance_beams: int = 3,
        persistence_threshold: float = 0.7,
        min_occlusion_width_deg: float = 5.0,
        treat_ge_max_as_inf: bool = True,
    ):
        self.history_size = history_size
        self.min_segment_beams = min_segment_beams
        self.gap_merge_beams = gap_merge_beams
        self.drift_tol = drift_tolerance_beams
        self.persistence_threshold = persistence_threshold
        self.min_occlusion_width_deg = min_occlusion_width_deg
        self.treat_ge_max_as_inf = treat_ge_max_as_inf

        self._mask_hist: deque[List[bool]] = deque(maxlen=history_size)
        self._segs_hist: deque[List[Tuple[int, int]]] = deque(maxlen=history_size)

    def add_scan(
        self,
        ranges: List[float],
        angle_min: float,
        angle_increment: float,
        range_max: float = 1e9,  # not used if treat_ge_max_as_inf=False
        range_min: float = 0.0
    ) -> Tuple[bool, List[OcclusionSegment]]:
        n = len(ranges)
        if n == 0 or angle_increment == 0.0:
            return (False, [])

        # 1) Build INF mask
        mask = [False] * n
        for i, r in enumerate(ranges):
            is_inf = math.isinf(r) or math.isnan(r)
            if not is_inf and self.treat_ge_max_as_inf:
                if r >= (range_max - 1e-6):
                    is_inf = True
            mask[i] = is_inf

        # 2) Segments
        segs = self._find_segments(mask, min_len=self.min_segment_beams, gap=self.gap_merge_beams)

        # 3) Stability vs history
        stability_scores = [self._segment_stability(seg) for seg in segs]

        # 4) Convert to angle ranges and filter
        occluded_segments: List[OcclusionSegment] = []
        for (seg, stab) in zip(segs, stability_scores):
            s_idx, e_idx = seg
            start_angle = angle_min + s_idx * angle_increment
            end_angle   = angle_min + e_idx   * angle_increment
            width_deg   = abs(e_idx - s_idx + 1) * abs(angle_increment) * 180.0 / math.pi
            if width_deg >= self.min_occlusion_width_deg and stab >= self.persistence_threshold:
                occluded_segments.append(
                    OcclusionSegment(
                        start_idx=s_idx,
                        end_idx=e_idx,
                        start_angle=start_angle,
                        end_angle=end_angle,
                        width_deg=width_deg,
                        stability=stab
                    )
                )

        # 5) Update history AFTER classification
        self._mask_hist.append(mask)
        self._segs_hist.append(segs)

        return (len(occluded_segments) > 0, occluded_segments)

    # ---- internals ----
    def _find_segments(self, mask: List[bool], min_len: int, gap: int) -> List[Tuple[int, int]]:
        n = len(mask)
        raw: List[Tuple[int, int]] = []
        i = 0
        while i < n:
            if mask[i]:
                j = i
                while j + 1 < n and mask[j + 1]:
                    j += 1
                raw.append((i, j))
                i = j + 1
            else:
                i += 1
        if not raw:
            return []

        merged: List[Tuple[int, int]] = []
        cur_s, cur_e = raw[0]
        for s, e in raw[1:]:
            if s - cur_e - 1 <= gap:
                cur_e = e
            else:
                if (cur_e - cur_s + 1) >= min_len:
                    merged.append((cur_s, cur_e))
                cur_s, cur_e = s, e
        if (cur_e - cur_s + 1) >= min_len:
            merged.append((cur_s, cur_e))
        return merged

    def _segment_stability(self, seg: Tuple[int, int]) -> float:
        if not self._segs_hist:
            return 0.0
        s0, e0 = seg
        set0 = set(range(s0, e0 + 1))
        overlaps: List[float] = []
        for past_segs in self._segs_hist:
            best_j = 0.0
            for (sp, ep) in past_segs:
                setp = set(range(sp, ep + 1))
                inter = len(set0 & setp)
                union = len(set0 | setp)
                if union == 0:
                    continue
                j = inter / union
                lb_drift = abs(sp - s0)
                rb_drift = abs(ep - e0)
                if lb_drift > self.drift_tol or rb_drift > self.drift_tol:
                    j *= 0.5
                if j > best_j:
                    best_j = j
            overlaps.append(best_j)
        return sum(overlaps) / len(overlaps) if overlaps else 0.0

# ----------------- CSV I/O wrapper -----------------

def parse_float_maybe_inf(s: str) -> float:
    s = s.strip().lower()
    if s in ("inf", "+inf", "infinity", "+infinity"):
        return float("inf")
    if s in ("-inf", "-infinity"):
        return float("-inf")
    try:
        return float(s)
    except ValueError:
        # Treat unknown tokens as NaN -> considered INF in detector (blocked/invalid)
        return float("nan")

def format_ranges_deg(segs: List[OcclusionSegment]) -> str:
    """Format angle ranges in degrees like: '-30.0–-12.0; 145.0–170.0' """
    if not segs:
        return ""
    parts = []
    for s in segs:
        a0 = math.degrees(s.start_angle)
        a1 = math.degrees(s.end_angle)
        # ensure consistent ordering left->right
        a_lo, a_hi = (a0, a1) if a0 <= a1 else (a1, a0)
        parts.append(f"{a_lo:.1f}–{a_hi:.1f}")
    return "; ".join(parts)

def detect_lidar_columns(header: List[str]) -> List[Tuple[int, str]]:
    cols = []
    for i, name in enumerate(header):
        if name.startswith("lidar_"):
            cols.append((i, name))
    # sort by numeric suffix just in case header order is shuffled
    def idx_of(name: str) -> int:
        try:
            return int(name.split("_", 1)[1])
        except Exception:
            return 10**9
    cols.sort(key=lambda t: idx_of(t[1]))
    return cols

def run(
    input_csv: str,
    output_csv: str,
    history_size: int = 10,
    min_segment_beams: int = 5,
    gap_merge_beams: int = 2,
    drift_tolerance_beams: int = 3,
    persistence_threshold: float = 0.7,
    min_occlusion_width_deg: float = 5.0,
    angle_min_deg: float = -180.0,
    angle_span_deg: float = 360.0,
    treat_ge_max_as_inf: bool = False  # for CSV, we usually rely on explicit 'inf'
):
    detector = LidarOcclusionDetector(
        history_size=history_size,
        min_segment_beams=min_segment_beams,
        gap_merge_beams=gap_merge_beams,
        drift_tolerance_beams=drift_tolerance_beams,
        persistence_threshold=persistence_threshold,
        min_occlusion_width_deg=min_occlusion_width_deg,
        treat_ge_max_as_inf=treat_ge_max_as_inf
    )

    with open(input_csv, "r", newline="") as f_in, open(output_csv, "w", newline="") as f_out:
        reader = csv.reader(f_in)
        header = next(reader)
        lidar_cols = detect_lidar_columns(header)
        if not lidar_cols:
            raise ValueError("No lidar_* columns found in CSV header.")

        # Resolve angle mapping
        n_beams = len(lidar_cols)
        angle_min = math.radians(angle_min_deg)
        angle_span = math.radians(angle_span_deg)
        angle_inc = angle_span / max(n_beams, 1)

        writer = csv.writer(f_out)
        out_header = [
            "timestep",
            "is_occluded",
            "num_segments",
            "occluded_ranges_deg",  # formatted "a0–a1; a2–a3"
            "segments_start_idx_end_idx",  # e.g., "12-33; 210-245"
            "segments_stability"          # e.g., "0.83; 0.77"
        ]
        writer.writerow(out_header)

        for row in reader:
            # timestep (string is OK, often an int/float)
            print(f"Row length={len(row)} expected={len(header)}")
            timestep = row[0]

            # build ranges array in lidar_0..lidar_N order
            ranges = [parse_float_maybe_inf(row[i]) for (i, _) in lidar_cols]

            is_occ, segs = detector.add_scan(
                ranges=ranges,
                angle_min=angle_min,
                angle_increment=angle_inc,
                range_max=1e9,
                range_min=0.0,
            )

            ranges_str = format_ranges_deg(segs)
            idx_str = "; ".join(f"{s.start_idx}-{s.end_idx}" for s in segs) if segs else ""
            stab_str = "; ".join(f"{s.stability:.2f}" for s in segs) if segs else ""

            writer.writerow([timestep, int(is_occ), len(segs), ranges_str, idx_str, stab_str])

# ----------------- CLI -----------------

def main():
    p = argparse.ArgumentParser(description="LiDAR occlusion detection from CSV.")
    p.add_argument("--input", required=True, help="Input CSV path with timestep,lidar_* columns.")
    p.add_argument("--output", required=True, help="Output CSV path for occlusion annotations.")
    p.add_argument("--history-size", type=int, default=30)
    p.add_argument("--min-segment-beams", type=int, default=5)
    p.add_argument("--gap-merge-beams", type=int, default=2)
    p.add_argument("--drift-tolerance-beams", type=int, default=3)
    p.add_argument("--persistence-threshold", type=float, default=0.7)
    p.add_argument("--min-occlusion-width-deg", type=float, default=5.0)
    p.add_argument("--angle-min-deg", type=float, default=-180.0,
                   help="Angle of lidar_0 in degrees (scanner frame).")
    p.add_argument("--angle-span-deg", type=float, default=360.0,
                   help="Total angular span covered by the lidar_* columns in degrees.")
    p.add_argument("--treat-ge-max-as-inf", action="store_true",
                   help="Also treat values >= range_max as INF (usually unnecessary for CSV).")
    args = p.parse_args()

    run(
        input_csv=args.input,
        output_csv=args.output,
        history_size=args.history_size,
        min_segment_beams=args.min_segment_beams,
        gap_merge_beams=args.gap_merge_beams,
        drift_tolerance_beams=args.drift_tolerance_beams,
        persistence_threshold=args.persistence_threshold,
        min_occlusion_width_deg=args.min_occlusion_width_deg,
        angle_min_deg=args.angle_min_deg,
        angle_span_deg=args.angle_span_deg,
        treat_ge_max_as_inf=args.treat_ge_max_as_inf,
    )

if __name__ == "__main__":
    main()
