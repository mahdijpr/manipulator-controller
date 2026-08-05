"""Analyze IMU diagnostic CSV data captured from the PlatformIO serial monitor."""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

import numpy as np


CSV_HEADER = [
    "timestamp_us", "dt_s", "raw_ax", "raw_ay", "raw_az", "raw_gx", "raw_gy",
    "raw_gz", "roll_deg", "pitch_deg", "gx_dps", "gy_dps", "gz_dps",
    "calibrated", "valid",
]
NUMERIC_COLUMNS = CSV_HEADER[:13]
MEASUREMENT_COLUMNS = [
    "raw_ax", "raw_ay", "raw_az", "raw_gx", "raw_gy", "raw_gz", "roll_deg",
    "pitch_deg", "gx_dps", "gy_dps", "gz_dps",
]


def parse_flag(value: str) -> int:
    """Parse firmware flags emitted as either numbers or common Boolean spellings."""
    normalized = value.strip().lower()
    if normalized in {"true", "yes", "on"}:
        return 1
    if normalized in {"false", "no", "off"}:
        return 0
    numeric = float(normalized)
    if not math.isfinite(numeric):
        raise ValueError("flag is not finite")
    return int(numeric != 0.0)


def looks_like_data_row(row: list[str]) -> bool:
    """Avoid treating PlatformIO/ESP32 status messages as rejected CSV samples."""
    if len(row) == len(CSV_HEADER):
        return True
    try:
        float(row[0].strip())
        return True
    except (IndexError, ValueError):
        return False


def parse_log(input_path: Path) -> tuple[dict[str, np.ndarray], int, int]:
    """Find the diagnostic header and parse complete, convertible samples after it."""
    accepted: list[list[float]] = []
    rejected = 0
    header_found = False
    with input_path.open("rb") as input_file:
        byte_order_mark = input_file.read(4)
    encoding = "utf-16" if byte_order_mark.startswith((b"\xff\xfe", b"\xfe\xff")) else "utf-8-sig"

    with input_path.open("r", encoding=encoding, errors="replace", newline="") as log_file:
        for row in csv.reader(log_file):
            if not row:
                continue
            # Some capture tools quote each complete serial line, leaving the CSV
            # reader with one field that itself contains the diagnostic record.
            if len(row) == 1 and "," in row[0]:
                row = next(csv.reader([row[0]]))
            if row == CSV_HEADER:
                header_found = True
                continue
            if not header_found or not looks_like_data_row(row):
                continue
            if len(row) != len(CSV_HEADER):
                rejected += 1
                continue
            try:
                numbers = [float(value.strip()) for value in row[:13]]
                # A non-finite dt_s is retained so timing analysis can report it as
                # excluded.  All timestamp and sensor values must be finite.
                if not math.isfinite(numbers[0]) or not all(math.isfinite(value) for value in numbers[2:]):
                    raise ValueError("non-finite timestamp or sensor value")
                numbers.extend((parse_flag(row[13]), parse_flag(row[14])))
            except ValueError:
                rejected += 1
                continue
            accepted.append(numbers)

    if not header_found:
        raise ValueError(f"CSV header was not found: {','.join(CSV_HEADER)}")

    if not accepted:
        return {name: np.array([], dtype=float) for name in CSV_HEADER}, 0, rejected

    matrix = np.asarray(accepted, dtype=float)
    return {name: matrix[:, index] for index, name in enumerate(CSV_HEADER)}, len(accepted), rejected


def descriptive_stats(values: np.ndarray) -> dict[str, float] | None:
    if len(values) == 0:
        return None
    return {
        "mean": float(np.mean(values)),
        "std": float(np.std(values)),
        "min": float(np.min(values)),
        "max": float(np.max(values)),
        "peak_to_peak": float(np.ptp(values)),
        "rms": float(np.sqrt(np.mean(np.square(values)))),
    }


def timing_stats(data: dict[str, np.ndarray]) -> dict[str, float | int | None]:
    timestamps = data["timestamp_us"]
    dt_values = data["dt_s"]
    count = len(timestamps)
    timing_dt = dt_values[np.isfinite(dt_values) & (dt_values > 0)]
    result: dict[str, float | int | None] = {
        "count": count,
        "dt_used": len(timing_dt),
        "dt_excluded": count - len(timing_dt),
    }
    if count == 0:
        return result

    result.update({
        "rate_from_mean_dt": 1.0 / float(np.mean(timing_dt)) if len(timing_dt) else None,
        "dt_mean": float(np.mean(timing_dt)) if len(timing_dt) else None,
        "dt_median": float(np.median(timing_dt)) if len(timing_dt) else None,
        "dt_std": float(np.std(timing_dt)) if len(timing_dt) else None,
        "dt_min": float(np.min(timing_dt)) if len(timing_dt) else None,
        "dt_max": float(np.max(timing_dt)) if len(timing_dt) else None,
        "dt_p95": float(np.percentile(timing_dt, 95)) if len(timing_dt) else None,
        "dt_p99": float(np.percentile(timing_dt, 99)) if len(timing_dt) else None,
    })

    intervals = np.diff(timestamps) / 1e6 if count > 1 else np.array([], dtype=float)
    positive_intervals = intervals[intervals > 0]
    result["non_increasing"] = int(np.count_nonzero(intervals <= 0))
    duration = float(np.sum(positive_intervals))
    result["duration"] = duration
    result["rate_from_timestamps"] = len(positive_intervals) / duration if duration > 0 else None
    result["jitter"] = float(np.std(positive_intervals)) if len(positive_intervals) else None
    median_interval = float(np.median(positive_intervals)) if len(positive_intervals) else None
    if median_interval is None or median_interval <= 0:
        result.update({"gap_events": 0, "missing_samples": 0, "timestamp_median_dt": None})
        return result

    gaps = positive_intervals[positive_intervals > 1.5 * median_interval]
    result["timestamp_median_dt"] = median_interval
    result["gap_events"] = int(len(gaps))
    result["missing_samples"] = int(sum(max(1, round(gap / median_interval) - 1) for gap in gaps))
    return result


def linear_drift(time_s: np.ndarray, values: np.ndarray) -> float | None:
    if len(time_s) < 2 or np.ptp(time_s) <= 0:
        return None
    return float(np.polyfit(time_s, values, 1)[0])


def fmt(value: float | int | None, digits: int = 6) -> str:
    return "n/a" if value is None else f"{value:.{digits}f}" if isinstance(value, float) else str(value)


def append_timing_report(lines: list[str], data: dict[str, np.ndarray], title: str) -> None:
    timing = timing_stats(data)
    lines.extend([f"\n{title}", "-" * len(title), f"Sample count: {timing['count']}"])
    if timing["count"] == 0:
        return
    lines.extend([
        f"Recording duration: {fmt(timing['duration'])} s",
        f"Actual sampling rate (timestamps): {fmt(timing['rate_from_timestamps'], 3)} Hz",
        f"dt_s values used / excluded: {timing['dt_used']} / {timing['dt_excluded']} (excluded non-positive or non-finite)",
        f"Sampling rate (mean dt_s): {fmt(timing['rate_from_mean_dt'], 3)} Hz",
        f"dt_s mean / median / std: {fmt(timing['dt_mean'])} / {fmt(timing['dt_median'])} / {fmt(timing['dt_std'])} s",
        f"dt_s min / max: {fmt(timing['dt_min'])} / {fmt(timing['dt_max'])} s",
        f"dt_s 95th / 99th percentile: {fmt(timing['dt_p95'])} / {fmt(timing['dt_p99'])} s",
        f"Sampling jitter (timestamp interval std): {fmt(timing['jitter'])} s",
        f"Duplicate or non-increasing timestamps: {timing['non_increasing']}",
        f"Suspected timing gaps (> 1.5 x median timestamp interval): {timing['gap_events']}",
        f"Estimated missing samples: {timing['missing_samples']}",
    ])


def append_measurement_report(lines: list[str], data: dict[str, np.ndarray], title: str) -> None:
    lines.extend([f"\n{title}", "-" * len(title)])
    for name in MEASUREMENT_COLUMNS:
        values = descriptive_stats(data[name])
        if values is None:
            lines.append(f"{name}: n/a")
            continue
        lines.append(
            f"{name}: mean={values['mean']:.6f}, std={values['std']:.6f}, "
            f"min={values['min']:.6f}, max={values['max']:.6f}, "
            f"p-p={values['peak_to_peak']:.6f}, rms={values['rms']:.6f}"
        )


def append_stability_report(lines: list[str], calibrated: dict[str, np.ndarray]) -> None:
    lines.extend(["\nCalibrated stability analysis", "-----------------------------"])
    if len(calibrated["timestamp_us"]) == 0:
        lines.append("ERROR: No calibrated rows are available; stability analysis was skipped.")
        return

    for angle in ("roll_deg", "pitch_deg"):
        values = descriptive_stats(calibrated[angle])
        assert values is not None
        lines.append(
            f"{angle}: bias={values['mean']:.6f} deg, std={values['std']:.6f} deg, "
            f"p-p={values['peak_to_peak']:.6f} deg"
        )
    for axis in ("gx_dps", "gy_dps", "gz_dps"):
        values = descriptive_stats(calibrated[axis])
        assert values is not None
        lines.append(f"{axis}: mean={values['mean']:.6f} dps, std={values['std']:.6f} dps")

    time_s = (calibrated["timestamp_us"] - calibrated["timestamp_us"][0]) / 1e6
    for angle in ("roll_deg", "pitch_deg"):
        drift = linear_drift(time_s, calibrated[angle])
        if drift is None:
            lines.append(f"{angle} drift: n/a (need at least two increasing timestamps)")
        else:
            lines.append(f"{angle} drift: {drift:.8f} deg/s ({drift * 60:.6f} deg/min)")


def subset(data: dict[str, np.ndarray], mask: np.ndarray) -> dict[str, np.ndarray]:
    return {name: values[mask] for name, values in data.items()}


def append_calibration_transition_report(
    lines: list[str], valid: dict[str, np.ndarray], warmup_samples: int,
    calibration_samples: int,
) -> None:
    expected_first_calibrated = warmup_samples + calibration_samples
    uncalibrated_count = int(np.count_nonzero(valid["calibrated"] == 0))
    calibrated_indices = np.flatnonzero(valid["calibrated"] == 1)
    lines.extend([
        "\nStartup calibration transition",
        "------------------------------",
        f"Uncalibrated valid rows: {uncalibrated_count}",
        f"Expected warm-up successful samples: {warmup_samples}",
        f"Expected calibration successful samples: {calibration_samples}",
        f"Expected first calibrated row (1-based valid CSV row): {expected_first_calibrated}",
    ])
    if len(calibrated_indices) == 0:
        lines.append("Actual first calibrated row: not present")
        return

    actual_first_calibrated = int(calibrated_indices[0]) + 1
    lines.extend([
        f"Actual first calibrated row (1-based valid CSV row): {actual_first_calibrated}",
        "Transition matches configured successful-sample counts: "
        f"{'yes' if actual_first_calibrated == expected_first_calibrated else 'no'}",
    ])


def build_report(
    data: dict[str, np.ndarray], accepted: int, rejected: int,
    warmup_samples: int, calibration_samples: int,
) -> str:
    valid = subset(data, data["valid"] == 1)
    pre_calibration = subset(valid, valid["calibrated"] == 0)
    post_calibration = subset(valid, valid["calibrated"] == 1)
    lines = [
        "==============================",
        " IMU DIAGNOSTICS ANALYSIS",
        "==============================",
        f"Accepted data rows: {accepted}",
        f"Rejected data rows: {rejected}",
        f"Valid rows used for main analysis: {len(valid['timestamp_us'])}",
        f"Invalid accepted rows excluded: {int(np.count_nonzero(data['valid'] == 0))}",
    ]
    append_timing_report(lines, valid, "Timing analysis (valid data)")
    append_measurement_report(lines, valid, "Descriptive statistics (valid data)")
    append_calibration_transition_report(
        lines, valid, warmup_samples, calibration_samples
    )
    if len(pre_calibration["timestamp_us"]) and len(post_calibration["timestamp_us"]):
        append_timing_report(lines, pre_calibration, "Pre-calibration results (valid data)")
        append_measurement_report(lines, pre_calibration, "Pre-calibration descriptive statistics")
        append_timing_report(lines, post_calibration, "Post-calibration results (valid data)")
        append_measurement_report(lines, post_calibration, "Post-calibration descriptive statistics")
    append_stability_report(lines, post_calibration)
    return "\n".join(lines) + "\n"


def make_plots(valid: dict[str, np.ndarray], output_dir: Path, save: bool, show: bool) -> None:
    if len(valid["timestamp_us"]) == 0:
        return
    import matplotlib.pyplot as plt

    time_s = (valid["timestamp_us"] - valid["timestamp_us"][0]) / 1e6
    pre_exists = bool(np.any(valid["calibrated"] == 0))
    post_exists = bool(np.any(valid["calibrated"] == 1))
    transition = time_s[np.flatnonzero(valid["calibrated"] == 1)[0]] if pre_exists and post_exists else None

    def finish(fig, filename: str) -> None:
        fig.tight_layout()
        if save:
            fig.savefig(output_dir / filename, dpi=150)

    def plot_series(columns: tuple[str, ...], title: str, ylabel: str, filename: str, transition_line: bool = False) -> None:
        fig, axis = plt.subplots(figsize=(10, 5))
        for name in columns:
            axis.plot(time_s, valid[name], label=name)
        if transition_line and transition is not None:
            axis.axvline(transition, color="black", linestyle="--", label="calibration transition")
        axis.set(title=title, xlabel="Time (s)", ylabel=ylabel)
        axis.grid(True, alpha=0.3)
        axis.legend()
        finish(fig, filename)

    plot_series(("roll_deg", "pitch_deg"), "Roll and pitch", "Degrees", "angles.png", True)
    plot_series(("gx_dps", "gy_dps", "gz_dps"), "Calibrated gyroscope", "Degrees/s", "gyro_dps.png", True)
    plot_series(("raw_ax", "raw_ay", "raw_az"), "Raw accelerometer", "Raw counts", "raw_accel.png")
    plot_series(("raw_gx", "raw_gy", "raw_gz"), "Raw gyroscope", "Raw counts", "raw_gyro.png")

    timing_dt_mask = np.isfinite(valid["dt_s"]) & (valid["dt_s"] > 0)
    timing_time = time_s[timing_dt_mask]
    timing_dt = valid["dt_s"][timing_dt_mask]

    fig, axis = plt.subplots(figsize=(10, 5))
    axis.plot(timing_time, timing_dt, label="dt_s")
    if len(timing_dt):
        axis.axhline(np.median(timing_dt), color="black", linestyle="--", label="median dt_s")
    axis.set(title="Sample interval", xlabel="Time (s)", ylabel="dt_s (s)")
    axis.grid(True, alpha=0.3)
    if len(timing_dt):
        axis.legend()
    else:
        axis.text(0.5, 0.5, "No positive finite dt_s values", ha="center", va="center", transform=axis.transAxes)
    finish(fig, "dt_s.png")

    fig, axis = plt.subplots(figsize=(8, 5))
    if len(timing_dt):
        axis.hist(timing_dt, bins="auto", edgecolor="black")
    else:
        axis.text(0.5, 0.5, "No positive finite dt_s values", ha="center", va="center", transform=axis.transAxes)
    axis.set(title="Sample interval distribution", xlabel="dt_s (s)", ylabel="Count")
    axis.grid(True, axis="y", alpha=0.3)
    finish(fig, "dt_s_histogram.png")
    if show:
        plt.show()
    else:
        plt.close("all")


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_path", type=Path, help="PlatformIO serial-monitor text or CSV log")
    parser.add_argument("--save-plots", action="store_true", help="save plots in imu_analysis")
    parser.add_argument("--show", action="store_true", help="display plots interactively")
    parser.add_argument(
        "--warmup-samples", type=int, default=50,
        help="successful warm-up samples configured in firmware (default: 50)",
    )
    parser.add_argument(
        "--calibration-samples", type=int, default=500,
        help="successful calibration samples configured in firmware (default: 500)",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_arguments()
    if not args.input_path.is_file():
        print(f"ERROR: Input file does not exist or is not a file: {args.input_path}", file=sys.stderr)
        return 1
    if args.warmup_samples < 0 or args.calibration_samples <= 0:
        print("ERROR: warm-up samples must be non-negative and calibration samples positive.", file=sys.stderr)
        return 1
    try:
        data, accepted, rejected = parse_log(args.input_path)
    except (OSError, ValueError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1
    valid_count = int(np.count_nonzero(data["valid"] == 1))
    if valid_count == 0:
        print("ERROR: No valid rows were found after parsing the log.", file=sys.stderr)
        return 1

    output_dir = args.input_path.parent / "imu_analysis"
    output_dir.mkdir(exist_ok=True)
    report = build_report(
        data, accepted, rejected, args.warmup_samples, args.calibration_samples
    )
    print(report, end="")
    (output_dir / "report.txt").write_text(report, encoding="utf-8")

    if args.save_plots or args.show:
        make_plots(subset(data, data["valid"] == 1), output_dir, args.save_plots, args.show)
        if args.save_plots:
            print(f"Plots saved in: {output_dir}")
    else:
        print(f"Report saved to: {output_dir / 'report.txt'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
