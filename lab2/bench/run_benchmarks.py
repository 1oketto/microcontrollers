#!/usr/bin/env python3
"""Import saved benchmark serial logs and draw SVG charts.

This tool only processes existing measurements; it does not build or calibrate
firmware and never invents missing results.
"""
import argparse
import csv
import html
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
HERE = Path(__file__).resolve().parent
VARIANTS = {
    "full": "All optimizations",
    "full_scan": "Without spatial filtering",
    "single_core": "Without second worker core",
    "pixel_stamps": "Without packed circle stamps",
    "reference_normal": "Without fast root / normal",
    "no_draw_cache": "Without duplicate draw cache",
    "no_text_cache": "Without text format cache",
}
TRIAL_FIELDS = ["variant", "clock_hz", "voltage_mv", "balls", "warmup_frames",
                "frames", "max_us", "mean_us", "misses", "passed"]


def write_csv(path, fields, rows):
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fields)
        writer.writeheader()
        writer.writerows(rows)


def collect(args):
    trials, capacities = [], []
    for path in args.logs:
        for raw in Path(path).read_text(errors="replace").splitlines():
            # Other boot output may precede a record on the same serial line.
            if "BENCH," in raw:
                values = raw[raw.index("BENCH,") + 6:].split(",")
                if len(values) != len(TRIAL_FIELDS) or values[0] not in VARIANTS:
                    continue
                try:
                    numbers = [int(value) for value in values[1:]]
                except ValueError:
                    continue
                row = dict(zip(TRIAL_FIELDS, [values[0], *numbers]))
                if row["balls"] < 0 or row["frames"] <= 0 or row["passed"] not in (0, 1):
                    continue
                trials.append(row)
            if "CAPACITY," in raw:
                values = raw[raw.index("CAPACITY,") + 9:].split(",")
                if len(values) == 4 and values[0] in VARIANTS:
                    try:
                        balls, ceiling, censored = map(int, values[1:])
                    except ValueError:
                        continue
                    if 0 <= balls <= ceiling and censored in (0, 1):
                        capacities.append({"variant": values[0], "balls": balls,
                                           "pool_limit": ceiling, "at_pool_limit": censored,
                                           "source_log": str(path)})
    if not trials and not capacities:
        raise SystemExit("No valid BENCH or CAPACITY records found; no results written.")
    write_csv(HERE / "trials.csv", TRIAL_FIELDS, trials)
    write_csv(HERE / "capacities.csv",
              ["variant", "balls", "pool_limit", "at_pool_limit", "source_log"], capacities)
    charts()
    print(f"Imported {len(trials)} trials and {len(capacities)} capacity results.")


def plot_svg(path, title, subtitle, ylabel, labels, values, notes, color):
    # A standalone vector chart: missing values are text, never zero-height bars.
    width, height = 1400, 760
    left, top, right, bottom = 120, 115, 1360, 510
    maximum = max([v for v in values if v is not None] + [1])
    maximum = max(1000, ((maximum + 4999) // 5000) * 5000)
    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
           '<rect width="100%" height="100%" fill="white"/>',
           '<style>text{font-family:Arial,sans-serif;fill:#172a3a}.tick{font-size:15px}.label{font-size:16px}</style>',
           f'<text x="{left}" y="40" font-size="26" font-weight="bold">{html.escape(title)}</text>',
           f'<text x="{left}" y="72" font-size="17">{html.escape(subtitle)}</text>']
    for tick in range(6):
        val = maximum * tick / 5
        y = bottom - (bottom - top) * tick / 5
        out.append(f'<path d="M{left},{y} H{right}" stroke="#dce3e8"/>')
        out.append(f'<text x="{left-16}" y="{y+5}" text-anchor="end" class="tick">{val:,.0f}</text>')
    step = (right - left) / len(labels)
    for i, (label, value, note) in enumerate(zip(labels, values, notes)):
        x = left + step * (i + .5)
        if value is None:
            out.append(f'<text x="{x}" y="{bottom-18}" text-anchor="middle" class="tick" fill="#667788">pending</text>')
        else:
            h = (bottom-top) * value / maximum
            out.append(f'<rect x="{x-step*.32}" y="{bottom-h}" width="{step*.64}" height="{h}" rx="4" fill="{color}"/>')
            out.append(f'<text x="{x}" y="{bottom-h-12}" text-anchor="middle" font-size="18">{value:,}{html.escape(note)}</text>')
        words, lines, line = label.split(), [], ""
        for word in words:
            if len(line + " " + word) > max(14, int(step/8)) and line:
                lines.append(line)
                line = word
            else:
                line = (line + " " + word).strip()
        lines.append(line)
        for j, line in enumerate(lines):
            out.append(f'<text x="{x}" y="{bottom+30+j*21}" text-anchor="middle" class="label">{html.escape(line)}</text>')
    out.append(f'<text x="35" y="{(top+bottom)/2}" transform="rotate(-90 35 {(top+bottom)/2})" text-anchor="middle" font-size="18">{html.escape(ylabel)}</text>')
    out.append('<text x="740" y="665" text-anchor="middle" font-size="18">Optimization / comparison build</text>')
    out.append(f'<text x="{left}" y="718" font-size="15">{html.escape("+ indicates a lower bound: configured ball pool limit was reached." if path.name == "ball_capacity.svg" else "Storage equivalents only. The three bars use different baselines and must not be added.")}</text>')
    out.append('</svg>')
    path.write_text("\n".join(out) + "\n")


def charts():
    measured = {}
    capacity_path = HERE / "capacities.csv"
    if capacity_path.exists():
        with capacity_path.open() as stream:
            for row in csv.DictReader(stream):
                # Across repeated boots use the smallest observed passing capacity.
                if row["variant"] not in measured or int(row["balls"]) < int(measured[row["variant"]]["balls"]):
                    measured[row["variant"]] = row
    values = [int(measured[v]["balls"]) if v in measured else None for v in VARIANTS]
    notes = ["+" if v in measured and int(measured[v]["at_pool_limit"]) else "" for v in VARIANTS]
    plot_svg(HERE / "ball_capacity.svg", "On-board ball capacity by optimization comparison",
             "No board measurements yet; pending is unknown, not zero." if not measured else
             "Minimum observed passing capacity across imported boots; same board and workload required.",
             "Balls passing the 16,666 us frame budget", list(VARIANTS.values()), values, notes, "#147d92")
    plot_svg(HERE / "memory_capacity.svg", "Memory savings expressed as ball storage capacity",
             "Calculated from framebuffer dimensions and record sizes; these are not frame-rate measurements.",
             "Additional storable ball records (analytical)",
             ["1-bit buffers vs 4-bit buffers", "Flash background vs RAM background",
              "10-byte records vs original 20-byte records (200 KB pool)"],
             [23040, 3840, 10000], ["", "", ""], "#526ec4")
    rows = []
    final = measured.get("full")
    for variant, label in VARIANTS.items():
        result = measured.get(variant)
        delta = ""
        if variant != "full" and final and result and not int(final["at_pool_limit"]) and not int(result["at_pool_limit"]):
            delta = int(final["balls"]) - int(result["balls"])
        rows.append({"variant": variant, "comparison": label,
                     "measured_balls": result["balls"] if result else "",
                     "additional_balls_with_optimization": delta,
                     "status": "measured lower bound" if result and int(result["at_pool_limit"]) else "measured" if result else "not measured"})
    write_csv(HERE / "optimization_results.csv",
              ["variant", "comparison", "measured_balls", "additional_balls_with_optimization", "status"], rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    cmd = commands.add_parser("collect")
    cmd.add_argument("logs", nargs="+")
    cmd.set_defaults(func=collect)
    cmd = commands.add_parser("charts")
    cmd.set_defaults(func=lambda args: charts())
    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
