"""Render uas2cot output (one CoT event per line) as an animated GIF map.

usage: python tools/cot_gif.py <events.cot> <out.gif> [--title TEXT]

Plots each event uid's latest position per second of event time, with a
short trail, coloured by CoT affiliation (f friend, a assumed friend,
n neutral, u unknown or pending, s suspect, h hostile). Needs matplotlib
and Pillow.
"""
import math
import sys
import xml.etree.ElementTree as ET
from collections import OrderedDict

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.animation import FuncAnimation, PillowWriter  # noqa: E402

COLORS = {"f": "#1f6fd1", "a": "#5b9be6", "n": "#2f9e44", "u": "#c9a400", "p": "#c9a400",
          "s": "#e8590c", "h": "#d6336c"}


def parse(path):
    frames = OrderedDict()
    for line in open(path, encoding="utf-8"):
        line = line.strip()
        if not line:
            continue
        ev = ET.fromstring(line.split("?>", 1)[1])
        t = ev.get("time")[:19]
        pt = ev.find("point")
        det = ev.find("detail")
        contact = det.find("contact") if det is not None else None
        frames.setdefault(t, {})[ev.get("uid")] = (
            float(pt.get("lat")), float(pt.get("lon")), ev.get("type"),
            contact.get("callsign") if contact is not None else ev.get("uid"))
    return frames


def main():
    args = sys.argv[1:]
    if len(args) < 2:
        print(__doc__)
        sys.exit(2)
    title = "uas2cot: MAVLink + MAVLink-M to CoT"
    if "--title" in args:
        title = args[args.index("--title") + 1]
    frames = parse(args[0])
    times = list(frames)
    state, trails = {}, {}
    snapshots = []
    for t in times:
        for uid, v in frames[t].items():
            state[uid] = v
            trails.setdefault(uid, []).append(v[:2])
        snapshots.append((t, dict(state), {u: list(p[-40:]) for u, p in trails.items()}))
    lats = [v[0] for f in frames.values() for v in f.values()]
    lons = [v[1] for f in frames.values() for v in f.values()]
    pad_lat = (max(lats) - min(lats)) * 0.12 + 1e-4
    pad_lon = (max(lons) - min(lons)) * 0.12 + 1e-4

    fig, ax = plt.subplots(figsize=(7.2, 6.0), dpi=90)

    def draw(i):
        t, st, tr = snapshots[i]
        ax.clear()
        ax.set_xlim(min(lons) - pad_lon, max(lons) + pad_lon)
        ax.set_ylim(min(lats) - pad_lat, max(lats) + pad_lat)
        ax.set_aspect(1 / math.cos(math.radians(sum(lats) / len(lats))))
        ax.set_facecolor("#f4f4ef")
        ax.grid(color="#dddddd", linewidth=0.6)
        ax.set_xlabel("longitude")
        ax.set_ylabel("latitude")
        ax.set_title(f"{title}\nevent time {t}Z (simulated data)", fontsize=10)
        for uid, (lat, lon, kind, cs) in st.items():
            aff = kind.split("-")[1] if kind.count("-") >= 2 else "u"
            c = COLORS.get(aff, "#555555")
            path = tr.get(uid, [])
            if len(path) > 1:
                ax.plot([p[1] for p in path], [p[0] for p in path], color=c, alpha=0.35, linewidth=1.5)
            marker = "s" if "-G" in kind else "^"
            ax.scatter([lon], [lat], color=c, marker=marker, s=70, edgecolors="black", linewidths=0.6, zorder=3)
            ax.annotate(f"{cs}\n{kind}", (lon, lat), textcoords="offset points", xytext=(7, 5), fontsize=7)
        return []

    step = max(1, len(snapshots) // 90)
    idx = list(range(0, len(snapshots), step))
    anim = FuncAnimation(fig, lambda k: draw(idx[k]), frames=len(idx), blit=False)
    anim.save(args[1], writer=PillowWriter(fps=8))
    print(f"{args[1]}: {len(idx)} frames from {len(times)} seconds of events")


if __name__ == "__main__":
    main()
