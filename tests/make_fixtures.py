#!/usr/bin/env python3
"""Generate point cloud fixtures for the loader tests.

The same logical points are written in several encodings so the tests can
assert that every decode path produces identical results. binary_compressed
is produced by PCL's own converter, which makes it a genuine
interoperability check rather than a test against our own writer.
"""
import math
import os
import struct
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "fixtures")

# Deterministic pseudo-random so fixtures are reproducible without numpy.
def lcg(seed):
    state = seed
    while True:
        state = (state * 6364136223846793005 + 1442695040888963407) & ((1 << 64) - 1)
        yield (state >> 11) / float(1 << 53)


def make_points(n, with_nan=False):
    """A helix with per-point attributes, plus optional NaN holes."""
    rng = lcg(12345)
    pts = []
    for i in range(n):
        t = i / n * 8.0 * math.pi
        jitter = next(rng) * 0.05
        x = 10.0 * math.cos(t) + jitter
        y = 10.0 * math.sin(t) + jitter
        z = i / n * 20.0 - 10.0
        if with_nan and i % 97 == 0:
            x = float("nan")
        intensity = int(next(rng) * 65535)        # uint16
        ring = i % 64                              # uint8
        label = (i // 128) % 1000                  # uint32
        stamp = 1700000000.0 + i * 1e-4            # float64, absolute unix time
        pts.append((x, y, z, intensity, ring, label, stamp))
    return pts


FIELDS = [
    ("x", "F", 4), ("y", "F", 4), ("z", "F", 4),
    ("intensity", "U", 2), ("ring", "U", 1), ("label", "U", 4), ("t", "F", 8),
]
PACK = "<fffHBId"   # must mirror FIELDS exactly


def header(n, data_kind):
    names = " ".join(f[0] for f in FIELDS)
    sizes = " ".join(str(f[2]) for f in FIELDS)
    types = " ".join(f[1] for f in FIELDS)
    counts = " ".join("1" for _ in FIELDS)
    return (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        f"FIELDS {names}\n"
        f"SIZE {sizes}\n"
        f"TYPE {types}\n"
        f"COUNT {counts}\n"
        f"WIDTH {n}\nHEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        f"DATA {data_kind}\n"
    )


def write_ascii(path, pts):
    with open(path, "w") as f:
        f.write(header(len(pts), "ascii"))
        for x, y, z, inten, ring, label, stamp in pts:
            xs = "nan" if math.isnan(x) else f"{x:.6f}"
            f.write(f"{xs} {y:.6f} {z:.6f} {inten} {ring} {label} {stamp:.6f}\n")


def write_binary(path, pts):
    with open(path, "wb") as f:
        f.write(header(len(pts), "binary").encode())
        for p in pts:
            f.write(struct.pack(PACK, *p))


def write_xyz_rgb_binary(path, pts):
    """Separate fixture exercising PCL's packed-RGB-inside-a-float quirk."""
    hdr = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n"
        f"WIDTH {len(pts)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(pts)}\nDATA binary\n"
    )
    with open(path, "wb") as f:
        f.write(hdr.encode())
        for i, (x, y, z, *_rest) in enumerate(pts):
            r, g, b = (i * 7) % 256, (i * 13) % 256, (i * 29) % 256
            packed = (r << 16) | (g << 8) | b
            # PCL stores the colour word in a float field, bits verbatim.
            as_float = struct.unpack("<f", struct.pack("<I", packed))[0]
            f.write(struct.pack("<ffff", x, y, z, as_float))


def write_kitti_bin(path, pts):
    with open(path, "wb") as f:
        for x, y, z, inten, *_rest in pts:
            f.write(struct.pack("<ffff", x, y, z, inten / 65535.0))


def main():
    os.makedirs(OUT, exist_ok=True)
    n = 5000
    pts = make_points(n)
    nan_pts = make_points(n, with_nan=True)

    write_ascii(os.path.join(OUT, "helix_ascii.pcd"), pts)
    write_binary(os.path.join(OUT, "helix_binary.pcd"), pts)
    write_ascii(os.path.join(OUT, "helix_nan_ascii.pcd"), nan_pts)
    write_binary(os.path.join(OUT, "helix_nan_binary.pcd"), nan_pts)
    write_xyz_rgb_binary(os.path.join(OUT, "helix_rgb.pcd"), pts)
    write_kitti_bin(os.path.join(OUT, "helix.bin"), pts)

    # Let PCL produce the compressed variant, so we are decoding a stream
    # written by the reference implementation rather than by ourselves.
    src = os.path.join(OUT, "helix_binary.pcd")
    dst = os.path.join(OUT, "helix_compressed.pcd")
    try:
        r = subprocess.run(
            ["pcl_convert_pcd_ascii_binary", src, dst, "2"],
            capture_output=True, text=True, timeout=120)
        if r.returncode != 0 or not os.path.exists(dst):
            print(f"  warning: PCL conversion failed: {r.stderr.strip()[:300]}", file=sys.stderr)
        else:
            print(f"  wrote {os.path.basename(dst)} via PCL")
    except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
        print(f"  warning: pcl_convert_pcd_ascii_binary unavailable ({exc})", file=sys.stderr)

    for name in sorted(os.listdir(OUT)):
        print(f"  {name:28s} {os.path.getsize(os.path.join(OUT, name)):>10,} bytes")


if __name__ == "__main__":
    main()
