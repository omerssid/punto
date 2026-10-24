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


def write_greyscale_pcd(path, pts):
    """Intensity replicated into R, G and B -- what a lot of LiDAR tooling
    emits, and what the greyscale detector has to recognise."""
    hdr = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n"
        f"WIDTH {len(pts)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(pts)}\nDATA binary\n"
    )
    with open(path, "wb") as f:
        f.write(hdr.encode())
        for x, y, z, inten, *_rest in pts:
            v = inten >> 8                       # one value in all three channels
            packed = (v << 16) | (v << 8) | v
            as_float = struct.unpack("<f", struct.pack("<I", packed))[0]
            f.write(struct.pack("<ffff", x, y, z, as_float))


def write_kitti_bin(path, pts):
    with open(path, "wb") as f:
        for x, y, z, inten, *_rest in pts:
            f.write(struct.pack("<ffff", x, y, z, inten / 65535.0))


PLY_TYPES = {"f4": "float", "u1": "uchar", "u2": "ushort", "f8": "double"}


def write_ply(path, pts, binary=None, with_faces=False, leading_element=False):
    """Write the helix as PLY.

    `binary` is None for ascii, "<" for little-endian, ">" for big-endian.
    `leading_element` emits a dummy element BEFORE vertex, which exercises
    the reader's ability to locate the vertex block past it.
    `with_faces` appends a face element after the vertices, the usual shape
    of a PLY in the wild.
    """
    props = [("x", "f4"), ("y", "f4"), ("z", "f4"),
             ("intensity", "u2"), ("red", "u1"), ("green", "u1"), ("blue", "u1")]

    fmt = "ascii 1.0" if binary is None else (
        "binary_little_endian 1.0" if binary == "<" else "binary_big_endian 1.0")
    lines = ["ply", f"format {fmt}", "comment generated by make_fixtures.py"]
    if leading_element:
        lines += ["element meta 3", "property float value"]
    lines += [f"element vertex {len(pts)}"]
    lines += [f"property {PLY_TYPES[t]} {n}" for n, t in props]
    if with_faces:
        lines += ["element face 2", "property list uchar int vertex_indices"]
    lines += ["end_header"]
    header = ("\n".join(lines) + "\n").encode()

    def row(i, p):
        x, y, z, inten, _ring, _label, _t = p
        r, g, b = (i * 7) % 256, (i * 13) % 256, (i * 29) % 256
        return x, y, z, inten, r, g, b

    with open(path, "wb") as f:
        f.write(header)
        if leading_element:
            if binary is None:
                f.write(b"1.0\n2.0\n3.0\n")
            else:
                for v in (1.0, 2.0, 3.0):
                    f.write(struct.pack(binary + "f", v))
        if binary is None:
            for i, p in enumerate(pts):
                x, y, z, inten, r, g, b = row(i, p)
                xs = "nan" if math.isnan(x) else f"{x:.6f}"
                f.write(f"{xs} {y:.6f} {z:.6f} {inten} {r} {g} {b}\n".encode())
        else:
            packer = struct.Struct(binary + "fffHBBB")
            for i, p in enumerate(pts):
                f.write(packer.pack(*row(i, p)))
        if with_faces:
            if binary is None:
                f.write(b"3 0 1 2\n3 1 2 3\n")
            else:
                for tri in ((0, 1, 2), (1, 2, 3)):
                    f.write(struct.pack(binary + "B", 3))
                    for idx in tri:
                        f.write(struct.pack(binary + "i", idx))


def write_las(path, pts, version=(1, 2), point_format=3, color16=True):
    """Write the helix as LAS.

    Coordinates become int32 via a scale/offset, which is how LAS stores
    them and the reason the decoder carries an affine transform.
    """
    scale = (0.001, 0.001, 0.001)
    offset = (0.0, 0.0, 0.0)

    if point_format == 3:
        rec_fmt = "<iiiHBBbBHdHHH"      # xyz, intensity, flags, class, angle,
        rec_len = 34                     # user, source, gps, rgb
    elif point_format == 7:
        rec_fmt = "<iiiHBBBBhHdHHH"
        rec_len = 36
    elif point_format == 0:
        rec_fmt = "<iiiHBBbBH"
        rec_len = 20
    else:
        raise ValueError(point_format)

    finite = [p for p in pts if not math.isnan(p[0])]
    xs = [p[0] for p in finite]; ys = [p[1] for p in finite]; zs = [p[2] for p in finite]

    header_size = 227 if version < (1, 4) else 375
    records = []
    for i, p in enumerate(pts):
        x, y, z, inten, ring, label, stamp = p
        xi = int(round((x - offset[0]) / scale[0]))
        yi = int(round((y - offset[1]) / scale[1]))
        zi = int(round((z - offset[2]) / scale[2]))
        r, g, b = (i * 7) % 256, (i * 13) % 256, (i * 29) % 256
        if color16:
            r, g, b = r * 257, g * 257, b * 257
        # Set the "synthetic" flag bit on some points, so the reader's
        # classification masking gets exercised.
        cls = (label % 32) | (0x20 if i % 50 == 0 else 0)
        if point_format == 3:
            records.append(struct.pack(rec_fmt, xi, yi, zi, inten, 0x11, cls, 0, 0,
                                       ring, stamp, r, g, b))
        elif point_format == 7:
            records.append(struct.pack(rec_fmt, xi, yi, zi, inten, 0x11, 0, cls & 0x1F, 0,
                                       0, ring, stamp, r, g, b))
        else:
            records.append(struct.pack(rec_fmt, xi, yi, zi, inten, 0x11, cls, 0, 0, ring))

    point_offset = header_size
    hdr = bytearray(header_size)
    hdr[0:4] = b"LASF"
    hdr[24] = version[0]
    hdr[25] = version[1]
    struct.pack_into("<H", hdr, 94, header_size)
    struct.pack_into("<I", hdr, 96, point_offset)
    struct.pack_into("<I", hdr, 100, 0)              # no VLRs
    hdr[104] = point_format
    struct.pack_into("<H", hdr, 105, rec_len)
    struct.pack_into("<I", hdr, 107, len(pts) if version < (1, 4) else 0)
    for i in range(3):
        struct.pack_into("<d", hdr, 131 + i * 8, scale[i])
        struct.pack_into("<d", hdr, 155 + i * 8, offset[i])
    struct.pack_into("<d", hdr, 179, max(xs)); struct.pack_into("<d", hdr, 187, min(xs))
    struct.pack_into("<d", hdr, 195, max(ys)); struct.pack_into("<d", hdr, 203, min(ys))
    struct.pack_into("<d", hdr, 211, max(zs)); struct.pack_into("<d", hdr, 219, min(zs))
    if version >= (1, 4):
        struct.pack_into("<Q", hdr, 247, len(pts))

    with open(path, "wb") as f:
        f.write(bytes(hdr))
        for rec in records:
            f.write(rec)


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
    write_greyscale_pcd(os.path.join(OUT, "helix_greyscale.pcd"), pts)

    write_ply(os.path.join(OUT, "helix_ascii.ply"), pts, binary=None)
    write_ply(os.path.join(OUT, "helix_le.ply"), pts, binary="<")
    write_ply(os.path.join(OUT, "helix_be.ply"), pts, binary=">")
    write_ply(os.path.join(OUT, "helix_faces.ply"), pts, binary="<", with_faces=True)
    write_ply(os.path.join(OUT, "helix_leading.ply"), pts, binary="<", leading_element=True)

    write_las(os.path.join(OUT, "helix_12_fmt3.las"), pts, version=(1, 2), point_format=3)
    write_las(os.path.join(OUT, "helix_14_fmt7.las"), pts, version=(1, 4), point_format=7)
    write_las(os.path.join(OUT, "helix_12_fmt0.las"), pts, version=(1, 2), point_format=0)
    write_las(os.path.join(OUT, "helix_8bitcolor.las"), pts, version=(1, 2), point_format=3,
              color16=False)

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
