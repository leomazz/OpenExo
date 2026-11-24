#!/usr/bin/env python3
"""Convert OpenExo LOGnnn.BIN files (FSR-free branch) to CSV.

Usage:
    python oexlog_to_csv.py LOG000.BIN [LOG001.BIN ...]
Writes LOG000.csv next to each input. Needs numpy (pip install numpy).

File layout (see ExoCode/src/DataLoggerSD.h):
    text header ending with "END\n" (fields:..., build:...)
    records: uint32 t_us, uint32 cycle, float32 x N  (little-endian)
"""
import sys
import numpy as np


def read_log(path):
    raw = open(path, "rb").read()
    end = raw.find(b"\nEND\n")
    if not raw.startswith(b"OEXLOG1") or end < 0:
        raise ValueError(f"{path}: not an OEXLOG1 file")
    header = raw[:end].decode("ascii", "replace").splitlines()
    fields = next(l for l in header if l.startswith("fields:"))[7:].split(",")
    build = next((l[6:] for l in header if l.startswith("build:")), "")
    body = raw[end + 5:]
    dtype = np.dtype([(fields[0], "<u4"), (fields[1], "<u4")] + [(f, "<f4") for f in fields[2:]])
    n = len(body) // dtype.itemsize
    data = np.frombuffer(body[: n * dtype.itemsize], dtype=dtype)
    return fields, build, data


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    for path in sys.argv[1:]:
        fields, build, data = read_log(path)
        out = path.rsplit(".", 1)[0] + ".csv"
        with open(out, "w") as f:
            f.write("# " + build + "\n")
            f.write(",".join(fields) + "\n")
            for rec in data:
                f.write(",".join(str(v) if i < 2 else f"{v:.6g}" for i, v in enumerate(rec)) + "\n")
        dur = (int(data["t_us"][-1]) - int(data["t_us"][0])) / 1e6 if len(data) > 1 else 0
        print(f"{path}: {len(data)} records, {dur:.1f} s -> {out}")
        print(f"   build: {build}")


if __name__ == "__main__":
    main()
