#!/usr/bin/env python3
"""Remove isolated occupied specks from a map_server PGM.

The saved real-robot map has single-cell obstacle noise left by scan matching.
Those specks land next to the robot and make Nav2 treat the robot cell as
lethal, so every path is rejected. Real walls are long connected components;
specks are 1-2 cells.

Usage:
    python3 tools/clean_map.py IN.pgm OUT.pgm --min-cluster 3
"""
import argparse
import shutil
from pathlib import Path

import numpy as np
from PIL import Image
from scipy.ndimage import label


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--min-cluster", type=int, default=3)
    options = parser.parse_args()

    image = np.array(Image.open(options.input))
    if image.ndim != 2:
        raise SystemExit("expected a single-channel PGM")
    occupied = image == 0
    labels, count = label(occupied, structure=np.ones((3, 3)))
    sizes = np.bincount(labels.ravel())
    removed = 0
    for index in range(1, count + 1):
        if sizes[index] < options.min_cluster:
            occupied[labels == index] = False
            removed += 1
    cleaned = image.copy()
    cleaned[occupied] = 0
    removed_cells = (image == 0) & ~occupied
    # A removed speck next to free space becomes free; one floating in unknown
    # space stays unknown, otherwise we would punch holes into unmapped area.
    neighbor_free = np.zeros_like(removed_cells)
    for shift_r, shift_c in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        neighbor_free |= np.roll(image == 254, (shift_r, shift_c), axis=(0, 1))
    cleaned[removed_cells & neighbor_free] = 254
    cleaned[removed_cells & ~neighbor_free] = 205
    Image.fromarray(cleaned).save(options.output)

    yaml_in = options.input.with_suffix(".yaml")
    yaml_out = options.output.with_suffix(".yaml")
    if yaml_in.exists():
        text = yaml_in.read_text()
        text = text.replace(options.input.name, options.output.name)
        yaml_out.write_text(text)

    print(
        f"{options.input.name}: clusters {count}, removed {removed}, "
        f"occupied {int((image == 0).sum())} -> {int(occupied.sum())}"
    )


if __name__ == "__main__":
    main()
