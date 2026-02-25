#!/usr/bin/env python3
import glob
import os
import sys

from PIL import Image


def main() -> int:
    if len(sys.argv) < 3:
        print("Usage: make_gif.py <frames_dir> <output_gif> [fps]")
        return 1

    frames_dir = sys.argv[1]
    output_gif = sys.argv[2]
    fps = float(sys.argv[3]) if len(sys.argv) >= 4 else 12.0
    fps = max(1.0, fps)

    pattern = os.path.join(frames_dir, "frame_*.png")
    frame_paths = sorted(glob.glob(pattern))
    if not frame_paths:
        print(f"No frames found in {frames_dir}")
        return 2

    images = [Image.open(p).convert("P", palette=Image.ADAPTIVE) for p in frame_paths]
    duration_ms = int(1000.0 / fps)

    os.makedirs(os.path.dirname(output_gif), exist_ok=True)
    images[0].save(
        output_gif,
        save_all=True,
        append_images=images[1:],
        loop=0,
        duration=duration_ms,
        optimize=False,
    )

    print(f"GIF saved to {output_gif} ({len(images)} frames, {fps:.1f} FPS)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
