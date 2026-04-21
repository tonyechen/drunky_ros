#!/usr/bin/env python3
"""
Group Bottle Capture Tool
Captures images of multiple bottles grouped together for training data.
Works alongside capture_bottles.py without disturbing existing data.

Usage:
    python3 capture_groups.py
"""

import cv2
import pyrealsense2 as rs
import numpy as np
import os
import glob
import shutil
import random

BASE_DIR = "dataset"
RAW_DIR = os.path.join(BASE_DIR, "raw")
GROUP_DIR = os.path.join(RAW_DIR, "group")
TRAIN_RATIO = 0.8


def setup_camera():
    """Initialize RealSense pipeline."""
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)
    pipeline.start(config)
    # Let auto-exposure settle
    for _ in range(30):
        pipeline.wait_for_frames()
    print("  Camera ready!\n")
    return pipeline


def capture_frame(pipeline):
    """Capture a single color frame."""
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    if not color_frame:
        return None
    return np.asanyarray(color_frame.get_data())


def get_next_index():
    """Find the next available index based on existing group images."""
    os.makedirs(GROUP_DIR, exist_ok=True)
    existing = glob.glob(os.path.join(GROUP_DIR, "group_*.jpg"))
    if not existing:
        return 0
    indices = []
    for path in existing:
        basename = os.path.basename(path)
        try:
            idx = int(basename.replace("group_", "").replace(".jpg", ""))
            indices.append(idx)
        except ValueError:
            continue
    return max(indices) + 1 if indices else 0


def capture_group_images():
    """Interactive capture loop for group bottle images."""
    print("Starting RealSense camera...")
    pipeline = setup_camera()

    start_index = get_next_index()
    existing_count = start_index
    current_index = start_index
    session_count = 0

    print("=" * 60)
    print("  GROUP BOTTLE CAPTURE TOOL")
    print(f"  Existing group images: {existing_count}")
    print(f"  New images will start at index: {start_index}")
    print("=" * 60)
    print("\n  Arrange multiple bottles in the frame.")
    print("  Press ENTER to capture, type 'q' + ENTER to finish.\n")

    while True:
        total = existing_count + session_count
        user_input = input(
            f"  [{session_count} captured this session, {total} total] "
            f"Press ENTER to capture (or 'q' to finish): "
        )

        if user_input.strip().lower() == "q":
            break

        img = capture_frame(pipeline)
        if img is None:
            print("  ERROR: Could not capture frame. Try again.")
            continue

        filename = f"group_{current_index:03d}.jpg"
        filepath = os.path.join(GROUP_DIR, filename)
        cv2.imwrite(filepath, img)
        print(f"  ✓ Saved {filepath}")

        current_index += 1
        session_count += 1

    pipeline.stop()

    print(f"\n{'=' * 60}")
    print(f"  SESSION COMPLETE — {session_count} new group images captured.")
    print(f"{'=' * 60}")

    return session_count


def get_existing_train_val_groups():
    """Find group images already in train/val to avoid re-splitting them."""
    existing = set()
    for split in ["train", "val"]:
        split_dir = os.path.join(BASE_DIR, split, "images")
        if os.path.exists(split_dir):
            for f in os.listdir(split_dir):
                if f.startswith("group_") and f.endswith(".jpg"):
                    existing.add(f)
    return existing


def organize_new_groups():
    """Add only newly captured group images into the train/val split."""
    for split in ["train", "val"]:
        os.makedirs(os.path.join(BASE_DIR, split, "images"), exist_ok=True)
        os.makedirs(os.path.join(BASE_DIR, split, "labels"), exist_ok=True)

    if not os.path.exists(GROUP_DIR):
        print("  No group images found.")
        return

    all_group_images = sorted(
        [f for f in os.listdir(GROUP_DIR) if f.startswith("group_") and f.endswith(".jpg")]
    )
    already_split = get_existing_train_val_groups()
    new_images = [f for f in all_group_images if f not in already_split]

    if not new_images:
        print("  No new group images to organize.")
        return

    print(f"\n  Organizing {len(new_images)} new group images into train/val...")

    random.shuffle(new_images)
    split_idx = int(len(new_images) * TRAIN_RATIO)
    # Ensure at least 1 goes to val if there are 2+ images
    if split_idx == len(new_images) and len(new_images) > 1:
        split_idx = len(new_images) - 1

    train_images = new_images[:split_idx]
    val_images = new_images[split_idx:]

    for img_name in train_images:
        shutil.copy2(
            os.path.join(GROUP_DIR, img_name),
            os.path.join(BASE_DIR, "train", "images", img_name),
        )

    for img_name in val_images:
        shutil.copy2(
            os.path.join(GROUP_DIR, img_name),
            os.path.join(BASE_DIR, "val", "images", img_name),
        )

    print(f"  Train: +{len(train_images)} group images")
    print(f"  Val:   +{len(val_images)} group images")

    print(f"\n{'=' * 60}")
    print("  REMINDER:")
    print("  Group images contain multiple bottles and need manual")
    print("  bounding-box labeling for each visible bottle class.")
    print("  Use Roboflow or labelImg to create .txt label files:")
    print("    pip install labelImg && labelImg dataset/train/images")
    print(f"{'=' * 60}")


if __name__ == "__main__":
    captured = capture_group_images()
    if captured > 0:
        organize_new_groups()
    else:
        print("\n  No images captured. Exiting.")