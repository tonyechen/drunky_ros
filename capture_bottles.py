#!/usr/bin/env python3
"""
Bottle Dataset Capture Tool
Captures 10 images per bottle class directly from a RealSense camera,
then organizes them into YOLO train/val structure.

Usage:
    python3 capture_bottles.py
"""

import cv2
import pyrealsense2 as rs
import numpy as np
import os
import shutil
import random

CLASSES = [
    "vodka",
    "whiskey",
    "gin",
    "tequila",
    "orange_juice",
    "ginger_beer",
    "coke",
    "tonic_water",
]

IMAGES_PER_CLASS = 10
BASE_DIR = "dataset"
RAW_DIR = os.path.join(BASE_DIR, "raw")
TRAIN_RATIO = 0.8


def setup_camera():
    """Initialize RealSense pipeline."""
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
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


def capture_all_bottles():
    """Main capture loop."""
    print("Starting RealSense camera...")
    pipeline = setup_camera()

    for cls in CLASSES:
        os.makedirs(os.path.join(RAW_DIR, cls), exist_ok=True)

    print("=" * 60)
    print("  BOTTLE DATASET CAPTURE TOOL")
    print(f"  Classes: {len(CLASSES)} | Images per class: {IMAGES_PER_CLASS}")
    print(f"  Total images to capture: {len(CLASSES) * IMAGES_PER_CLASS}")
    print("=" * 60)

    for class_idx, cls in enumerate(CLASSES):
        print(f"\n{'='*60}")
        print(f"  CLASS {class_idx + 1}/{len(CLASSES)}: {cls.upper()}")
        print(f"{'='*60}")
        input(f"\n  Place the [{cls}] bottle under the camera and press ENTER...")

        for i in range(IMAGES_PER_CLASS):
            if i > 0:
                input(f"  [{cls}] Reposition bottle and press ENTER ({i + 1}/{IMAGES_PER_CLASS})...")

            img = capture_frame(pipeline)
            if img is None:
                print("  ERROR: Could not capture frame.")
                continue

            filename = f"{cls}_{i:03d}.jpg"
            filepath = os.path.join(RAW_DIR, cls, filename)
            cv2.imwrite(filepath, img)
            print(f"  ✓ Saved {filepath}")

        print(f"\n  Done with [{cls}] — {IMAGES_PER_CLASS} images captured.")

    pipeline.stop()

    print(f"\n{'='*60}")
    print("  ALL CAPTURES COMPLETE!")
    print(f"{'='*60}")


def organize_dataset():
    """Split raw images into train/val YOLO structure."""
    for split in ["train", "val"]:
        os.makedirs(os.path.join(BASE_DIR, split, "images"), exist_ok=True)
        os.makedirs(os.path.join(BASE_DIR, split, "labels"), exist_ok=True)

    print("\nOrganizing dataset into train/val split...")

    train_count = 0
    val_count = 0

    for cls in CLASSES:
        cls_dir = os.path.join(RAW_DIR, cls)
        if not os.path.exists(cls_dir):
            print(f"  WARNING: No images found for {cls}")
            continue

        images = sorted([f for f in os.listdir(cls_dir) if f.endswith(".jpg")])
        random.shuffle(images)

        split_idx = int(len(images) * TRAIN_RATIO)
        train_images = images[:split_idx]
        val_images = images[split_idx:]

        for img_name in train_images:
            shutil.copy2(
                os.path.join(cls_dir, img_name),
                os.path.join(BASE_DIR, "train", "images", img_name),
            )
            train_count += 1

        for img_name in val_images:
            shutil.copy2(
                os.path.join(cls_dir, img_name),
                os.path.join(BASE_DIR, "val", "images", img_name),
            )
            val_count += 1

        print(f"  {cls}: {len(train_images)} train, {len(val_images)} val")

    # Create data.yaml
    data_yaml = f"""train: ./train/images
val: ./val/images

nc: {len(CLASSES)}
names: {CLASSES}
"""
    yaml_path = os.path.join(BASE_DIR, "data.yaml")
    with open(yaml_path, "w") as f:
        f.write(data_yaml)

    print(f"\n  Train: {train_count} images")
    print(f"  Val:   {val_count} images")
    print(f"  Config: {yaml_path}")

    print(f"\n{'='*60}")
    print("  NEXT STEPS:")
    print("  1. Label images with Roboflow or labelImg")
    print("     pip install labelImg && labelImg dataset/train/images")
    print("  2. Put .txt label files in train/labels and val/labels")
    print("  3. Train:")
    print("     from ultralytics import YOLO")
    print("     model = YOLO('yolov8m.pt')")
    print("     model.train(data='dataset/data.yaml', epochs=50, imgsz=640)")
    print(f"{'='*60}")


if __name__ == "__main__":
    capture_all_bottles()
    organize_dataset()