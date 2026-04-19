from ultralytics import YOLO
import os

DATASET_YAML = os.path.abspath("../dataset/data.yaml")
OUTPUT_DIR   = os.path.abspath(".")

def train():
    model = YOLO("yolo11n.pt")

    model.train(
        data=DATASET_YAML,
        epochs=100,
        imgsz=640,
        batch=8,
        patience=20,          # early stop if no improvement for 20 epochs
        lr0=0.001,            # lower LR — fine-tuning, not training from scratch
        augment=True,
        project=OUTPUT_DIR,
        name="run",
        exist_ok=True,
    )

    best_pt = os.path.join(OUTPUT_DIR, "run", "weights", "best.pt")
    print(f"\nTraining complete. Best model: {best_pt}")


if __name__ == "__main__":
    train()
