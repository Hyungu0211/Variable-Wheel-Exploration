"""Train on the team's annotated pipe dataset and export a matching NCNN model."""
import argparse
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("data", type=Path, help="YOLO dataset YAML")
    parser.add_argument("--base", default="yolov8n.pt")
    parser.add_argument("--epochs", type=int, default=50)
    parser.add_argument("--imgsz", type=int, default=320)
    args = parser.parse_args()
    if not args.data.is_file() or args.epochs < 1 or args.imgsz < 32 or args.imgsz % 32:
        parser.error("Check dataset path, epochs and image size (multiple of 32).")
    from ultralytics import YOLO
    model = YOLO(args.base)
    model.train(data=str(args.data.resolve()), epochs=args.epochs, imgsz=args.imgsz)
    best = Path(model.trainer.best)
    exported = YOLO(str(best)).export(format="ncnn", imgsz=args.imgsz)
    print("Copy the entire NCNN directory to the Pi:", exported)


if __name__ == "__main__":
    main()
