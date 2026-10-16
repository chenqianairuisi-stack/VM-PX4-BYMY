#!/usr/bin/env python3
"""GPU-only training, usable on Windows and Linux. Outputs best.pt and ONNX."""
import argparse
import json
from pathlib import Path
import shutil


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data', type=Path, default=Path(__file__).resolve().parent / 'dataset')
    parser.add_argument('--epochs', type=int, default=100)
    parser.add_argument('--batch', type=int, default=8)
    parser.add_argument('--imgsz', type=int, default=640)
    parser.add_argument('--workers', type=int, default=2)
    parser.add_argument('--weights', default='yolov8n.pt')
    args = parser.parse_args()
    import torch
    import yaml
    from ultralytics import YOLO
    if not torch.cuda.is_available():
        raise SystemExit('CUDA is unavailable. Run on Windows with the NVIDIA GPU and CUDA PyTorch; CPU fallback is disabled.')
    root = args.data.resolve()
    if not (root / 'data.yaml').is_file():
        raise SystemExit(f'Missing dataset: {root / "data.yaml"}')
    config = yaml.safe_load((root/'data.yaml').read_text(encoding='utf-8'))
    config['path'] = str(root)
    config_path = root/'data.local.yaml'
    config_path.write_text(yaml.safe_dump(config), encoding='utf-8')
    print('Training GPU:', torch.cuda.get_device_name(0), flush=True)
    model = YOLO(args.weights)
    result = model.train(data=str(config_path), epochs=args.epochs, imgsz=args.imgsz,
                        batch=args.batch, workers=args.workers, device=0,
                        project=str(root.parent/'runs'), name='drone', exist_ok=False,
                        seed=2026, deterministic=True, patience=25, close_mosaic=10,
                        fliplr=.5, flipud=0.0, degrees=12, translate=.15, scale=.5,
                        hsv_h=.015, hsv_s=.5, hsv_v=.35)
    best = Path(model.trainer.best)
    trained = YOLO(str(best))
    test = trained.val(data=str(config_path), split='test', device=0, imgsz=args.imgsz)
    exported = trained.export(format='onnx', imgsz=416, dynamic=False, simplify=False,
                             opset=12, device='cpu')
    destination = root.parent/'trained_model'
    destination.mkdir(exist_ok=True)
    shutil.copy2(best, destination/'drone.pt')
    shutil.copy2(exported, destination/'drone.onnx')
    (destination/'metrics.json').write_text(json.dumps({
        'gpu': torch.cuda.get_device_name(0), 'epochs_requested': args.epochs,
        'test_metrics': test.results_dict, 'train_directory': str(model.trainer.save_dir),
        'classes': trained.names, 'status': 'training_complete_requires_simulation_validation',
    }, indent=2), encoding='utf-8')
    print(f'Completed. Models and held-out test metrics: {destination}', flush=True)


if __name__ == '__main__':
    # Required for Windows DataLoader child processes.
    main()
