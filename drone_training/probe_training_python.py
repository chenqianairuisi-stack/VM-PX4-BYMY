"""Report installed dependencies and verify real CUDA and torchvision kernels."""
import importlib
import json
from pathlib import Path
import sys

report = {'python': sys.executable, 'version': sys.version, 'ready': False}
errors = []
modules = {}
for name in ('torch', 'torchvision', 'ultralytics'):
    try:
        modules[name] = importlib.import_module(name)
        report[name] = modules[name].__version__
    except Exception as exc:
        errors.append(f'{name}: {type(exc).__name__}: {exc}')
if 'torch' in modules:
    torch = modules['torch']
    try:
        report['cuda'] = torch.cuda.is_available()
        report['torch_cuda_version'] = torch.version.cuda
        if report['cuda']:
            report['gpu'] = torch.cuda.get_device_name(0)
            assert (torch.ones(4, device='cuda:0') * 2).sum().item() == 8
            if 'torchvision' in modules:
                modules['torchvision'].ops.nms(
                    torch.tensor([[0., 0., 2., 2.]], device='cuda:0'),
                    torch.tensor([.9], device='cuda:0'), .5,
                )
            torch.cuda.synchronize()
        else:
            errors.append('PyTorch cannot access CUDA in this interpreter')
    except Exception as exc:
        errors.append(f'CUDA execution: {type(exc).__name__}: {exc}')
report['ready'] = not errors and len(modules) == 3
report['errors'] = errors
Path(sys.argv[1]).write_text(json.dumps(report, indent=2), encoding='utf-8')
print(json.dumps(report), flush=True)
sys.exit(0 if report['ready'] else 1)
