#!/usr/bin/env python3
"""Validate captured YOLO data and make a contact sheet without changing labels."""
import argparse
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('dataset', type=Path)
    args = parser.parse_args()
    root = args.dataset
    report = {'classes': ['drone'], 'splits': {}, 'duplicate_images': 0}
    hashes = set()
    tiles = []
    for split in ['train','val','test']:
        images = sorted((root/'images'/split).glob('*.jpg'))
        labels = sorted((root/'labels'/split).glob('*.txt'))
        assert {p.stem for p in images} == {p.stem for p in labels}, split
        positive = negative = 0
        widths, heights = [], []
        selected = set(np.linspace(0,len(images)-1,12,dtype=int))
        for index, path in enumerate(images):
            data = path.read_bytes()
            digest = hashlib.sha256(data).hexdigest()
            if digest in hashes:
                report['duplicate_images'] += 1
            hashes.add(digest)
            frame = cv2.imread(str(path))
            assert frame is not None and frame.shape == (480,640,3), path
            label = (root/'labels'/split/(path.stem+'.txt')).read_text().strip()
            if label:
                values = list(map(float,label.split()))
                assert len(values) == 5 and values[0] == 0, path
                _,x,y,w,h = values
                assert all(np.isfinite(values)) and 0<w<=1 and 0<h<=1, path
                assert -.000001<=x-w/2<=x+w/2<=1.000001, path
                assert -.000001<=y-h/2<=y+h/2<=1.000001, path
                widths.append(w*640); heights.append(h*480)
                positive += 1
                cv2.rectangle(frame,(round((x-w/2)*640),round((y-h/2)*480)),
                              (round((x+w/2)*640),round((y+h/2)*480)),(0,255,0),1)
            else:
                negative += 1
            if index in selected:
                cv2.putText(frame,split+'/'+path.stem,(8,22),cv2.FONT_HERSHEY_SIMPLEX,.5,(255,255,255),1)
                tiles.append(cv2.resize(frame,(320,240)))
        report['splits'][split] = dict(images=len(images),positive=positive,negative=negative,
                                      width_px=[min(widths),max(widths)],height_px=[min(heights),max(heights)])
    records = json.loads((root/'capture_manifest.json').read_text())
    scenes = {split: {r['scene'] for r in records if r['file'].startswith('images/'+split+'/')}
              for split in report['splits']}
    assert not (scenes['train'] & scenes['val'] or scenes['train'] & scenes['test'] or scenes['val'] & scenes['test'])
    report['scene_splits'] = {k:sorted(v) for k,v in scenes.items()}
    report['total_images'] = sum(s['images'] for s in report['splits'].values())
    report['status'] = 'labels_validated_training_not_run'
    (root/'dataset_report.json').write_text(json.dumps(report,indent=2))
    cv2.imwrite(str(root/'preview.jpg'),cv2.vconcat([cv2.hconcat(tiles[i:i+4]) for i in range(0,len(tiles),4)]))
    print(json.dumps(report,indent=2))


if __name__ == '__main__':
    main()
