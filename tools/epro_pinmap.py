"""Extract only electrical records from user-supplied LCEDA epro2 archives.

No user/profile metadata is exported. The input archives are not committed.
"""
import argparse
import hashlib
import json
import zipfile
from collections import defaultdict
from pathlib import Path


def extract(path):
    data = Path(path).read_bytes()
    with zipfile.ZipFile(path) as archive:
        name = next(n for n in archive.namelist() if n.endswith('.epru'))
        text = archive.read(name).decode('utf-8')
    decoder, objects, i = json.JSONDecoder(), [], 0
    while i < len(text):
        start = text.find('{', i)
        if start < 0:
            break
        try:
            obj, i = decoder.raw_decode(text, start)
            objects.append(obj)
        except ValueError:
            i = start + 1
    starts = [i for i, o in enumerate(objects) if o.get('type') == 'DOCHEAD']
    components, mcu_names = {}, {}
    for k, start in enumerate(starts):
        end = starts[k + 1] if k + 1 < len(starts) else len(objects)
        seg = objects[start:end]
        if len(seg) < 2:
            continue
        kind = seg[1].get('docType')
        records = [(h, seg[j + 1]) for j, h in enumerate(seg[:-1]) if 'type' in h]
        attrs = defaultdict(dict)
        for head, body in records:
            if head['type'] == 'ATTR':
                attrs[body.get('parentId')][body.get('key')] = body.get('value')
        title = next((b.get('title') for h, b in records if h['type'] == 'META'), '')
        if kind == 'SYMBOL' and title == 'CH32V307VCT6':
            for values in attrs.values():
                if values.get('Pin Number'):
                    mcu_names[values['Pin Number']] = values.get('Pin Name')
        if kind != 'PCB':
            continue
        by_id = {h['id']: attrs[h['id']].get('Designator')
                 for h, b in records if h['type'] == 'COMPONENT'}
        for head, body in records:
            if head['type'] != 'PAD_NET':
                continue
            key = json.loads(head['id'])
            ref = by_id.get(key[1])
            if ref:
                components.setdefault(ref, {})[key[2]] = body.get('padNet', '')
    return {'source_name': Path(path).name, 'sha256': hashlib.sha256(data).hexdigest(),
            'components': components, 'mcu_pin_names': mcu_names}


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('mainboard')
    p.add_argument('driver')
    p.add_argument('--output', required=True)
    a = p.parse_args()
    result = {'mainboard': extract(a.mainboard), 'driver': extract(a.driver)}
    target = Path(a.output)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(target)
