"""Read-only integrity/path check; never opens hardware or installs services."""
from pathlib import Path
import hashlib, json, re
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
runtime = ROOT / 'pynq'
manifest = json.loads((runtime / 'manifest.json').read_text('utf-8'))
for name, entry in manifest['upload_files'].items():
    p = (runtime / name).resolve()
    assert p.is_relative_to(runtime.resolve()) and p.is_file(), name
    assert hashlib.sha256(p.read_bytes()).hexdigest() == entry['sha256'], name
bad = []
for p in ROOT.rglob('*'):
    rel = p.relative_to(ROOT)
    if any(part in ('.git', '__pycache__', 'generated') for part in rel.parts):
        continue
    if not all(re.fullmatch(r'[a-z0-9_.-]+', part) for part in rel.parts):
        bad.append(rel.as_posix())
assert not bad, bad
project = ROOT / 'firmware/bisai300/mdk/rt1064.uvprojx'
tree = ET.parse(project)
for el in tree.findall('.//FilePath') + tree.findall('.//ScatterFile'):
    if el.text:
        p = (project.parent / el.text.replace('\\', '/')).resolve()
        assert p.is_relative_to(ROOT.resolve()) and p.is_file(), el.text
print('PACKAGE_OK', len(manifest['upload_files']), 'runtime hashes; paths and Keil references OK')
