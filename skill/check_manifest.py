"""Read-only SHA256 validation reusable across local embedded release folders."""
from pathlib import Path
import argparse, hashlib, json

parser = argparse.ArgumentParser()
parser.add_argument('--manifest', type=Path, required=True)
parser.add_argument('--root', type=Path, required=True)
args = parser.parse_args()
root = args.root.resolve()
entries = json.loads(args.manifest.read_text('utf-8'))['upload_files']
for name, record in entries.items():
    target = (root/name).resolve()
    if not target.is_relative_to(root) or not target.is_file():
        raise SystemExit('Missing or out-of-root path: '+name)
    if hashlib.sha256(target.read_bytes()).hexdigest() != record['sha256']:
        raise SystemExit('Hash mismatch: '+name)
print('MANIFEST_OK', len(entries))
