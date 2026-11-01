"""Run the existing application regression suite without board access."""
from pathlib import Path
import subprocess, sys

ROOT = Path(__file__).resolve().parents[1]
out = ROOT / 'sim/generated'
out.mkdir(exist_ok=True)
command = [sys.executable, '-m', 'unittest', 'discover', '-s', str(ROOT / 'pynq/verification'), '-p', 'test_*.py']
r = subprocess.run(command, cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
(out / 'application_tests.log').write_bytes(r.stdout)
print(r.stdout.decode('utf-8', errors='replace'))
raise SystemExit(r.returncode)
