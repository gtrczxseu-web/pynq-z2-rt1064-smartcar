"""Build ARM runtime libraries on PYNQ, without replacing deployed files."""
from pathlib import Path
import platform, subprocess

ROOT = Path(__file__).resolve().parents[1]
if platform.machine() not in ('armv7l', 'armv8l'):
    raise SystemExit('Run on the 32-bit ARM PYNQ target, not an x86 desktop.')
out = ROOT / 'build/generated'
out.mkdir(exist_ok=True)
for source, name in [('uart_capture.c', 'libuart_receiver_rt.so'), ('uart_tx_native.c', 'libuart_tx_v1.so')]:
    subprocess.run(['gcc', '-O2', '-std=c11', '-shared', '-fPIC', '-pthread',
                    str(ROOT / 'pynq' / source), '-o', str(out / name), '-latomic'], check=True)
print('Built in', out, '; no deployed file changed. Verify before deliberate replacement.')
