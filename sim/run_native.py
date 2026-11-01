"""Rebuild actual C controller against ideal hardware stubs, then exercise UART."""
from pathlib import Path
import argparse, os, subprocess, sys

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument('--gcc', default='gcc')
args = parser.parse_args()
src = ROOT / 'firmware/bisai300/code'
tests = ROOT / 'sim/native'
out = ROOT / 'sim/generated'
out.mkdir(exist_ok=True)
env = dict(os.environ)
compiler = Path(args.gcc)
if compiler.is_absolute():
    env['PATH'] = str(compiler.parent) + os.pathsep + env.get('PATH', '')
common = [args.gcc, '-std=c99', '-Wall', '-Wextra', '-Werror', '-Wno-error=unused-function', '-O2',
          '-I'+str(tests/'stubs'), '-I'+str(tests), '-I'+str(src)]
actual = [str(src/n) for n in ('route_v7.c', 'route_board.c', 'position.c', 'control.c', 'pid.c')]
for name in ('adapter_host_test', 'adapter_recipe_host_test', 'adapter_spin_host_test', 'adapter_bypass_host_test'):
    target = out / (name + ('.exe' if sys.platform=='win32' else ''))
    subprocess.run(common + actual + [str(tests/(name+'.c')), '-o', str(target), '-lm'], env=env, check=True)
    subprocess.run([str(target)], env=env, check=True)
library = tests / ('adapter_bridge.dll' if sys.platform=='win32' else 'adapter_bridge.so')
flags = ['-shared', '-Wl,--export-all-symbols'] if sys.platform=='win32' else ['-shared', '-fPIC']
subprocess.run(common + flags + actual + [str(tests/'adapter_bridge.c'), '-o', str(library), '-lm'], env=env, check=True)
subprocess.run([sys.executable, str(tests/'test_wire_bypass.py')], cwd=ROOT, env=env, check=True)
