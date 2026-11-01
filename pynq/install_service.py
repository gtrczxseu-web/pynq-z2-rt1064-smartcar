"""Default is a read-only package check; --install explicitly switches V6 to V7."""
import argparse,hashlib,json,os,platform,shutil,subprocess
from pathlib import Path

FOLDER=Path(__file__).resolve().parent
UNIT=Path('/etc/systemd/system/pynq-sensor-autostart.service')
BACKUP=Path('/etc/systemd/system/pynq-sensor-autostart.service.before-v7')

def run(*args,check=True):
    p=subprocess.run(list(args),text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=40)
    print(p.stdout,flush=True)
    if check and p.returncode:raise RuntimeError('Command failed: '+' '.join(args))
    return p

def verify():
    manifest=json.loads((FOLDER/'manifest.json').read_text('utf8'))
    assert manifest['release']=='motion-v7.4-step-pause-20261025'
    for name,entry in manifest['upload_files'].items():
        p=FOLDER/name
        assert p.is_file() and hashlib.sha256(p.read_bytes()).hexdigest()==entry['sha256'],name
    print('V7_PACKAGE_OK',len(manifest['upload_files']),flush=True)

def main():
    parser=argparse.ArgumentParser();g=parser.add_mutually_exclusive_group()
    g.add_argument('--install',action='store_true');g.add_argument('--rollback',action='store_true')
    args=parser.parse_args();verify()
    if not args.install and not args.rollback:
        print('Read-only check. No service or hardware changed.');return
    if platform.machine()!='armv7l' or os.geteuid()!=0:raise RuntimeError('Run as root on PYNQ-Z2')
    if FOLDER!=Path('/home/xilinx/jupyter_notebooks/hud_motion_v7'):raise RuntimeError('Upload the entire board_upload CONTENTS to hud_motion_v7 first')
    if args.rollback:
        if not BACKUP.is_file():raise RuntimeError('No saved pre-V7 service unit')
        run('systemctl','stop',UNIT.name);shutil.copy2(BACKUP,UNIT)
        run('systemctl','daemon-reload');run('systemctl','enable',UNIT.name);run('systemctl','start',UNIT.name)
        print('Previous service restored. This does not flash/restore the MCU firmware.');return
    new=FOLDER/UNIT.name
    assert 'hud_motion_v7/standalone.py' in new.read_text('utf8')
    if UNIT.is_file() and UNIT.read_bytes()!=new.read_bytes() and not BACKUP.exists():shutil.copy2(UNIT,BACKUP)
    run('systemctl','stop',UNIT.name,check=False)
    shutil.copy2(new,UNIT);os.chmod(UNIT,0o644)
    run('systemctl','daemon-reload');run('systemctl','enable',UNIT.name);run('systemctl','start',UNIT.name)
    print('V7 installed and enabled. It waits for calibration, V7 MCU status and an explicit BTN0 press. Motors have not been commanded to start.')

if __name__=='__main__':main()
