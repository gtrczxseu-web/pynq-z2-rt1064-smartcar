from pathlib import Path
import argparse,subprocess,sys
root=Path(__file__).resolve().parents[1];sim=root/'sim'
parser=argparse.ArgumentParser();parser.add_argument('--iverilog',default='C:/iverilog/bin/iverilog.exe');parser.add_argument('--vvp',default='C:/iverilog/bin/vvp.exe');args=parser.parse_args()
subprocess.run([sys.executable,str(sim/'make_vectors.py')],check=True)
out=root/'generated/sim';out.mkdir(parents=True,exist_ok=True)
logs=[]
for top in ['tb_math','tb_sp21_axi']:
    exe=out/(top+'.vvp')
    command=[args.iverilog,'-g2012','-Wall','-I',str(sim),'-s',top,'-o',str(exe)]+[str(p) for p in sorted((root/'rtl').glob('*.v'))]+[str(sim/(top+'.sv'))]
    result=subprocess.run(command,capture_output=True,text=True);logs.append(result.stdout+result.stderr)
    if result.returncode:break
    result=subprocess.run([args.vvp,str(exe)],cwd=sim,capture_output=True,text=True);logs.append(result.stdout+result.stderr)
    if result.returncode:break
reports=root/'reports';reports.mkdir(exist_ok=True)
(reports/'simulation.log').write_text('\n'.join(logs),encoding='utf-8')
print('\n'.join(logs));raise SystemExit(result.returncode)
