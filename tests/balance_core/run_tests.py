"""Compile and run the same pure C V2 core on the host; no board access."""
import argparse
import os
import shutil
from pathlib import Path
import subprocess
import sys

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sanitize', action='store_true')
    parser.add_argument('--compiler', type=Path)
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[2]
    build=Path(__file__).resolve().parent/'build'
    build.mkdir(exist_ok=True)
    compilers=sorted((root/'.host_tools'/'zig').glob('*/zig.exe'), reverse=True)
    compiler=args.compiler or shutil.which("zig") or (compilers[0] if compilers else None)
    if not compiler:
        raise RuntimeError('No host compiler; provide --compiler')
    tag='ubsan' if args.sanitize else 'normal'
    exe=build/('balance_test_'+tag+('.exe' if os.name=='nt' else ''))
    component=root/'firmware'/'components'/'balance_core'
    command=[str(compiler)]+(['cc'] if Path(compiler).stem=='zig' else [])+[
        '-std=c11','-Wall','-Wextra','-Werror','-pedantic','-I',str(component/'include'),
        str(component/'balance_core.c'),str(Path(__file__).with_name('test_balance.c')),
        '-o',str(exe)]
    if args.sanitize:
        command+=['-fsanitize=undefined','-fsanitize-trap=undefined','-fno-sanitize-recover=all']
    env=dict(os.environ, ZIG_LOCAL_CACHE_DIR=str(build/'zig-local-cache'), ZIG_GLOBAL_CACHE_DIR=str(build/'zig-global-cache'))
    subprocess.run(command,env=env,check=True)
    result=subprocess.run([str(exe)],capture_output=True,text=True)
    output=result.stdout+result.stderr
    (build/('last_'+tag+'.txt')).write_text(output,encoding='utf-8')
    print(output,end='')
    if result.returncode:
        return 2
    csv=['source,case,expected_mass_g,expected_angle_deg,computed_mass_g,computed_angle_deg']
    csv+=['synthetic,'+line[4:] for line in result.stdout.splitlines() if line.startswith('SIM,')]
    (build/('simulation_'+tag+'.csv')).write_text('\n'.join(csv)+'\n',encoding='utf-8')
    return 0

if __name__=='__main__':
    raise SystemExit(main())
