"""Compile shared production product engine/job and influence/average core."""
import argparse
import os
import shutil
from pathlib import Path
import subprocess
p=argparse.ArgumentParser();p.add_argument('--assessment',action='store_true');p.add_argument('--sanitize',action='store_true');p.add_argument('--trial',action='store_true');p.add_argument('--auto-axis',action='store_true');p.add_argument('--retest',action='store_true');p.add_argument('--window',action='store_true');p.add_argument('--speed',action='store_true');a=p.parse_args()
root=Path(__file__).resolve().parents[2];here=Path(__file__).resolve().parent;build=here/'build';build.mkdir(exist_ok=True)
parts=root/'firmware/components'
cc=shutil.which('zig') or next(iter(sorted((root/'.host_tools/zig').glob('*/zig.exe'),reverse=True)),None)
if not cc:
    p.error('Install Zig and add it to PATH.')
names=['product_session','balance_core','coherent_average','coast_selection','coast_analysis','sync_analysis']
cmd=[str(cc),'cc','-std=c11','-O2','-Wall','-Wextra','-Werror','-pedantic']
for n in names:cmd+=['-I',str(parts/n/'include')]
cmd+=[str(parts/'product_session/product_window.c'),str(parts/'product_session/product_speed.c'),str(parts/'product_session/product_engine.c'),str(parts/'product_session/product_assessment.c'),str(parts/'product_session/product_job.c'),
      str(parts/'product_session/product_trial.c'),str(parts/'product_session/product_auto_axis.c'),str(parts/'product_session/product_retest.c'),str(parts/'balance_core/balance_core.c'),str(parts/'coherent_average/coherent_average.c'),str(here/('test_assessment.c' if a.assessment else 'test_window.c' if a.window else 'test_speed.c' if a.speed else 'test_retest.c' if a.retest else 'test_auto_axis.c' if a.auto_axis else 'test_trial.c' if a.trial else 'test_product.c'))]
tag='ubsan' if a.sanitize else 'normal';exe=build/(('assessment_' if a.assessment else 'window_' if a.window else 'speed_' if a.speed else 'retest_' if a.retest else 'auto_axis_' if a.auto_axis else 'trial_' if a.trial else 'product_')+tag+'.exe');cmd+=['-o',str(exe)]
if a.sanitize:cmd+=['-fsanitize=undefined','-fsanitize-trap=undefined','-fno-sanitize-recover=all']
env=dict(os.environ,ZIG_LOCAL_CACHE_DIR=str(build/'zig-local-cache'),ZIG_GLOBAL_CACHE_DIR=str(root/'tests/live_analysis/build/zig-global-cache'))
subprocess.run(cmd,check=True,env=env);subprocess.run([str(exe)],check=True)
