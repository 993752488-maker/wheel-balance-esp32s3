"""Build the actual C device analysis path; never accesses serial hardware."""
import hashlib
import csv
import json
import math
import os
import shutil
from pathlib import Path
import subprocess
from raw_data import load_capture_csv

ROOT=Path(__file__).resolve().parents[1]
def build():
    parts=ROOT/'firmware/components';folder=ROOT/'tests/product_session/build';folder.mkdir(exist_ok=True)
    names=['product_session','balance_core','coherent_average','coast_selection','coast_analysis','sync_analysis','live_analysis']
    sources=[parts/p for p in ['product_session/product_engine.c','product_session/product_assessment.c','product_session/product_job.c','product_session/product_window.c','product_session/product_speed.c','product_session/product_auto_axis.c','balance_core/balance_core.c','coherent_average/coherent_average.c',
        'coast_selection/coast_selection.c','coast_analysis/coast_analysis.c','sync_analysis/sync_analysis.c','live_analysis/live_analysis_core.c']]
    sources+=[ROOT/'tools/native/product_session_cli.c'];inputs=list(sources)
    for name in names:inputs+=list((parts/name/'include').glob('*.h'))
    digest=hashlib.sha256(b''.join(p.relative_to(ROOT).as_posix().encode()+p.read_bytes() for p in inputs)).hexdigest()
    exe=folder/'product_replay.exe';manifest=folder/'product_replay_source.json'
    if not(exe.exists() and manifest.exists() and json.loads(manifest.read_text())['sha256']==digest):
        cc=shutil.which('zig') or next(iter(sorted((ROOT/'.host_tools/zig').glob('*/zig.exe'),reverse=True)),None)
        if not cc:
            raise RuntimeError('Install Zig and add it to PATH.')
        cmd=[str(cc),'cc','-std=c11','-O2','-Wall','-Wextra','-Werror','-pedantic']
        for name in names:cmd+=['-I',str(parts/name/'include')]
        cmd += [str(p) for p in sources]+['-o',str(exe)]
        env=dict(os.environ,ZIG_LOCAL_CACHE_DIR=str(folder/'zig-local-cache'),ZIG_GLOBAL_CACHE_DIR=str(ROOT/'tests/live_analysis/build/zig-global-cache'))
        subprocess.run(cmd,env=env,check=True);manifest.write_text(json.dumps({'sha256':digest})+'\n',encoding='utf-8')
    return exe,digest
def demo():
    exe,digest=build();p=subprocess.run([str(exe),'--demo'],capture_output=True,text=True,check=True)
    result=json.loads(p.stdout);result['result']['assessment']=result.get('assessment');result['native_source_sha256']=digest;return result

def _target_field(target_rpm):
    target_rpm=float(target_rpm)
    if not math.isfinite(target_rpm) or target_rpm<=0:
        raise ValueError('invalid timed target RPM')
    return f',{target_rpm}'

def profile(path,axis=1,*,automatic=False,settling=False,target_rpm=None):
    """Recorded input through actual C clock, selector and product filter.

    A supplied target_rpm enables the device's timed window selection.
    Does not call a balancing solve or invent an influence coefficient.
    """
    if axis not in (0,1,2):raise ValueError('invalid axis')
    path=Path(path).resolve();raw,info=load_capture_csv(path)
    if not info['integrity_ok'] or info['faults'] or info['queue_drops'] or info['data_issues']:
        raise ValueError('invalid capture cannot enter profile replay')
    if info['metadata'].get('accel_lsb_per_g')!='8192':raise ValueError('unsupported acceleration scale')
    rows=list(csv.DictReader(s for s in raw.decode('utf-8').splitlines() if not s.startswith('#')))
    epochs={int(r['epoch']) for r in rows if r['record_type']=='SAMPLE'}
    if len(epochs)!=1:raise ValueError('mixed acquisition epochs')
    epoch=epochs.pop();meta=info['metadata']
    target='' if target_rpm is None else _target_field(target_rpm)
    lines=[f'P,{axis}\n',f"D,{info['capture_id']},{epoch},{info['samples']},{info['references']},{meta['start_us']},{meta['end_us']},1{target}\n"]
    for r in rows:
        keys=['sequence','epoch','count_begin_us','count_end_us','pending_frames','batch_index','raw_x','raw_y','raw_z'] if r['record_type']=='SAMPLE' else ['sequence','reference_us','has_period']
        lines.append(('S' if r['record_type']=='SAMPLE' else 'R')+','+','.join(r[k] for k in keys)+'\n')
    lines.append('E\n');exe,digest=build()
    options=['--timed'] if target else []
    mode='--profile-all' if automatic else '--profile'
    if settling and not target:mode+='-window'
    p=subprocess.run([str(exe),*options,mode],input=''.join(lines),capture_output=True,text=True,check=True,timeout=45)
    file=path.relative_to(ROOT) if path.is_relative_to(ROOT) else path
    result=json.loads(p.stdout);result.update(native_source_sha256=digest,provenance={
        'file':file.as_posix(),'sha256':info['csv_sha256'],'boot_id':info['boot_id'],
        'firmware':info['firmware'],'epoch':epoch,'capture_id':info['capture_id'],'start_us':int(meta['start_us']),'end_us':int(meta['end_us'])})
    if target:result.update(capture_mode='timed',replay_target_rpm=float(target[1:]))
    return result

def replay(paths,config=(10,270,0,270,1,1),comparison_kind='restored_baseline_NOT_actual_correction_verification',*,pair_only=False,repeat_only=False,automatic=False,target_rpms=None):
    """Replay recorded windows; pass one target RPM per stage for timed mode."""
    if pair_only and repeat_only:raise ValueError('replay modes must be exclusive')
    if len(paths)!=(4 if repeat_only else 2 if pair_only else 3):raise ValueError('two stages for pair-only;three for correction comparison;four for repeat observation')
    targets=['']*len(paths) if target_rpms is None else [_target_field(rpm) for rpm in target_rpms]
    if len(targets)!=len(paths):raise ValueError('timed replay requires a target RPM for each capture')
    lines=['C,'+','.join(map(str,config))+'\n'];inputs=[];contexts=[]
    for p,target in zip(paths,targets):
        path=Path(p).resolve()
        raw,info=load_capture_csv(path)
        if not info['integrity_ok'] or info['faults'] or info['queue_drops'] or info['data_issues']:
            raise ValueError('invalid capture cannot enter product replay: '+str(path))
        rows=list(csv.DictReader(s for s in raw.decode('utf-8').splitlines() if not s.startswith('#')))
        epochs={int(r['epoch']) for r in rows if r['record_type']=='SAMPLE'}
        if len(epochs)!=1:raise ValueError('mixed acquisition epochs')
        epoch=epochs.pop();meta=info['metadata'];contexts.append((info['boot_id'],info['firmware'],epoch))
        lines.append(f"D,{info['capture_id']},{epoch},{info['samples']},{info['references']},{meta['start_us']},{meta['end_us']},1{target}\n")
        for r in rows:
            keys=['sequence','epoch','count_begin_us','count_end_us','pending_frames','batch_index','raw_x','raw_y','raw_z'] if r['record_type']=='SAMPLE' else ['sequence','reference_us','has_period']
            lines.append(('S' if r['record_type']=='SAMPLE' else 'R')+','+','.join(r[k] for k in keys)+'\n')
        lines.append('E\n')
        file=path.relative_to(ROOT) if path.is_relative_to(ROOT) else path
        inputs.append({'file':file.as_posix(),'sha256':info['csv_sha256'],'boot_id':info['boot_id'],'firmware':info['firmware'],'epoch':epoch})
        if target:inputs[-1]['replay_target_rpm']=float(target[1:])
    if len(set(contexts))!=1:raise ValueError('different recorded boot/firmware/epoch contexts')
    options=['--timed'] if target_rpms is not None else []
    if repeat_only:options.append('--auto-repeat' if automatic and options else '--repeat')
    elif automatic:options.append('--auto-pair-window' if pair_only else '--auto-window')
    elif pair_only:options.append('--pair')
    exe,digest=build();p=subprocess.run([str(exe),*options],input=''.join(lines),capture_output=True,text=True,check=True,timeout=45)
    result=json.loads(p.stdout);result.update(native_source_sha256=digest,inputs=inputs,third_stage_kind=comparison_kind,
        execution='offline_host_replay_shared_device_C',new_hardware_test=False)
    if target_rpms is not None:result['capture_mode']='timed'
    return result
if __name__=='__main__':print(json.dumps(demo(),indent=2))
