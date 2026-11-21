"""Independent validation of bounded full-rate recorder CSV (no phase claims)."""
import csv
import hashlib
import io
import json
from pathlib import Path
import re

COLUMNS='record_type,capture_id,ingest_us,sequence,epoch,readout_us,count_begin_us,count_end_us,pending_frames,batch_index,raw_x,raw_y,raw_z,reference_us,has_period,resynced'.split(',')

QUARANTINE_REGISTRY = Path(__file__).with_name('capture_quarantine.json')


def capture_review(path, content, registry_path=None):
    """Review file identity, independently of transport/CRC completeness.

    Known bad bytes remain quarantined after copying or renaming the CSV.
    Missing reviews never certify sensor integrity or physical precision.
    """
    path = Path(path)
    fingerprint = hashlib.sha256(content).hexdigest()
    registry = json.loads(Path(registry_path or QUARANTINE_REGISTRY).read_text(encoding='utf-8-sig'))
    if not isinstance(registry, dict) or registry.get('schema') != 'wheelbalance_quarantine_v1' or not isinstance(registry.get('captures'), dict):
        raise ValueError('invalid capture quarantine registry')
    for fingerprint_key, entry in registry['captures'].items():
        if not re.fullmatch(r'[0-9a-f]{64}', fingerprint_key) or not isinstance(entry, dict) or not isinstance(entry.get('reason'), str) or not entry['reason']:
            raise ValueError('invalid capture quarantine entry')
    known = registry['captures'].get(fingerprint)
    review_path = path.with_suffix('.validity_review.json')
    review = None
    if review_path.exists():
        review = json.loads(review_path.read_text(encoding='utf-8-sig'))
        if not isinstance(review, dict) or type(review.get('sensor_integrity')) is not bool:
            raise ValueError('invalid sensor validity review: ' + str(review_path))
        if review.get('raw_csv_sha256') != fingerprint:
            raise ValueError('sensor validity review CSV hash mismatch: ' + str(review_path))
    return {
        'csv_sha256': fingerprint,
        'quarantined': known is not None or (review is not None and not review['sensor_integrity']),
        'known_bad_hash': known is not None,
        'sensor_integrity': False if known is not None else review['sensor_integrity'] if review is not None else None,
        'reason': known.get('reason', 'known bad capture') if isinstance(known, dict) else review.get('reason') if review else None,
        'review_path': str(review_path.resolve()) if review is not None else None,
        'registry_path': str(Path(registry_path or QUARANTINE_REGISTRY).resolve()),
    }


def load_capture_csv(path, allow_synthetic=False):
    """Shared fail-closed entry for analysis; byte validation stays forensic."""
    path = Path(path)
    content = path.read_bytes()
    review = capture_review(path, content)
    if review['quarantined']:
        raise ValueError('quarantined sensor data: ' + str(path) + '; ' + str(review['reason']))
    info = validate_csv(content, allow_synthetic)
    info.update(path=str(path.resolve()), csv_sha256=review['csv_sha256'], sensor_review=review)
    return content, info

def validate_csv(content,allow_synthetic=False):
    text=content.decode('utf-8') if isinstance(content,bytes) else content
    if not text.endswith('# end=1\n'):
        raise ValueError('missing complete export footer')
    meta={};rows=[]
    for line in text.splitlines():
        if line.startswith('# '):
            key,sep,value=line[2:].partition('=')
            if not sep or key in meta:raise ValueError('invalid/duplicate metadata')
            meta[key]=value
        else:rows.append(line)
    allowed=('live_device','synthetic_fixture') if allow_synthetic else ('live_device',)
    if meta.get('schema')!='wheelbalance_raw_v1' or meta.get('source') not in allowed or meta.get('measurement_qualified')!='0':
        raise ValueError('unsupported source or qualification')
    if not re.fullmatch('[0-9a-f]{16}',meta.get('boot_id','')):raise ValueError('invalid boot identity')
    if meta.get('time_basis')!='host_fifo_observations_not_exact_sample_times':raise ValueError('unknown timing semantics')
    if not rows or next(csv.reader([rows[0]]))!=COLUMNS:raise ValueError('invalid columns')
    capture=int(meta['capture_id']);start=int(meta['start_us']);end=int(meta['end_us']);faults=int(meta['faults'])
    if not 0<capture<=0xffffffff or start<=0 or end<start:raise ValueError('invalid window identity/time')
    counts={'SAMPLE':0,'REFERENCE':0};last_sample=None;last_ref=None;issues=set();reference_section=False
    first_readout=last_readout=None;first_sequence=last_sequence=None
    for row in csv.reader(rows[1:]):
        if len(row)!=16 or row[0] not in counts:raise ValueError('invalid row shape')
        kind=row[0]
        if int(row[1])!=capture:raise ValueError('mixed capture IDs')
        ingest,seq=int(row[2]),int(row[3]);counts[kind]+=1
        if not start<=ingest<=end:issues.add('outside_admission_window')
        if seq<=0:issues.add('zero_sequence')
        if kind=='SAMPLE':
            if reference_section or any(row[13:]):raise ValueError('invalid sample row order/fields')
            epoch,readout,begin,finish,pending,index=map(int,row[4:10]);raw=list(map(int,row[10:13]))
            if any(v<-32768 or v>32767 for v in raw):raise ValueError('raw acceleration outside int16')
            if any(v>=32760 or v<=-32760 for v in raw):issues.add('saturation')
            if epoch<=0 or pending<=0 or not 0<=index<pending:issues.add('sample_metadata')
            if not 0<begin<=finish<=readout<=ingest:issues.add('sample_time_order')
            if last_sample:
                prev_seq,prev_epoch,prev_ingest,prev_readout=last_sample
                if seq!=prev_seq+1:issues.add('sample_sequence')
                if epoch!=prev_epoch:issues.add('epoch_changed')
                if ingest<prev_ingest or readout<prev_readout:issues.add('sample_time_reverse')
            else:first_readout=readout;first_sequence=seq
            last_sample=(seq,epoch,ingest,readout);last_readout=readout;last_sequence=seq
        else:
            reference_section=True
            if any(row[4:13]):raise ValueError('invalid reference blank fields')
            observed,period,resynced=map(int,row[13:])
            if period not in (0,1) or resynced not in (0,1):raise ValueError('invalid reference booleans')
            if not 0<observed<=ingest:issues.add('reference_time_order')
            if last_ref:
                if seq!=last_ref[0]+1:issues.add('reference_sequence')
                if observed<=last_ref[1] or ingest<last_ref[2]:issues.add('reference_time_reverse')
            last_ref=(seq,observed,ingest)
    for key,kind,maximum in [('samples','SAMPLE',64000),('references','REFERENCE',1024)]:
        if counts[kind]!=int(meta[key]) or counts[kind]!=int(meta['exported_'+key]) or counts[kind]>maximum:
            raise ValueError('row count/footer mismatch')
    if not counts['SAMPLE']:issues.add('empty_samples')
    integrity=meta.get('integrity_ok')=='1'
    if integrity and (faults or int(meta['queue_drops']) or issues):raise ValueError('false integrity claim')
    hz=None
    if first_readout is not None and last_readout>first_readout:
        hz=(counts['SAMPLE']-1)*1e6/(last_readout-first_readout)
    return {'capture_id':capture,'boot_id':meta['boot_id'],'firmware':meta.get('firmware'),
        'samples':counts['SAMPLE'],'references':counts['REFERENCE'],'faults':faults,
        'queue_drops':int(meta['queue_drops']),'integrity_ok':integrity,'data_issues':sorted(issues),
        'first_sequence':first_sequence,'last_sequence':last_sequence,
        'readout_span_rate_hz':hz,'measurement_qualified':False,'metadata':meta}
