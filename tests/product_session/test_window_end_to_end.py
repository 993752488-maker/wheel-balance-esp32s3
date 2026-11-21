"""500 Hz numerical FIFO -> real clock/selector/window -> actual H/Uc/retest.

All input is synthetic and explicitly labelled. Does not touch a device.
"""
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import unittest

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tools'))
from native_product_session import build


def dataset(stage,rpm=100,alpha=0,duration=18,disturbance=True,ripple=0):
    start=1_000_000+stage*160_000_000;w=rpm*2*math.pi/60
    ramp=2 if disturbance else 0;gain=w*.12
    def angle(t):
        if ramp and t<ramp:return (w-gain*ramp)*t+.5*gain*t*t
        v=t-ramp;mod=w*.35
        return (w*ramp-.5*gain*ramp*ramp)+w*v+.5*alpha*v*v+ripple*w*(1-math.cos(mod*v))/mod
    z=complex(.05358983848622454,-.8928203230275509)
    z=z+complex(1,-.5) if stage==1 else z*.2 if stage==2 else z
    samples=[]
    for i in range(round(duration*500)):
        t=i*.002;theta=angle(t);v=z.real*math.cos(theta)-z.imag*math.sin(theta)
        # Non-synchronous hand/frame disturbance and a short initial ramp.
        if disturbance and .8<t<4:v+=2*math.exp(-2*(t-.8))*math.sin(2*math.pi*1.3*(t-.8))
        newest=(i//10*10+9)*2000+start;midpoint=newest+1000
        raw=round(v*8192/9.80665)
        samples.append(f'S,{stage*70000+i+1},1,{midpoint-100},{midpoint+100},10,{i%10},0,{raw},8192\n')
    refs=[];k=0
    while 2*math.pi*k<angle(duration-.002):
        low=0.;high=duration
        for _ in range(45):
            mid=(low+high)/2
            if angle(mid)<2*math.pi*k:low=mid
            else:high=mid
        refs.append(f'R,{stage*1100+k+1},{start+round((low+high)*.5e6)},{int(k>0)}\n');k+=1
    return f'D,{stage+1},1,{len(samples)},{len(refs)},{start},{start+round(duration*1e6)},1\n'+''.join(samples+refs)+'E\n'


class WindowPipeline(unittest.TestCase):
    def test_steady_coast_and_long_low_speed(self):
        exe,digest=build();cases=[]
        for name,rpm,alpha,duration,ripple in [('steady',100,0,18,0),('steady_controller_ripple',100,0,22,.015),('coast',180,-.3,18,0),('long_low_speed',20,0,58,0),('whole_coast110seconds',180,-.1,110,0)]:
            data='C,5,100,0,100,1,1\n'+''.join(dataset(i,rpm,alpha,duration,ripple=ripple) for i in range(3))
            proc=subprocess.run([str(exe),'--synthetic-window'],input=data,capture_output=True,text=True,timeout=45)
            self.assertEqual(proc.returncode,0,name+': '+proc.stderr)
            out=json.loads(proc.stdout)
            self.assertEqual(out['source'],'synthetic');self.assertFalse(out['recommendation_valid'])
            self.assertTrue(out['result']['calculation_valid'],out['result'])
            self.assertAlmostEqual(out['result']['mass_g'],4,delta=.06)
            self.assertAlmostEqual(out['result']['angle_deg'],120,delta=1.5)
            self.assertTrue(out['comparison']['available'])
            self.assertAlmostEqual(out['comparison']['improvement_percent'],80,delta=1)
            for v in out['capture_window_stages']:
                self.assertGreaterEqual(v['usable_turns'],3)
                self.assertGreater(v['skipped_turns'],0)
                self.assertGreater(v['first_usable_us'],v['start_us']+1_500_000)
            if name=='long_low_speed':self.assertGreater(out['stages'][0]['samples'],16064)
            cases.append({'case':name,'input_source':'explicit_numerical_FIFO_fixture','native_source_sha256':digest,'result':out})
        folder=Path(os.environ.get('WHEEL_TEST_OUTPUT',ROOT/'tests/product_session/build/window-end-to-end'));folder.mkdir(exist_ok=True,parents=True)
        (folder/'window-end-to-end.json').write_text(json.dumps({'scope':'software_only_no_physical_balance_validation','cases':cases},indent=2)+'\n',encoding='utf-8')


if __name__=='__main__':unittest.main()
