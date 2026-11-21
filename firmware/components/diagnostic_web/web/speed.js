/* Display only. Paired report metrics come from the device, never re-solved here. */
(()=>{
  'use strict';
  const $=id=>document.getElementById(id),ns='http://www.w3.org/2000/svg';
  const colors=['#ad791c','#8050c5'],names=['原轮','校正后'];
  let last=null,lastKey='',rendered=null;
  function text(id,value){const e=$(id);if(e.textContent!==value)e.textContent=value;}
  function valid(s,p){
    if(!s||!p||p.available!==true||p.schema_version!==1||p.metric!=='RMS_of_per_turn_1x_peak_magnitudes_mps2'||
       p.session_id!==s.id||p.snapshot_revision!==s.revision||p.boot_id!==s.boot_id||p.axis!==s.config.axis||p.source!==s.source||
       !Number.isSafeInteger(p.generation)||p.generation<0||!Array.isArray(p.stages)||p.stages.length!==3)return false;
    return p.stages.every((a,i)=>{
      if(!a||!['empty','preview','frozen','invalid'].includes(a.state)||!Array.isArray(a.points)||a.points.length>14)return false;
      if(a.state==='preview'&&(s.historical||!['RECORDING','PROCESSING'].includes(s.state)||s.stage!==i))return false;
      if(a.state==='frozen'&&(!s.stages[i].captured||s.stages[i].data_integrity_valid!==true||a.capture_id!==s.stages[i].capture_id||a.epoch!==s.stages[i].epoch||
         a.start_us!==s.stages[i].start_us||a.end_us!==s.stages[i].end_us))return false;
      if(a.points.length&&!['preview','frozen'].includes(a.state))return false;
      const bins=new Set();let count=0;
      for(const b of a.points){if(!b||!Number.isInteger(b.lower_rpm)||b.lower_rpm<20||b.lower_rpm>=300||b.lower_rpm%20||b.upper_rpm!==b.lower_rpm+20||
          bins.has(b.lower_rpm)||!Number.isInteger(b.count)||b.count<1||b.count>96||
          ![b.rms,b.low,b.high].every(Number.isFinite)||b.low<0||b.high<b.low||b.rms<0||b.rms>1e6||
          b.rms<b.low-1e-5||b.rms>b.high+1e-5)return false;bins.add(b.lower_rpm);count+=b.count;}
      return count===a.used_turns&&count<=96&&(!count||(Number.isInteger(a.capture_id)&&a.capture_id>0&&Number.isInteger(a.epoch)&&a.epoch>0));
    });
  }

  function element(tag,attrs={},content){const e=document.createElementNS(ns,tag);for(const[k,v]of Object.entries(attrs))e.setAttribute(k,String(v));if(content!==undefined)e.textContent=content;return e;}
  function value(v){return Number.isFinite(v)?Number(v.toPrecision(3)).toString():'—';}
  function comparison(s){const c=s?.comparison,a=s?.stages?.[0],b=s?.stages?.[2];
    return c?.available===true&&Number.isFinite(c.original_peak_mps2)&&c.original_peak_mps2>0&&Number.isFinite(c.retest_peak_mps2)&&c.retest_peak_mps2>=0&&
      Number.isFinite(c.improvement_percent)&&c.matched_turns>=3&&a?.captured&&b?.captured&&a.data_integrity_valid===true&&b.data_integrity_valid===true&&
      a.epoch===b.epoch&&a.capture_id>0&&b.capture_id>0&&b.start_us>a.end_us&&Number.isFinite(c.lower_rpm??s.result.lower_rpm)&&Number.isFinite(c.upper_rpm??s.result.upper_rpm);
  }
  function model(s,dirty){
    if(!s||dirty)return null;const p=s.speed_profile,okay=valid(s,p),c=s.comparison,paired=comparison(s);
    if(s.stage===2&&s.state==='NO_RESULT'&&!paired)return null;
    const lower=c?.lower_rpm??s.result.lower_rpm,upper=c?.upper_rpm??s.result.upper_rpm;
    if(paired&&!$('pSpeedFull').checked){return {kind:'paired',series:[c.original_peak_mps2,c.retest_peak_mps2].map(v=>[{lower_rpm:lower,upper_rpm:upper,rms:v,count:c.matched_turns}]),preview:false,paired:true,legacy:!okay};}
    if(!okay)return null;
    const a=p.stages[0],b=p.stages[2],filter=x=>x.count>=3&&($('pSpeedFull').checked||!s.result.calculation_valid||x.lower_rpm>=s.result.lower_rpm&&x.upper_rpm<=s.result.upper_rpm);
    const base=a.points.filter(filter),after=b.points.filter(filter),common=new Set(base.filter(x=>after.some(y=>y.lower_rpm===x.lower_rpm)).map(x=>x.lower_rpm));
    const running=s.stage===2&&['ARMED','RECORDING','PROCESSING'].includes(s.state);
    if(!common.size&&running)return {kind:'bins',series:[base,[]],preview:true,paired:false};
    return {kind:'bins',series:[base.filter(x=>common.has(x.lower_rpm)),after.filter(x=>common.has(x.lower_rpm))],preview:b.state==='preview',paired:false};
  }
  function drawPaired(svg,series,maximum){
    const start=28,trackWidth=544;
    svg.append(element('rect',{x:0,y:13,width:600,height:311,rx:25,fill:'#faf7fd'}));
    svg.append(element('text',{x:start,y:52,class:'speed-unit'},'同速 1× 振动 · m/s²'));
    series.forEach(([bin],index)=>{
      const y=117+index*108,color=colors[index];
      const labelColor=index===0?'#81580f':'#6434a3';
      const row=element('g',{'data-speed-stage':index===0?'0':'2'});
      row.append(
        element('text',{x:start,y:y-16,fill:labelColor,'font-size':27,'font-weight':650},names[index]),
        element('text',{x:start+trackWidth,y:y-16,fill:labelColor,'font-size':30,'font-weight':700,'text-anchor':'end'},value(bin.rms)),
        element('rect',{x:start,y,width:trackWidth,height:22,rx:8,fill:'#ebe2f1'}),
        element('rect',{x:start,y,width:trackWidth*bin.rms/maximum,height:22,rx:8,fill:color,'data-speed-bar':''})
      );
      row.append(element('title',{},names[index]+' '+value(bin.rms)+' m/s²'));
      svg.append(row);
    });
    for(const fraction of [0,.5,1]){
      const label=fraction===0?'start':fraction===1?'end':'middle';
      svg.append(element('text',{x:start+trackWidth*fraction,y:293,'text-anchor':label,class:'speed-tick'},value(maximum*fraction)));
    }
  }
  function draw(m){const svg=$('pSpeedPlot');svg.replaceChildren();
    svg.dataset.stage0=JSON.stringify(m?.series[0]||[]);svg.dataset.stage1='[]';svg.dataset.stage2=JSON.stringify(m?.series[1]||[]);svg.dataset.kind=m?.kind||'empty';
    if(!m)return;const points=m.series.flat();if(!points.length)return;
    const min=Math.min(...points.map(x=>x.lower_rpm)),max=Math.max(...points.map(x=>x.upper_rpm));
    let top=Math.max(...points.map(x=>x.high??x.rms),1e-6)*1.18;const power=10**Math.floor(Math.log10(top));top=[1,1.25,1.5,2,2.5,5,10].find(v=>v*power>=top)*power;
    if(m.paired){drawPaired(svg,m.series,top);return;}
    const left=76,right=572,yt=35,bottom=265,x=r=>left+(r-min)/(max-min)*(right-left),y=v=>bottom-v/top*(bottom-yt);
    for(let i=0;i<=4;++i){const v=top*i/4,py=y(v);svg.append(element('path',{d:`M${left} ${py}H${right}`,stroke:'#eee8f3',fill:'none'}),element('text',{x:left-10,y:py+6,'text-anchor':'end',class:'speed-tick'},value(v)));}
    const ticks=Math.min(4,Math.max(1,Math.round((max-min)/20)));
    for(let i=0;i<=ticks;++i){const rpm=min+(max-min)*i/ticks;svg.append(element('text',{x:x(rpm),y:bottom+29,'text-anchor':'middle',class:'speed-tick'},String(Math.round(rpm))));}
    svg.append(element('text',{x:left,y:19,class:'speed-unit'},'1×强度 · m/s²'),element('text',{x:right,y:330,'text-anchor':'end',class:'speed-unit'},'转速 RPM'));
    m.series.forEach((bins,index)=>{const g=element('g',{'data-speed-stage':index===0?'0':'2'}),color=colors[index];let path='',previous=null;
      for(const b of bins.slice().sort((a,b)=>a.lower_rpm-b.lower_rpm)){const px=x((b.lower_rpm+b.upper_rpm)/2),py=y(b.rms),preview=index===1&&m.preview;
        if(m.kind==='bins'&&b.count>=3)g.append(element('rect',{x:x(b.lower_rpm)+2,y:y(b.high),width:Math.max(1,x(b.upper_rpm)-x(b.lower_rpm)-4),height:Math.max(1,y(b.low)-y(b.high)),fill:color,opacity:.1}));
        path+=(previous&&b.lower_rpm===previous.upper_rpm?'L':'M')+px+' '+py;
        g.append(element('path',{d:`M${x(b.lower_rpm)+5} ${py}H${x(b.upper_rpm)-5}`,stroke:color,'stroke-width':index===1?4:3,'stroke-linecap':'round','stroke-dasharray':preview?'7 6':'none'}));
        const dot=element('circle',{cx:px,cy:py,r:index===1?5:4,fill:preview?'white':color,stroke:color,'stroke-width':2});dot.append(element('title',{},`${names[index]} ${b.lower_rpm}–${b.upper_rpm} RPM：${value(b.rms)} m/s²`));g.append(dot);
        previous=b;
      }
      if(path)g.prepend(element('path',{d:path,fill:'none',stroke:color,'stroke-width':index===1?4:3,'stroke-dasharray':index===1&&m.preview?'7 6':'none'}));svg.append(g);
    });
  }
  function render(s,{dirty=false,uptime=0,uiFixture=false,connected=true}={}){
    last={s,options:{dirty,uptime,uiFixture,connected}};
    const wanted=!!s&&!dirty&&(s.stage===2&&(s.stages[2].capture_id||['ARMED','RECORDING','PROCESSING'].includes(s.state))||s.comparison.available);
    $('pSpeedCard').hidden=!wanted;
    const m=wanted?model(s,dirty):null,has=!!m?.series.flat().length,c=s?.comparison,paired=!!s&&comparison(s),variable=!!s?.retest_history?.group_variable_observed;
    $('pSpeedPlot').toggleAttribute('hidden',!has);$('pSpeedLegend').hidden=!has||m.paired;$('pSaveChart').disabled=!has||!paired;
    const failed=s?.stage===2&&s.state==='NO_RESULT'&&!paired;
    $('pSpeedFull').disabled=failed||!valid(s,s?.speed_profile);$('pShowPlacement').hidden=!s?.result.calculation_valid||!paired;
    $('pSpeedTag').hidden=!uiFixture&&s?.source!=='synthetic'&&!s?.historical&&!m?.preview;
    text('pSpeedTag',uiFixture?'界面测试':s?.source==='synthetic'?'合成 · 非实测':s?.historical?'历史结果':'复测预览');
    text('pSpeedChange',variable?'多次复测差异较大':paired?(c.improvement_percent>=0?'降低 ':'增大 ')+Math.abs(c.improvement_percent).toFixed(1).replace(/\.0$/,'')+'%':s?.state==='NO_RESULT'?'这次暂不可比较':m?.preview?'正在观察校正后':'等待校正后复测');
    $('pSpeedCard').classList.toggle('worse',paired&&c.improvement_percent<0);
    text('pSpeedBand',paired?`${c.lower_rpm??s.result.lower_rpm}–${c.upper_rpm??s.result.upper_rpm} RPM · ${c.matched_turns} 对整圈`:s?.result.calculation_valid?`参照速度 ${s.result.lower_rpm}–${s.result.upper_rpm} RPM`:'自动寻找共同转速范围');
    let hint=paired?'':has?'等待校正后数据':'等待复测结果';
    if(failed)hint=s.reason==='NO_COMMON_SPEED'?'共同转速段不足，记录已保存。':'本段未计算改善率，原因已保存。';
    if(m?.preview)hint=s.state==='PROCESSING'?'预览 · 正在分析':!connected?'预览 · 等待更新':'复测预览';
    text('pSpeedHint',hint);text('pSpeedMatched',paired?`原轮 ${value(c.original_peak_mps2)} → 校正后 ${value(c.retest_peak_mps2)} m/s²。默认图使用报告中的同一组配对汇总。`:failed?'该段记录可在会话报告中查看。':'预览只绘出原轮和复测共同覆盖的速度段；测完以整段复算为准。');
    text('pSpeedResponse',m?.legacy?'此图来自历史配对摘要，没有补造旧版完整转速曲线。':variable?'各次结果保留在历史中，当前响应差异较大，暂不据此继续配重。':'试重用于后台标定，本图只比较原轮和校正后。工程原型，尚未完成精度认证。');
    const key=JSON.stringify([s?.id,s?.revision,s?.boot_id,m]);if(key!==lastKey){lastKey=key;rendered=m;draw(m);}
    const legend=$('pSpeedLegend');if(!legend.childElementCount)names.forEach((name,i)=>{const el=document.createElement('span');el.className='speed-legend-item';el.style.setProperty('--series',colors[i]);el.textContent=name;legend.append(el);});
  }
  async function saveImage(){
    if(!last||!rendered||!comparison(last.s))return;const s=last.s,saved={change:$('pSpeedChange').textContent,band:$('pSpeedBand').textContent,source:s.source,fixture:last.options.uiFixture,firmware:s.firmware,id:s.id,historical:s.historical,kind:rendered.kind},svg=$('pSpeedPlot').cloneNode(true);
    const style=element('style',{},'svg{font-family:system-ui,sans-serif}.speed-tick,.speed-unit{fill:#685b77;font-size:19px}');svg.prepend(style);svg.removeAttribute('hidden');svg.setAttribute('xmlns',ns);svg.setAttribute('width','600');svg.setAttribute('height','340');
    const url=URL.createObjectURL(new Blob([new XMLSerializer().serializeToString(svg)],{type:'image/svg+xml;charset=utf-8'}));
    try{const img=new Image();await new Promise((resolve,reject)=>{img.onload=resolve;img.onerror=reject;img.src=url;});
      const canvas=document.createElement('canvas');canvas.width=1200;canvas.height=980;const ctx=canvas.getContext('2d');ctx.scale(2,2);ctx.fillStyle='#fff';ctx.fillRect(0,0,600,490);
      ctx.fillStyle='#382343';ctx.font='600 25px system-ui,sans-serif';ctx.fillText('车轮动平衡 · 原轮与校正后',32,42);ctx.fillStyle='#8050c5';ctx.font='22px system-ui,sans-serif';ctx.fillText(saved.change,32,79);ctx.fillStyle='#93849d';ctx.font='14px system-ui,sans-serif';ctx.fillText(saved.band,260,78);
      ctx.drawImage(img,0,99,600,340);ctx.fillStyle='#8d7a99';ctx.font='13px system-ui,sans-serif';ctx.fillText((saved.fixture?'界面测试 · 非实测':saved.source==='synthetic'?'合成演示 · 非实测':'工程记录 · 精度尚未认证')+' · 固件 '+saved.firmware,32,457);ctx.fillText('会话 '+saved.id+(saved.historical?' · 历史结果':'')+(saved.kind==='bins'?' · 全转速汇总，改善率取配对段':''),32,479);
      const blob=await new Promise(resolve=>canvas.toBlob(resolve,'image/png'));if(!blob)throw Error('PNG');const download=URL.createObjectURL(blob),a=document.createElement('a');a.href=download;a.download='wheelbalance_compare_'+saved.id+'.png';document.body.append(a);a.click();a.remove();setTimeout(()=>URL.revokeObjectURL(download),1000);
    }finally{URL.revokeObjectURL(url);}
  }
  $('pSpeedFull').addEventListener('change',()=>{if(last){lastKey='';render(last.s,last.options);}});
  $('pSaveChart').addEventListener('click',()=>saveImage().catch(()=>text('pSpeedHint','图片未保存成功，可以先导出会话报告。')));
  window.WheelSpeed={render,valid,saveImage};
})();
