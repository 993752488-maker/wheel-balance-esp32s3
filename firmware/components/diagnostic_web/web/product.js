/* Device-owned product session. UI previews use explicitly marked fixtures. */
(() => {
  'use strict';
  const $=id=>document.getElementById(id);
  const deviceLink=window.WheelDeviceLink;
  const reconnectHint='会话暂未更新，正在自动重连；操作不会自动重复提交。';
  let pollTimer=null,pollPromise=null,refreshRequested=false,pollFailures=0;
  const ui={data:null,viewId:null,busy:false,pendingRevision:null,pendingUntil:0,online:false,dirty:false,initialized:false,lastSuccess:0,pollSequence:0,acceptedSequence:0,confirmationContext:null,planContext:null,planDirty:false,customCorrectionRadius:false,showPlacement:false};
  const busyStates=['ARMED','RECORDING','PROCESSING'];
  const reasons={NONE:'',SENSOR_OFFLINE:'传感器离线，请检查供电与接线。',DATA_FAULT:'本段数据有采集异常，已结束并保存原因。',REFERENCE_INVALID:'光电参考异常，请检查白标与探头。',TOO_FEW_TURNS:'本段可用整圈不足，记录已保存。',NO_COMMON_SPEED:'两段没有足够的共同速度数据，本次未生成估计。',WEAK_TRIAL_RESPONSE:'本段试重差量仍较弱，暂未形成可用估计。',INCONSISTENT_RESPONSE:'原轮、试重或两者差量的方向不够一致，暂不能确定配重位置。',SETUP_CHANGED:'采集状态改变，请新建会话重新测量。',NUMERIC_ERROR:'本次计算不可用，记录与原因已保存。',DEVICE_BUSY:'设备正被其他记录或下载占用，请稍后重试。',WAIT_TIMEOUT:'等待转动已超时，可重新开始。',BASELINE_NEAR_ZERO:'原始振动接近零，改善率无法比较。',STORAGE_ERROR:'历史保存失败，可先导出本次报告。'};
  const fieldMap={pVehicle:'vehicle',pMass:'trial_mass_g',pRadius:'trial_radius_mm',pTrialAngle:'trial_angle_deg',pCorrectionRadius:'correction_radius_mm',pAxis:'axis',pDirection:'direction'};
  reasons.ESTIMATE_CONFLICT='克数待确认。停稳后取下试重，导出报告。';
  reasons.CAPTURE_LIMIT='达到两分钟上限，原始记录已保存。';
  function session(){return ui.data&&ui.data.session;}
  function goodSession(s){return !!(s&&Number.isInteger(s.id)&&s.id>0&&Number.isInteger(s.revision)&&s.revision>0&&s.config&&Array.isArray(s.stages)&&s.stages.length===3&&s.result&&s.comparison&&['device','synthetic'].includes(s.source)&&typeof s.historical==='boolean');}
  function geometry(angle,direction){if(!Number.isFinite(angle)||!['cw','ccw'].includes(direction))return null;const a=((angle%360)+360)%360,sign=direction==='cw'?1:-1,t=a*sign*Math.PI/180;return {rotation:a*sign,x:120+88*Math.sin(t),y:120-88*Math.cos(t)};}
  function text(id,value){
    const node=$(id);
    if(node.textContent!==value)node.textContent=value;
  }
  function dialArc(rotation){
    if(!rotation)return '';
    const radius=60,rad=rotation*Math.PI/180;
    const x=(120+radius*Math.sin(rad)).toFixed(2),y=(120-radius*Math.cos(rad)).toFixed(2);
    return 'M120 60 A60 60 0 '+(Math.abs(rotation)>180?1:0)+' '+(rotation>0?1:0)+' '+x+' '+y;
  }
  function guide(s){return s.trial_guide&&s.trial_guide.available===true?s.trial_guide:null;}
  function grams(value){return Number(value.toFixed(2)).toString();}
  function setForm(c){for(const [id,key] of Object.entries(fieldMap))$(id).value=String(c[key]);const g=guide(session()),flow=session()?.capture_flow;$('pAxis').value=g?.axis_selection?.mode==='manual'?String(c.axis):'auto';$('pPiece').value=g?g.piece_mass_g:5;$('pTrialMode').value=g&&g.plan_source==='manual'?'manual':'guided';$('pVehicleType').value=flow?.vehicle_type||'bicycle';$('pCaptureMode').value=flow?.mode||'coast';$('pCaptureSeconds').value=flow?.duration_seconds||30;ui.customCorrectionRadius=Math.abs(c.correction_radius_mm-c.trial_radius_mm)>1e-8;ui.initialized=true;}
  function nextAction(s){
    if(s.historical)return 'new';
    if(s.state==='RECORDING')return 'finish';
    if(s.result.can_start_retest)return s.source==='synthetic'?'demo_retest':'retest';
    if(s.result.calculation_valid)return 'export';
    if(s.state==='NO_RESULT'&&!(s.stage===1&&guide(s)?.retry_available&&guide(s)?.editable))return 'export';
    if(!s.stages[0].captured)return 'original';
    return s.source==='synthetic'?'demo_trial':'trial';
  }
  function renderTrialTask(s,visible,connected,locked){
    const g=guide(s),mass=s.config.trial_mass_g,piece=g?g.piece_mass_g:5;
    const key=[ui.data.boot_id,s.id,mass,g?.attempt_count||0].join(':');
    if(ui.planContext!==key){$('pTrialActualMass').value=mass;ui.planContext=key;ui.planDirty=false;}
    $('pTrialTask').hidden=!visible;
    const ended=g&&!g.editable&&g.attempt_count>0;
    const last=g?.attempts?.at(-1);
    text('pTrialTaskSource',ended?'试重已结束':g?.plan_source==='response_projection'?'建议试重':g?.plan_source==='starter_policy'?'两片起步探测':'指定试重');
    text('pTrialTaskMass',ended?grams(last?.mass_g??mass)+' g':'额外添加 '+grams(mass)+' g');
    const count=mass/piece,whole=Number.isFinite(count)&&Math.abs(count-Math.round(count))<1e-7;
    text('pTrialTaskPieces',whole?Math.round(count)+' 片 × '+grams(piece)+' g':'每片 '+grams(piece)+' g');
    const direction=s.config.direction==='cw'?'顺时针':'逆时针';
    const angle=s.config.trial_angle_deg;
    const mark=geometry(angle,s.config.direction);
    $('pTrialMark').toggleAttribute('hidden',!mark||!!ended);
    if(mark)$('pTrialMark').setAttribute('transform','rotate('+mark.rotation+' 120 120)');
    text('pTrialTaskLocation',(angle===0?'白标径向':direction+' '+angle+'°')+' · '+s.config.trial_radius_mm+' mm');
    text('pTrialTaskReplace',ended?'停稳后取下试重。':
      last&&Math.abs(last.mass_g-mass)>1e-8?'停稳后取下上一组 '+grams(last.mass_g)+'g，再贴本次总量。':
      last?'本次仍用 '+grams(mass)+'g，无需加贴。':'停稳后加贴试重，留出白标。');
    let advice='';
    if(g?.attempt_count&&!s.result.calculation_valid){
      if(g.next_reason==='RESPONSE_PROJECTION')advice='上次变化偏弱，建议换成 '+grams(g.suggested_mass_g)+'g。';
      else if(g.next_reason==='ATTEMPT_LIMIT')advice='试重已结束，可导出报告。';
      else if(g.next_reason==='HARD_FAULT')advice='数据异常，请保存报告。';
      else advice='当前数据不足以计算可信的增重建议。';
    }
    text('pTrialAdvice',advice);
    const suggestion=Number.isFinite(g?.suggested_mass_g)&&Math.abs(g.suggested_mass_g-mass)>1e-8;
    $('pUseTrialAdvice').hidden=!suggestion;
    text('pUseTrialAdvice',suggestion?'改为总共 '+grams(g.suggested_mass_g)+'g':'采用下一组建议');
    $('pUseTrialAdvice').disabled=!connected||locked||!g?.editable;
    $('pApplyTrialMass').disabled=!connected||locked||!g?.editable||!ui.planDirty;
    $('pTrialActualMass').disabled=locked||!g?.editable;
    $('pTrialInstalled').closest('label').hidden=!visible||!!ended;
    $('pTrialActualMass').closest('details').hidden=!!ended;
    text('pTrialInstalledText','已停稳，'+grams(mass)+'g 试重已贴牢');
  }
  function retestHistory(s){return s?.retest_history?.available?s.retest_history:null;}
  function repeatText(entry){const r=entry?.repeat;if(!r||r.state==='NOT_CHECKED')return '尚无同状态重复性观察。';
    if(r.state==='NOT_COMPARABLE')return '同状态记录暂不可比较：'+(reasons[r.reason]||'缺少可比数据')+'。';
    return '与记录 #'+r.previous_capture_id+' 的同状态幅值相差 '+Math.abs(r.amplitude_change_percent).toFixed(1)+'%。'+
      (r.state==='VARIABLE_RESPONSE'?'响应差异较大，暂不据此继续配重。':'这是本次观察，不代表精度已验证。');}
  function renderRetests(s){const h=retestHistory(s),box=$('pRetestRows');box.replaceChildren();
    for(const e of (h?.entries||[]).slice().reverse()){const row=document.createElement('div'),title=document.createElement('strong'),value=document.createElement('p'),note=document.createElement('small');
      row.className='product-retest-row';title.textContent=(e.legacy?'旧版摘要':('第 '+e.ordinal+' 次复测'))+' · 记录 #'+e.capture_id;
      value.textContent=e.comparison_available?'相对原轮：1×指标'+(e.improvement_percent>=0?'改善 ':'增大 ')+Math.abs(e.improvement_percent).toFixed(1)+'%':('本次不可比较：'+(reasons[e.comparison_reason]||reasons[e.reason]||'数据不足'));
      note.textContent=(e.condition==='unchanged'?'配重未变。':e.condition==='correction_changed'?'校正块已调整，重新分组。':e.legacy?'旧版只保存最后一段，其他记录无法恢复。':'本组首次复测。')+repeatText(e);
      row.append(title,value,note);box.append(row);}
    text('pRetestRetention',h?.entries?.length?'保留最近 '+h.capacity+' 次复测摘要'+(h.dropped_count?'；较早 '+h.dropped_count+' 次已超出保存数量':'')+'。完整原始CSV仅保留最新一段，请及时下载。'+(h.legacy_count_unknown?'旧版总次数未知。':''):'尚无复测记录。完整原始CSV仅保留最新一段。');
  }
  function estimateNote(s){
    const q=s.result?.assessment;
    if(s.result?.reason==='ESTIMATE_CONFLICT')return '候选配重不一致，当前记录仅供分析。';
    if(q?.state==='ASSESSMENT_MISSING')return '分段评估未能恢复，计算记录已保留，暂不显示贴块位置。';
    if(q?.state==='LEGACY_NOT_ASSESSED')return '旧版历史结果，未做新版多段评估；精度未认证。';
    if(!q?.available)return '估计精度尚未认证；只用于本次安装的受控复测。';
    if(q.state==='VARIABLE_ESTIMATES'){
      const cause=q.warning_flags&32?'不同测量轴存在分歧':q.warning_flags&16?'不同速度段存在分歧':'同一速度段前后存在变化';
      const range=q.valid_bands>1&&Number.isFinite(q.observed_mass_min_g)&&Number.isFinite(q.observed_mass_max_g)?'（分段约 '+q.observed_mass_min_g.toFixed(1)+'–'+q.observed_mass_max_g.toFixed(1)+'g）':'';
      return cause+range+'。本次为粗略工程估计，须复测确认；范围不是误差保证。';
    }
    return (q.joint_used?'已联合 '+q.joint_bands+' 个速度段计算。':'当前采用单一可比速度段。')+(q.state==='OBSERVATIONS_CLOSE'?'分段观察接近，仍需实物复测。':'支持数据有限，结果待复测。');
  }
  function render(){
    const s=session(),connected=ui.online&&performance.now()-ui.lastSuccess<4000;
    if(!goodSession(s)){text('pHeading',connected?'产品会话暂不可用':'等待连接设备');text('pHint','连接设备热点后，这里会显示测量步骤。');$('pPrimary').disabled=true;if(window.WheelSpeed)window.WheelSpeed.render(null);return;}
    if(!ui.initialized&&ui.pendingRevision===null)setForm(s.config);
    const failedRetest=s.stage===2&&s.state==='NO_RESULT';
    $('productApp').dataset.view=!ui.dirty&&s.comparison.available?'comparison':s.result.calculation_valid&&!failedRetest?'placement':'measurement';
    const running=busyStates.includes(s.state),locked=ui.busy||ui.pendingRevision!==null||ui.data.export_busy===true;
    const synthetic=s.source==='synthetic',history=s.historical,action=nextAction(s),retests=retestHistory(s);
    const timed=s.capture_flow?.mode==='timed',duration=s.capture_flow?.duration_seconds||30;
    $('pVehiclePicker').hidden=synthetic||history||running||s.stages[0].captured&&!ui.dirty;
    $('pVehicleType').disabled=running||locked;
    for(const field of document.querySelectorAll('#pSetup input,#pSetup select,#pSetup button'))field.disabled=running||locked;
    $('pCaptureSeconds').disabled=running||locked||$('pCaptureMode').value!=='timed';
    text('pCaptureModeHint',$('pCaptureMode').value==='timed'?'匀速记录 '+$('pCaptureSeconds').value+' 秒':'等车轮转完');
    const latestRetest=retests?.entries?.at(-1),variable=!!retests?.group_variable_observed;
    const confirmationContext=[ui.data.boot_id,s.id,s.state,s.stages[1].capture_id,s.stages[2].capture_id,s.config.trial_mass_g].join(':');
    if(ui.confirmationContext!==confirmationContext){$('pTrialRemoved').checked=false;$('pTrialInstalled').checked=false;$('pRetestCondition').value='';ui.confirmationContext=confirmationContext;}
    const conflict=s.result.reason==='ESTIMATE_CONFLICT';
    const noCorrection=s.result.no_correction_required===true;
    const checkVisible=action==='retest'&&!ui.dirty&&!running;
    $('pRetestCheck').hidden=!checkVisible;
    $('pRetestConditionBox').hidden=!checkVisible||!retests?.entries?.length;
    text('pRetestCheckText',s.stage===2?'已核对当前校正块，试重已移除':'已移除 '+s.config.trial_mass_g+'g 试重，并按图贴好校正块');
    const planned=!synthetic&&!history&&!running&&!ui.dirty;
    const trialTask=planned&&s.stages[0].captured&&!s.result.calculation_valid&&(!guide(s)||guide(s).editable||guide(s).attempt_count>0);
    renderTrialTask(s,trialTask,connected,locked);$('pTrialPlan').hidden=!planned||trialTask||s.stages[0].captured;
    $('pManualMassField').hidden=$('pTrialMode').value!=='manual';
    text('pRadiusNote',ui.customCorrectionRadius?'高级校正半径当前为 '+$('pCorrectionRadius').value+' mm。':'试重和校正默认使用同一贴块半径。');
    text('pTrialPlan',s.state==='NO_RESULT'?'记录已保存。':'');
    text('pSource',[ui.data.ui_fixture?'界面测试':'',history?'历史':'',synthetic?'合成':''].filter(Boolean).join(' · '));
    $('pSource').hidden=!ui.data.ui_fixture&&!history&&!synthetic;
    text('pVehicleName',s.config.vehicle||'本次测量');
    let heading='从原轮开始',hint='保留原有配重，先测原轮。';
    if(s.stages[0].captured){heading='原轮已保存';hint=trialTask?'':'准备试重。';}
    if(s.result.calculation_valid){heading=s.result.engineering_available?'准备复测':'暂无配重指引';hint='';}
    if(noCorrection){heading='本次无需配重';hint='原轮振动较低。停稳后取下试重。';}
    if(s.comparison.available){heading='这次测量完成了';hint='';}
    if(variable&&!running){heading='复测结果差异较大';hint='查看已保存的复测。';}
    if(s.state==='ARMED'){heading='等待车轮转动';hint=timed?'低速匀速，记录 '+duration+' 秒。':'轻拨后松手，等车轮转完。';}
    if(s.state==='RECORDING'){heading='正在记录'+['原轮','试重后','校正后'][s.stage]+'整段转动';hint=timed?'保持可控低速。':'让车轮自然停稳。';}
    if(s.state==='PROCESSING'){heading='正在处理本段记录';hint='';}
    if(s.state==='NO_RESULT'){heading=s.stage===2?'复测暂不可比较':'本次无可用估计';hint=reasons[s.reason]||'记录已保存，可查看报告。';}
    if(failedRetest&&action==='retest')hint=(s.reason==='NO_COMMON_SPEED'?'两次转速重合不足。':reasons[s.reason]||'本段无法比较。')+' 原轮和试重已保留。';
    if(s.state==='NO_RESULT'&&guide(s)?.retry_available&&guide(s)?.editable)hint=reasons[s.reason]||'核对试重后可重测。';
    if(conflict){heading='候选配重有分歧';hint=reasons.ESTIMATE_CONFLICT;}
    if(s.state==='CANCELLED'){heading='本段已取消';hint='可重新开始本段。';}
    if(s.state==='TIMEOUT'){heading='等待转动已超时';hint='准备好后可重试。';}
    if(synthetic&&!running&&!['NO_RESULT','CANCELLED','TIMEOUT'].includes(s.state))hint='';
    if(history){heading='已保存的历史记录';hint='';}
    if(connected&&ui.data.export_busy===true&&!running)hint='正在导出记录。';
    if(!connected){heading='网页正在重连';hint='保持热点连接，等待更新。';}
    if(ui.dirty){heading='设置已修改';hint='应用后重新测量原轮。';}
    text('pHeading',heading);text('pHint',hint);
    const flow=s.capture_flow,flowOkay=flow?.available===true&&flow.session_id===s.id&&flow.snapshot_revision===s.revision&&flow.boot_id===s.boot_id&&flow.source===s.source;
    const active=flowOkay?flow.stages?.[s.stage]:null;
    const elapsed=active?.start_us?Math.max(0,Math.floor((active.end_us-active.start_us)/1e6)):0;
    $('pCaptureFlow').hidden=!running;
    $('pCaptureSettle').className=s.state==='RECORDING'?'active':s.state==='PROCESSING'?'done':'';
    $('pCaptureCollect').className=s.state==='PROCESSING'?'done':'';
    $('pCaptureFinish').className=s.state==='PROCESSING'?'active':'';
    text('pCaptureSettle',timed?'定时记录':'整段记录');text('pCaptureCollect',timed?'定时完成':'停转确认');
    text('pCaptureCount',s.state==='ARMED'?'等你开始转轮':s.state==='PROCESSING'?'整段统一分析中':timed?'已记录 '+elapsed+' / '+duration+' 秒':'已记录 '+elapsed+' 秒 · 等待停转');
    text('pCaptureNote',s.state==='RECORDING'&&!timed?'停稳后自动分析':'');
    text('pRecordLabel',history?'会话 '+s.id:synthetic?'合成演示 · 非实测':s.storage_ok?'':'摘要尚未保存');
    const labels={original:'开始原轮测量',trial:(guide(s)?.retry_available?'重新测量':'开始测量')+'这组 '+grams(s.config.trial_mass_g)+'g 试重',retest:s.stage===2?'再次复测':'开始校正后复测',demo_trial:'演示试重测量',demo_retest:s.comparison.available?'再次演示复测':'演示校正后复测',new:'新建测量',export:'导出报告，结束本次',finish:'结束本段并分析'};
    text('pPrimary',ui.data.export_busy===true?'正在导出记录…':ui.dirty?'应用参数并新建测量':running?(s.state==='ARMED'?'等待转动中…':s.state==='RECORDING'?(timed?'定时采集中…':'等待车轮转完…'):'正在处理…'):labels[action]);
    $('pPrimary').disabled=((action!=='export'||ui.dirty)&&!connected)||locked||running||(action==='trial'&&!ui.dirty&&(!$('pTrialInstalled').checked||ui.planDirty))||(action==='retest'&&!ui.dirty&&(!$('pTrialRemoved').checked||(retests?.entries?.length&&!$('pRetestCondition').value)))||(!synthetic&&!history&&!ui.data.device_online&&!ui.dirty&&action!=='export');
    $('pCancel').hidden=!running||s.state==='PROCESSING';$('pCancel').disabled=!connected||locked;
    $('pFinish').hidden=s.state!=='RECORDING';$('pFinish').disabled=!connected||locked;
    $('pNew').disabled=!connected||locked||running;$('pDemo').disabled=!connected||locked||running;
    $('pSaveConfig').disabled=!connected||locked||running;
    $('pExport').disabled=!goodSession(s)||locked;
    $('pBack').hidden=!ui.viewId;
    const completed=[s.stages[0].captured,s.stages[1].captured,(s.result.engineering_available&&!variable)||noCorrection,s.comparison.available];
    const next=running?(s.stage===2?3:s.stage):s.stage===2&&action==='retest'?3:!completed[0]?0:trialTask||!completed[1]?1:2;
    completed.forEach((done,index)=>{
      const step=$('pStep'+index),active=index===next&&!history&&!noCorrection&&(running||!s.comparison.available);
      step.className=active?'current':done?'done':'';
      if(active)step.setAttribute('aria-current','step');
      else step.removeAttribute('aria-current');
    });
    $('pProgress').hidden=!running;if(s.state==='RECORDING'&&!timed)$('pProgress').removeAttribute('value');else $('pProgress').value=s.state==='PROCESSING'?100:timed&&s.state==='RECORDING'?Math.min(99,elapsed/duration*100):0;
    const r=s.result,show=!variable&&!ui.dirty&&r.engineering_available===true&&r.calculation_valid===true&&Number.isFinite(r.mass_g)&&r.mass_g>0&&Number.isFinite(r.angle_deg)&&geometry(r.angle_deg,s.config.direction);
    const comparisonFocus=s.comparison.available||s.stage===2&&running;
    $('pPlacementCard').hidden=!r.calculation_valid||running||comparisonFocus&&!ui.showPlacement;
    $('pTarget').toggleAttribute('hidden',!show);const g=show&&geometry(r.angle_deg,s.config.direction);
    text('pResultTag',conflict?'克数待确认':history?'历史结果':synthetic?'合成结果':noCorrection?'无需配重':variable?'复测差异较大':s.comparison.available?'本次估计 · 已复测':r.engineering_available?(r.assessment?.state==='VARIABLE_ESTIMATES'?'工程估计 · 分段有差异':'工程估计 · 待复测'):r.calculation_valid?'暂无配重指引':'等待测量');
    text('pResultMass',show?'≈ '+r.mass_g.toFixed(1)+' g':noCorrection?'0 g':'— g');
    text('pResultAngle',show?Math.round(r.angle_deg)%360+'°':'— °');
    const direction=s.config.direction==='cw'?'顺时针':'逆时针';
    text('pResultDirection',show?direction+' · 半径 '+s.config.correction_radius_mm+' mm':'');
    text('pPlacementNote',show?(synthetic?'合成位置 · 非实测':history?'历史位置 · 供查看':s.stage===2?'本会话原轮和试重测得的位置。':'停稳后取下试重。保持同侧，按图贴块。'):'');
    if(conflict)text('pPlacementNote','各速度段和测量轴的结果见报告。');
    if(r.assessment?.state==='ASSESSMENT_MISSING')text('pPlacementNote',estimateNote(s));
    if(variable)text('pPlacementNote','同一配重的复测响应差异较大，暂不显示继续贴块指引。原工程估计和各次负改善率均保留在测量详情与报告。');
    if(show){
      $('pWeight').setAttribute('transform','rotate('+g.rotation+' 120 120)');
      $('pRadial').setAttribute('d','M120 120 L'+g.x.toFixed(2)+' '+g.y.toFixed(2));
      $('pSweep').setAttribute('d',dialArc(g.rotation));
    }
    text('pRightAngle',s.config.direction==='cw'?'90°':'270°');text('pLeftAngle',s.config.direction==='cw'?'270°':'90°');
    const comparison=s.comparison;$('pComparison').hidden=!comparison.available;
    const phaseNote=comparison.phase_estimate_quality==='WEAK_OR_INCOHERENT'?'选段1×方向分散，幅值比较仍保留，但不能据此验证配重角度。':'';
    $('pComparison').classList.toggle('worse',comparison.available&&comparison.improvement_percent<0);
    const rawComparison=comparison.available?'1×指标'+(comparison.improvement_percent>=0?'改善 ':'增大 ')+Math.abs(comparison.improvement_percent).toFixed(1)+'%':'—';
    text('pImprovement',variable?'复测结果差异较大':rawComparison);
    text('pCompareDetail',comparison.available?(variable?'本次相对原轮：'+rawComparison+'。':'')+['X','Y','Z'][s.config.axis]+'轴 · '+(Number.isFinite(comparison.lower_rpm)?comparison.lower_rpm:s.result.lower_rpm)+'–'+(Number.isFinite(comparison.upper_rpm)?comparison.upper_rpm:s.result.upper_rpm)+' RPM · '+comparison.matched_turns+'对整圈。1×幅值RMS：'+comparison.original_peak_mps2.toPrecision(3)+' → '+comparison.retest_peak_mps2.toPrecision(3)+' m/s²。'+phaseNote+(s.retest_weight_confirmation==='not_recorded'&&!synthetic?'此旧记录未保存试重移除确认。':'')+'不等同于静态重心或整车总振动。':'');
    text('pRepeatQuality',latestRetest?repeatText(latestRetest):'尚无同状态重复性观察。');
    text('pQuality',r.calculation_valid&&!synthetic&&!noCorrection?estimateNote(s):synthetic?'此会话全部为合成数据。':'');
    text('pStorage',s.storage_ok?'摘要自动保存 · 可导出报告':'本次摘要尚未持久保存，请导出报告。');
    text('pSummary',s.stages.map((v,i)=>['原轮','试重','复测'][i]+' #'+(v.capture_id||'—')+'：'+(v.captured?v.usable_turns+'个可用整圈，'+Math.round(v.min_rpm)+'–'+Math.round(v.max_rpm)+' RPM':v.capture_id?'已记录，'+(reasons[v.reason]||'无可用估计'):'未记录')).join('\n')+(r.matched_turns?'\n参考比较速度段 '+r.matched_turns+' 对整圈，'+r.lower_rpm+'–'+r.upper_rpm+' RPM。':'')+'\n通道：'+(guide(s)?.axis_selection?.mode==='auto'?'自动'+(guide(s).axis_selection.resolved?'选定'+['X','Y','Z'][s.config.axis]+'轴':'，试重后按响应确认'):['X','Y','Z'][s.config.axis]+'轴（手动）')+'\n会话 '+s.id+' · 固件 '+s.firmware);
    if(variable&&r.calculation_valid)$('pSummary').textContent+='\n原工程估计：约 '+r.mass_g.toFixed(1)+'g／'+Math.round(r.angle_deg)+'°，当前不作继续贴块指引。';
    renderRetests(s);
    if(window.WheelSpeed)window.WheelSpeed.render(s,{dirty:ui.dirty,uptime:ui.data.uptime_us||0,uiFixture:ui.data.ui_fixture===true,connected});
    const box=$('pHistory');box.replaceChildren();for(const h of ui.data.history||[]){const button=document.createElement('button');button.type='button';button.textContent=(h.vehicle||'测量')+' · '+(h.source==='synthetic'?'合成':'实测')+' · '+h.id;button.addEventListener('click',()=>{ui.viewId=h.id;ui.initialized=false;poll(true);});box.appendChild(button);}
  }
  async function request(action,extra={}){const s=session();if(!goodSession(s)||ui.busy||ui.pendingRevision!==null||ui.data.export_busy===true)return;
    const current=ui.data;ui.busy=true;text('pMessage','');render();
    const body={...extra,action,boot_id:current.boot_id,id:current.current_id,revision:s.revision};
    try{
      if(action==='retest'){if(!$('pTrialRemoved').checked)throw Error('请先确认已移除试重并贴好校正块。');body.trial_removed=true;
        body.retest_condition=retestHistory(s)?.entries?.length?$('pRetestCondition').value:'first';
        if(!body.retest_condition)throw Error('请确认本次配重与上次复测是否相同。');}
      if(action==='trial'){if(ui.planDirty)throw Error('请先确认修改后的试重总量。');if(!$('pTrialInstalled').checked)throw Error('请先按任务卡确认实际新增试重总量。');body.trial_installed=true;body.confirmed_mass_g=s.config.trial_mass_g;}
      if(action==='trial_plan'&&(!Number.isFinite(body.mass_g)||body.mass_g<.1||body.mass_g>1000))throw Error('请输入有效的本次试重总量。');
      if(action==='new'){const config={};for(const [id,key] of Object.entries(fieldMap))config[key]=['vehicle','direction'].includes(key)?$(id).value:Number($(id).value);
        const seconds=Number($('pCaptureSeconds').value);if(!Number.isInteger(seconds)||seconds<10||seconds>60)throw Error('定时记录应为10至60秒的整数。');
        body.capture_settings={vehicle_type:$('pVehicleType').value,mode:$('pCaptureMode').value,duration_seconds:seconds};
        body.axis_mode=$('pAxis').value==='auto'?'auto':'manual';
        if(body.axis_mode==='auto')config.axis=s.config.axis;
        const piece=Number($('pPiece').value),mode=$('pTrialMode').value;
        if(!Number.isFinite(piece)||piece<.1||piece>500)throw Error('请填写有效的每片配重质量。');
        if(mode==='guided')config.trial_mass_g=2*piece;
        if(![config.trial_mass_g,config.trial_radius_mm,config.correction_radius_mm].every(v=>Number.isFinite(v)&&v>0)||!Number.isFinite(config.trial_angle_deg)||config.trial_angle_deg<0||config.trial_angle_deg>=360)throw Error('请填写有效的质量、半径和0–359°试重角度。');body.config=s.source==='synthetic'&&!ui.dirty&&ui.data.real_config?{...ui.data.real_config,trial_mass_g:config.trial_mass_g}:config;
        body.trial_guide={mode,piece_mass_g:piece};
        if(extra.beginner_defaults){body.axis_mode='auto';body.config.trial_angle_deg=0;body.config.correction_radius_mm=body.config.trial_radius_mm;body.config.trial_mass_g=2*piece;body.trial_guide.mode='guided';delete body.beginner_defaults;}}
      if(ui.viewId){ui.viewId=null;ui.initialized=false;await poll(true);body.id=ui.data.current_id;body.revision=session().revision;body.boot_id=ui.data.boot_id;}
      const reply=await deviceLink.json('/api/session',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body),timeoutMs:5000,priority:true});
      if(!reply.request_queued)throw Error('设备暂未接受操作，请稍后重试。');ui.pendingRevision=body.revision;ui.pendingUntil=performance.now()+6000;
      if(action==='new'||action==='demo'){ui.dirty=false;ui.initialized=false;ui.showPlacement=false;}
      await poll(true);
    }catch(e){text('pMessage',e.httpStatus?'设备暂未接受操作，请核对当前会话后重试。':e.message||'操作尚未确认，请等待连接恢复。');}
    finally{ui.busy=false;render();}
  }
  async function pollOnce(force){
    const view=ui.viewId,ticket=++ui.pollSequence;
    try{const q=view?'?id='+encodeURIComponent(view):'';const data=await deviceLink.json('/api/session'+q,{cache:'no-store',timeoutMs:3000});if(data.schema_version!==2||!goodSession(data.session)||!/^[a-f0-9]{16}$/.test(data.boot_id))throw Error('schema');
      if(view!==ui.viewId||ticket<ui.acceptedSequence)return;ui.acceptedSequence=ticket;
      if(ui.data&&ui.data.boot_id!==data.boot_id){ui.pendingRevision=null;ui.dirty=false;ui.initialized=false;}
      if(ui.pendingRevision!==null&&data.session.revision!==ui.pendingRevision)ui.pendingRevision=null;
      if(ui.pendingRevision!==null&&performance.now()>ui.pendingUntil){ui.pendingRevision=null;text('pMessage','操作尚未执行，请重试。');}
      ui.data=data;ui.online=true;ui.lastSuccess=performance.now();pollFailures=0;
      if($('pMessage').textContent===reconnectHint)text('pMessage','');
    }catch(e){pollFailures=Math.min(pollFailures+1,4);if(ticket<ui.acceptedSequence||view!==ui.viewId)return;if(force)text('pMessage',reconnectHint);}
    render();
  }
  function poll(force=false){
    clearTimeout(pollTimer);
    if(pollPromise){if(force)refreshRequested=true;return pollPromise;}
    // A forced refresh during a request is coalesced and awaited before the
    // shared promise settles (history -> current -> command keeps its identity).
    pollPromise=(async()=>{do{refreshRequested=false;await pollOnce(force);force=true;}while(refreshRequested);})()
      .finally(()=>{pollPromise=null;pollTimer=setTimeout(()=>poll(),pollFailures?Math.min(8000,1000*2**(pollFailures-1)):1000);});
    return pollPromise;
  }
  async function exportReport(){
    if(!ui.data||ui.busy)return;const s=session();ui.busy=true;render();
    const controller=new AbortController(),timeout=setTimeout(()=>controller.abort(),12000);
    try{
      const response=await fetch('/api/session/report?id='+encodeURIComponent(s.id),{cache:'no-store',signal:controller.signal});
      if(!response.ok)throw Error('设备报告返回 '+response.status);
      const payload=await response.json();
      if(payload.session?.id!==s.id||payload.session?.source!==s.source||payload.session?.boot_id!==s.boot_id)throw Error('报告来源与当前会话不一致');
      const report={...payload,schema_version:2,report_kind:s.source==='synthetic'?'synthetic_product_session':'engineering_product_session',
        ui_fixture:payload.ui_fixture===true,exported_at:new Date().toISOString(),angle_convention:'white_mark_zero_along_rotation_same_view',recommendation_valid:false};
      const url=URL.createObjectURL(new Blob([JSON.stringify(report,null,2)+'\n'],{type:'application/json'})),a=document.createElement('a');
      a.href=url;a.download='wheelbalance_session_'+s.id+'.json';document.body.appendChild(a);a.click();a.remove();setTimeout(()=>URL.revokeObjectURL(url),1000);
      text('pMessage','完整会话报告已导出。');
    }catch(error){text('pMessage','完整报告导出失败，请重试：'+error.message);}
    finally{clearTimeout(timeout);ui.busy=false;render();}
  }
  $('pPrimary').addEventListener('click',()=>{const action=ui.dirty?'new':nextAction(session());if(action==='export')exportReport();else request(action,action==='new'&&!ui.dirty?{beginner_defaults:true}:{});});
  $('pNew').addEventListener('click',()=>request('new',{beginner_defaults:true}));$('pSaveConfig').addEventListener('click',()=>request('new'));
  $('pCancel').addEventListener('click',()=>request('cancel'));$('pDemo').addEventListener('click',()=>request('demo'));
  $('pFinish').addEventListener('click',()=>request('finish'));
  $('pExport').addEventListener('click',exportReport);$('pExportCompare').addEventListener('click',exportReport);$('pShowPlacement').addEventListener('click',()=>{ui.showPlacement=!ui.showPlacement;render();});$('pBack').addEventListener('click',()=>{ui.viewId=null;ui.initialized=false;poll(true);});
  $('pTrialRemoved').addEventListener('change',render);
  $('pRetestCondition').addEventListener('change',render);
  $('pTrialInstalled').addEventListener('change',render);
  $('pTrialActualMass').addEventListener('input',()=>{ui.planDirty=Number($('pTrialActualMass').value)!==session().config.trial_mass_g;$('pTrialInstalled').checked=false;text('pMessage',ui.planDirty?'实际总量已修改，请先点“确认本次总量”。':'');render();});
  $('pApplyTrialMass').addEventListener('click',()=>request('trial_plan',{mass_g:Number($('pTrialActualMass').value),plan_source:'manual'}));
  $('pUseTrialAdvice').addEventListener('click',()=>request('trial_plan',{mass_g:guide(session())?.suggested_mass_g,plan_source:'response_projection'}));
  $('pResetAdvanced').addEventListener('click',()=>{$('pAxis').value='auto';$('pTrialAngle').value='0';$('pTrialMode').value='guided';$('pCorrectionRadius').value=$('pRadius').value;ui.customCorrectionRadius=false;ui.dirty=true;render();});
  $('pRadius').addEventListener('input',()=>{if(!ui.customCorrectionRadius)$('pCorrectionRadius').value=$('pRadius').value;});
  $('pCorrectionRadius').addEventListener('input',()=>{ui.customCorrectionRadius=Number($('pCorrectionRadius').value)!==Number($('pRadius').value);});
  for(const id of ['pPiece','pTrialMode'])$(id).addEventListener('input',()=>{ui.dirty=true;render();});
  $('pVehicleType').addEventListener('change',()=>{const electric=$('pVehicleType').value==='electric';$('pCaptureMode').value=electric?'timed':'coast';$('pCaptureSeconds').value=30;
    if(['自行车 · 后轮','电动车 · 车轮','其它 · 车轮'].includes($('pVehicle').value))$('pVehicle').value=electric?'电动车 · 车轮':$('pVehicleType').value==='bicycle'?'自行车 · 后轮':'其它 · 车轮';ui.dirty=true;render();});
  for(const id of ['pCaptureMode','pCaptureSeconds'])$(id).addEventListener('input',()=>{ui.dirty=true;render();});
  for(const id of Object.keys(fieldMap))$(id).addEventListener('input',()=>{ui.dirty=true;render();});
  window.WheelProduct={geometry,render,poll,request,acceptFixture(data){if(!data||data.ui_fixture!==true)throw Error('explicit fixture required');ui.data=data;ui.online=true;ui.lastSuccess=performance.now();render();}};
  poll();setInterval(render,500);
})();
