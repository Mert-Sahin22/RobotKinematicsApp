// ================= ui.js =================
// Tüm DOM kontrollerini (parametre girişleri, eksen kaydırıcıları, sekmeler, görünüm
// onay kutuları, IK paneli) kurar. initUI() olarak dışa açılır çünkü params/thetasDeg
// (app state) main.js'te tanımlanır ve bu fonksiyon o tanımlardan SONRA çağrılmalıdır.

function initUI(){

  // ---------- Params UI ----------
  const paramsRow = document.getElementById('params-row');
  const paramDefs = [
    {k:'d1', label:'d1 (taban yüksekliği)'},
    {k:'a2', label:'a2 (üst kol uzunluğu)'},
    {k:'a3', label:'a3 (ön kol uzunluğu)'},
    {k:'d4', label:'d4 (bilek ofseti)'},
    {k:'d5', label:'d5'},
    {k:'d6', label:'d6 (uç efektör uzunluğu)'},
  ];
  paramDefs.forEach(pd=>{
    const grp = document.createElement('div'); grp.className='grp';
    const lab = document.createElement('span'); lab.className='plabel'; lab.textContent = pd.label + ':';
    const inp = document.createElement('input'); inp.type='number'; inp.value = params[pd.k]; inp.step='0.1';
    inp.addEventListener('input', ()=>{ params[pd.k] = parseFloat(inp.value)||0; updateFK(); });
    const unit = document.createElement('span'); unit.className='unit'; unit.textContent='mm';
    grp.appendChild(lab); grp.appendChild(inp); grp.appendChild(unit);
    paramsRow.appendChild(grp);
  });

  // ---------- Joint sliders UI ----------
  const slidersHolder = document.getElementById('joint-sliders');
  const sliderEls = [], numberEls = [];
  for (let i=0;i<6;i++){
    const row = document.createElement('div'); row.className='joint-row';
    const head = document.createElement('div'); head.className='jhead';
    const label = document.createElement('label'); label.textContent = 'θ' + (i+1);
    const num = document.createElement('input'); num.type='number'; num.value = thetasDeg[i]; num.step='1';
    head.appendChild(label); head.appendChild(num);
    const slider = document.createElement('input'); slider.type='range'; slider.min=-180; slider.max=180; slider.step=1; slider.value=thetasDeg[i];
    row.appendChild(head); row.appendChild(slider);
    slidersHolder.appendChild(row);
    sliderEls.push(slider); numberEls.push(num);
    slider.addEventListener('input', ()=>{ thetasDeg[i]=parseFloat(slider.value); num.value=slider.value; updateFK(); });
    num.addEventListener('input', ()=>{ thetasDeg[i]=parseFloat(num.value)||0; slider.value = Math.max(-180,Math.min(180,thetasDeg[i])); updateFK(); });
  }
  document.getElementById('reset-btn').addEventListener('click', ()=>{
    for (let i=0;i<6;i++){ thetasDeg[i]=0; sliderEls[i].value=0; numberEls[i].value=0; }
    updateFK();
  });

  // ---------- Tabs ----------
  document.querySelectorAll('.tab-btn').forEach(btn=>{
    btn.addEventListener('click', ()=>{
      document.querySelectorAll('.tab-btn').forEach(b=>b.classList.remove('active'));
      document.querySelectorAll('.tab-content').forEach(c=>c.classList.remove('active'));
      btn.classList.add('active');
      document.getElementById('tab-'+btn.dataset.tab).classList.add('active');
    });
  });

  // ---------- View options ----------
  function bindViewCheckbox(id, key){
    const el = document.getElementById(id);
    el.checked = viewState[key];
    el.addEventListener('change', ()=>{
      viewState[key] = el.checked;
      if(key==='tcpPath' && el.checked) tcpPathPoints = []; // start a fresh trace
      applyViewState();
    });
  }
  [['vs-grid','grid'],['vs-jointFrames','jointFrames'],['vs-toolFrame','toolFrame'],
   ['vs-jointLabels','jointLabels'],['vs-linkLabels','linkLabels'],['vs-collision','collision'],
   ['vs-workspace','workspace'],['vs-tcpPath','tcpPath']].forEach(([id,key])=>bindViewCheckbox(id,key));
  applyViewState();

  // ---------- FK -> IK handoff ----------
  document.getElementById('copy-to-ik-btn').addEventListener('click', ()=>{
    const {p, roll, pitch, yaw} = window._lastPose;
    document.getElementById('ik-px').value = p[0].toFixed(1);
    document.getElementById('ik-py').value = p[1].toFixed(1);
    document.getElementById('ik-pz').value = p[2].toFixed(1);
    document.getElementById('ik-roll').value = roll.toFixed(1);
    document.getElementById('ik-pitch').value = pitch.toFixed(1);
    document.getElementById('ik-yaw').value = yaw.toFixed(1);
    document.querySelector('.tab-btn[data-tab="ik"]').click();
  });

  // ---------- IK solve UI ----------
  const solutionsHolder = document.getElementById('ik-solutions');
  const ikMsg = document.getElementById('ik-msg');

  document.getElementById('solve-ik-btn').addEventListener('click', ()=>{
    const px = parseFloat(document.getElementById('ik-px').value)||0;
    const py = parseFloat(document.getElementById('ik-py').value)||0;
    const pz = parseFloat(document.getElementById('ik-pz').value)||0;
    const roll = parseFloat(document.getElementById('ik-roll').value)||0;
    const pitch = parseFloat(document.getElementById('ik-pitch').value)||0;
    const yaw = parseFloat(document.getElementById('ik-yaw').value)||0;
    const R = rotFromRPY(roll, pitch, yaw);

    targetMarker.visible = true; targetGizmo.visible = true;
    targetMarker.position.copy(toThree([px,py,pz]));
    applyGizmoPose(targetGizmo, R, toThree([px,py,pz]));

    solutionsHolder.innerHTML = '';
    ikMsg.style.display = 'none';
    let anyValid = false;
    const labels = { shoulder:{1:'Omuz A',[-1]:'Omuz B'}, wrist:{1:'Bilek yukarı',[-1]:'Bilek aşağı'}, elbow:{1:'Dirsek yukarı',[-1]:'Dirsek aşağı'} };

    [1,-1].forEach(shoulder=>{
      [1,-1].forEach(wrist=>{
        [1,-1].forEach(elbow=>{
          const res = ik(R, [px,py,pz], params, {shoulder,wrist,elbow});
          const card = document.createElement('div');
          const tagText = labels.shoulder[shoulder]+' &middot; '+labels.elbow[elbow]+' &middot; '+labels.wrist[wrist];
          if (res.error){
            card.className = 'sol-card invalid';
            card.innerHTML = '<div class="sh"><span class="tag"><span class="status-dot" style="background:var(--bad)"></span>'+tagText+'</span></div>' +
              '<div class="th-vals" style="color:var(--text2)">'+res.error+'</div>';
          } else {
            anyValid = true;
            const degs = res.thetas.map(t=>normDeg(toDeg(t)));
            // physical check: joint limits / ground / elbow inversion / self-collision, on top of
            // the analytic (math-only) solution the IK solver produced
            const cumCheck = fkChain(res.thetas, params);
            const ptsCheck = [[0,0,0]];
            for(const T of cumCheck) ptsCheck.push([T[0][3],T[1][3],T[2][3]]);
            const phys = evaluatePoseValidity(res.thetas, ptsCheck);
            card.className = 'sol-card' + (phys.ok ? '' : ' warn');
            const dotColor = phys.ok ? 'var(--ok)' : 'var(--warn)';
            const extra = phys.ok ? '' : '<div class="th-vals" style="color:var(--warn)">⚠ '+phys.reason+'</div>';
            card.innerHTML = '<div class="sh"><span class="tag"><span class="status-dot" style="background:'+dotColor+'"></span>'+tagText+'</span>' +
              '<button class="btn small apply-btn">Uygula</button></div>' +
              '<div class="th-vals">' + degs.map((d,i)=>'θ'+(i+1)+'='+d.toFixed(1)+'°').join('  ') + '</div>' + extra;
            card.querySelector('.apply-btn').addEventListener('click', ()=>{
              document.querySelectorAll('.sol-card').forEach(c=>c.classList.remove('selected'));
              card.classList.add('selected');
              for (let i=0;i<6;i++){
                thetasDeg[i] = degs[i];
                sliderEls[i].value = Math.max(-180,Math.min(180,degs[i]));
                numberEls[i].value = degs[i].toFixed(1);
              }
              updateFK();
            });
          }
          solutionsHolder.appendChild(card);
        });
      });
    });
    if (!anyValid){
      ikMsg.textContent = 'Bu poz için 8 dalın hiçbiri geçerli değil (kolun erişim sınırları dışında olabilir).';
      ikMsg.style.display='block';
    }
  });
}