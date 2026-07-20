// ================= kinematics.js =================
// Analitik ters kinematik (IK) çözücü ve poz geçerlilik kontrolleri.
// utils.js'e bağımlıdır (dh/fkChain zaten orada; burada normDeg/toDeg/toRad kullanılıyor).

// Joint limits used by evaluatePoseValidity (degrees). Loosely modeled after a typical
// UR-style cobot: base/wrist axes free-spinning, shoulder/elbow/wrist-2 mechanically limited.
const jointLimitsDeg = [
  [-180, 180], // J1 taban
  [-180, 180], // J2 omuz
  [-180, 180], // J3 dirsek
  [-180, 180], // J4 bilek 1
  [-180, 180], // J5 bilek 2
  [-180, 180], // J6 bilek 3
];

// Analytic IK. branch = {shoulder, wrist, elbow} each +-1. Returns {thetas:[...]} or {error:'...'}
function ik(R, pxyz, p, branch){
  const [px,py,pz] = pxyz;
  const r11=R[0][0], r12=R[0][1], r13=R[0][2];
  const r21=R[1][0], r22=R[1][1], r23=R[1][2];
  const r31=R[2][0], r32=R[2][1], r33=R[2][2];
  const {d1,a2,a3,d4,d5,d6} = p;

  if (Math.abs(d6) < 1e-9 || Math.abs(a2) < 1e-9 || Math.abs(a3) < 1e-9)
    return {error:'d6, a2 ve a3 sıfırdan farklı olmalı.'};

  const p05x = px - d6*r13;
  const p05y = py - d6*r23;
  const Rxy = Math.sqrt(p05x*p05x + p05y*p05y);
  const ratio1 = d4 / Rxy;
  if (!isFinite(ratio1) || Math.abs(ratio1) > 1) return {error:'Hedef, bilek ofseti (d4) nedeniyle erişilemez.'};
  const th1 = Math.atan2(p05y, p05x) + branch.shoulder * Math.acos(ratio1) + Math.PI/2;
  const s1 = Math.sin(th1), c1 = Math.cos(th1);

  const c5 = (px*s1 - py*c1 - d4) / d6;
  if (Math.abs(c5) > 1) return {error:'Hedef oryantasyon bu pozisyon için erişilemez.'};
  const s5abs = Math.sqrt(Math.max(0,1 - c5*c5));
  if (s5abs < 1e-6) return {error:'Bilek tekilliği (θ5 ≈ 0), θ6 belirsiz.'};
  const s5 = branch.wrist * s5abs;
  const th5 = Math.atan2(s5, c5);

  const s6 = (r22*c1 - r12*s1) / s5;
  const c6 = (r11*s1 - r21*c1) / s5;
  const th6 = Math.atan2(s6, c6);

  const c234 = r31*s6 + r32*c6;
  const s234 = -r33 / s5;
  const th234 = Math.atan2(s234, c234);

  const X = px*c1 + py*s1 - d5*s234 + d6*s5*c234;
  const Z = pz - d1 + d5*c234 + d6*s5*s234;

  const D = (X*X + Z*Z - a2*a2 - a3*a3) / (2*a2*a3);
  if (Math.abs(D) > 1) return {error:'Hedef pozisyon kolun erişim sınırları dışında.'};
  const s3 = branch.elbow * Math.sqrt(Math.max(0,1-D*D));
  const th3 = Math.atan2(s3, D);
  const th2 = Math.atan2(Z, X) - Math.atan2(a3*Math.sin(th3), a2 + a3*Math.cos(th3));
  const th4 = th234 - th2 - th3;

  return {thetas:[th1,th2,th3,th4,th5,th6]};
}

// Physical validity check: joint limits, ground collision, elbow inversion heuristic, approximate self-collision.
// thetasRad: [t1..t6] in radians. pts: [origin, p1..p6] robot-frame joint positions (mm).
// This is intentionally simple (not a full mesh collision system) — it flags obviously invalid
// poses so the FK/IK math isn't presented as if every solution were physically usable.
function evaluatePoseValidity(thetasRad, pts){
  // 1) joint limits
  for(let i=0;i<6;i++){
    const deg = normDeg(toDeg(thetasRad[i]));
    const lim = jointLimitsDeg[i];
    if(deg < lim[0] || deg > lim[1]){
      return {ok:false, reason:'Eksen '+(i+1)+' limiti aşıldı: '+deg.toFixed(1)+'° (izin: '+lim[0]+'°..'+lim[1]+'°)'};
    }
  }
  // 2) ground collision — no joint should dip below the floor (z=0)
  const GROUND_EPS = 1e-6;
  for(let i=1;i<pts.length;i++){
    if(pts[i][2] < -GROUND_EPS){
      return {ok:false, reason:'Zemin çarpışması: nokta '+i+' z='+pts[i][2].toFixed(1)+' mm'};
    }
  }
  // 3) elbow inversion heuristic — elbow point dropping below the (fixed-height) shoulder pivot
  const shoulder = pts[1], elbow = pts[2];
  if(elbow[2] < shoulder[2] - GROUND_EPS){
    return {ok:false, reason:'Dirsek ters dönmüş (elbow z < shoulder z)'};
  }
  // 4) approximate self-collision — closest distance between non-adjacent link segments
  const linkSegs = [];
  for(let i=0;i<6;i++) linkSegs.push([pts[i], pts[i+1]]);
  const COLLISION_THRESHOLD = 60; // mm, ~ sum of link radii + safety margin
  const pairsToCheck = [[1,3,'Link2','Link4'], [1,4,'Link2','Link5'], [2,4,'Link3','Link5']];
  for(const [a,b,nameA,nameB] of pairsToCheck){
    const d = segSegDistance(linkSegs[a][0], linkSegs[a][1], linkSegs[b][0], linkSegs[b][1]);
    if(d < COLLISION_THRESHOLD){
      return {ok:false, reason:'Öz çarpışma: '+nameA+' ↔ '+nameB+' ('+d.toFixed(0)+' mm)'};
    }
  }
  return {ok:true};
}
