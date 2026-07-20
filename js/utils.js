// ================= utils.js =================
// Genel matematik / DH dönüşümü / geometri yardımcı fonksiyonları.
// Başka hiçbir dosyaya bağımlı değildir; en başta yüklenmelidir.

function dh(theta, d, a, alpha) {
  const ct = Math.cos(theta), st = Math.sin(theta);
  const ca = Math.cos(alpha), sa = Math.sin(alpha);
  return [
    [ct, -st*ca,  st*sa, a*ct],
    [st,  ct*ca, -ct*sa, a*st],
    [0,   sa,     ca,    d],
    [0,   0,      0,     1]
  ];
}

function matmul(A,B){
  const R=[[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,1]];
  for(let i=0;i<4;i++) for(let j=0;j<4;j++){ let s=0; for(let k=0;k<4;k++) s+=A[i][k]*B[k][j]; R[i][j]=s; }
  return R;
}

// Returns each intermediate transform T01..T06 (cumulative), for drawing joints.
function fkChain(thetas, p){
  const [t1,t2,t3,t4,t5,t6] = thetas;
  const mats = [
    dh(t1, p.d1, 0, Math.PI/2),
    dh(t2, 0, p.a2, 0),
    dh(t3, 0, p.a3, 0),
    dh(t4, p.d4, 0, Math.PI/2),
    dh(t5, p.d5, 0, -Math.PI/2),
    dh(t6, p.d6, 0, 0)
  ];
  const cum = [mats[0]];
  for (let i=1;i<6;i++) cum.push(matmul(cum[i-1], mats[i]));
  return cum; // [T01, T02, T03, T04, T05, T06]
}

function rotFromRPY(rollDeg, pitchDeg, yawDeg){
  const r = rollDeg*Math.PI/180, pch = pitchDeg*Math.PI/180, y = yawDeg*Math.PI/180;
  const cr=Math.cos(r), sr=Math.sin(r), cp=Math.cos(pch), sp=Math.sin(pch), cy=Math.cos(y), sy=Math.sin(y);
  // R = Rz(y) * Ry(p) * Rx(r)
  return [
    [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
    [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
    [-sp,   cp*sr,            cp*cr]
  ];
}

function rpyFromRot(R){
  const pitch = Math.atan2(-R[2][0], Math.sqrt(R[0][0]**2 + R[1][0]**2));
  const yaw = Math.atan2(R[1][0], R[0][0]);
  const roll = Math.atan2(R[2][1], R[2][2]);
  return [roll*180/Math.PI, pitch*180/Math.PI, yaw*180/Math.PI];
}

function normDeg(d){ let x=((d+180)%360+360)%360-180; return x; }
function toDeg(r){ return r*180/Math.PI; }
function toRad(d){ return d*Math.PI/180; }

// ================= Geometry helpers for validity / collision checks =================
function v3sub(a,b){ return [a[0]-b[0], a[1]-b[1], a[2]-b[2]]; }
function v3add(a,b){ return [a[0]+b[0], a[1]+b[1], a[2]+b[2]]; }
function v3dot(a,b){ return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]; }
function v3scale(a,s){ return [a[0]*s, a[1]*s, a[2]*s]; }
function v3dist(a,b){ const d=v3sub(a,b); return Math.sqrt(v3dot(d,d)); }
function clamp01(v){ return Math.max(0, Math.min(1, v)); }

// Closest distance between segment p1-q1 and segment p2-q2 (classic segment-segment distance).
function segSegDistance(p1,q1,p2,q2){
  const d1=v3sub(q1,p1), d2=v3sub(q2,p2), r=v3sub(p1,p2);
  const a=v3dot(d1,d1), e=v3dot(d2,d2), f=v3dot(d2,r);
  const EPS=1e-9;
  let s, t;
  if(a<=EPS && e<=EPS){ s=0; t=0; }
  else if(a<=EPS){ s=0; t=clamp01(f/e); }
  else{
    const c=v3dot(d1,r);
    if(e<=EPS){ t=0; s=clamp01(-c/a); }
    else{
      const b=v3dot(d1,d2);
      const denom=a*e-b*b;
      s = denom>EPS ? clamp01((b*f-c*e)/denom) : 0;
      t = (b*s+f)/e;
      if(t<0){ t=0; s=clamp01(-c/a); }
      else if(t>1){ t=1; s=clamp01((b-c)/a); }
    }
  }
  const c1=v3add(p1, v3scale(d1,s));
  const c2=v3add(p2, v3scale(d2,t));
  return v3dist(c1,c2);
}
