// ================= robot.js =================
// Robotun 3B görsel temsili: linkler, eklem gövdeleri, çerçeve gizmoları, etiketler,
// IK hedef göstergesi, workspace küresi, TCP izi ve her karede pozu güncelleyen updateFK().
// scene.js (robotGroup, scene, camera, renderer, grid) ve kinematics.js/utils.js'e bağımlıdır.
// params / thetasDeg (app state) main.js'te tanımlanır; updateFK() sadece çağrıldığı anda
// bunları okur, bu yüzden yükleme sırası önemli değildir.

// base
const baseMat = new THREE.MeshStandardMaterial({color:0x3a3f4a, metalness:0.3, roughness:0.6});
const baseMesh = new THREE.Mesh(new THREE.CylinderGeometry(70,80,40,24), baseMat);
robotGroup.add(baseMesh);

const LINK_COLOR = 0x4d9dff, LINK_BAD_COLOR = 0xff5c5c;
const linkMat = new THREE.MeshStandardMaterial({color:LINK_COLOR, metalness:0.25, roughness:0.5});

// Real cobots (UR5e etc.) don't look like thin straight rods joining ball centers — each joint is
// a chunky cylindrical "housing" oriented along its own rotation axis, with a colored cap band,
// and the links plug into the side of that housing rather than passing straight through its
// center. That's what makes the arm read as curved/organic instead of a stick figure. We build
// that housing as a small group (grey body + two colored end caps) and reuse one per joint.
// Radii/lengths taper from base (thick) to wrist (thin), matching how a real arm looks.
const JOINT_RADII = [22, 22, 22, 22, 22, 22];
const JOINT_LENS   = [45, 45, 45, 45, 45, 45];
const LINK_RADII   = [22, 22, 22, 22, 22, 22];
const HOUSING_CAP_COLOR = 0x35c9d1; 
const housingBodyMat = new THREE.MeshStandardMaterial({color:0x8a8f99, metalness:0.35, roughness:0.45});
const housingCapMat = new THREE.MeshStandardMaterial({color:HOUSING_CAP_COLOR, metalness:0.2, roughness:0.4});

function makeJointHousing(radius, length){
  const group = new THREE.Group();
  const body = new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, length, 28), housingBodyMat);
  group.add(body);
  const capLen = length * 0.22;
  const capTop = new THREE.Mesh(new THREE.CylinderGeometry(radius*1.015, radius*1.015, capLen, 28), housingCapMat);
  capTop.position.y = length/2 - capLen/2;
  group.add(capTop);
  const capBot = capTop.clone();
  capBot.position.y = -(length/2 - capLen/2);
  group.add(capBot);
  group.userData.radius = radius;
  group.userData.length = length;
  return group;
}

const linkMeshes = [], jointMeshes = [];
for (let i=0;i<6;i++){
  const link = new THREE.Mesh(new THREE.CylinderGeometry(LINK_RADII[i],LINK_RADII[i],1,16), linkMat.clone());
  robotGroup.add(link); linkMeshes.push(link);
  const joint = makeJointHousing(JOINT_RADII[i], JOINT_LENS[i]);
  robotGroup.add(joint); jointMeshes.push(joint);
}

// RGB axes gizmo (X=red, Y=green, Z=blue), reused for every frame + the IK target
function makeAxesGizmo(len){
  const g = new THREE.Group();
  const mk = (color, dir) => {
    const mat = new THREE.LineBasicMaterial({color});
    const geo = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0,0,0), dir.clone().multiplyScalar(len)]);
    return new THREE.Line(geo, mat);
  };
  g.add(mk(0xff5c5c, new THREE.Vector3(1,0,0))); // X = red
  g.add(mk(0x4ddd8a, new THREE.Vector3(0,1,0))); // Y = green
  g.add(mk(0x4d9dff, new THREE.Vector3(0,0,1))); // Z = blue
  return g;
}

// Small canvas-texture text label, billboarded via THREE.Sprite.
function makeTextSprite(text, opts={}){
  const fontSize = opts.fontSize || 40;
  const color = opts.color || '#eceef2';
  const bg = opts.background || 'rgba(20,22,26,0.78)';
  const canvas = document.createElement('canvas');
  const ctx = canvas.getContext('2d');
  ctx.font = 'bold '+fontSize+'px monospace';
  const padding = 16;
  const textWidth = ctx.measureText(text).width;
  canvas.width = Math.ceil(textWidth + padding*2);
  canvas.height = fontSize + padding*2;
  ctx.font = 'bold '+fontSize+'px monospace'; // resizing the canvas resets context state
  ctx.fillStyle = bg;
  ctx.fillRect(0,0,canvas.width, canvas.height);
  ctx.fillStyle = color;
  ctx.textBaseline = 'middle';
  ctx.textAlign = 'center';
  ctx.fillText(text, canvas.width/2, canvas.height/2 + 1);
  const texture = new THREE.CanvasTexture(canvas);
  texture.minFilter = THREE.LinearFilter;
  const mat = new THREE.SpriteMaterial({map:texture, depthTest:false, depthWrite:false, transparent:true});
  const sprite = new THREE.Sprite(mat);
  const worldScale = opts.worldScale || 0.55;
  sprite.scale.set(canvas.width*worldScale*0.5, canvas.height*worldScale*0.5, 1);
  sprite.renderOrder = 999;
  return sprite;
}

// Per-frame axis gizmos: index 0 = base frame, index i (1..6) = frame after joint i (T0i).
// Each is positioned/oriented directly from the DH cumulative transforms every update — a single
// pose call per frame drives the gizmo, its joint sphere is handled separately since jointMeshes
// already existed above; keeping them as siblings (not nested) avoids re-deriving relative
// transforms that the existing absolute-matrix FK already computes correctly.
const frameAxisGizmos = [];
for(let i=0;i<=6;i++){
  const g = makeAxesGizmo(55);
  robotGroup.add(g);
  frameAxisGizmos.push(g);
}
const IDENTITY_R = [[1,0,0],[0,1,0],[0,0,1]];

// Joint labels J1..J6 and link labels L1..L6
const jointLabels = [];
for(let i=0;i<6;i++){
  const s = makeTextSprite('J'+(i+1));
  robotGroup.add(s);
  jointLabels.push(s);
}
// görsel yönle eşleştirmek için. Renkler mevcut eksen kuralıyla tutarlı: X=yeşil, Y=kırmızı.
const GROUND_AXIS_DIST = 800; // mm, zeminde ne kadar uzağa yazılsın
const groundLabelX = makeTextSprite('+X', {color:'#ff5c5c', fontSize:90, background:'rgba(20,22,26,0.55)'});
groundLabelX.position.copy(toThree([GROUND_AXIS_DIST, 0, 0]));
robotGroup.add(groundLabelX);

const groundLabelY = makeTextSprite('+Y', {color:'#4ddd8a', fontSize:90, background:'rgba(20,22,26,0.55)'});
groundLabelY.position.copy(toThree([0, GROUND_AXIS_DIST, 0]));
robotGroup.add(groundLabelY)

const linkLabels = [];
for(let i=0;i<6;i++){
  const s = makeTextSprite('L'+(i+1), {color:'#a8adb8', background:'rgba(20,22,26,0.6)', fontSize:32});
  robotGroup.add(s);
  linkLabels.push(s);
}


// Tool (TCP) frame reuses frameAxisGizmos[6] — no separate duplicate gizmo needed.
const toolFrameGizmo = frameAxisGizmos[6];

// IK target gizmo/marker — lives in robotGroup too, so it stays visually aligned with the
// robot's own TCP frame once a solution is applied.
const targetGizmo = makeAxesGizmo(90);
targetGizmo.visible = false;
robotGroup.add(targetGizmo);
const targetMarker = new THREE.Mesh(new THREE.SphereGeometry(14,12,12), new THREE.MeshBasicMaterial({color:0xff5c5c, wireframe:true}));
targetMarker.visible = false;
robotGroup.add(targetMarker);

// Approximate reachable-workspace envelope (sphere), centered on the shoulder pivot (which sits
// at a fixed height above the base regardless of joint1 rotation).
let lastWorkspaceRadius = -1;
const workspaceSphere = new THREE.Mesh(
  new THREE.SphereGeometry(1, 24, 16),
  new THREE.MeshBasicMaterial({color:0x4d9dff, wireframe:true, transparent:true, opacity:0.12})
);
robotGroup.add(workspaceSphere);

// TCP trail
const TCP_PATH_MAX_POINTS = 600;
let tcpPathPoints = [];
const tcpPathLine = new THREE.Line(
  new THREE.BufferGeometry(),
  new THREE.LineBasicMaterial({color:0xff8a5c})
);
robotGroup.add(tcpPathLine);

// ---------- View options ----------
const viewState = {
  grid:true, jointFrames:true, toolFrame:true, jointLabels:true, linkLabels:false,
  collision:false, workspace:false, tcpPath:false
};
function applyViewState(){
  grid.visible = viewState.grid;
  for(let i=0;i<=5;i++) frameAxisGizmos[i].visible = viewState.jointFrames;
  toolFrameGizmo.visible = viewState.toolFrame;
  jointLabels.forEach(s=> s.visible = viewState.jointLabels);
  linkLabels.forEach(s=> s.visible = viewState.linkLabels);
  workspaceSphere.visible = viewState.workspace;
  tcpPathLine.visible = viewState.tcpPath;
}
applyViewState();

// three.js coordinate mapping: robot uses Z-up (per DH), three.js is Y-up.
// Swapping two axes alone flips handedness (mirrors the scene); negating the third component
// keeps it a proper (determinant +1) right-handed remap: robot (x,y,z) -> three (x, z, -y).
// The robotGroup's 180° Y rotation above is a separate, purely cosmetic choice (which way the
// robot "faces" the default camera) — it does not affect this remap's correctness.
function toThree(v){ return new THREE.Vector3(v[0], v[2], -v[1]); }

function placeCylinderBetween(mesh, pA, pB, radius){
  const a = toThree(pA), b = toThree(pB);
  const mid = a.clone().add(b).multiplyScalar(0.5);
  const dir = b.clone().sub(a);
  const len = dir.length();
  mesh.position.copy(mid);
  mesh.geometry.dispose();
  mesh.geometry = new THREE.CylinderGeometry(radius, radius, Math.max(len,0.01), 14);
  if (len > 1e-6){
    const up = new THREE.Vector3(0,1,0);
    const axis = new THREE.Vector3().crossVectors(up, dir.clone().normalize());
    const angle = Math.acos(Math.max(-1,Math.min(1,up.dot(dir.clone().normalize()))));
    mesh.quaternion.setFromAxisAngle(axis.lengthSq()>1e-8 ? axis.normalize() : new THREE.Vector3(1,0,0), angle);
  }
}

// Orients a joint housing so its cylinder axis (local Y) lines up with the joint's actual
// rotation axis in world space. That axis is the Z axis of the DH frame *before* this joint's
// own rotation is applied (frame i for joint i+1, with frame 0 = base/identity) — i.e. exactly
// the axis the joint spins around. Using this (instead of just centering a sphere) is what makes
// each joint look like a real pivoting "puck" that the neighboring links bend around.
function orientJointHousing(group, R, posThree){
  group.position.copy(posThree);
  const remap = (col) => new THREE.Vector3(col[0], col[2], -col[1]);
  const zAxis = remap([R[0][2], R[1][2], R[2][2]]).normalize();
  const up = new THREE.Vector3(0,1,0);
  const axis = new THREE.Vector3().crossVectors(up, zAxis);
  const dot = Math.max(-1, Math.min(1, up.dot(zAxis)));
  const angle = Math.acos(dot);
  if (axis.lengthSq() > 1e-8){
    group.quaternion.setFromAxisAngle(axis.normalize(), angle);
  } else if (dot < 0){
    group.quaternion.setFromAxisAngle(new THREE.Vector3(1,0,0), Math.PI);
  } else {
    group.quaternion.identity();
  }
}

// Rotation of the DH frame *before* joint (i+1)'s own rotation is applied — i.e. the frame the
// joint's rotation axis (its local Z) is expressed in. i=0 -> base (identity), i=1..5 -> rotation
// part of cum[i-1] (T0,i).
function frameRotationBefore(i, cum){
  if (i === 0) return IDENTITY_R;
  const T = cum[i-1];
  return [[T[0][0],T[0][1],T[0][2]],[T[1][0],T[1][1],T[1][2]],[T[2][0],T[2][1],T[2][2]]];
}

function applyGizmoPose(gizmo, R, posThree){
  gizmo.position.copy(posThree);
  const m = new THREE.Matrix4();
  // R is robot-frame rotation; remap basis vectors the same way as toThree (swap y/z rows)
  const remap = (col) => new THREE.Vector3(col[0], col[2], -col[1]);
  const xAxis = remap([R[0][0], R[1][0], R[2][0]]);
  const yAxis = remap([R[0][1], R[1][1], R[2][1]]);
  const zAxis = remap([R[0][2], R[1][2], R[2][2]]);
  m.makeBasis(xAxis, yAxis, zAxis);
  gizmo.quaternion.setFromRotationMatrix(m);
}

const hud = document.getElementById('hud');
const fkReadout = document.getElementById('fk-readout');
const poseStatusEl = document.getElementById('pose-status-readout');

function updateFK(){
  const thetasRad = thetasDeg.map(toRad);
  const cum = fkChain(thetasRad, params);
  const pts = [[0,0,0]];
  for (const T of cum) pts.push([T[0][3], T[1][3], T[2][3]]);

  // links (radius tapers from base to wrist, matching the joint housings below)
  placeCylinderBetween(linkMeshes[0], pts[0], pts[1], LINK_RADII[0]);
  for (let i=1;i<6;i++) placeCylinderBetween(linkMeshes[i], pts[i], pts[i+1], LINK_RADII[i]);
  // joints — each housing is oriented along its own rotation axis (frame *before* its own
  // rotation is applied), which is what gives the arm its curved, "real robot" silhouette
  // instead of straight rods meeting at featureless ball joints.
  for (let i=0;i<6;i++){
    const Rprev = frameRotationBefore(i, cum);
    orientJointHousing(jointMeshes[i], Rprev, toThree(pts[i]));
  }

  // per-frame axis gizmos: frame0 = base (identity), frame i = T0i for i=1..6
  applyGizmoPose(frameAxisGizmos[0], IDENTITY_R, toThree(pts[0]));
  for (let i=0;i<cum.length;i++){
    const T = cum[i];
    const R = [[T[0][0],T[0][1],T[0][2]],[T[1][0],T[1][1],T[1][2]],[T[2][0],T[2][1],T[2][2]]];
    applyGizmoPose(frameAxisGizmos[i+1], R, toThree(pts[i+1]));
  }

  // labels
  for(let i=0;i<6;i++) jointLabels[i].position.copy(toThree(pts[i])).add(new THREE.Vector3(0,34,0));
  for(let i=0;i<6;i++){
    const mid = [ (pts[i][0]+pts[i+1][0])/2, (pts[i][1]+pts[i+1][1])/2, (pts[i][2]+pts[i+1][2])/2 ];
    linkLabels[i].position.copy(toThree(mid)).add(new THREE.Vector3(0,22,0));
  }

  // workspace envelope
  const wr = params.a2 + params.a3 + params.d4 + params.d5 + params.d6;
  if(Math.abs(wr - lastWorkspaceRadius) > 0.5){
    workspaceSphere.geometry.dispose();
    workspaceSphere.geometry = new THREE.SphereGeometry(Math.max(wr,1), 24, 16);
    lastWorkspaceRadius = wr;
  }
  workspaceSphere.position.copy(toThree([0,0,params.d1]));

  // TCP trail
  if(viewState.tcpPath){
    const tip = toThree(pts[6]);
    tcpPathPoints.push(tip.x, tip.y, tip.z);
    if(tcpPathPoints.length > TCP_PATH_MAX_POINTS*3) tcpPathPoints.splice(0, tcpPathPoints.length - TCP_PATH_MAX_POINTS*3);
    tcpPathLine.geometry.dispose();
    tcpPathLine.geometry = new THREE.BufferGeometry();
    tcpPathLine.geometry.setAttribute('position', new THREE.Float32BufferAttribute(tcpPathPoints, 3));
  }

  // physical validity (joint limits / ground / elbow inversion / approximate self-collision)
  const validity = evaluatePoseValidity(thetasRad, pts);
  if(viewState.collision){
    linkMeshes.forEach(m=> m.material.color.setHex(validity.ok ? LINK_COLOR : LINK_BAD_COLOR));
  } else {
    linkMeshes.forEach(m=> m.material.color.setHex(LINK_COLOR));
  }
  const statusLine = validity.ok
    ? '<b style="color:var(--ok)">✓ Poz geçerli</b>'
    : '<b style="color:var(--bad)">✕ Poz geçersiz</b><br>'+validity.reason;
  poseStatusEl.innerHTML = statusLine;

  const T06 = cum[5];
  const R = [[T06[0][0],T06[0][1],T06[0][2]],[T06[1][0],T06[1][1],T06[1][2]],[T06[2][0],T06[2][1],T06[2][2]]];
  const p = [T06[0][3], T06[1][3], T06[2][3]];

  const [roll,pitch,yaw] = rpyFromRot(R);
  hud.innerHTML =
    '<b>Uç efektör pozu</b><br>' +
    'x: <b>'+p[0].toFixed(1)+'</b>  y: <b>'+p[1].toFixed(1)+'</b>  z: <b>'+p[2].toFixed(1)+'</b> mm<br>' +
    'roll: <b>'+roll.toFixed(1)+'</b>  pitch: <b>'+pitch.toFixed(1)+'</b>  yaw: <b>'+yaw.toFixed(1)+'</b> °<br>' +
    (validity.ok ? '<b style="color:var(--ok)">Poz geçerli</b>' : '<b style="color:var(--bad)">Geçersiz:</b> '+validity.reason);

  fkReadout.innerHTML =
    'px = <b>'+p[0].toFixed(2)+'</b> mm<br>' +
    'py = <b>'+p[1].toFixed(2)+'</b> mm<br>' +
    'pz = <b>'+p[2].toFixed(2)+'</b> mm<br>' +
    'roll = <b>'+roll.toFixed(2)+'</b>°  pitch = <b>'+pitch.toFixed(2)+'</b>°  yaw = <b>'+yaw.toFixed(2)+'</b>°';

  window._lastPose = {p, R, roll, pitch, yaw};
}
