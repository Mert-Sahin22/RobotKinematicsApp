// ================= scene.js =================
// Three.js temel sahne kurulumu: kamera, renderer, ışıklar, grid, kamera orbit kontrolleri.
// three.js (CDN) yüklendikten sonra çalışır. robot.js ve main.js buradaki globallere erişir.

const holder = document.getElementById('canvas-holder');
const scene = new THREE.Scene();

// All robot-frame content (links, joints, frame axes, labels, IK target, TCP path) lives under
// one group. The DH math stays entirely in the robot's own coordinate convention; this group
// only carries a cosmetic 180° turn so the robot faces the default camera sensibly. Nothing about
// FK/IK computation changes — only where the assembled meshes are parented.
const robotGroup = new THREE.Group();
scene.add(robotGroup);
robotGroup.rotation.y = Math.PI;

const camera = new THREE.PerspectiveCamera(45, 1, 1, 10000);
const renderer = new THREE.WebGLRenderer({antialias:true});
renderer.setPixelRatio(window.devicePixelRatio);
holder.appendChild(renderer.domElement);

scene.add(new THREE.AmbientLight(0xffffff, 0.55));
const dl = new THREE.DirectionalLight(0xffffff, 0.8);
dl.position.set(600, 900, 700);
scene.add(dl);
const dl2 = new THREE.DirectionalLight(0xffffff, 0.3);
dl2.position.set(-500, -300, -400);
scene.add(dl2);

// ground grid — kept outside robotGroup since it's a world-space reference plane, not robot geometry
const grid = new THREE.GridHelper(1600, 16, 0x333844, 0x22252d);
scene.add(grid);

// camera orbit controls (hand-rolled, no addons needed)
let camTheta = 0.9, camPhi = 1.0, camRadius = 1900, camTarget = new THREE.Vector3(0,250,0);
function updateCamera(){
  camPhi = Math.max(0.15, Math.min(Math.PI-0.15, camPhi));
  camera.position.set(
    camTarget.x + camRadius*Math.sin(camPhi)*Math.cos(camTheta),
    camTarget.y + camRadius*Math.cos(camPhi),
    camTarget.z + camRadius*Math.sin(camPhi)*Math.sin(camTheta)
  );
  camera.up.set(0,1,0);
  camera.lookAt(camTarget);
}
let dragging=false, lastX=0, lastY=0;
renderer.domElement.addEventListener('pointerdown', e=>{ dragging=true; lastX=e.clientX; lastY=e.clientY; });
window.addEventListener('pointerup', ()=> dragging=false);
window.addEventListener('pointermove', e=>{
  if(!dragging) return;
  camTheta += (e.clientX-lastX)*0.006;
  camPhi -= (e.clientY-lastY)*0.006;
  lastX=e.clientX; lastY=e.clientY;
  updateCamera();
});
renderer.domElement.addEventListener('wheel', e=>{
  e.preventDefault();
  camRadius *= (1 + e.deltaY*0.001);
  camRadius = Math.max(300, Math.min(6000, camRadius));
  updateCamera();
}, {passive:false});

function resize(){
  const w = holder.clientWidth, h = holder.clientHeight;
  renderer.setSize(w,h);
  camera.aspect = w/h;
  camera.updateProjectionMatrix();
}
new ResizeObserver(resize).observe(holder);
