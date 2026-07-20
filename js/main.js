// ================= main.js =================
// Giriş noktası. Uygulama durumunu (DH parametreleri, eksen açıları) tanımlar,
// arayüzü kurar (initUI — params/thetasDeg'e ihtiyaç duyduğu için burada, state
// tanımlandıktan SONRA çağrılır) ve render döngüsünü başlatır.
// Yükleme sırası: three.js (CDN) -> utils.js -> kinematics.js -> scene.js -> robot.js -> ui.js -> main.js

// ---------- App state ----------
const params = { d1:162.5, a2:425, a3:392.3, d4:133.3, d5:99.6, d6:99.6 };
const thetasDeg = [0, 0, 90, 0, 90, 0]; // start pose, degrees

// ---------- Bootstrap ----------
initUI();

resize();
updateCamera();
updateFK();
(function animate(){ requestAnimationFrame(animate); renderer.render(scene, camera); })();
