# RobotKinematicsApp
Interactive robot kinematics simulator using Three.js with Forward and Inverse Kinematics.


# 6 Eksenli Cobot — İleri &amp; Ters Kinematik Simülatörü

Tarayıcıda çalışan, kurulum gerektirmeyen bir 6 eksenli endüstriyel kol (cobot) simülatörü.
Standart Denavit–Hartenberg (DH) parametreleriyle ileri kinematik (FK) hesaplar, analitik
yöntemle ters kinematik (IK) çözer ve robotu Three.js ile 3 boyutlu olarak görselleştirir.

## Özellikler

- **İleri kinematik (FK):** 6 eksen açısını kaydırıcılarla değiştir, uç efektörün (TCP)
  konum ve oryantasyonunu anlık gör.
- **Ters kinematik (IK):** Hedef konum + oryantasyon gir, robot kolun bu hedefe ulaşması
  için gereken eksen açılarını (olası 8 çözüm dalı: omuz/dirsek/bilek kombinasyonları) hesapla.
- **3B görselleştirme:** Eklem çerçeveleri, uç efektör çerçevesi, etiketler, erişim alanı
  (workspace) küresi, TCP izi — hepsi görünüm panelinden açılıp kapatılabilir.
- **Ayarlanabilir DH parametreleri:** d1, a2, a3, d4, d5, d6 değerlerini değiştirip farklı
  kol geometrilerini deneyebilirsin.

## Proje yapısı

```
RobotProjesi/
├── index.html          # Sayfa iskeleti, tüm script/link etiketleri
├── css/
│   └── style.css        # Tüm görsel stiller
├── js/
│   ├── utils.js          # DH dönüşümleri, RPY↔rotasyon, vektör yardımcıları (bağımsız)
│   ├── kinematics.js      # Analitik IK çözücü + poz geçerlilik/çarpışma kontrolü
│   ├── scene.js           # Three.js sahne/kamera/ışık/orbit kontrolleri kurulumu
│   ├── robot.js           # Robot mesh'leri, gizmolar, etiketler, updateFK() render döngüsü
│   ├── ui.js              # Tüm arayüz kontrolleri (initUI() fonksiyonu olarak)
│   └── main.js            # Uygulama durumu (params, thetasDeg) + başlatma sırası
└── assets/               # Gerekirse ek görsel/statik dosyalar için
```

### Dosyalar arası bağımlılık ve yükleme sırası

`index.html` script'leri şu sırayla yükler, sıra değiştirilmemeli:

```
three.js (CDN) → utils.js → kinematics.js → scene.js → robot.js → ui.js → main.js
```

Sebep: her dosya bir öncekinin tanımladığı fonksiyon/değişkenlere ihtiyaç duyar
(ör. `kinematics.js`, `utils.js`'teki `toDeg`/`normDeg` fonksiyonlarını kullanır).
`ui.js` içeriği `initUI()` adlı tek bir fonksiyona sarılmıştır çünkü `params` ve
`thetasDeg` değişkenlerine ihtiyaç duyar; bunlar `main.js`'te tanımlanır ve `main.js`
kendi state'ini tanımladıktan **sonra** `initUI()`'ı çağırır.

## Nasıl çalıştırılır

Herhangi bir build aracı veya sunucu gerekmez, saf HTML/CSS/JS'dir.

**Yerelde açmak için:**
```bash
git clone https://github.com/Mert-Sahin22/RobotKinematicsApp.git
cd RobotKinematicsApp
```

## Kullanılanlar

- [Three.js](https://threejs.org/) (r128, CDN üzerinden) — 3B render
- Saf JavaScript (framework yok), HTML5, CSS3