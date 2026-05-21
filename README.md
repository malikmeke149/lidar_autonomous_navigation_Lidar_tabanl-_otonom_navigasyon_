[README (1).md](https://github.com/user-attachments/files/28114344/README.1.md)
# Sensör Füzyonu ve Lokalizasyon Kullanılarak LiDAR Tabanlı Otonom Navigasyon Benchmark Analizi

## 1. Giriş ve Senaryo Tanımı
[cite_start]Bu proje kapsamında, 80x40 metre boyutlarında iki boyutlu (2B) bir çalışma alanında, non-holonomic (diferansiyel sürüş kısıtlı) bir mobil robot modeli için gelişmiş otonom navigasyon ve sensör füzyonu mimarisi tasarlanmıştır[cite: 1326]. 
[cite_start]Robotun görevi, harita üzerinde (4.0, 8.0, 0.0) başlangıç noktasından harekete geçerek 10 adet dengeli dağıtılmış dikdörtgen formda statik engeli aşıp (75.0, 36.0) uzak hedef noktasına güvenli bir şekilde ulaşmaktır[cite: 1329]. [cite_start]Simülasyonda tekerlek enkoderi, IMU ve 2B LiDAR sensör karakteristikleri stokastik gürültü koşullarıyla birlikte modellenmiştir[cite: 1331].

## 2. Kullanılan Yöntemler
Projede üç temel mimari kullanılmıştır:
* [cite_start]**LiDAR Veri İşleme ve Kümeleme:** Robotun çevresine 36 adet ışınsal (ray-casting) LiDAR tarama vektörü fırlatılarak ham mesafe verileri toplanır[cite: 1335]. [cite_start]Bu gürültülü veriler eşikleme filtresine tabi tutulduktan sonra 3.0 tolerans katsayılı Öklid mesafesi tabanlı bir yakınlık analizi (Clustering) ile engeller tespit edilir[cite: 1336, 1337].
* [cite_start]**Genişletilmiş Kalman Filtresi (EKF):** Robotun kinematik durum vektörü $x_t = [x, y, \theta]^T$ tahmini için tekerlek enkoderlerinden gelen hız vektörü ve IMU üzerinden alınan baş açısı ölçümleri EKF sensör füzyonu ile harmanlanmaktadır[cite: 1338, 1339].
* [cite_start]**Yol Planlama (Benchmark):** Geliştirilen çalışma alanında 7 farklı küresel ve reaktif yol planlama algoritması kıyaslanmıştır: A*, Dijkstra, D* Lite (Reaktif), RRT, RRT*, PRM ve Q-Learning[cite: 1349].

## 3. Sonuçlar ve Grafikler
Gerçekleştirilen çoklu algoritma navigasyon testleri sonucunda A*, Dijkstra, RRT ve RRT* algoritmalarının, engeller etrafından güvenli bir şekilde dolaşarak hedef tolerans alanına girmeyi başardığı ve görevi tamamladığı gözlemlenmiştir[cite: 1462]. EKF sensör füzyonu algoritması, simülasyondaki stokastik gürültüye rağmen gerçek konum ile tahmin arasındaki sapmayı dar bir bantta tutmayı başarmıştır[cite: 1465].
1.<img width="627" height="312" alt="image" src="https://github.com/user-attachments/assets/d9ee4140-b705-4630-8fc2-dc91107185fa" />

2.<img width="627" height="234" alt="image" src="https://github.com/user-attachments/assets/b80d8577-c898-498f-a830-ee2ee0f13b0a" />

3.<img width="596" height="370" alt="image" src="https://github.com/user-attachments/assets/114cf153-ce5b-4887-b6d7-462d513047bc" />

4.<img width="590" height="471" alt="image" src="https://github.com/user-attachments/assets/5067948c-208e-4a43-af9d-731c50783cb8" />

*(Not: Simülasyondaki grafikler kodu çalıştırdığınızda otomatik olarak oluşturulmaktadır. Görsel çıktılar ayrıca depoda yer almaktadır.)*

## 4. Hata Analizi ve Kısa Tartışma
Elde edilen performans metrikleri (Yörünge Boyu, Çalışma Süresi, Konum RMSE) şu sonuçları ortaya koymuştur:
* [cite_start]**Başarı Durumu:** En kısa yörüngeyi 76.79 m ile A* üretirken, en yüksek hesaplama yükü 2.49 sn ile RRT* algoritmasında gözlenmiştir[cite: 1463].
* [cite_start]**Başarısız Görevler:** D* Lite, PRM ve model tabansız Q-Learning ajanları, lokal dar geçitler ve haritaya atanan kinematik kısıtlar nedeniyle kısıtlı bir mesafe kat ettikten sonra hedefe ulaşamayarak başarısız olmuşlardır[cite: 1464].
* [cite_start]**Lokalizasyon ve RMSE:** EKF mekanizması sensör hatalarını başarıyla izole etmiş ve anlık gürültü salınımlarını sönümleyerek konum tahminindeki kararlılığını kanıtlamıştır[cite: 1465].

## 5. Kaynaklar ve Yapay Zeka Kullanım Beyanı

* [cite_start]**Yapay Zeka Kullanım Beyanı:** Bu projede Gemini Advanced kullanılarak EKF matematiksel denklemlerinin LaTeX diline dönüştürülmesi, raporun akademik formatlanması ve tablo bileşenlerinin renklendirilmesinde destek alınmıştır[cite: 1468, 1469]. [cite_start]Proje senaryosunun tasarımı, kodlanması, test edilmesi ve grafik analizi tamamen öğrenci çalışmasıdır[cite: 1470].

* **Kaynaklar:**
  1. [cite_start]V. Ušinskis, M. Nowicki, A. Dzedzickis ve V. Bučinskas, "Sensor-fusion based navigation for autonomous mobile robot," *Sensors*, cilt 25, sayı 4, makale 1248, 2025[cite: 1472].
  2. [cite_start]Y. Ou, Y. Cai, Y. Sun ve T. Qin, "Autonomous navigation by mobile robot with sensor fusion based on deep reinforcement learning," *Sensors*, cilt 24, sayı 12, makale 3895, 2024[cite: 1473].
  3. [cite_start]B. Zhang ve C. Li, "The optimization and application research of the RRT-APF-based path planning algorithm," *Electronics*, cilt 13, sayı 24, makale 4963, 2024[cite: 1474].

## 6. Kurulum ve Kullanım Talimatları
Bu proje Python programlama dili ile geliştirilmiştir. Projeyi kendi bilgisayarınızda çalıştırabilmeniz için aşağıdaki adımları izleyin.

**Gereksinimler:**
* Python 3.x
* Matplotlib kütüphanesi (`matplotlib`)
* Numpy kütüphanesi (`numpy`)

**Kurulum Adımları:**
1. Proje dosyalarını bilgisayarınıza indirin veya bu depoyu klonlayın.
2. Terminalinizi veya komut satırınızı açıp şu komutu yazarak gerekli kütüphaneleri kurun:
   ```bash
   pip install numpy matplotlib
