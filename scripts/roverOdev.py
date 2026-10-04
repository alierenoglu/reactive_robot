#!/usr/bin/env python3
import math  
import time  
import rospy  # ROS 1 haberlesmesi.
from sensor_msgs.msg import LaserScan  # Hazir lazer mesaji 
from geometry_msgs.msg import Twist  # Hazir ileri ve donus hizi mesaji
from gazebo_msgs.srv import DeleteLight  # Gazebo isik silmeK icin
from nav_msgs.msg import Odometry  # Robotun yonunu tasiyan mesaj icin



sonLazerKaydi = None  # scanCallBack son lazer mesajini ve alinma zamanini buraya koyucaz
sonOdomKaydi = None  # odomCallBack son yon mesajini ve alinma zamanini buraya koyucaz


pi = math.pi


def scanCallBack(msg):     # subscriberindan yeni olcumler gelince bu fonksiyon cagrilir
    global sonLazerKaydi    
    sonLazerKaydi = (msg, time.monotonic())


def odomCallBack(msg):   #subscriberindan yeni olcumler gelince bu fonksiyon cagrilir
    global sonOdomKaydi    
    sonOdomKaydi = (msg, time.monotonic())





def normalize(aci):
    return pi - (pi -aci) % (2*pi)  #  açıyı -180 ile 180 arasına alıyoruz kontrolü daha kolay


def lazerAnlz(msg, sensorx, sensory):   # lazer verilerini alip engellerin x ve y koordinatlarini elde edecek
    if not msg.ranges or msg.angle_increment <= 0:   #lazer verisi almamissak direkt none dödürüyoruz
    
        return None  

    engelNoktalari = []  # başlangıçta boş bi liste
    onMesafe = float("inf")  # sonsuz koyuyoruz daha yakın gelince minle güncellicez
    onIsinSayisi = 0 


    for siraNo, mesafe in enumerate(msg.ranges): # mesajı enumarete ile sıra numarasıyla değişkene atıyoruz
        aci = msg.angle_min + siraNo * msg.angle_increment  # açı hesapla
        
        aci = normalize(aci)

        onAlan = abs(aci) < math.radians(100)  # on alandakileri alıyoruz



        if onAlan:
            onIsinSayisi += 1  # on alandaki olcum sayisini artir

        if math.isinf(mesafe) and mesafe > 0.0:  # eger mesafe sonsuz ise engel yok diye kabul ediyoruz
            continue  # bu olcumu atla

        if not (msg.range_min <= mesafe <= msg.range_max):  # mesafe range in  içinde değilse veriyi almayıp atlıyoruz 
            if onAlan:
                return None
            continue


        engelX = sensorx + mesafe * math.cos(aci)  # engel noktasinin x koordinatini hesapla
        engelY = sensory + mesafe * math.sin(aci)  # engel noktasinin y koordinatini hesapla
        engelNoktalari.append((engelX, engelY))  # engel noktasini listeye ekle

        if abs(aci)<= math.radians(20):
            onMesafe = min(onMesafe, math.hypot (engelX,engelY))  # 40 derece önümüzdeki engeli değişkene kaydediyoruz daha sonra durumda yazdırıcaz

    if onIsinSayisi == 0:
        return None             # on olcum yoksa hareket secilemez.

    return engelNoktalari, onMesafe    


#Haraket Tahmin 


def hareketTahmin(ileriHizi, donusHizi, tahminSuresi, adimSuresi, donusSuresi):  # robot için farklı hız değerleri için tahmini yolunu buluyoruz
    
    tahminiYol = [(0.0, 0.0)]  # Baslangic robot merkezi

  



    adimSayisi = round(tahminSuresi / adimSuresi)  # adım sayısını buluyoruz 
    duzHareket = abs(donusHizi) < 1e-6  # dönmüyorsak düz hareket kabul edicez


    if not duzHareket:
        donusYaricapi = ileriHizi / donusHizi  #  lineer hızı açısal hıza bölünce dönülen yarıçapı elde ederiz R = v / ω

    for adimNo in range(1, adimSayisi + 1):            # 1 den adım sayısı kadar 

        zaman = adimNo * adimSuresi  # tahmin süresi adım 

        if duzHareket:
            robotX = ileriHizi * zaman
            robotY =  0.0
        else:
            donulenSure = min(zaman, donusSuresi)  # Hedef yone ulasilana kadar gecen sure.
            duzMesafe = ileriHizi * (zaman - donulenSure)  # Donusten sonra gidilen duz yol.
            donusAcisi = donusHizi * donulenSure
            sinus = math.sin(donusAcisi)
            kosinus = math.cos(donusAcisi)
            robotX = donusYaricapi * sinus + duzMesafe * kosinus
            robotY = donusYaricapi * (1.0 - kosinus) + duzMesafe * sinus


        tahminiYol.append((robotX, robotY))
    return tahminiYol





def crashKontrol(tahminiYol, engelNoktalar, kontrolRSqr):  #  çarpışma kontrolü yapmak için kullanılacak

    for robotX, robotY in tahminiYol:               # her robot x i ve y si için her engel x i ve y sini karşılaştırıp çarpışma kontrolü yapılıyor 
        for engelX, engelY in engelNoktalar:
            farkX = robotX - engelX
            farkY = robotY - engelY

            if farkX ** 2 + farkY **2 <= kontrolRSqr:
                return True                 # tahmini yolumuz için engellerinden birinde çarpışma gerçekleşirse true döner

    return False



def hedefSec(engelNoktalar, yonFarki, ayarlar):  #    robotun ilerleyeceği yönü seçme
    
        
    robotYaricapi = ayarlar["robot_radius"] 
    robotYaricapKare = robotYaricapi**2
    yonFootSize = 2.0 * ayarlar["maksimumYonFarki"] / (ayarlar["yonAdaySayisi"] - 1)   # aday yönler arasındaki derece
    enIyiYonPuani = -float("inf")  # negatiften başlatıyoruz
    hedefAcisi = None
    hedefUzakligi = 0.0

    for yonNo in range(ayarlar["yonAdaySayisi"]):

        #koridorAcisi = -ayarlar["maksimumYonFarki"] + yonNo * yonFootSize  # başlangıç yönüne göre aday yönün açısı
        koridorAcisi = ayarlar["maksimumYonFarki"] - yonNo * yonFootSize  # soldan (+) baslayip saga (-) dogru tara
        adayAcisi = koridorAcisi - yonFarki             # robotun anlık yönüne göre aday yönün açısı


        if abs(adayAcisi) > pi / 1.95:
            continue            # Robotun baktığı yönü yaklaşık olarak 180 dereceyle sınırladım

        kosinus = math.cos(adayAcisi)
        sinus = math.sin(adayAcisi)
        acikMesafe = ayarlar["hedefAramaMesafesi"]  # robotun önü açıksa boş kabul edeceği uzaklık

        for engelX, engelY in engelNoktalar:
            boyunaMesafe = engelX * kosinus + engelY * sinus
            yanMesafe = -engelX * sinus + engelY * kosinus


            if boyunaMesafe > 0.0 and abs(yanMesafe) < robotYaricapi:
                ilkTemasMesafesi = boyunaMesafe - math.sqrt(max(0.0, robotYaricapKare - yanMesafe ** 2))
                acikMesafe = min(acikMesafe, max(0.0, ilkTemasMesafesi))                #aday yönde ilerlerken ilk temas anlarını kaydeder ve aday yöndeki açık mesafeyi kaydeder
                if acikMesafe == 0.0:
                    break               # yon kapalı 

        if acikMesafe == 0.0:               #yon kapalı olduğu durumda puanlama yapma   
            continue

        yonPuani = acikMesafe * math.cos(koridorAcisi) - ayarlar["donusDegisimAgirligi"] * abs(adayAcisi)   # başlangıç dan x ekseninde pozitife doğru ilerlemeyi ödüllendirdim , dönüş sıklığını azaltmak için ceza verdim

        if yonPuani > enIyiYonPuani:            #daha iyisini buldukça hedef açıyı belirliyoruz
            enIyiYonPuani = yonPuani
            hedefAcisi = adayAcisi
            hedefUzakligi = acikMesafe  
    return hedefAcisi, hedefUzakligi


def hizSec(engelNoktalar, onMesafe, prDonusHizi, yonFarki, ayarlar):        # seçtiğimiz hedefe ilerlerken ileri hızı ve dönüş hızını belirliyor 
  
    hedefAcisi, hedefUzakligi = hedefSec(engelNoktalar, yonFarki, ayarlar)

    if hedefAcisi is None:
        rospy.loginfo_throttle(1.0, "Durdu, acik hedef bulunamadi. On mesafe: %.3f m", onMesafe)
        return 0.0, 0.0

    hedefX = hedefUzakligi * math.cos(hedefAcisi)       # hedefin koordinatlarını alıyoruz
    hedefY = hedefUzakligi * math.sin(hedefAcisi)

    hizAdimi = (ayarlar["maksimumIleriHiz"] - ayarlar["minimumIleriHiz"]) / (ayarlar["hizAdaySayisi"] - 1)      #denenecek ileri hız adaylarını belirliyor
    ileriBestPoint = -float("inf")
    
    donusSiniri = ayarlar["maksimumDonusHiz"] 
    donusAdimi = 2.0 * donusSiniri / (ayarlar["donusAdaySayisi"] - 1)     #deneyeceğimiz dönüş hızlarının adım farkı
    
    kontrolRSqr = (ayarlar["robot_radius"] )**2   




    secilenHareket = (0.0, 0.0)
    carpismaSayisi = 0
    yonElemeSayisi = 0

    for hizNo in range(ayarlar["hizAdaySayisi"] - 1, -1, -1):    # her ileri hız için her dönüş hızını denicez

        ileriHizi = ayarlar["minimumIleriHiz"] + hizNo * hizAdimi   # maksimum denenecek hızdan başlayarak hepsini deniyoruz
        


        for donusNo in range(ayarlar["donusAdaySayisi"]):

            #donusHizi = -donusSiniri + donusNo * donusAdimi        # sağa doğru dönüşten başlayarak dönüş hızı için adaylar belirleniyor 
            donusHizi = donusSiniri - donusNo * donusAdimi        # sola donusten (+) baslayip saga (-) dogru adaylar
            donusSuresi = ayarlar["tahminSuresi"]


            if abs(donusHizi) < 1e-6:                   #donus hızı çok çok küçükse sıfır kabul ediyoruz         
                donusHizi = 0.0  
            

            if donusHizi * hedefAcisi > 0.0:
                donusSuresi = min(donusSuresi, abs(hedefAcisi / donusHizi))         # açı = açısal hız * süre yani bu hızla dönersem hedefe bakmam kaç sn süreri cevaplar
            sonrakiYon = yonFarki + donusHizi * donusSuresi

            if abs(sonrakiYon) > ayarlar["maksimumYonFarki"]:     # başlangıç yönünden maksimumYonFarki'ndan fazla sapacak adayları eliyoruz
                yonElemeSayisi += 1
                continue

          
            tahminiYol = hareketTahmin(ileriHizi, donusHizi, ayarlar["tahminSuresi"], ayarlar["adimSuresi"], donusSuresi)
          
            if crashKontrol(tahminiYol, engelNoktalar, kontrolRSqr):

                carpismaSayisi += 1                                     # çarpışma varsa eleniyor ve sayım yapılıyor
                continue
            
            sonX, sonY = tahminiYol[-1]

            hedefeUzaklik = math.hypot(hedefX - sonX, hedefY - sonY)  # hedefe kalan mesafe

            hedefYonHatasi = abs(hedefAcisi - donusHizi * donusSuresi)

            donusHiziDegisimi = abs(donusHizi - prDonusHizi)
            
            point = ileriHizi - ayarlar["ilerlemeAgirligi"] * hedefeUzaklik - ayarlar["hedefYonAgirligi"] * hedefYonHatasi - ayarlar["donusDegisimAgirligi"] * donusHiziDegisimi
            if point > ileriBestPoint:
                ileriBestPoint = point
                secilenHareket = (ileriHizi, donusHizi)

    if secilenHareket == (0.0, 0.0):
        rospy.loginfo_throttle(1.0, "Durdu: on mesafe=%.3f m, hedef=%.1f derece, yonElenen=%d, carpismaElenen=%d", onMesafe, math.degrees(hedefAcisi), yonElemeSayisi, carpismaSayisi)
    return secilenHareket


def hizGonder (hizPub, ileriHizi, donusHizi):
    hizmsg = Twist()            # ros un hazir hiz mesajindan yeni bir mesaj olustur
    hizmsg.linear.x = ileriHizi  # lineer  hizini ayarla
    hizmsg.angular.z = donusHizi  # acisal  hizini ayarla
    hizPub.publish(hizmsg)

def rosOdevBitti(hizPub,scanSub, odomSub):
    hizGonder(hizPub, 0.0, 0.0)  # robotu durdur
    scanSub.unregister()  # sublar kapatıldı
    odomSub.unregister()  

    rospy.loginfo("60 Saniye doldu. Robot durduruldu. Abonelikler kapatildi." )  # bilgi mesaji yazdir

    try:
        rospy.wait_for_service('/gazebo/delete_light', timeout=2)  # delete_light servisini bekle
        isikSil = rospy.ServiceProxy('/gazebo/delete_light', DeleteLight)  # delete_light servisine baglan
        sonuc = isikSil("sun")

        if sonuc.success:
            rospy.loginfo("Sun isigi silindi.")  # isik silme basariliysa bilgi mesaji yazdir
        else:
            rospy.logwarn("Sun isigi silinemedi: %s", sonuc.status_message)  # isik silme basarisizsa uyari mesaji yazdir
    except (rospy.ServiceException, rospy.ROSException) as error:
        rospy.logwarn("Sun isigi silinemedi: %s", error)  # servis cagrisi basarisizsa uyari mesaji yazdir

    finally:
        rospy.signal_shutdown("Program tamamlandi.")

def main():
    rospy.init_node("roverOdev")  # ROS node unu baslatıyoruz

    ayarlar = {
        "maksimumIleriHiz": rospy.get_param("~maksimumIleriHiz"),  
        "minimumIleriHiz": rospy.get_param("~minimumIleriHiz"),  
        "maksimumDonusHiz": rospy.get_param("~maksimumDonusHiz"),  
        "maksimumYonFarki": rospy.get_param("~maksimumYonFarki"),  
        "tahminSuresi": rospy.get_param("~tahminSuresi"),  
        "adimSuresi": rospy.get_param("~adimSuresi"),  
        "robot_radius": rospy.get_param("~robot_radius"),  
        "lazerX": rospy.get_param("~lazerX"), 
        "lazerY": rospy.get_param("~lazerY"), 
        "lazerZamanAsimi": rospy.get_param("~lazerZamanAsimi"),  
        "hedefAramaMesafesi": rospy.get_param("~hedefAramaMesafesi"), 
        "ilerlemeAgirligi": rospy.get_param("~ilerlemeAgirligi"),  
        "hedefYonAgirligi": rospy.get_param("~hedefYonAgirligi"),  
        "donusDegisimAgirligi": rospy.get_param("~donusDegisimAgirligi"), 
        "yonAdaySayisi": rospy.get_param("~yonAdaySayisi"),  
        "hizAdaySayisi": rospy.get_param("~hizAdaySayisi"), 
        "donusAdaySayisi": rospy.get_param("~donusAdaySayisi"),  
    }
# ayarlar için hata kontrolleri
    for isim in ("yonAdaySayisi", "hizAdaySayisi", "donusAdaySayisi"):
        if not isinstance(ayarlar[isim], int) or ayarlar[isim] < 2:
            raise ValueError(isim + " en az 2 olan bir tam sayi olmali.")
    if ayarlar["yonAdaySayisi"] % 2 == 0 or ayarlar["donusAdaySayisi"] % 2 == 0:
        raise ValueError("yonAdaySayisi ve donusAdaySayisi tek sayi olmali. Ornek: 71 ve 21.")
    if ayarlar["minimumIleriHiz"] > ayarlar["maksimumIleriHiz"]:
        raise ValueError("minimumIleriHiz, maksimumIleriHiz degerini asamaz.")
    if not 0.0 < ayarlar["maksimumYonFarki"] < math.pi / 2.0:
        raise ValueError("maksimumYonFarki 0 ile pi/2 arasinda olmali. Ornek: 1.5.")

    



    hizPub =rospy.Publisher("/cmd_vel", Twist, queue_size=1)                # cmd_vel topicine publisher olusturur
    scanSub =rospy.Subscriber("/scan", LaserScan, scanCallBack, queue_size=1) # yeni gelen lazer mesajlarini scanCallBack e aktaracak
    odomSub = rospy.Subscriber("/odom", Odometry, odomCallBack, queue_size=1)  # topic adı mesaj tipi callback fonksiyonu


    def durdurucu():
        hizGonder(hizPub, 0.0, 0.0)  # robotu durdur


    rospy.on_shutdown(durdurucu)  # ROS kapanirken durdurucu fonksiyonunu cagir


    baslangicZamani = None # baslangic zamanini None olarak baslat, hareket gelene kadar sure baslamadi
    baslangicYonu = None
    prIleriHizi = 0.0  # onceki ileri hizi 
    prDonusHizi = 0.0  # onceki donus hizi
    prDurum = None # onceki hareket durum

    rospy.loginfo("Rover odev basladi. Hareket baslayinca 60 saniye sayac basladi.")  # bilgi mesaji yazdir

    dongu = rospy.Rate(1.0 / ayarlar["adimSuresi"])  # saniyede kac tur donecegi (Hz)

    while not rospy.is_shutdown():
         
     
        lazerKaydi = sonLazerKaydi             # global degiskenden lazer kaydini al
        odomKaydi = sonOdomKaydi            # global degiskenden odom kaydini al

        if lazerKaydi is None or time.monotonic() - lazerKaydi[1] > ayarlar["lazerZamanAsimi"]:
            ileriHizi, donusHizi = 0.0, 0.0
            durum = "Lazer verisi yok veya eski"

        elif odomKaydi is None or time.monotonic() - odomKaydi[1] > ayarlar["lazerZamanAsimi"]:
            ileriHizi, donusHizi = 0.0, 0.0
            durum = "Yon verisi yok veya eski"

        else:
            yon = odomKaydi[0].pose.pose.orientation
            robotYonu = math.atan2(2.0 * (yon.w * yon.z + yon.x * yon.y), 1.0 - 2.0 * (yon.y * yon.y + yon.z * yon.z))

            if baslangicYonu is None:
                baslangicYonu = robotYonu

            yonFarki = math.atan2(math.sin(robotYonu - baslangicYonu), math.cos(robotYonu - baslangicYonu))  # Yon farkini -180 ile +180 derece arasinda hesapla.

          

            tarama = lazerAnlz(lazerKaydi[0], ayarlar["lazerX"], ayarlar["lazerY"])

            if tarama is None:
                ileriHizi, donusHizi = 0.0, 0.0 # lazer verisi gecersizse robotu durdur
                durum = "Lazer verisi gecersiz"  # durum mesajini ayarla
            else:
                engelNoktalar, onMesafe = tarama  # engel noktalarini ve on mesafeyi al
                ileriHizi, donusHizi = hizSec(engelNoktalar, onMesafe, prDonusHizi,yonFarki, ayarlar) # hiz ve donus hizini sec

                if ileriHizi == 0.0 :
                    durum = "Duruyor"
                elif donusHizi > 0.0001:
                    durum = "Sola donuyor"
                elif donusHizi < -0.0001:
                    durum = "Saga donuyor"
                else:
                    durum = "Duz gidiyor"

        if baslangicZamani is not None:
            gecenSure = (rospy.Time.now() - baslangicZamani).to_sec()
            if gecenSure >= 60.0:
                rosOdevBitti(hizPub, scanSub, odomSub)
                break

        hizGonder(hizPub, ileriHizi, donusHizi)  # hiz ve donus hizini gonder

        if baslangicZamani is None and ileriHizi != 0.0:
            baslangicZamani = rospy.Time.now()              # hareket basladiginda baslangic zamanini ayarla
            rospy.loginfo("Hareket basladi. Zaman baslatildi.")  # bilgi mesaji yazdir

        if durum != prDurum or abs(ileriHizi - prIleriHizi) >= 0.02 or abs(donusHizi- prDonusHizi) >= 0.1:
            rospy.loginfo("%s - linear hizi: %.3f m/s, angular hizi: %.3f rad/s", durum, ileriHizi, donusHizi)
            prDurum = durum # durum guncellenir

        prIleriHizi = ileriHizi
        prDonusHizi = donusHizi      # önceki hizlar guncellenir
       
        dongu.sleep()  # turun toplam suresi adimSuresi olacak sekilde bekle



if __name__ == "__main__":  # Dosya dogrudan calistirilinca programi baslatir.
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except (KeyError, ValueError) as hata:
        rospy.logerr("Ayar veya veri hatasi: %s", hata)
        rospy.signal_shutdown("Program durduruldu.")
        raise SystemExit(1)
