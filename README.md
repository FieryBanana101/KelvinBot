# Kelvinbot v0.1 [Thermal visualizer robot]

![kelvinbot](https://github.com/FieryBanana101/KelvinBot/blob/main/media/Screenshot%20from%202024-12-29%2014-47-39.png)

Kelvinbot adalah rancangan robot beroda yang dapat melakukan visualisasi suhu pada lingkungan sekitarnya. Motivasi dari dibuatnya robot ini yaitu dapat menjadi ide bagi rancangan robot *Search and Rescue* ataupun robot bawah air yang memiliki penglihatan minim.

## Tools:
#### - Ros2 Jazzy Calisco
#### - Gazebo harmonic
#### - OpenCV2
#### - Python3

## Panduan Instalasi
Pertama buat sebuah workspace ros2 (atau jika sudah ada, langsung pindah ke direktori workspace),

```bash
mkdir ros2_ws && cd ros2_ws
```

Kemudian, clone repositori ini dan lakukan build dengan colcon serta lakukan setup.

```bash
git clone https://github.com/FieryBanana101/KelvinBot
colcon build --symlink-install
source install/setup.bash
```
Instalasi selesai, jalankan simulasi dengan menggunakan ```ros2 launch```:

```bash
ros2 launch kelvinbot kelvinbot.launch.py
```

**Notes:**
Setiap membuka terminal baru perlu dilakukan setup dengan ```install/setup.bash```, agar tidak harus selalu setup manual maka tambahkan command berikut ke ```~/.bashrc```:

```bash
echo "source $PATH_TO_WORKSPACE/install/setup.bash" >> ~/.bashrc
```

Ganti ```$PATH_TO_WORKSPACE``` dengan path workspace yang dimiliki.


## Manajemen Direktori

```bash
kelvinbot
    ├── CMakeLists.txt
    ├── include
    │   └── kelvinbot
    ├── package.xml
    └── src
        ├── config
        ├── scripts
        │   ├── kelvinbot.launch.py
        │   ├── object_detection.py
        │   └── __pycache__
        │       ├── kelvinbot.launch.cpython-312.pyc
        │       ├── launch.cpython-312.pyc
        │       ├── launcher.cpython-312.pyc
        │       ├── launcher_src.cpython-312.pyc
        │       └── object_detection.cpython-312.pyc
        ├── sdf
        │   └── world.sdf
        └── urdf
            ├── inertial_macros.xacro
            ├── robot_body.xacro
            ├── robot.urdf
            ├── robot.urdf.xacro
            ├── thermal_camera.xacro
            └── wheel_drive.xacro
```
Komponen-komponen robot akan dibangun melalui urdf file yang terletak di ```./src/urdf``` dibagi menjadi beberapa bagian, dan akan dikombinasikan menggunakan xml macro (xacro). File komponen-komponen robot diantaranya:
- inertial_macros.xacro: macro untuk tensor inersia objek 3d  
- robot_body.xacro: bagian utama robot  
- wheel_drive.xacro: bagian roda dan sistem kendali robot  
- thermal_camera.xacro: bagian kamera termal sebagai penglihatan robot  
- robot.urdf.xacro: keseluruhan sistem robot  
- robot.urdf: file urdff akhir yang akan dijadikan model pada simulasi

![kelvinbot](https://github.com/FieryBanana101/KelvinBot/blob/main/media/Screenshot%20from%202024-12-29%2014-52-52.png) 
![kelvinbot](https://github.com/FieryBanana101/KelvinBot/blob/main/media/Screenshot%20from%202024-12-29%2014-53-49.png) 
![kelvinbot](https://github.com/FieryBanana101/KelvinBot/blob/main/media/Screenshot%20from%202024-12-29%2014-52-37.png)
![kelvinbot](https://github.com/FieryBanana101/KelvinBot/blob/main/media/Screenshot%20from%202024-12-29%2014-53-27.png)
![kelvinbot](https://github.com/FieryBanana101/KelvinBot/blob/main/media/Screenshot%20from%202024-12-29%2014-53-17.png)


Komponen utama robot terdiri dari balok yang terhubung dengan 4 roda yang menggunakan *differential drive*, pada bagian atas balok terdapat silinder yang terhubung dengan kamera termal 16bit.

Robot akan disimulasikan pada lingkungan yang telah dirancang pada ```./src/sdf/world.sdf```. Pada lingkungan tersebut telah disiapkan objek dengan suhu bermacam-macam pada rentang 250 K hingga 1000 K.

Terdapat sebuah launch file ```./src/scripts/kelvinbot.launch.py``` dan sebuah script computer vision ```./src/scripts/object_detection.py``` yang menjalani logika dari pemrosesan gambar thermal.

Pada dasarnya, Launch File akan melakukan:
- Inisialisasi world.sdf pada gazebo
- Spawn robot.urdf pada gazebo
- Aktivasi ```ros_gz_bridge``` untuk topic ```/thermal_camera```
- Aktivasi object_detection.py
- Inisialisasi ```rqt_image_view```

Untuk memudahkan prosess build, terdapat file ```package.xml``` dan ```CMakeLists.txt``` yang akan mengatur dependensi dan kepentingan build lainnya seperti instalasi launch file, mengeksport environment variable yang diperlukan dan lain-lain.

## Alur Kerja ROS2 dan Gazebo

![kelvinbot](https://github.com/FieryBanana101/KelvinBot/blob/main/media/Screenshot%20from%202024-12-29%2015-31-26.png)


Terdapat system plugin differential drive yang menghubungkan kedua roda depan dari robot. Plugin tersebut akan menerima data dari topic gazebo ```/cmd_vel``` berupa tipe pesan ```geometry_msgs/msg/Twist``` (Kecepatan linear di sumbu-x dan Rotasi terhadap sumbu-z).

Pusat kendali robot adalah melalui arrow key. Input akan didapat melalui GUI plugin gazebo ```key publisher```, plugin akan melakukan publish data tipe ```gz.msgs.Int32``` ke topic ```/keyboard/keypress``` yang berupa kode keyboard yang ditekan.

Topic ```/keyboard/keypress``` kemudian akan melakukan publish data ke system plugin gazebo ```triggered publisher``` yang akan melakukan publish tipe data ```geometry_msgs/msg/Twist``` yang sesuai ke ```/cmd_vel```. Akan dicocokkan kode keyboard yang datang dengan pesan tipe ``````geometry_msgs/msg/Twist`````` yang harus dikirim ke differential drive.

Thermal camera 16 bit Gazebo akan melakukan publish data tipe ```gz.msgs.Image``` ke topic ```/thermal_camera```.

Kemudian dengan menggunakan ```ros_gz_bridge``` topic ```/thermal_camera``` akan dihubungkan ke sistem ROS. 

Dengan menggunakan ```rclpy```, setiap frame dari ```/thermal_camera``` akan diambil. Selain itu juga akan dibuat topic ```/image_processor``` dan topic ```/final_image```.

Topic ```/image_processor``` akan melakukan pemrosesan gambar dari ```/thermal_camera``` dengan konversi dari ```gz.msgs.Image``` (tipe gazebo) menjadi ```sensor_msgs/msg/Image``` (tipe ROS) melalui ```cv_bridge``` dan objek deteksi melalui ```opencv``` dan ```numpy```. 

Frame yang telah diproses akan dipublish ke topic ```/final_image``` dan akan dapat dilihat melaui ```rqt_image_view```

## Penjelasan alur kerja dan demonstrasi:
```console
https://youtu.be/ck__dUD6nkg
```
