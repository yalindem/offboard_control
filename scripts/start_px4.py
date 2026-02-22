import os
import subprocess
import argparse
import time
import signal
import sys

# Global süreç değişkenleri
agent_process = None
px4_process = None
qgc_process = None
ros_process = None

def start_px4(drone_type: str, world: str):
    global px4_process
    try:
        os.chdir(os.path.expanduser("~/PX4-Autopilot"))
        print("--- [1/4] PX4 SITL Başlatılıyor... ---")
        
        if world is None:
            command = f"make px4_sitl {drone_type}"
        else:
            command = f"PX4_GZ_WORLD={world} make px4_sitl {drone_type}"
        
        # Süreç grubunu (process group) yönetebilmek için start_new_session=True kullanıyoruz
        px4_process = subprocess.Popen(command, shell=True, preexec_fn=os.setsid)
        print(f"PX4 Komutu: {command}")
    except Exception as e:
        print(f"PX4 başlatılırken hata: {e}")

def start_dds():
    global agent_process
    try:
        print("--- [2/4] MicroXRCEAgent Başlatılıyor... ---")
        # Agent genelde path'e ekli olduğu için direkt çağrılabilir
        agent_process = subprocess.Popen(
            ["MicroXRCEAgent", "udp4", "-p", "8888"], 
            preexec_fn=os.setsid
        )
        print(f"MicroXRCEAgent PID: {agent_process.pid}")
    except Exception as e:
        print(f"DDS Agent başlatılırken hata: {e}")

def start_qgc():
    global qgc_process
    try:
        print("--- [3/4] QGroundControl Başlatılıyor... ---")
        # Sisteminizde 'qgc' komutu tanımlı değilse tam yolunu yazmalısınız
        qgc_process = subprocess.Popen(["qgc"], preexec_fn=os.setsid)
    except Exception as e:
        print(f"QGC başlatılamadı (Yolun doğru olduğundan emin olun): {e}")

def start_ros_launch():
    global ros_process
    try:
        print("--- [4/4] ROS2 Launch Başlatılıyor... ---")
        # Önce workspace'i source eder, sonra launch komutunu çalıştırır
        command = "source ~/workspace/ros_ws/install/setup.bash && ros2 launch offboard_control drone_bringup.launch.py"
        
        # executable='/bin/bash' ekliyoruz çünkü 'source' bir bash komutudur
        ros_process = subprocess.Popen(
            command, 
            shell=True, 
            executable='/bin/bash', 
            preexec_fn=os.setsid
        )
    except Exception as e:
        print(f"ROS2 Launch error: {e}")
def signal_handler(sig, frame):
    print('\n\n' + '!'*20)
    print("Kapatma sinyali alındı. Tüm süreçler sonlandırılıyor...")
    
    # Başlattığımız tüm süreçleri gruplarıyla beraber öldürüyoruz
    processes = {
        "ROS2": ros_process,
        "DDS Agent": agent_process,
        "PX4": px4_process,
        "QGC": qgc_process
    }

    for name, proc in processes.items():
        if proc and proc.poll() is None:
            try:
                print(f"{name} sonlandırılıyor...")
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            except Exception as e:
                print(f"{name} kapatılırken hata: {e}")

    print("Temizlik tamamlandı. Çıkış yapılıyor.")
    sys.exit(0)

def run(args):
    # Ctrl+C yakalayıcı
    signal.signal(signal.SIGINT, signal_handler)

    # 1. QGroundControl (Opsiyonel, en başta açılması iyidir)
    start_qgc()
    time.sleep(2)

    # 2. PX4 SITL (Gazebo ile birlikte)
    start_px4(args.drone, args.world)
    # Gazebo'nun yüklenmesi ve PX4'ün initialize olması için uzunca bekleme
    print("Gazebo ve PX4'ün açılması bekleniyor (10sn)...")
    time.sleep(10)

    # 3. DDS Agent (PX4 ile ROS2 arasındaki köprü)
    start_dds()
    time.sleep(15)

    # 4. ROS2 Nodes (Senin yazdığın kontrol kodu)
    start_ros_launch()

    print("\n" + "="*40)
    print("TÜM SİSTEMLER ÇALIŞIYOR")
    print("Durdurmak için: CTRL+C")
    print("="*40)

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="PX4 SITL ve ROS2 Otomasyonu")
    parser.add_argument(
        "--drone", "-d",
        default="gz_x500",
        help="Drone tipi (Örn: gz_x500, gz_iris). Varsayılan: gz_x500"
    )
    parser.add_argument(
        "--world", "-w",
        default=None,
        help="Gazebo dünyası (Örn: baylands, khtspacelab vb.)"
    )
    
    args = parser.parse_args()
    run(args)
