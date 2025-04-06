#!/usr/bin/env python3

import subprocess
import os

def main():
    # İlk komut: cd ~/Micro-XRCE-DDS-Agent/build
    agent_build_path = os.path.expanduser('~/Micro-XRCE-DDS-Agent/build')
    os.chdir(agent_build_path)

    # İkinci komut: ./MicroXRCEAgent udp4 -p 8888
    agent_process = subprocess.Popen(['./MicroXRCEAgent', 'udp4', '-p', '8888'])

    # Üçüncü komut: PX4_GZ_WORLD=walls make px4_sitl gz_x500_depth
    px4_env = os.environ.copy()
    px4_env['PX4_GZ_WORLD'] = 'walls'
    px4_process = subprocess.Popen(['make', 'px4_sitl', 'gz_x500_depth'], env=px4_env)

    # İşlemlerin tamamlanmasını bekleyin
    agent_process.wait()
    px4_process.wait()

if __name__ == '__main__':
    main()
