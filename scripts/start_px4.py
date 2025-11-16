import os
import subprocess
import argparse
import time
import signal
import sys

agent_process = None

def start_px4(drone_type: str):
    try:
        os.chdir(os.path.expanduser("~/PX4-Autopilot"))
        print("Building and starting PX4 SITL...")
        subprocess.Popen(["make", "px4_sitl", drone_type])
    except FileNotFoundError as e:
        print(f"Error: Directory not found - {e}")
    except subprocess.CalledProcessError as e:
        print(f"Error: Command failed - {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")


def start_dds():
    global agent_process
    try:
        os.chdir(os.path.expanduser("~/Micro-XRCE-DDS-Agent"))
        print("Starting MicroXRCEAgent...")
        agent_process = subprocess.Popen(["MicroXRCEAgent", "udp4", "-p", "8888"])
        print(f"MicroXRCEAgent started with PID: {agent_process.pid}")
    except FileNotFoundError as e:
        print(f"Error: Directory not found - {e}")
    except subprocess.CalledProcessError as e:
        print(f"Error: Command failed - {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")


def start_qgc():
    try:
        print("Starting QGroundControl...")
        qgc_process = subprocess.Popen(["qgc"])
    except FileNotFoundError as e:
        print(f"Error: Directory not found - {e}")
    except Exception as e:
        print(f"An unexpected error occurred while starting QGC: {e}")

def get_process_id(process_name):
    try:
        pid_output = subprocess.check_output(
            ["pgrep", "-f", process_name],
            text=True,
            stderr=subprocess.PIPE
        ).strip()
        if pid_output:
            return pid_output.split()[0]
        else:
            return None
    except subprocess.CalledProcessError as e:
        if e.returncode == 1:
            return None
        else:
            raise


def kill_process_by_id(id: str):
    if id:
        print(f"Terminating process with PID: {id}")
        try:
            subprocess.run(["kill", "-9", id], check=True)
            print(f"Process {id} successfully killed.")
        except subprocess.CalledProcessError as e:
            print(f"Error killing process {id}: {e}")
        except Exception as e:
            print(f"An unexpected error occurred while killing process: {e}")
    else:
        print("No process ID provided to kill.")

def signal_handler(sig, frame):
    print('\n\nCtrl+C (SIGINT) received. Initiating graceful shutdown...')
    global agent_process

    if agent_process and agent_process.poll() is None:
        print(f"Killing MicroXRCEAgent (PID: {agent_process.pid}) started by this script...")
        agent_process.kill()
    else:
        dds_pid = get_process_id("MicroXRCEAgent")
        if dds_pid:
            kill_process_by_id(dds_pid)
        else:
            print("MicroXRCEAgent process not found or already terminated.")

    sys.exit(0)


def run(args):
    signal.signal(signal.SIGINT, signal_handler)

    # Start the processes
    start_qgc()
    time.sleep(1)
    start_px4(args.drone)
    time.sleep(1)
    start_dds()
    time.sleep(1)

    print("\nPress Ctrl+C to terminate all processes and exit.")
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        # This block is theoretically unreachable because the signal handler exits the script.
        pass


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Start PX4 SITL with specified drone type.")
    parser.add_argument(
        "--drone", "-d",
        default="gz_x500",
        help="Drone type to use (e.g., gz_iris, gz_x500, gz_standard_vtol). Default: gz_x500"
    )
    args = parser.parse_args()
    run(args)