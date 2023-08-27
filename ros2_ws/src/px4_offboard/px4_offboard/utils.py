import psutil
import subprocess
import time

def capture_external_stdout(self, pid):
        ready = False
        self.get_logger().info(f" *** waiting for px4 to be ready ...***")
        try:
            while not ready:
                process = subprocess.run(["tail", "-f", f"/proc/{pid}/fd/1"], 
                                         stdout=subprocess.PIPE, 
                                         stderr=subprocess.PIPE, 
                                         text=True,
                                         timeout=1.0)
                if process.returncode == 0:
                    if "Ready for takeoff!" in process.stdout:
                        self.get_logger().info(f" *** ready to takeoff detected! ***")
                        ready = True
                else:
                    self.get_logger().info(f" *** Error: {process.stderr}")
                time.sleep(1.0)
                
        except Exception as e:
            self.get_logger().info(f" **** An error occurred: {e}")

def wait_for_px4(self):
    try:
        for process in psutil.process_iter(['pid', 'cmdline']):
            cmdline = process.info['cmdline']
        if cmdline and "px4" in " ".join(cmdline):
            pid = process.info['pid']
            self.get_logger().info(f" *** got px4,  pid = {pid}")
            self.capture_external_stdout(pid)
    except Exception as e:
            print(f"An error occurred: {e}")

