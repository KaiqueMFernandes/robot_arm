import subprocess
import time
import sys

def main():
    print("[Orchestrator] Starting Python Plotter...")
    # 1. Start the plotter script asynchronously
    # Replace 'python plotter.py' with your plotter command and arguments
    plotter_process = subprocess.Popen([sys.executable, "tools/live_logger.py"])

    # Give the plotter a brief moment to open its window / bind its port
    time.sleep(1.5) 

    print("[Orchestrator] Starting C++ Simulator...")
    try:
        # 2. Start the C++ simulator
        # .subprocess.Popen allows it to run concurrently, or use .run() if you want to block until it finishes
        sim_process = subprocess.Popen(["./build/robot_arm"])

        # Wait for the C++ simulator to finish
        sim_process.wait()

    except KeyboardInterrupt:
        print("\n[Orchestrator] Shutting down simulation gracefully...")
    
    finally:
        # Ensure plotter closes when the C++ sim exits
        plotter_process.terminate()
        plotter_process.wait()
        print("[Orchestrator] Clean shutdown complete.")

if __name__ == "__main__":
    main()