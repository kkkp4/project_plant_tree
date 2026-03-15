import subprocess
import sys

def run_script(script):

    print(f"\n🚀 Running {script}\n")

    result = subprocess.run(
        [sys.executable, script],
        capture_output=False
    )

    print(f"\n✅ {script} finished\n")


def main():

    # STEP 1
    run_script("pot_face_auto_close.py")

    # STEP 2
    run_script("top_pot_stop_auto_close.py")

    # STEP 3
    run_script("apriltag_auto_close.py")

    # STEP 4
    run_script("obj.py")

    print("🎉 Mission Completed")


if __name__ == "__main__":
    main()
