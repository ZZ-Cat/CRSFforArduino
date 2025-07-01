import subprocess
import sys

if __name__ == "__main__":
    # Run the PlatformIO build command
    command = "pio run -e development"
    try:
        subprocess.run(command, shell=True, check=True, text=True, capture_output=True)
    except subprocess.CalledProcessError as e:
        print(f"{e.stdout.strip()}")
        print(f"{e.stderr.strip()}")
        print("Tests failed. Please resolve the issues before proceeding.")
        exit(1)

    # Exit with code 0 if everything is fine.
    exit(0)
