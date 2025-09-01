import subprocess
import sys

if __name__ == "__main__":
    # Run the defect detector to check for any issues in the code.
    command = "pio check -e defect_detector --fail-on-defect=low --fail-on-defect=medium --fail-on-defect=high"
    try:
        subprocess.run(command, shell=True, check=True, text=True, capture_output=True)
    except subprocess.CalledProcessError as e:
        print(f"{e.stdout.strip()}")
        print(f"{e.stderr.strip()}")
        print("There were issues detected in the code-base. Please resolve them before proceeding.")
        exit(1)

    # Get the command line arguments.
    args = sys.argv[1:]
    # If --deploy was specified as a command line argument, deploy the code to the device.
    if "--deploy" in args:
        # Run the PlatformIO upload command.
        command = "pio run -e development -t upload"
        try:
            subprocess.run(command, shell=True, check=True, text=True, capture_output=True)
        except subprocess.CalledProcessError as e:
            print(f"{e.stdout.strip()}")
            print(f"{e.stderr.strip()}")
            print("Deployment failed. Please resolve the issues before proceeding.")
            exit(1)

    # If --deploy-and-monitor was specified as a command line argument,
    # deploy the code to the device, wait for it to finish, and then start the serial monitor.
    elif "--deploy-and-monitor" in args:
        # Run the PlatformIO upload command.
        command = "pio run -e development -t upload"
        try:
            subprocess.run(command, shell=True, check=True, text=True, capture_output=True)
        except subprocess.CalledProcessError as e:
            print(f"{e.stdout.strip()}")
            print(f"{e.stderr.strip()}")
            print("Deployment failed. Please resolve the issues before proceeding.")
            exit(1)
        # Start the serial monitor.
        command = "pio device monitor"
        try:
            subprocess.run(command, shell=True, check=True, text=True)
        except subprocess.CalledProcessError as e:
            print(f"{e.stdout.strip()}")
            print(f"{e.stderr.strip()}")
            print("Failed to start the serial monitor. Please resolve the issues before proceeding.")
            exit(1)
    
    # Check compatibility across verified devices.
    # TO-DO:
    #   - Conduct static analysis across verified devices.
    #   - May need to update platformio.ini and the other targets files for this.
    #   - For now, this just works.
    elif "--build-on-compatible-devices" in args:
        # No arguments provided—defaults to building across commonly used devices.
        # Because these are the ones that are tested and verified It Just Works.
        command = "pio run"
        try:
            subprocess.run(command, shell=True, check=True, capture_output=True)
        except subprocess.CalledProcessError as e:
            print(f"{e.stdout.strip()}")
            print(f"{e.stderr.strip()}")
            print("Build failed. One or more devices are not compatible.")
            exit(1)

    # If no arguments were specified, run the build command.
    else:
        # Run the PlatformIO build command.
        command = "pio run -e development"
        try:
            subprocess.run(command, shell=True, check=True, text=True, capture_output=True)
        except subprocess.CalledProcessError as e:
            print(f"{e.stdout.strip()}")
            print(f"{e.stderr.strip()}")
            print("Build failed. Please resolve the issues before proceeding.")
            exit(1)

    # Exit with code 0 if everything is fine.
    exit(0)
