import subprocess
import sys

if __name__ == "__main__":
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

    # # Run the PlatformIO build command
    # command = "pio run -e development"
    # try:
    #     subprocess.run(command, shell=True, check=True, text=True, capture_output=True)
    # except subprocess.CalledProcessError as e:
    #     print(f"{e.stdout.strip()}")
    #     print(f"{e.stderr.strip()}")
    #     print("Tests failed. Please resolve the issues before proceeding.")
    #     exit(1)

    # # Exit with code 0 if everything is fine.
    # exit(0)
