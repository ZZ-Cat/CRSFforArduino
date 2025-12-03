import subprocess
import sys

# This script automates the build, deployment, and monitoring process for embedded device code using PlatformIO.
# It includes quality checks before proceeding with deployment or monitoring and allows specifying target environments.
# 
# Usage:
# --deploy                  Deploy the code to the device after building.
# --deploy-and-monitor      Deploy the code to the device and then start the serial monitor.
# --target <environment>    Specify the target environment to build for. This should match the environment names in the PlatformIO configuration file.
#                           Example: --target adafruit_metro_m4
#                           If --target is omitted, the default environment will be used.
# 
# Usage examples:
# python build.py
# python build.py --deploy --target adafruit_metro_m4
# python build.py --deploy-and-monitor --target adafruit_metro_m4

command_argument_deploy = "--deploy"
command_argument_deploy_and_monitor = "--deploy-and-monitor"
command_argument_target = "--target"
command_run_build = "pio run"
command_run_monitor = "pio device monitor"
command_run_qc = "pio check --fail-on-defect=low --fail-on-defect=medium --fail-on-defect=high"

if __name__ == "__main__":
    # Get the command line arguments
    arguments = sys.argv[1:]
    if command_argument_target in arguments:
        # Get the index of the target argument
        target_index = arguments.index(command_argument_target) + 1
        if target_index < len(arguments):
            target_environment = arguments[target_index]
            command_run_build += f" -e {target_environment}"
            command_run_qc += f" -e {target_environment}"
        else:
            print("Error: No target environment specified after --target.")
            exit(1)
    # Run quality checks
    try:
        subprocess.run(command_run_qc, shell=True, check=True, text=True, capture_output=True)
    except subprocess.CalledProcessError as e:
        print(f"{e.stdout.strip()}")
        print(f"{e.stderr.strip()}")
        # print("There were issues detected in the code-base. Please resolve them before proceeding.")
        exit(1)
    # If --deploy was specified as a command line argument, deploy the code to the device.
    if command_argument_deploy in arguments:
        # Run the PlatformIO upload command.
        command_upload = f"{command_run_build} -t upload"
        try:
            subprocess.run(command_upload, shell=True, check=True, text=True, capture_output=True)
        except subprocess.CalledProcessError as e:
            print(f"{e.stdout.strip()}")
            print(f"{e.stderr.strip()}")
            print("Deployment failed. Please resolve the issues before proceeding.")
            exit(1)
    # If --deploy-and-monitor was specified as a command line argument,
    # deploy the code to the device, wait for it to finish, and then start the serial monitor.
    elif command_argument_deploy_and_monitor in arguments:
        # Run the PlatformIO upload command.
        command_upload = f"{command_run_build} -t upload"
        try:
            subprocess.run(command_upload, shell=True, check=True, text=True, capture_output=True)
        except subprocess.CalledProcessError as e:
            print(f"{e.stdout.strip()}")
            print(f"{e.stderr.strip()}")
            print("Deployment failed. Please resolve the issues before proceeding.")
            exit(1)
        # Start the serial monitor.
        try:
            subprocess.run(command_run_monitor, shell=True, check=True, text=True)
        except subprocess.CalledProcessError as e:
            print(f"{e.stdout.strip()}")
            print(f"{e.stderr.strip()}")
            print("Failed to start the serial monitor. Please resolve the issues before proceeding.")
            exit(1)
    # If --target was specified without deploy or monitor, just build for that target.
    elif command_argument_target in arguments:
        try:
            subprocess.run(command_run_build, shell=True, check=True, text=True, capture_output=True)
        except subprocess.CalledProcessError as e:
            print(f"{e.stdout.strip()}")
            print(f"{e.stderr.strip()}")
            print("Build failed. Please resolve the issues before proceeding.")
            exit(1)
    # If no arguments were provided, just build the project.
    elif not arguments:
        try:
            subprocess.run(command_run_build, shell=True, check=True, text=True, capture_output=True)
        except subprocess.CalledProcessError as e:
            print(f"{e.stdout.strip()}")
            print(f"{e.stderr.strip()}")
            print("Build failed. Please resolve the issues before proceeding.")
            exit(1)
    exit(0)
