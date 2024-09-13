This repo contains microROS dev environments and bash scripts used by the **BYU Agricultural Robotics Team**. 
If you're doing code development, feel free to clone this repo directly, make a new branch, and start coding and pushing some changes.
Otherwise (if you're looking to set up a new robot from scratch), see this repo for instructions on how to get our custom Docker image running instead: https://github.com/BYUAgrobotics/AgrobotSetup

A quick high-level overview of the repo:
- **agrobot/** - PlatformIO dev environment for the Teensy.
Source code and header files are included in "agrobot/src/" and "agrobot/include/", and dependencies can be imported by modifying the "agrobot/platformio.ini" file.
Different sensors can be enabled/disabled by commenting out the #define statements at the top of "agrobot/src/main.cpp" and rebuilding.
- **firmware_options/** - pre-compiled firmware hex files that can be uploaded to the vehicles.
Custom hex files added to this directory can be uploaded to the Teensy boards using the "upload.sh" script.
- **scripts (build.sh, upload.sh, etc)** - automates helpful software tasks on the robot.
For example, running "bash msg_update.sh" will remove the microROS library from the workspace and rebuild it using updated message and service declarations from "agrobot/extra_packages."
A description of what each script does is included as a header comment in the file.
