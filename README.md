# Rover

This repository contains the default code for all boards in the CanEduDev Rover.

For more documentation, visit [CanEduDev's Documentation website](https://www.canedudev.com/getting-started-with-the-rover/).

Feel free to submit issues if anything is unclear.

## Working with the repository

This project is built using [Meson](https://mesonbuild.com/) with the Ninja backend.

Supported OS: Ubuntu 24.04. If you're using Windows, you can utilize the Ubuntu 24.04 [WSL](https://learn.microsoft.com/en-us/windows/wsl/) distribution.

### Prerequisites

1. Run `./scripts/bootstrap.sh` to set up the build environment. This step installs all dependencies and prepares the build folder.
2. Enter virtual environment: `source .venv/bin/activate`

It is required to activate the virtual environment to run most tasks in the repo. As such, all the next sections assume the environment has been sourced.

## Building the Firmware

Execute `meson compile -C build` to initiate the build process. The build output can be found in the `build` folder.

## Building the ROS2 Gateway

There is a ROS gateway available which exposes the Rover's CAN messages as ROS topics, and allows the user to control the Rover via ROS. The gateway is deployed as a docker container and available as part of this repository's Github packages.

The gateway can be built locally using `meson compile -C build ros-gateway`.

## Building the Code Documentation

To build the documentation, run `meson compile -C build docs`. The HTML output is located in the `build/docs/html` folder.

## Building a Release

Generate a zip file containing the board binaries by running `meson compile -C build release`. The output is stored in the `build` folder.

## Running Tests

To run all unit tests, execute `meson test -C build`.

Additionally, there are integration tests that run against the boards. Note that these tests require hardware that supports canlib, such as the [Kvaser Leaf Light](https://www.kvaser.com/product/kvaser-leaf-light-hs-v2/).

The integration tests are found in `rover_py/tests`. They can be run using `python <path-to-test>.py`.

## Using STM32CubeMX to Generate Code

The hardware initialization code for various boards was initially generated using [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html). STM32CubeMX projects are defined in `.ioc` files located in the `boards` directory.

Follow these steps to generate code:
1. Load a project in STM32CubeMX.
2. Click "GENERATE CODE" in the top right corner.
3. If prompted to download firmware, proceed with the download.

## Flashing Binaries onto the Board

There are two steps to flash the binaries.
1. Flash the board via SWD using an STM32 programmer such as the [STLINK-V3SET programmer](https://www.st.com/en/development-tools/stlink-v3set.html) along with the [STM32CubeProgrammer software](https://www.st.com/en/development-tools/stm32cubeprog.html). For rovers delivered with a working software, this step can be skipped.

2. Flash the binary and configuration via CAN using a Kvaser interface. Use the "fw_update.py" program by calling "python fw_update.py" from the release folder. This will flash all boards in a system with the correct binary and configuration. Do not interrupt this process as this will brick your devices. 

Bricked devices can be restored by following step 1.

### Upgrading a working system's firmware
To update a working system, download the latest release and follow step 2 above.
