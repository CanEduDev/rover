# FrSky Setup Instructions

This guide details how to set up a FrSky VX10, fresh from the factory, with CanEduDev's default settings.

## Connecting the FrSky radio to PC

1. Power on the transmitter.
2. Connect the transmitter to the PC via USB port.
3. On the transmitter's screen, select "Ethos Suite" from the menu.

The instructions differ for Windows and Linux, see below.

## Windows

1. Install [ETHOS Suite](https://ethos.frsky-rc.com).
2. Open ETHOS Suite and go to "Radio information" in the menu. The transmitter should show up. Click "Manage model", which opens the backup location picker.
3. In the backup location picker, choose the folder in this repo containing the `ETHOS_*.zip` file. If you did this correctly, the interface should show the last backup time.
4. Click "RESTORE" to load the settings into the transmitter. Click OK.
5. Unplug the transmitter.

## Linux

1. Mount the USB storage device that was connected. Usually, your desktop environment gives a notification for this when you connect the transmitter via USB.
2. Extract the `ETHOS_*.zip` file from this repo. This will create a folder named "ETHOS_*". Copy the contents of the new folder to the root of the mounted folder. You will be prompted to overwrite the files. Do that.
3. Unplug the transmitter.

## Final steps

Finally, the model name on the transmitter screen should read "Rover", confirming the settings were loaded. Swipe right on the screen to see the Channel mix. Try moving the sticks and see if everything works.

## Switch reference

In the final configuration, the SA switch is the radio override and the SF switch is the calibration button.

## Receiver

The receiver choice and binding procedure have not been finalized yet. This section will be updated once that's decided.
