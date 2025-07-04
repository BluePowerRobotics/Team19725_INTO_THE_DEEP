package org.firstinspires.ftc.teamcode.Controllers;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.Controllers.RobotStates.INSTALL_RUNMODE;
public class InstallerController {
    HardwareMap hardwareMap;
    Gamepad gamepad1;
    Gamepad gamepad2;
    Telemetry telemetry;
    Servo clipInstallPuller,clipInstaller;
    INSTALL_RUNMODE installStates = INSTALL_RUNMODE.PREPARING;
    public void initInstaller(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        // Initialize the installer with the provided hardware map and game pads
        // This method can be used to set up any necessary components or configurations
        // for the installation process.
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        clipInstaller = this.hardwareMap.get(Servo.class,"");
        clipInstallPuller = this.hardwareMap.get(Servo.class,"");
        clipInstaller.setDirection(Servo.Direction.FORWARD);
        clipInstallPuller.setDirection(Servo.Direction.FORWARD);
        // Example initialization code (to be replaced with actual implementation):
        telemetry.addData("Installer", "Initialization started");
        telemetry.update();

        // Perform any setup tasks here...

        telemetry.addData("Installer", "Initialization complete");
        telemetry.update();
    }
    boolean PrepareInited = false;
    long prepareStartTime = 0;
    boolean InstallInited = false;
    long installStartTime = 0;
    public void setMode(INSTALL_RUNMODE installStates) {
        // Set the installation mode to the specified run mode
        this.installStates = installStates;
        switch (this.installStates) {
            case WAITING:
                PrepareInited = false;
                clipInstallPuller.setPosition(0.5);
                clipInstaller.setPosition(0);
                break;
            case PREPARING:
                clipInstaller.setPosition(0);
                if(!PrepareInited) {
                    // Perform any necessary preparation tasks here
                    // For example, initializing components or setting up configurations
                    telemetry.addData("Installer", "Preparing for installation");
                    telemetry.update();
                    prepareStartTime = System.currentTimeMillis();
                    PrepareInited = true;
                }
                if(System.currentTimeMillis()-prepareStartTime < 1000) {
                    clipInstallPuller.setPosition(0);
                    telemetry.addData("Installer", "Preparing");
                    telemetry.update();
                }
                break;
            case INSTALLING:
                PrepareInited = false;
                clipInstallPuller.setPosition(0.5);

                if(!InstallInited) {
                    // Perform any necessary preparation tasks here
                    // For example, initializing components or setting up configurations
                    telemetry.addData("Installer", "install init");
                    telemetry.update();
                    installStartTime = System.currentTimeMillis();
                    InstallInited = true;
                }
                if(System.currentTimeMillis()-installStartTime < 1000) {
                    clipInstaller.setPosition(0.8);
                    telemetry.addData("Installer", "Installing");
                    telemetry.update();
                }else{
                    clipInstaller.setPosition(0);
                    telemetry.addData("Installer", "Installation complete");
                }
                break;
        }
    }
}
