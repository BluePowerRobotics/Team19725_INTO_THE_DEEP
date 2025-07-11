package org.firstinspires.ftc.teamcode.Controllers.Installer;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.sun.tools.javac.code.Attribute;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.Controllers.DisSensor;
import org.firstinspires.ftc.teamcode.Controllers.RobotStates.INSTALL_RUNMODE;
public class InstallerController{
    boolean isUpping = false;
    double Not_Installing = 0;
    double Install_Finished = 0.2;
    double InstallPos = 80;
    double ClipLength = 10;
    int CurrentNum = 1;
    HardwareMap hardwareMap;
    Gamepad gamepad1;
    Gamepad gamepad2;
    Telemetry telemetry;
    Servo clipInstaller,clipInstallPuller,beamSpinner;
    INSTALL_RUNMODE installStates = INSTALL_RUNMODE.WAITING;
    DisSensor disSensor = new DisSensor();
    public InstallerController(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        // Initialize the installer with the provided hardware map and game pads
        // This method can be used to set up any necessary components or configurations
        // for the installation process.
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        clipInstaller = this.hardwareMap.get(Servo.class,"servoc0");
        clipInstallPuller = this.hardwareMap.get(Servo.class,"servoc1");
        beamSpinner = this.hardwareMap.get(Servo.class,"servoc2");
        clipInstaller.setDirection(Servo.Direction.FORWARD);
        clipInstallPuller.setDirection(Servo.Direction.FORWARD);
        beamSpinner.setDirection(Servo.Direction.FORWARD);

        disSensor.init(hardwareMap);

        // Example initialization code (to be replaced with actual implementation):
        telemetry.addData("Installer", "Initialization started");
        telemetry.update();

        // Perform any setup tasks here...

        telemetry.addData("Installer", "Initialization complete");
        telemetry.update();
    }
    boolean PrepareInited = false;
    long UpStartTime = 0;
    boolean InstallInited = false;
    long installStartTime = 0;
    public void SetCurrentNum(int currentNum) {
        // Set the current number of clip
        this.CurrentNum = currentNum;
    }
    public void BeamSpinner(boolean ifdown){
        if (ifdown) {
            if(!isUpping){
                beamSpinner.setPosition(0.8);
                UpStartTime = System.currentTimeMillis();
                isUpping = true;
            }
            if(System.currentTimeMillis() - UpStartTime > 500 && System.currentTimeMillis() - UpStartTime < 1000){
                beamSpinner.setPosition(0);
            }
        } else {
            beamSpinner.setPosition(0.6);
        }
    }
    public void setMode(INSTALL_RUNMODE installStates) {
        // Set the installation mode to the specified run mode
        this.installStates = installStates;
        switch (this.installStates) {
            case WAITING:
                clipInstaller.setPosition(Not_Installing);
                clipInstallPuller.setPosition(1);
                if(disSensor.getDis() > InstallPos){
                    clipInstallPuller.setPosition(0.5);
                }
                break;
            case EATING:
                clipInstallPuller.setPosition(0);
                if(disSensor.getDis() < (6 - CurrentNum) * ClipLength) {
                    clipInstallPuller.setPosition(0.5);
                    this.installStates = INSTALL_RUNMODE.WAITING;
                } else {
                    clipInstallPuller.setPosition(0);
                }
                break;
            case INSTALLING:
                clipInstaller.setPosition(Install_Finished);
                break;
            case BACKING:
                if(disSensor.getDis() > InstallPos + 10) {
                    clipInstallPuller.setPosition(0.5);
                }
                else{
                    clipInstallPuller.setPosition(1);
                }
                break;
        }
    }
}
