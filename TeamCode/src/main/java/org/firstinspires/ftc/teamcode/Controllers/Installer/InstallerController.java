package org.firstinspires.ftc.teamcode.Controllers.Installer;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.sun.tools.javac.code.Attribute;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.Controllers.DisSensor;
import org.firstinspires.ftc.teamcode.Controllers.RobotStates;
import org.firstinspires.ftc.teamcode.Controllers.RobotStates.INSTALL_RUNMODE;
@Config
public class InstallerController{
    public boolean isUpping = false;
    public static double Beam_low = 0.13;//与installer对接
    public static double Beam_mid = 0.25;//从观察区取clip
    public static double Beam_high = 0.35;//提起clip，使之脱离观察区樯

    //todo: find out the correct values for these constants
    public static double Not_Installing = 0;
    public static double Install_Finished = 0.7;

    public double WaitStartTime = 0;

    public static double InstallPos = 216;
    public static double SixthClipInstallPos = 56;
    double ClipLength = 25;
    int CurrentNum = 1;
    HardwareMap hardwareMap;
    Gamepad gamepad1;
    Gamepad gamepad2;
    Telemetry telemetry;
    Servo clipInstaller,clipInstallPuller,beamSpinner;
    INSTALL_RUNMODE installStates = INSTALL_RUNMODE.WAITING;
    public DisSensor disSensor = new DisSensor();
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
        // telemetry.addData("Installer", "Initialization started");
        // telemetry.update();
        beamSpinner.setPosition(Beam_mid);
        // Perform any setup tasks here...

        // telemetry.addData("Installer", "Initialization complete");
        // telemetry.update();
    }
    long UpStartTime = 0;
    boolean InstallInited = false;
    long installStartTime = 0;
    public int getCurrentNum() {
        // Set the current number of clip
        return CurrentNum;
    }
    public RobotStates.INSTALL_RUNMODE getInstallStates(){return this.installStates;}
    public double getdis(){
        return disSensor.getDis();
    }


    public void BeamSpinner(boolean ifdown){
        if (ifdown) {
            isUpping = false;
            beamSpinner.setPosition(Beam_mid);
        }
        else {
             beamSpinner.setPosition(Beam_high);
             if(!isUpping){
                 UpStartTime = System.currentTimeMillis();
                 isUpping = true;
             }
             if(System.currentTimeMillis() - UpStartTime > 300 && System.currentTimeMillis() - UpStartTime < 1000){
                 beamSpinner.setPosition(Beam_low);
                 this.CurrentNum = 1;

            }
        }
    }

    public void SinglePullerControl(double position) {
        // Control the clip installation puller servo to the specified speed

        clipInstallPuller.setPosition(position);
    }
    public void Install() {
        clipInstaller.setPosition(Install_Finished);
    }
    public void NotInstall() {
        clipInstaller.setPosition(Not_Installing);
    }


    // public void SingleBeamSpinnerControl(double position) {
    //     // Control the clip installation puller servo to the specified speed
    //     beamSpinner.setPosition(position);
    // }
    //todo run（）和setmode（）同步更改！！！！！！！
    //0right 1left
    public void setMode(INSTALL_RUNMODE installStates) {
        // Set the installation mode to the specified run mode
        this.installStates = installStates;
        switch (this.installStates) {
            case WAITING:
                //clipInstaller.setPosition(Not_Installing);
                if(disSensor.getDis() > InstallPos){
                    clipInstallPuller.setPosition(0.5);
                }
                else{
                    clipInstallPuller.setPosition(0);
                }
                break;
            case EATING:
                clipInstaller.setPosition(Not_Installing);
                clipInstallPuller.setPosition(1);
                if(disSensor.getDis() < SixthClipInstallPos + (6 - CurrentNum) * ClipLength) {
                    this.CurrentNum++;
                    clipInstallPuller.setPosition(0.5);
                    this.installStates = INSTALL_RUNMODE.WAITING;

                } else {
                    clipInstallPuller.setPosition(1);
                }
                break;
            case INSTALLING:
                clipInstaller.setPosition(Install_Finished);
                break;
            case BACKING:
                clipInstaller.setPosition(Install_Finished);
                if(disSensor.getDis() > InstallPos + 40) {
                    clipInstallPuller.setPosition(0.5);
                    WaitStartTime = System.currentTimeMillis();
                    this.installStates = INSTALL_RUNMODE.RETURNING;
                }
                else{
                    clipInstallPuller.setPosition(0);
                }
                break;
            case RETURNING:
                //clipInstaller.setPosition(Not_Installing);
                if(System.currentTimeMillis() - WaitStartTime > 500){
                    clipInstallPuller.setPosition(1);
                }
                if(disSensor.getDis() < InstallPos){
                    clipInstallPuller.setPosition(0.5);
                    this.installStates = INSTALL_RUNMODE.WAITING;
                }
                break;
        }
    }
    public void run() {
        switch (this.installStates) {
            case WAITING:
                //clipInstaller.setPosition(Not_Installing);

                if(disSensor.getDis() > InstallPos){
                    clipInstallPuller.setPosition(0.5);
                }
                else{
                    clipInstallPuller.setPosition(0);
                }
                break;
            case EATING:
                clipInstaller.setPosition(Not_Installing);
                clipInstallPuller.setPosition(1);
                if(disSensor.getDis() < SixthClipInstallPos + (6 - CurrentNum) * ClipLength) {
                    this.CurrentNum++;
                    clipInstallPuller.setPosition(0.5);
                    this.installStates = INSTALL_RUNMODE.WAITING;
                } else {
                    clipInstallPuller.setPosition(1);
                }
                break;
            case INSTALLING:
                clipInstaller.setPosition(Install_Finished);
                break;
            case BACKING:
                clipInstaller.setPosition(Install_Finished);
                if(disSensor.getDis() > InstallPos + 40) {
                    clipInstallPuller.setPosition(0.5);
                    WaitStartTime = System.currentTimeMillis();
                    this.installStates = INSTALL_RUNMODE.RETURNING;
                }
                else{
                    clipInstallPuller.setPosition(0);
                }
                break;
            case RETURNING:
                //clipInstaller.setPosition(Not_Installing);
                if(System.currentTimeMillis() - WaitStartTime > 500){
                    clipInstallPuller.setPosition(1);
                }
                if(disSensor.getDis() < InstallPos){
                    clipInstallPuller.setPosition(0.5);
                    this.installStates = INSTALL_RUNMODE.WAITING;
                }
                break;

        }
    }
}
