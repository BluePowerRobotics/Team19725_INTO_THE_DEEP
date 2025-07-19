package org.firstinspires.ftc.teamcode.Controllers.Installer;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Controllers.DisSensor;
import org.firstinspires.ftc.teamcode.Controllers.RobotStates.INSTALL_RUNMODE;

public class InstallerAction {
    public boolean isUpping = false;
    public static double Beam_low = 0.15;//与installer对接
    public static double Beam_mid = 0.16666666666;//从观察区取clip
    public static double Beam_high = 0.35;//提起clip，使之脱离观察区樯
    double InstallPos = 233;
    double SixthClipInstallPos = 58;
    double ClipLength = 25;
    double SpitDis = 40;
    int CurrentNum = 1;


    boolean IFstartBacking = false;
    double WaitStartTime = 0;
    HardwareMap hardwareMap;
    Gamepad gamepad1;
    Gamepad gamepad2;
    Telemetry telemetry;
    Servo clipInstaller,clipInstallPuller,beamSpinner;
    INSTALL_RUNMODE installStates = INSTALL_RUNMODE.WAITING;
    DisSensor disSensor = new DisSensor();
    public InstallerAction(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
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
    long prepareStartTime = 0;
    boolean InstallInited = false;
    long installStartTime = 0;
    public class ClipInstaller implements Action{
        private final boolean install;
        public ClipInstaller(boolean install) {
            this.install = install;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (install) {
                clipInstaller.setPosition(0.8);
            } else {
                clipInstaller.setPosition(0);
            }
            telemetry.addData("Clip Installer", clipInstaller.getPosition());
            return false;
        }
    }
    public Action clipInstaller(boolean install) {
        return new ClipInstaller(install);
    }
    public class InstallerPuller implements Action{
        private boolean initialized = false;
        int state = 0;
        double ClipPosition;
        long UpstartTime;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            double dis = disSensor.getDis();
            if (!initialized) {
                clipInstallPuller.setPosition(1);
                initialized = true;
            }
            if(CurrentNum <= 6) {
                if (state == 0) {
                    ClipPosition = SixthClipInstallPos + (6 - CurrentNum) * ClipLength;
                    clipInstallPuller.setPosition(1);
                    UpstartTime = System.currentTimeMillis();
                    state = 1;
                    telemetry.addData("Puller", "Moving to clip: " + CurrentNum);
                    return true;
                }
                if (state == 1) {
                    if (dis <= ClipPosition ||
                            //todo 更改时间
                            System.currentTimeMillis() - UpstartTime > 2000) {

                        clipInstallPuller.setPosition(0.5);
                        state = 2;
                        telemetry.addData("Puller", "Reached clip: " + CurrentNum);
                    }
                    return true;
                }
                if (state == 2) {
                    clipInstallPuller.setPosition(0);
                    if (dis >= InstallPos ||
                            System.currentTimeMillis() - UpstartTime > 2000 * 2) {
                        CurrentNum++;
                        state = 3;
                        telemetry.addData("Puller", "Returned to center");
                        return false;
                    }
                }
                telemetry.addData("Clip Install Puller", "Pulling clip " + CurrentNum);
                telemetry.update();
            }
            return true;
        }
    }
    public Action installerPuller() {
        // Create an action to pull the installer
        return new InstallerPuller();
    }
    public class SpitClip implements Action{
        private boolean initialized = false;
        int state = 3;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            double dis = disSensor.getDis();
            if (!initialized) {
                clipInstallPuller.setPosition(0);
                initialized = true;
            }
            if (CurrentNum <= 6) {
                if (state == 3) {
                    clipInstallPuller.setPosition(0);
                    if (dis >= InstallPos + SpitDis) {
                        clipInstallPuller.setPosition(0.5);
                        WaitStartTime = System.currentTimeMillis();
                        state = 4;
                        telemetry.addData("Puller", "Spit specimen");
                    }
                    return true;
                }
                if (state == 4) {
                    clipInstallPuller.setPosition(0.5);
                    if (System.currentTimeMillis() - WaitStartTime > 500){
                        IFstartBacking = true;
                        clipInstallPuller.setPosition(1);
                    }
                    if (dis <= InstallPos && IFstartBacking) {
                        clipInstallPuller.setPosition(0.5);
                        telemetry.addData("Puller", "Final center");
                        IFstartBacking = false;
                        return false;
                    }
                    return true;
                }
                telemetry.addData("Clip Install Puller", "Pulling clip " + CurrentNum);
                telemetry.update();
            }
            return false;
        }
    }
    public Action spitClip() {
        // Create an action to pull the installer
        return new SpitClip();
    }
    long UpStartTime = 0;
    public class BeamSpinner implements Action{
        private final boolean ifdown;
        public BeamSpinner(boolean ifdown) {
            this.ifdown = ifdown;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (ifdown) {
                isUpping = false;
                beamSpinner.setPosition(Beam_mid);
                return false;
            }
            else {
                beamSpinner.setPosition(Beam_high);
                if(!isUpping){
                    UpStartTime = System.currentTimeMillis();
                    isUpping = true;
                }
                if(System.currentTimeMillis() - UpStartTime > 300 && System.currentTimeMillis() - UpStartTime < 1000){
                    beamSpinner.setPosition(Beam_low);
                    return false;
                }
            }
            telemetry.addData("Beam Spinner", beamSpinner.getPosition());
            telemetry.update();
            return System.currentTimeMillis() - UpStartTime <= 300 || System.currentTimeMillis() - UpStartTime >= 1000;
        }
    }
    public Action beamSpinner(boolean ifdown) {
        return new BeamSpinner(ifdown);
    }
//    public void setMode(INSTALL_RUNMODE installStates) {
//        // Set the installation mode to the specified run mode
//        this.installStates = installStates;
//        switch (this.installStates) {
//            case WAITING:
//                PrepareInited = false;
//                clipInstallPuller.setPosition(0.5);
//                clipInstaller.setPosition(0);
//                break;
//            case EATING:
//                clipInstaller.setPosition(0);
//                if(!PrepareInited) {
//                    // Perform any necessary preparation tasks here
//                    // For example, initializing components or setting up configurations
//                    telemetry.addData("Installer", "Preparing for installation");
//                    telemetry.update();
//                    prepareStartTime = System.currentTimeMillis();
//                    PrepareInited = true;
//                }
//                if(System.currentTimeMillis()-prepareStartTime < 1000) {
//                    clipInstallPuller.setPosition(0);
//                    telemetry.addData("Installer", "Preparing");
//                    telemetry.update();
//                }
//                break;
//            case INSTALLING:
//                PrepareInited = false;
//                clipInstallPuller.setPosition(0.5);
//
//                if(!InstallInited) {
//                    // Perform any necessary preparation tasks here
//                    // For examp le, initializing components or setting up configurations
//                    telemetry.addData("Installer", "install init");
//                    telemetry.update();
//                    installStartTime = System.currentTimeMillis();
//                    InstallInited = true;
//                }
//                if(System.currentTimeMillis()-installStartTime < 1000) {
//                    clipInstaller.setPosition(0.8);
//                    telemetry.addData("Installer", "Installing");
//                    telemetry.update();
//                }else{
//                    clipInstaller.setPosition(0);
//                    telemetry.addData("Installer", "Installation complete");
//                }
//                break;
//        }
//    }
}
