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
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            double dis = disSensor.getDis();
            if (!initialized) {
                clipInstallPuller.setPosition(0);
                initialized = true;
            }
            if (dis > 0) {
                return true;
            } else {
                clipInstallPuller.setPosition(0.5);
            }
            telemetry.addData("Clip Install Puller", clipInstallPuller.getPosition());
            return false;
        }
    }
    public Action installerPuller() {
        // Create an action to pull the installer
        return new InstallerPuller();
    }
    public class BeamSpinner implements Action{
        private final int beamPosition;
        public BeamSpinner(int beanPosition) {
            this.beamPosition = beanPosition;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (beamPosition == 1) {
                beamSpinner.setPosition(0.8);
            } else if (beamPosition == 2) {
                beamSpinner.setPosition(0.6);
            } else if (beamPosition == 3) {
                beamSpinner.setPosition(0);
            }
            telemetry.addData("Beam Spinner", beamSpinner.getPosition());
            telemetry.update();
            return false;
        }
    }
    public Action beamSpinner(int beamPosition) {
        return new BeamSpinner(beamPosition);
    }
    public void setMode(INSTALL_RUNMODE installStates) {
        // Set the installation mode to the specified run mode
        this.installStates = installStates;
        switch (this.installStates) {
            case WAITING:
                PrepareInited = false;
                clipInstallPuller.setPosition(0.5);
                clipInstaller.setPosition(0);
                break;
            case EATING:
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
                    // For examp le, initializing components or setting up configurations
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
