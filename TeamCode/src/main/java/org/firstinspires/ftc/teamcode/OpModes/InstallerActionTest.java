package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.roadrunner.SequentialAction;
import org.firstinspires.ftc.teamcode.Controllers.Installer.InstallerAction;
import org.firstinspires.ftc.teamcode.Controllers.Installer.InstallerController;
import org.firstinspires.ftc.teamcode.Controllers.RobotStates;

@TeleOp
@Config
public class InstallerActionTest extends LinearOpMode {
    public static double pos = 0.8;
    public void runOpMode() {
        boolean ifdown = false;
InstallerController tmp = new InstallerController(hardwareMap, gamepad1, gamepad2, telemetry);
        InstallerAction installerController = new InstallerAction(hardwareMap, gamepad1, gamepad2, telemetry);
        waitForStart();
        Actions.runBlocking(
                new SequentialAction(
                        //in         stallerController.clipInstaller(false),
                        installerController.installerPuller()
                )
        );
        while(opModeIsActive()){
            if(gamepad1.a){
                Actions.runBlocking(
                        installerController.spitClip()
                );

                //installerController.SetCurrentNum(1);
            }
            if(gamepad1.b){
                Actions.runBlocking(
                        installerController.installerPuller()
                );
            }
            if(gamepad1.x){
                ifdown = true;
                Actions.runBlocking(
                        installerController.beamSpinner(ifdown)
                );
            }
            if(gamepad1.y){
                ifdown = false;
                Actions.runBlocking(
                        installerController.beamSpinner(ifdown)
                );
            }
            if(gamepad2.a){
                tmp.setMode(RobotStates.INSTALL_RUNMODE.BACKING);
            }
            if(gamepad2.b){
                tmp.setMode(RobotStates.INSTALL_RUNMODE.EATING);
            }
            tmp.run();

//                //installerController.SingleInstallerControl(pos);
//
//            telemetry.addData("currentnum", installerController.getCurrentNum());
//            telemetry.addData("ifdown", ifdown);
//            telemetry.addData("dis" , installerController.disSensor.getDis());
//            telemetry.addData("InstallerState", installerController.getInstallStates());
//            telemetry.update();
//            installerController.run();

        }
    }
}