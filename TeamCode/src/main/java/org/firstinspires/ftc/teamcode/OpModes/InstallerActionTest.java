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

        InstallerController installerController = new InstallerController(hardwareMap, gamepad1, gamepad2, telemetry);
        installerController.setMode(RobotStates.INSTALL_RUNMODE.WAITING);
        waitForStart();
//        Actions.runBlocking(
//
//                new SequentialAction(
//                        installerController.clipInstaller(false),
//                        installerController.installerPuller(),
//                        installerController.beamSpinner(1)
//                )
//        );
        while(opModeIsActive()){
            if(gamepad1.a){
                installerController.setMode(RobotStates.INSTALL_RUNMODE.EATING);
                //installerController.SetCurrentNum(1);
            }
            if(gamepad1.left_bumper){
                ifdown = false;
                installerController.BeamSpinner(ifdown);
            }
            if(gamepad1.right_bumper){
                ifdown = true;
                installerController.BeamSpinner(ifdown);
            }
                //installerController.SingleInstallerControl(pos);

            telemetry.addData("currentnum", installerController.getCurrentNum());
            telemetry.addData("ifdown", ifdown);
            telemetry.addData("dis" , installerController.disSensor.getDis());
            telemetry.addData("InstallerState", installerController.getInstallStates());
            telemetry.update();
            installerController.run();

        }
    }
}