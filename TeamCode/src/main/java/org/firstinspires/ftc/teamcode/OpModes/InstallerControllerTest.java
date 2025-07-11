package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.roadrunner.SequentialAction;
import org.firstinspires.ftc.teamcode.Controllers.InstallerController;
@TeleOp

public class InstallerControllerTest extends LinearOpMode {
    public void runOpMode() {
        InstallerController installerController = new InstallerController(hardwareMap, gamepad1, gamepad2, telemetry);
        waitForStart();
        Actions.runBlocking(

                new SequentialAction(
                        installerController.clipInstaller(false),
                        installerController.installerPuller(),
                        installerController.beamSpinner(1)
                )
        );
    }
}