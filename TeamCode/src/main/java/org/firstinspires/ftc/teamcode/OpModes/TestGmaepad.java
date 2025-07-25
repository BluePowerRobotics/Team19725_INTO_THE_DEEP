package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TestGmaepad extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while( opModeIsActive() ) {
            telemetry.addData("Left Stick X", gamepad1.left_stick_x);
            telemetry.addData("Left Stick Y", gamepad1.left_stick_y);
            telemetry.addData("Right Stick X", gamepad1.right_stick_x);
            telemetry.addData("Right Stick Y", gamepad1.right_stick_y);
            telemetry.addData("Right trigger", gamepad1.right_trigger);
            telemetry.addData("Left trigger", gamepad1.left_trigger);
            telemetry.addData("Left Bumper", gamepad1.left_bumper);
            telemetry.addData("Right Bumper", gamepad1.right_bumper);
            telemetry.addData("A Button", gamepad1.a);
            telemetry.addData("B Button", gamepad1.b);
            telemetry.addData("X Button", gamepad1.x);
            telemetry.addData("Y Button", gamepad1.y);
            telemetry.addData("Dpad_down",gamepad1.dpad_down);
            telemetry.addData("Dpad_down", gamepad1.dpad_up);
            telemetry.addData("Dpad_left", gamepad1.dpad_left);
            telemetry.addData("Dpad_right", gamepad1.dpad_right);
            telemetry.update();
        }
    }
}
