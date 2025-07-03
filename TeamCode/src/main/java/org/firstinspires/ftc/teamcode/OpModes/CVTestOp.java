package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RoadRunner.*;
import org.firstinspires.ftc.teamcode.VisualColor.*;
import org.firstinspires.ftc.teamcode.Controllers.*;

@TeleOp
public class CVTestOp extends LinearOpMode {
    //MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
    FindCandidate CVmoudle = new FindCandidate();
    private DcMotor armMotor;
    @Override
    public void runOpMode() {
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Initialize the camera and other components here
        CVmoudle.init(hardwareMap, telemetry);

        waitForStart();

        while (opModeIsActive()) {
            // Process camera frames and detect colors
            telemetry.addData("running to", CVmoudle.findCandidate().toString());
            if(CVmoudle.findCandidate().suggestion == 1) {
                telemetry.addData("suggestion", "需要车辆左移");
            }
            else if(CVmoudle.findCandidate().suggestion == 2) {
                telemetry.addData("suggestion", "需要车辆右移");
            }
            else if(CVmoudle.findCandidate().suggestion == 3) {
                telemetry.addData("suggestion", "需要滑轨前移");
            }
            else {
                telemetry.addData("suggestion", "不需要移动");
            }
            double armPower = 0;
            if (gamepad2.right_trigger > 0.1) {
                armPower = gamepad2.right_trigger;
            } else if (gamepad2.left_trigger > 0.1) {
                armPower = -gamepad2.left_trigger;
            }
            armMotor.setPower(armPower);
            telemetry.addData("Arm Position", armMotor.getCurrentPosition());
            telemetry.addData("Arm Power", armPower);
//            drive.setDrivePowers(
//                new PoseVelocity2d(
//                    new Vector2d(gamepad1.left_stick_x, gamepad1.left_stick_y),
//                    gamepad1.right_stick_x
//                )
//            );

            // Display detected colors and other information

            telemetry.update();
        }
    }

}
