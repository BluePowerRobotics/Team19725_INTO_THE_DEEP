package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Controllers.SixServoArm.ServoValueOutputter;
import org.firstinspires.ftc.teamcode.Controllers.SixServoArm.SixServoArmAction;
import org.firstinspires.ftc.teamcode.RoadRunner.*;
import org.firstinspires.ftc.teamcode.Vision.*;

@TeleOp
public class CVTestOp_BLUE extends LinearOpMode {

    ServoValueOutputter.ClipPosition CurrentClipPosition = ServoValueOutputter.ClipPosition.UNLOCKED;
    MecanumDrive drive;
    SixServoArmAction sixServoArmController;
    FindCandidate CVModule = new FindCandidate();
    private DcMotor armMotor;
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        sixServoArmController = new SixServoArmAction(hardwareMap, telemetry,gamepad2);
//        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
//        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Initialize the camera and other components here
        //***   0:Blue, 1:Red, 2:Yellow
        //todo: 这里的颜色需要根据实际情况调整!!!!!!!
        CVModule.init(hardwareMap, telemetry, 0);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        double t = System.currentTimeMillis(); // 获取当前时间





        //waitForStart();
        boolean rbispressed = false;
        boolean isIntaking = false;
        while (opModeIsActive() || opModeInInit()) {
            if(System.currentTimeMillis() - t > 1000){
                CurrentClipPosition = ServoValueOutputter.ClipPosition.HALF_LOCKED;
            }

            if(gamepad1.right_bumper){
                if(!rbispressed) {
                    t = System.currentTimeMillis();
                    rbispressed = true;
                }
            }else{
                if(System.currentTimeMillis() - t < 500){
                    if(CurrentClipPosition == ServoValueOutputter.ClipPosition.LOCKED){
                        CurrentClipPosition = ServoValueOutputter.ClipPosition.UNLOCKED;
                    }
                    else if(CurrentClipPosition == ServoValueOutputter.ClipPosition.UNLOCKED){
                        CurrentClipPosition = ServoValueOutputter.ClipPosition.LOCKED;
                    }
                    else{
                        CurrentClipPosition = ServoValueOutputter.ClipPosition.LOCKED;
                    }
                }
            }
            Actions.runBlocking(
                    new SequentialAction(
                            sixServoArmController.SixServoArmSetClip(CurrentClipPosition)
                    )

            );
            if(isIntaking){
                Actions.runBlocking(
                        new SequentialAction(
                                sixServoArmController.SixServoArmRunToPosition(CVModule.findCandidate()),
                                sixServoArmController.SixServoArmSetClip(ServoValueOutputter.ClipPosition.LOCKED)
                        )

                );            }
            // Process camera frames and detect colors
            telemetry.addData("running toX", CVModule.findCandidate().GoToX);
            telemetry.addData("running toY", CVModule.findCandidate().GoToY);
            if(CVModule.findCandidate().suggestion == 1) {
                telemetry.addData("suggestion", "需要车辆左移");
            }
            else if(CVModule.findCandidate().suggestion == 2) {
                telemetry.addData("suggestion", "需要车辆右移");
            }
            else if(CVModule.findCandidate().suggestion == 3) {
                telemetry.addData("suggestion", "需要滑轨前移");
            }
            else {
                telemetry.addData("suggestion", "不需要移动");
            }
//            double armPower = 0;
//            if (gamepad2.right_trigger > 0.1) {
//                armPower = gamepad2.right_trigger;
//            } else if (gamepad2.left_trigger > 0.1) {
//                armPower = -gamepad2.left_trigger;
//            }
//            armMotor.setPower(armPower);
//            telemetry.addData("Arm Position", armMotor.getCurrentPosition());
//            telemetry.addData("Arm Power", armPower);
            drive.setDrivePowers(
                new PoseVelocity2d(
                    new Vector2d(gamepad1.left_stick_x, gamepad1.left_stick_y),
                    gamepad1.right_stick_x
                )
            );

            telemetry.update();
        }
    }

}
