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

import org.firstinspires.ftc.teamcode.Controllers.SixServoArm.ServoRadianEasyCalculator;
import org.firstinspires.ftc.teamcode.Controllers.SixServoArm.ServoValueEasyOutputter;
import org.firstinspires.ftc.teamcode.Controllers.SixServoArm.SixServoArmEasyAction;
import org.firstinspires.ftc.teamcode.Controllers.SixServoArm.SixServoArmEasyController;
import org.firstinspires.ftc.teamcode.RoadRunner.*;
import org.firstinspires.ftc.teamcode.Vision.*;
import org.firstinspires.ftc.teamcode.Vision.model.ArmAction;

@TeleOp
public class CVTestOp_BLUE extends LinearOpMode {

    ServoValueEasyOutputter.ClipPosition CurrentClipPosition = ServoValueEasyOutputter.ClipPosition.UNLOCKED;
    MecanumDrive drive;
    SixServoArmEasyController sixServoArmController;
    ServoValueEasyOutputter sixServoValueEasyOutputter;
    FindCandidate CVModule = new FindCandidate();
    private DcMotor armMotor;
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        sixServoArmController = new SixServoArmEasyController(hardwareMap, telemetry);
        sixServoValueEasyOutputter = new ServoValueEasyOutputter(hardwareMap,telemetry, ServoRadianEasyCalculator.getInstance());
//        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
//        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Initialize the camera and other components here
        //***   0:Blue, 1:Red, 2:Yellow
        //todo: 这里的颜色需要根据实际情况调整!!!!!!!
        CVModule.init(hardwareMap, telemetry, 1);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        double t = System.currentTimeMillis(); // 获取当前时间

        ArmAction Command = new ArmAction(0,0,0,0,-1);



        //waitForStart();
        boolean rbispressed = false;
        boolean isIntaking = false;
        boolean hasIntaked = true;
        
        waitForStart();
        
        
        while (opModeIsActive()) {
            if(System.currentTimeMillis() - t > 1000){
                CurrentClipPosition = ServoValueEasyOutputter.ClipPosition.HALF_LOCKED;
            }
            if(gamepad2.left_bumper){
                isIntaking = true;
            }
            else{
                isIntaking = false;
            }
            if(gamepad2.right_bumper){
                if(!rbispressed) {
                    t = System.currentTimeMillis();
                    rbispressed = true;
                }
            }else{
                if(System.currentTimeMillis() - t < 500){
                    if(CurrentClipPosition == ServoValueEasyOutputter.ClipPosition.LOCKED){
                        CurrentClipPosition = ServoValueEasyOutputter.ClipPosition.UNLOCKED;
                    }
                    else if(CurrentClipPosition == ServoValueEasyOutputter.ClipPosition.UNLOCKED){
                        CurrentClipPosition = ServoValueEasyOutputter.ClipPosition.LOCKED;
                    }
                    else{
                        CurrentClipPosition = ServoValueEasyOutputter.ClipPosition.LOCKED;
                    }
                }
            }
            sixServoValueEasyOutputter.setClip(CurrentClipPosition);

            if(isIntaking){
                Command = CVModule.CalculateAverage(CVModule);
                if(isIntaking && Command.suggestion != -2 && !hasIntaked){
                    sixServoArmController.setTargetPosition(CVModule.findCandidate()).update();
                    hasIntaked = true;
                }
            }
            
            // Process camera frames and detect colors
            telemetry.addData("running toX", Command.GoToX);
            telemetry.addData("running toY", Command.GoToY);
            telemetry.addData("suggestion", Command.suggestion);
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
