package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Autonomous.WRCAutoRightBlue;
import org.firstinspires.ftc.teamcode.Controllers.Installer.InstallerController;
import org.firstinspires.ftc.teamcode.Controllers.SixServoArm.ServoValueEasyOutputter;
import org.firstinspires.ftc.teamcode.Controllers.SixServoArm.SixServoArmAction;
import org.firstinspires.ftc.teamcode.RoadRunner.Drawing;
import org.firstinspires.ftc.teamcode.RoadRunner.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.tuning.TuningOpModes;
import org.firstinspires.ftc.teamcode.Controllers.*;
import org.firstinspires.ftc.teamcode.Controllers.IntakeLength.*;
import org.firstinspires.ftc.teamcode.Controllers.SixServoArm.*;
import org.firstinspires.ftc.teamcode.VisualColor.FindCandidate;

import com.acmerobotics.roadrunner.ftc.FlightRecorder;


@TeleOp(name = "OpModeWRC_BLUE")
public class OpModeWRC_BLUE extends LinearOpMode {

    //12
    ServoValueEasyOutputter.ClipPosition CurrentClipPosition = ServoValueEasyOutputter.ClipPosition.UNLOCKED;
    FlightRecorder recorder;
    GoBildaPinpointDriver odo;
    MecanumDrive drive;
    boolean pad1_lbispressed = false;
    boolean pad1_rbispressed = false;
    long pad2xpressTime = 0;
    boolean pad2_xispressed = false;
    boolean pad2_rbispressed = false;
    boolean isIntaking = false;
    boolean ifslow = false;
    boolean ifRoadRunner = true;//是否使用roadrunner控制底盘移动
    boolean ifSixServoArm = true;//是否使用六伺服臂





    //Single Servo Control

    boolean ifGivenCommand = false;
    boolean pad2_lstickispressed = false;
    boolean pad2_rstickispressed = false;
    int servo_select = 1;
    double servo_position = 0.5;





    double kpad;
    public double t = 0;//当前时间
    public double move_x_l;
    public double move_y_l;
    public double move_x_r;
    public double move_y_r;


    ChassisController chassisController;
    InstallerController installerController;
    MotorLineIntakeLengthController intakeLengthController;
    SixServoArmEasyAction sixServoArmEasyController;
    //OutputController outputController;
    FindCandidate CVModule;
    private void initall(){
        kpad = 1;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //todo 读取自动化结束位置
        drive = new MecanumDrive(hardwareMap, WRCAutoRightBlue.EndPose);
        chassisController = new ChassisController();
        chassisController.initChassis(hardwareMap, gamepad1, gamepad2, telemetry);
        installerController = new InstallerController(hardwareMap, gamepad1, gamepad2, telemetry);
        intakeLengthController = new MotorLineIntakeLengthController(hardwareMap);
        sixServoArmEasyController = new SixServoArmEasyAction(hardwareMap, telemetry, gamepad2);
        //outputController = new OutputController(hardwareMap);
        CVModule = new FindCandidate();
        //todo: 这里的颜色需要根据实际情况调整!!!!!!!
        CVModule.init(hardwareMap, telemetry, 0);
        Actions.runBlocking(sixServoArmEasyController.SixServoArmInit());
        double t = System.currentTimeMillis(); // 获取当前时间
        //硬件初始化
    }

    private void move(){
        if(gamepad1.right_bumper){
            if(!pad1_rbispressed){
                if(ifRoadRunner){
                    ifRoadRunner = false;
                }
                else{
                    ifRoadRunner = true;
                }
                pad1_rbispressed = true;
            }
        }else{
            pad1_rbispressed = false;
        }
        if(gamepad1.left_bumper){
            if(!pad1_lbispressed){
                if(ifslow){
                    ifslow = false;
                }
                else{
                    ifslow = true;
                }
                pad1_lbispressed = true;
            }
        }else{
            pad1_lbispressed = false;
        }
        if(ifslow){
            kpad = 0.3;
        }
        else {
            kpad = 1;
        }
        if(ifRoadRunner){
            double realx = gamepad1.left_stick_y * kpad;
            double realy = gamepad1.left_stick_x * kpad;//*****已经转换过X,Y轴，并写过负号
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            realx,
                            realy
                    ),
                    gamepad1.right_stick_x * kpad
            ));
        }
        else{
            move_x_l = gamepad1.left_stick_x + gamepad2.left_stick_x;
            move_y_l = gamepad1.left_stick_y + gamepad2.left_stick_y;
            move_x_r = gamepad1.right_stick_x + gamepad2.right_stick_x;
            move_y_r = gamepad1.right_stick_y + gamepad2.right_stick_y;
            chassisController.chassisController(move_x_l, -move_x_r, move_y_l + move_y_r, kpad);
        }
        drive.updatePoseEstimate();
    }
    private void teleprint(){
        telemetry.addData("ifSlow", ifslow);
        telemetry.addData("ifRoadRunner", ifRoadRunner);
        if(ifSixServoArm){
            telemetry.addData("ServoSelected(0-5)", -1);
        }
        else{
            telemetry.addData("ServoSelected(0-5)", servo_select);
        }

        telemetry.addData("xpad(实际上是y轴)", -gamepad1.left_stick_y * kpad);
        telemetry.addData("ypad(实际上是x轴)", -gamepad1.left_stick_x * kpad);

        telemetry.addData("xencoder", odo.getEncoderX());
        telemetry.addData("yencoder", odo.getEncoderY());
        Pose2d pose = drive.localizer.getPose();
        telemetry.addData("x", pose.position.x);
        telemetry.addData("y", pose.position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
//        //telemetry.addData("angledeg0", angledeg0);
//        telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
//        Pose2D pos = odo.getPosition();
//        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
//        telemetry.addData("Position", data);
//        Pose2D vel = odo.getVelocity();
//        String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", vel.getX(DistanceUnit.MM), vel.getY(DistanceUnit.MM), vel.getHeading(AngleUnit.DEGREES));
//        telemetry.addData("Velocity", velocity);
        telemetry.addLine();
//        telemetry.addData("角度", colorLocator.LocateAll().angle);
//        telemetry.addData("X", colorLocator.LocateAll().x);
//        telemetry.addData("Y", colorLocator.LocateAll().y);

        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();

        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), pose);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
    private  void controllers() {
        if (gamepad1.b) {
            installerController.BeamSpinner(false);
        }
        if (gamepad1.a) {
            installerController.BeamSpinner(true);
        }


//        intakeLengthController.SetFowardSpeed(gamepad2.left_trigger);
//        intakeLengthController.SetBackwardSpeed(gamepad2.right_trigger);




        if (gamepad2.right_bumper) {
            if (!pad2_rbispressed) {
                t = System.currentTimeMillis();
                pad2_rbispressed = true;
            }
            if (System.currentTimeMillis() - t > 1000) {
                CurrentClipPosition = ServoValueEasyOutputter.ClipPosition.HALF_LOCKED;
            }
        } else {
            if (System.currentTimeMillis() - t < 500) {
                if (CurrentClipPosition == ServoValueEasyOutputter.ClipPosition.LOCKED) {
                    CurrentClipPosition = ServoValueEasyOutputter.ClipPosition.UNLOCKED;
                }else {
                    CurrentClipPosition = ServoValueEasyOutputter.ClipPosition.LOCKED;
                }
            }
            pad2_rbispressed = false;
        }
        Actions.runBlocking(
                new SequentialAction(
                        sixServoArmEasyController.SixServoArmSetClip(CurrentClipPosition)
                )
        );

        if(gamepad2.left_bumper && ifSixServoArm){
            Actions.runBlocking(
                    new SequentialAction(
                            sixServoArmEasyController.SixServoArmRunToPosition(CVModule.findCandidate())
                    )
            );
        }

        if(gamepad2.a){
            //installerController.
        }

        if(gamepad2.y){
            //installerController.setInstallerPosition(InstallerController.InstallerPosition.INSTALLER_UP);
        }

        if(gamepad2.b){
            //outputController.setTargetOutputHeight(100);
        }
    }


    public void switchMode() {
        if (gamepad2.x) {
            if (!pad2_xispressed) {
                t = System.currentTimeMillis();
                pad2_xispressed = true;
            }
        } else {
            if(System.currentTimeMillis() - t > 1000) {
                if (ifSixServoArm) {
                    ifSixServoArm = false;
                } else {
                    ifSixServoArm = true;
                }
            }
            pad1_lbispressed = false;
        }
    }
    public void SingleServoControl() {
        if(gamepad2.left_stick_button) {
            if (!pad2_lstickispressed) {
                servo_select--;
                pad2_lstickispressed = true;
                ifGivenCommand = false; // Reset command flag when changing servo selection
            }
        } else {
            pad2_lstickispressed = false;
        }
        if(gamepad2.right_stick_button) {
            if (!pad2_rstickispressed) {
                servo_select++;
                pad2_rstickispressed = true;
                ifGivenCommand = false;
            }
        } else {
            pad2_rstickispressed = false;
        }
        if (servo_select < 0) {
            servo_select = 5;
        }
        if(servo_select > 5){
            servo_select = 0;
        }

        if(gamepad2.left_stick_x > 0.1 && gamepad2.left_stick_x < 0.6) {
            servo_position += 0.01;
            ifGivenCommand = true;
        }
        else if(gamepad2.left_stick_x < -0.1 && gamepad2.left_stick_x > -0.6) {
            servo_position -= 0.01;
            ifGivenCommand = true;
        }
        if(gamepad2.left_stick_x > 0.6) {
            servo_position += 0.08;
            ifGivenCommand = true;
        }
        else if(gamepad2.left_stick_x <-0.6) {
            servo_position -= 0.08;
            ifGivenCommand = true;
        }

        if (servo_position > 1)
            servo_position = 1;
        if (servo_position < 0)
            servo_position = 0;

        if(ifGivenCommand){
            ServoValueEasyOutputter.getInstance(hardwareMap, telemetry,ServoRadianEasyCalculator.getInstance()).SingleServoControl(servo_select, servo_position);
        }
    }





    public void runOpMode() throws InterruptedException {

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            initall();

            waitForStart();
            while (opModeIsActive()) {
                move();
                switchMode();
                controllers();
                SingleServoControl();
                teleprint();
            }
        }
    }
}
