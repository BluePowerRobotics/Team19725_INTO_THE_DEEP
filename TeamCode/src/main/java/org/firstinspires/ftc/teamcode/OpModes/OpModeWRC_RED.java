package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Autonomous.WRCAutoRightBlue;
import org.firstinspires.ftc.teamcode.Autonomous.WRCAutoRightRed;
import org.firstinspires.ftc.teamcode.Controllers.ArmController;
import org.firstinspires.ftc.teamcode.Controllers.ChassisController;
import org.firstinspires.ftc.teamcode.Controllers.InstallerController;
import org.firstinspires.ftc.teamcode.Controllers.IntakeLength.MotorLineIntakeLengthController;
import org.firstinspires.ftc.teamcode.Controllers.SixServoArm.ServoValueOutputter;
import org.firstinspires.ftc.teamcode.Controllers.SixServoArm.SixServoArmAction;
import org.firstinspires.ftc.teamcode.RoadRunner.Drawing;
import org.firstinspires.ftc.teamcode.RoadRunner.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.tuning.TuningOpModes;
import org.firstinspires.ftc.teamcode.VisualColor.Alignment;
import org.firstinspires.ftc.teamcode.VisualColor.FindCandidate;


@TeleOp(name = "OpMode2025")
public class OpModeWRC_RED extends LinearOpMode {

    //12
    //12
    ServoValueOutputter.ClipPosition CurrentClipPosition = ServoValueOutputter.ClipPosition.UNLOCKED;
    FlightRecorder recorder;
    GoBildaPinpointDriver odo;
    MecanumDrive drive;
    boolean pad1_lbispressed = false;
    boolean pad1_rbispressed = false;

    boolean pad2_rbispressed = false;
    boolean isIntaking = false;
    boolean ifslow = false;
    boolean ifRoadRunner = true;//是否使用roadrunner控制底盘移动
    double kpad;
    public double t = 0;//当前时间
    public double move_x_l;
    public double move_y_l;
    public double move_x_r;
    public double move_y_r;


    ChassisController chassisController;
    InstallerController installerController;
    MotorLineIntakeLengthController intakeLengthController;
    SixServoArmAction sixServoArmController;
    //OutputController outputController;
    FindCandidate CVModule;
    private void initall(){
        kpad = 1;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //todo 读取自动化结束位置
        drive = new MecanumDrive(hardwareMap, WRCAutoRightRed.EndPose);
        chassisController = new ChassisController();
        chassisController.initChassis(hardwareMap, gamepad1, gamepad2, telemetry);
        installerController = new InstallerController(hardwareMap, gamepad1, gamepad2, telemetry);
        intakeLengthController = new MotorLineIntakeLengthController(hardwareMap);
        sixServoArmController = new SixServoArmAction(hardwareMap, telemetry, gamepad2);
        //outputController = new OutputController(hardwareMap);
        CVModule = new FindCandidate();
        //todo: 这里的颜色需要根据实际情况调整!!!!!!!
        CVModule.init(hardwareMap, telemetry, 1);
        Actions.runBlocking(sixServoArmController.SixServoArmInit());
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
        telemetry.addData("x(实际上是y轴)", -gamepad1.left_stick_y * kpad);
        telemetry.addData("y(实际上是x轴)", -gamepad1.left_stick_x * kpad);
        telemetry.addData("xencoder", odo.getEncoderX());
        telemetry.addData("yencoder", odo.getEncoderY());
        Pose2d pose = drive.localizer.getPose();
        telemetry.addData("x", pose.position.x);
        telemetry.addData("y", pose.position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));

        telemetry.addData("ifslow", ifslow);
        telemetry.addData("ifRoadRunner", ifRoadRunner);
        telemetry.addData("CurrentClipPosition", CurrentClipPosition);
        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();

        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), pose);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
    private  void controllers() {
        if (gamepad1.left_trigger > 0.1) {
//            installerController.;
        }
        if (gamepad1.right_trigger > 0.1) {
            //    installerController.;
        }


//        intakeLengthController.SetFowardSpeed(gamepad2.left_trigger);
//        intakeLengthController.SetBackwardSpeed(gamepad2.right_trigger);




        if (gamepad1.right_bumper) {
            if (!pad2_rbispressed) {
                t = System.currentTimeMillis();
                pad2_rbispressed = true;
            }
            if (System.currentTimeMillis() - t > 1000) {
                CurrentClipPosition = ServoValueOutputter.ClipPosition.HALF_LOCKED;
            }
        } else {
            if (System.currentTimeMillis() - t < 500) {
                if (CurrentClipPosition == ServoValueOutputter.ClipPosition.LOCKED) {
                    CurrentClipPosition = ServoValueOutputter.ClipPosition.UNLOCKED;
                }else {
                    CurrentClipPosition = ServoValueOutputter.ClipPosition.LOCKED;
                }
            }
            pad2_rbispressed = false;
        }
        Actions.runBlocking(
                new SequentialAction(
                        sixServoArmController.SixServoArmSetClip(CurrentClipPosition)
                )
        );

        if(gamepad2.left_bumper){
            Actions.runBlocking(
                    new SequentialAction(
                            sixServoArmController.SixServoArmRunToPosition(CVModule.findCandidate())
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





    public void runOpMode() throws InterruptedException {

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            initall();
            teleprint();
            waitForStart();
            while (opModeIsActive()) {
                move();
                controllers();
                teleprint();
            }
        }
    }
}
