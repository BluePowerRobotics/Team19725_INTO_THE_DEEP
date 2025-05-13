package org.firstinspires.ftc.teamcode.OpModes;

import android.os.Environment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RoadRunner.Drawing;
import org.firstinspires.ftc.teamcode.RoadRunner.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.tuning.TuningOpModes;

import com.acmerobotics.roadrunner.ftc.FlightRecorder;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.messages.MecanumLocalizerInputsMessage;
import org.firstinspires.ftc.teamcode.messages.PoseMessage;


@TeleOp
public class OpMode2025 extends LinearOpMode {

    //12
    FlightRecorder recorder;
    GoBildaPinpointDriver odo;
    MecanumDrive drive;
    //boolean ifchanged = false;
    boolean lbispressed = false;
    boolean rbispressed = false;
    boolean ifslow = false;
    boolean ifgyw = false;

    boolean ifblue = true;
    double kpad;
    ColorLocator colorLocator;
    ChassisController ChassisController=new ChassisController();//构建class实例
    ArmController ArmController = new ArmController();
    Alignment AutoFollow;
    public double t = 0;//当前时间
    public double move_x_l;
    public double move_y_l;
    public double move_x_r;
    public double move_y_r;
    public double move_x2;

    private void initall(){
        kpad = 1;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Pose2d  initposeRedLeft = new Pose2d(-23.62,-70.86,64);
        Pose2d  initposeRedRight = new Pose2d(+23.62,-70.86,90);
        Pose2d  initposeBlueLeft = new Pose2d(+23.62,70.86,-90);
        Pose2d initposeBlueRight = new Pose2d(-23.62,70.86,-90);
        drive = new MecanumDrive(hardwareMap, initposeRedLeft);
        //telemetry.addData("设置tele", 1);
        //drive.settele(telemetry);



        ChassisController.initChassis(hardwareMap,gamepad1,gamepad2);
        ArmController.initArm(hardwareMap,gamepad1,gamepad2,telemetry);
        //AutoFollow.init(drive, hardwareMap, ifblue);



        colorLocator = new ColorLocator(hardwareMap.get(WebcamName.class, "Webcam 1"), ifblue);


        odo = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        //odo.setOffsets(-142.0, 120.0); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.resetPosAndIMU();
    }

    private void move(){
        if(gamepad1.right_bumper){
            if(!rbispressed){
                if(ifgyw){
                    ifgyw = false;
                }
                else{
                    ifgyw = true;
                }
                rbispressed = true;
            }
        }else{
            rbispressed = false;
        }
        if(gamepad1.left_bumper){
            if(!lbispressed){
                if(ifslow){
                    ifslow = false;
                }
                else{
                    ifslow = true;
                }
                lbispressed = true;
            }
        }else{
            lbispressed = false;
        }
        if(ifslow){
            kpad = 0.3;
        }
        else{
            kpad = 1;
        }//唐氏小按键
        //使用按键防抖代替
        if(!ifgyw){
            double realx = gamepad1.left_stick_y * kpad;
            double realy = gamepad1.left_stick_x * kpad;//*****已经转换过X,Y轴，并写过负号
            //double nowx = -realy * Math.sin(Math.toRadians((pos.getHeading(AngleUnit.DEGREES)) - angledeg0))   +     realx * Math.cos(Math.toRadians((pos.getHeading(AngleUnit.DEGREES)) - angledeg0));;
            //double nowy = realy * Math.cos(Math.toRadians((pos.getHeading(AngleUnit.DEGREES)) - angledeg0))    -     realx * Math.sin(Math.toRadians((pos.getHeading(AngleUnit.DEGREES)) - angledeg0));;
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
            ChassisController.chassisController(move_x_l, -move_x_r, move_y_l + move_y_r, kpad);
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
//        //telemetry.addData("angledeg0", angledeg0);
//        telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
//        Pose2D pos = odo.getPosition();
//        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
//        telemetry.addData("Position", data);
//        Pose2D vel = odo.getVelocity();
//        String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", vel.getX(DistanceUnit.MM), vel.getY(DistanceUnit.MM), vel.getHeading(AngleUnit.DEGREES));
//        telemetry.addData("Velocity", velocity);
        telemetry.addLine();
        telemetry.addData("角度", colorLocator.LocateAll().angle);
        telemetry.addData("X", colorLocator.LocateAll().x);
        telemetry.addData("Y", colorLocator.LocateAll().y);
        telemetry.addData("ifslow", ifslow);
        telemetry.addData("ifgyw", ifgyw);
        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();

        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), pose);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
    public void telecolor(){
        telemetry.addData("角度", colorLocator.LocateAll().angle);
        telemetry.addData("X", colorLocator.LocateAll().x);
        telemetry.addData("Y", colorLocator.LocateAll().y);
        telemetry.update();
    }




    public void runOpMode() throws InterruptedException {

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            initall();
            while(opModeInInit() && !opModeIsActive()){
                telecolor();
            }
            waitForStart();
            while (opModeIsActive()) {
                //odo.update();
                move();
                teleprint();
                //ChassisController.chassisController(move_x_l,-move_x_r,move_y_l+move_y_r);
                ArmController.armController();
                FlightRecorder.write("Pose", new PoseMessage(
                        drive.localizer.getPose()));


                if(gamepad2.x){
                    //AutoFollow.follow();
                }
            }
            //File log = new File(Environment.getExternalStorageDirectory(), "FIRST/roadrunner/logs");
        }
    }
}
