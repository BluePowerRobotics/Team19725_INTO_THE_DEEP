package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Controllers.ChassisController;
import org.firstinspires.ftc.teamcode.Controllers.IntakeLength.*;
import org.firstinspires.ftc.teamcode.Controllers.SixServoArm.ServoValueEasyOutputter;
import org.firstinspires.ftc.teamcode.Controllers.SixServoArm.SixServoArmEasyController;

@TeleOp

public class OP07142 extends LinearOpMode{
    IntakeLengthControllerInterface intakeLengthController = MotorLineIntakeLengthController.getInstance();
    ServoValueEasyOutputter servoValueEasyOutputter;
    SixServoArmEasyController sixServoArmController;
    ChassisController rbmove = new ChassisController();// 构建Move_GYW（）class实例
    static DcMotor armPuller;
    static DcMotor leftFront, leftBack, rightBack, rightFront,intake,output;
    Servo servos3, servos4, servos5;


    IMU imu;
    int logoFacingDirectionPosition = 0;
    int usbFacingDirectionPosition = 2;
    boolean orientationIsValid = true;
    YawPitchRollAngles orientation;

    public double angletime = 0;

    public double t = 0;// 当前时间
    public double move_x_l;
    public double move_y_l;
    public double move_x_r;
    public double move_y_r;
    public double move_x2;
    public double degree = 0;
    public double thita = 0;
    public double nuleftFront = 0.7071068;// 二分之根号二

    public boolean noheadmode = false;
    public boolean yhasbeenpressed = false;
    public boolean thitalock = false;
    public boolean xhasbeenpressed = false;

    public boolean ahasbeenpressed = false, lbhasbeenpressed = false, rbhasbeenpressed = false;
    public int servo_select = 3;
    public double servo_position = 0.5;
    // servos5:0.86~1
    //ArmDirectController armDirectController = new ArmDirectController();
    ChassisController chassisController = new ChassisController();
    //EasyClimb easyClimb = new EasyClimb();

    private void inithardware() {
        sixServoArmController=SixServoArmEasyController.getInstance(hardwareMap,telemetry);
        servoValueEasyOutputter=sixServoArmController.servoValueOutputter;
        // control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        // expansion_Hub_3 = hardwareMap.get(Blinker.class, "Expansion Hub 3");

        leftFront = hardwareMap.get(DcMotor.class, "leftFront"); // Configure the robot to use these 4 motor names,
        leftBack = hardwareMap.get(DcMotor.class, "leftBack"); // or change these strings to match your existing Robot
        // Configuration.
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        intake = hardwareMap.get(DcMotor.class, "IntakeLengthMotor");
        output  = hardwareMap.get(DcMotor.class, "OutputLengthMotor");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        output.setDirection(DcMotorSimple.Direction.FORWARD);
//        armPuller = hardwareMap.get(DcMotor.class,"armPuller");
//        armPuller.setDirection(DcMotorSimple.Direction.FORWARD);
//        armPuller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        servos3 = hardwareMap.get(Servo.class, "servoc3");
        servos4 = hardwareMap.get(Servo.class, "servoc4");
        servos5 = hardwareMap.get(Servo.class, "servoc5");
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        t = System.currentTimeMillis();// 获取当前时间

        imu = hardwareMap.get(IMU.class, "imu");
        logoFacingDirectionPosition = 0; // Up
        usbFacingDirectionPosition = 2; // Forward
        orientation = imu.getRobotYawPitchRollAngles();
        imu.resetYaw();
        updateOrientation();

    }

    public void fps_and_tele() {


        telemetry.addData("fps", 1000 / (System.currentTimeMillis() - t));// fps

        telemetry.addData("thita", orientation.getYaw(AngleUnit.DEGREES));
        telemetry.addData("move_x_l/旋转--逆-顺+", move_x_l);
        telemetry.addData("move_y_l+move_y_r/前-后+", move_y_l + move_y_r);
        telemetry.addData("move_x_r/左-右+", move_x_r);
        telemetry.addData("degree", degree);

        telemetry.addData("x",x);
        telemetry.addData("y",y);
        telemetry.addData("intake",intake.getCurrentPosition());
        telemetry.addData("output",output.getCurrentPosition());

        telemetry.update();
        t = System.currentTimeMillis();// 获取当前时间
    }

    void updateOrientation() {

        /*
         * try {
         * RevHubOrientationOnRobot orientationOnRobot = new
         * RevHubOrientationOnRobot(logo, usb);
         * imu.initialize(new IMU.Parameters(orientationOnRobot));
         * orientationIsValid = true;
         * } catch (IllegalArgumentException e) {
         * orientationIsValid = false;
         * }
         */
        orientation = imu.getRobotYawPitchRollAngles();
    }

    public void runOpMode() {
        inithardware();
        chassisController.initChassis(hardwareMap,gamepad1,gamepad2,telemetry);
        waitForStart();
        while (opModeIsActive()) {
            updateOrientation();
            thita = orientation.getYaw(AngleUnit.DEGREES);

            move_x_l = gamepad1.left_stick_x;
            move_y_l = gamepad1.left_stick_y;
            move_x_r = gamepad1.right_stick_x;
            move_y_r = gamepad1.right_stick_y;
            chassisController.chassisController(move_x_l, -move_x_r, move_y_l + move_y_r);

            // moveselect(move_x_l,-move_x_r,move_y_l+move_y_r);
            // moveselect(move_x_l,-move_y_l-move_y_r,-move_x_r);
            fps_and_tele();

            degree = 0.0036984 * orientation.getYaw(AngleUnit.DEGREES) + 0.499286;
            motorcontrol();
            servocontrol();
            /*
            if(gamepad1.x){
                armPuller.setPower(0.1);
            }else if(gamepad1.y){
                armPuller.setPower(-0.1);
            }else{
                armPuller.setPower(0);
            }*/
        }
    }

    boolean bhasbeenpressed = false, cliplock = false;
    public void motorcontrol(){
        if(gamepad2.x){
            intake.setPower(0.5);
        }else if(gamepad2.y){
            intake.setPower(-0.5);

        }else{
            intake.setPower(0);
        }
        if(gamepad2.a){
            output.setPower(0.5);
        }else if(gamepad2.b){
            output.setPower(-0.5);

        }else{
            output.setPower(0);
        }
        if(gamepad2.left_bumper){
            intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if(gamepad2.right_bumper){
            output.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            output.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
    double x=0,y=10;
    boolean leftstickbuttonhasbeenpressed= false;
    boolean locked=false;
    public void servocontrol() {
        x+=gamepad2.left_stick_x*0.5;
        y-=gamepad2.left_stick_y*0.5;
        if(!gamepad2.left_bumper)
            sixServoArmController.setTargetPosition(x,y,Math.PI,0).update();
        else
            servoValueEasyOutputter.moveToLeft();
        ServoValueEasyOutputter.ClipPosition pos;
        if(gamepad2.left_stick_button){
            if(!leftstickbuttonhasbeenpressed){
                locked=!locked;
                leftstickbuttonhasbeenpressed=true;
            }
        }else{
            leftstickbuttonhasbeenpressed=false;
        }
        if(locked){
            pos=ServoValueEasyOutputter.ClipPosition.LOCKED;
        }else{
            pos=ServoValueEasyOutputter.ClipPosition.UNLOCKED;
        }
        if(gamepad2.right_stick_button)
            pos=ServoValueEasyOutputter.ClipPosition.HALF_LOCKED;
        servoValueEasyOutputter.setClip(pos);
    }
}