package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
//import org.firstinspires.ftc.teamcode.Controllers.ArmController;
import org.firstinspires.ftc.teamcode.Controllers.ChassisController;

@TeleOp

public class Auto0513Test extends LinearOpMode {

    org.firstinspires.ftc.teamcode.Controllers.ChassisController ChassisController = new ChassisController();// 构建class实例
    //org.firstinspires.ftc.teamcode.Controllers.ArmController ArmController = new ArmController(hardwareMap, telemetry);// 构建class实例

    static RevHubOrientationOnRobot.LogoFacingDirection[] logoFacingDirections = RevHubOrientationOnRobot.LogoFacingDirection
            .values();
    static RevHubOrientationOnRobot.UsbFacingDirection[] usbFacingDirections = RevHubOrientationOnRobot.UsbFacingDirection
            .values();

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

    public boolean noheadmode = false;
    public boolean yhasbeenpressed = false;
    public boolean thitalock = false;
    public boolean xhasbeenpressed = false;
    public boolean lthasbeenpressed = false, rthasbeenpressed = false;
    // servoe5:0.86~1
    // servoe4:0;0.515;0.95
    // servoe3:0.7~1

    private void inithardware() {
        // control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        // expansion_Hub_3 = hardwareMap.get(Blinker.class, "Expansion Hub 3");
        /*
         * leftFront = hardwareMap.get(DcMotor.class, "leftFront"); // Configure the
         * robot to use these 4 motor names,
         * leftBack = hardwareMap.get(DcMotor.class, "leftBack"); // or change these
         * strings to match your existing Robot Configuration.
         * rightBack = hardwareMap.get(DcMotor.class, "rightBack");
         * rightFront = hardwareMap.get(DcMotor.class, "rightFront");
         * servoe3 = hardwareMap.get(Servo.class, "servoe3");
         * servoe4 = hardwareMap.get(Servo.class, "servoe4");
         * servoe5 = hardwareMap.get(Servo.class, "servoe5");
         * leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         * leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         * rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         * rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         * 
         * rightFront.setDirection(DcMotor.Direction.REVERSE);
         * leftFront.setDirection(DcMotor.Direction.FORWARD);
         * rightBack.setDirection(DcMotor.Direction.REVERSE);
         * leftBack.setDirection(DcMotor.Direction.FORWARD);
         * 
         */
        t = System.currentTimeMillis();// 获取当前时间

        // imu = hardwareMap.get(IMU.class, "imu");
        imu = hardwareMap.get(IMU.class, "eimu");
        logoFacingDirectionPosition = 0; // Up
        usbFacingDirectionPosition = 2; // Forward
        orientation = imu.getRobotYawPitchRollAngles();
        imu.resetYaw();
        updateOrientation();

    }

    public void fps_and_tele() {


        telemetry.addData("fps", 1000 / (System.currentTimeMillis() - t));// fps


        telemetry.addData("move_x_l/旋转--逆-顺+", move_x_l);
        telemetry.addData("move_y_l+move_y_r/前-后+", move_y_l + move_y_r);
        telemetry.addData("move_x_r/左-右+", move_x_r);
        telemetry.addData("degree", degree);



        //telemetry.addData("servo_position", ArmController.servo_position);

        //telemetry.addData("length", ArmController.motorNowLength / ArmController.motorLength);

        telemetry.update();
        t = System.currentTimeMillis();// 获取当前时间
    }

    void updateOrientation() {

        RevHubOrientationOnRobot.LogoFacingDirection logo = logoFacingDirections[logoFacingDirectionPosition];
        RevHubOrientationOnRobot.UsbFacingDirection usb = usbFacingDirections[usbFacingDirectionPosition];
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
        ChassisController.initChassis(hardwareMap, gamepad1,gamepad2,telemetry);
//        ArmController.initArm(hardwareMap, gamepad1, gamepad2,telemetry);
        waitForStart();
//        ArmController.armup = true;
//        ArmController.clipPosition = 0;
//        ArmController.armUpTime = t + 1600;
//        ArmController.armMotor.setTargetPosition(0);
//        ArmController.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        ArmController.armMotor.setPower(1);
        ChassisController.setMode(org.firstinspires.ftc.teamcode.Controllers.ChassisController.CHASSIS_RUNMODE.STOP_AND_RESET_LOCATOR);
        ChassisController.setTargetLocation(100,100,0);
        ChassisController.setSpeed(0.1);
        while (opModeIsActive()) {
            /*
             * updateOrientation();
             * thita = orientation.getYaw(AngleUnit.DEGREES);
             */

            ChassisController.setMode(org.firstinspires.ftc.teamcode.Controllers.ChassisController.CHASSIS_RUNMODE.RUN_TO_LOCATION);
            //ArmController.armController();

            fps_and_tele();


        }
    }


}
