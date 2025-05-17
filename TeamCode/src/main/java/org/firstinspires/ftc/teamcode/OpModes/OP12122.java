package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp

public class OP12122 extends LinearOpMode {

    ChassisController rbmove = new ChassisController();// 构建Move_GYW（）class实例
    static DcMotor armPuller;
    static DcMotor leftFront, leftBack, rightBack, rightFront;
    Servo servos3, servos4, servos5;
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
    public double nuleftFront = 0.7071068;// 二分之根号二

    public boolean noheadmode = false;
    public boolean yhasbeenpressed = false;
    public boolean thitalock = false;
    public boolean xhasbeenpressed = false;

    public boolean ahasbeenpressed = false, lbhasbeenpressed = false, rbhasbeenpressed = false;
    public int servo_select = 3;
    public double servo_position = 0.5;
    // servos5:0.86~1
    ArmDirectController armDirectController = new ArmDirectController();
    ChassisController chassisController = new ChassisController();
    EasyClimb easyClimb = new EasyClimb();

    private void inithardware() {
        // control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        // expansion_Hub_3 = hardwareMap.get(Blinker.class, "Expansion Hub 3");

        leftFront = hardwareMap.get(DcMotor.class, "leftFront"); // Configure the robot to use these 4 motor names,
        leftBack = hardwareMap.get(DcMotor.class, "leftBack"); // or change these strings to match your existing Robot
                                                               // Configuration.
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
//        armPuller = hardwareMap.get(DcMotor.class,"armPuller");
//        armPuller.setDirection(DcMotorSimple.Direction.FORWARD);
//        armPuller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        servos3 = hardwareMap.get(Servo.class, "servos3");
        servos4 = hardwareMap.get(Servo.class, "servos4");
        servos5 = hardwareMap.get(Servo.class, "servos5");
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

        //telemetry.addData("armPuller",armPuller.getCurrentPosition());
        telemetry.addData("fps", 1000 / (System.currentTimeMillis() - t));// fps

        telemetry.addData("thita", orientation.getYaw(AngleUnit.DEGREES));
        telemetry.addData("move_x_l/旋转--逆-顺+", move_x_l);
        telemetry.addData("move_y_l+move_y_r/前-后+", move_y_l + move_y_r);
        telemetry.addData("move_x_r/左-右+", move_x_r);
        telemetry.addData("degree", degree);
        telemetry.addData("leftFrontPower", rbmove.leftFrontPower);
        telemetry.addData("leftBackPower", rbmove.leftBackPower);
        telemetry.addData("rightBackPower", rbmove.rightBackPower);
        telemetry.addData("rightFrontPower", rbmove.rightFrontPower);

        telemetry.addData("px", rbmove.px);
        telemetry.addData("py", rbmove.py);
        telemetry.addData("omiga", rbmove.omiga);
        telemetry.addData("alpha", rbmove.alpha);
        telemetry.addData("angle", rbmove.angle);

        telemetry.addData("servo_select", servo_select);
        telemetry.addData("servo_position", servo_position);

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
        armDirectController.initArm(hardwareMap,gamepad2,telemetry);
        armDirectController.initArmAction();
        chassisController.initChassis(hardwareMap,gamepad1,gamepad2);
        easyClimb.initClimb(hardwareMap,gamepad2);
        waitForStart();
        while (opModeIsActive()) {
            updateOrientation();
            thita = orientation.getYaw(AngleUnit.DEGREES);
            armDirectController.armSpinnerController(gamepad2.left_stick_y);
            armDirectController.armMotorController(-gamepad2.left_trigger+ gamepad2.right_trigger);
            armDirectController.servoe4Controller(gamepad2.right_stick_y);
            armDirectController.servoe5Controller(gamepad2.a);
            armDirectController.servoe2Controller(gamepad2.y);

            move_x_l = gamepad1.left_stick_x;
            move_y_l = gamepad1.left_stick_y;
            move_x_r = gamepad1.right_stick_x;
            move_y_r = gamepad1.right_stick_y;
            chassisController.chassisController(move_x_l, -move_x_r, move_y_l + move_y_r);

            // moveselect(move_x_l,-move_x_r,move_y_l+move_y_r);
            // moveselect(move_x_l,-move_y_l-move_y_r,-move_x_r);
            fps_and_tele();
            /*
            degree = 0.0036984 * orientation.getYaw(AngleUnit.DEGREES) + 0.499286;
            servocontrol();
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

    public void servocontrol() {

        /*
         * if (gamepad1.b){
         * if (!bhasbeenpressed){
         * if (!cliplock) cliplock = true;
         * else if (cliplock) cliplock = false;
         * bhasbeenpressed = true;
         * }else{
         * bhasbeenpressed = true;
         * }
         * }else{
         * bhasbeenpressed = false;
         * }
         * if(cliplock) servos5.setPosition(0.86);
         * else servos5.setPosition(1);
         */

        if (gamepad1.a) {
            if (!ahasbeenpressed) {
                servo_select++;
                ahasbeenpressed = true;
            }
        } else {
            ahasbeenpressed = false;
        }
        if (servo_select > 5) {
            servo_select = 3;
        }

        if (gamepad1.left_bumper) {

            servo_position += 0.005;

        }
        if (gamepad1.right_bumper) {

            servo_position -= 0.005;

        }

        if (servo_position > 1)
            servo_position = 1;
        if (servo_position < 0)
            servo_position = 0;

        if (servo_select == 3) {
            servos3.setPosition(servo_position);
        } else if (servo_select == 4) {
            servos4.setPosition(servo_position);
        } else {
            servos5.setPosition(servo_position);
        }
    }

    public void moveselect(double r, double y, double x) {
        if (gamepad1.y) {
            if (!yhasbeenpressed) {
                if (!noheadmode)
                    noheadmode = true;
                else if (noheadmode)
                    noheadmode = false;
                yhasbeenpressed = true;
            } else {
                yhasbeenpressed = true;
            }
            rbmove.setthita = thita;

        } else {
            yhasbeenpressed = false;
        }

        if (gamepad1.x) {
            if (!xhasbeenpressed) {
                if (!thitalock)
                    thitalock = true;
                else if (thitalock)
                    thitalock = false;
                xhasbeenpressed = true;
            } else {
                xhasbeenpressed = true;
            }
            rbmove.lock_thita = thita;
        } else {
            xhasbeenpressed = false;
        }

        rbmove.getthita = thita;

        if (thitalock) {
            r = rbmove.thitalock();
        } else {
            if (System.currentTimeMillis() - angletime >= 10) {
                if (y < 0) {
                    r = -r;
                }
                rbmove.lock_thita -= 2 * r;
                // if (rbmove.lock_thita<=-180) rbmove.lock_thita +=360;
                // if (rbmove.lock_thita>=180) rbmove.lock_thita-=360;
                angletime = System.currentTimeMillis();
            }
            r = rbmove.thitalock();
        }

        if (!noheadmode) {
            rbmove.move(r, y, x);
        } else {
            rbmove.noheadmove(r, y, x, 1);
        }
    }

}
