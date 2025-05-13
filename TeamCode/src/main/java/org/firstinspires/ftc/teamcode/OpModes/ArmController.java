//package org.firstinspires.ftc.teamcode.OpModes;
//
//import com.qualcomm.robotcore.hardware.Blinker;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.HardwareDevice;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Gyroscope;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.Gamepad;
//
//public class ArmController {
//    private Blinker control_Hub;
//    private Blinker expansion_Hub_2;
//    private HardwareDevice webcam_1;
//    private DcMotor armMotor;
//    private DcMotor armTest;
//    private Gyroscope eimu;
//    private IMU imu;
//    private DcMotor leftBack;
//    private DcMotor leftFront;
//    private DcMotor rightBack;
//    private DcMotor rightFront;
//    private Servo servoe0;
//    private Servo servoe1;
//    private Servo servoe2;
//    private Servo servoe3;
//    private Servo servoe4;
//    private Servo servoe5;
//    private Servo servos0;
//    private Servo servos1;
//    private Servo servos2;
//    private Servo servos3;
//    private Servo servos4;
//    private Servo servos5;
//    private HardwareMap hardwaremap;
//    private Gamepad gamepad1;
//    private Gamepad gamepad2;
//
//    private static final int HIGHBLANKET = 0;
//    private static final int LOWBLANKET = 1;
//    private static final int HIGHRAILING = 2;
//    private static final int LOWRAILING = 3;
//
//    public int MODE = 0;
//
//    // todo: write your code here
//
//    double t = 0;
//
//    // ArmController
//
//    boolean bhasbeenpressed = false, cliplock = false;
//    boolean armup = false;
//    boolean ahasbeenpressed = false;
//    boolean y2hasbeenpressed = false;
//    double servo_position = 0.9;
//
//    double motorTime = 0;
//    double motorLength = 100000;
//    double motorNowLength = 0;
//    double motorPower = 0;
//
//    double clipLockPos = 0.7;
//    double clipUnlockPos = 1;
//    double clipPosition = 0;
//    double clipUpPos = 0.3;
//    double clipDownPos = 0.675;
//    double armUpPos = 0;
//    double armPosMax = 1;
//    double armPosMin = 0.7;
//    double[] armMotorPosition;
//
//    int armUpSpendHalfMS;
//    int armUpLength;
//
//    void initArm(HardwareMap hardwaremaprc, Gamepad gamepadrc, Gamepad gamepadrc2) {
//        t = System.currentTimeMillis();// 获取当前时间
//        motorTime = t;
//        gamepad1 = gamepadrc;
//        gamepad2 = gamepadrc2;
//        hardwaremap = hardwaremaprc;
//        armMotor = hardwaremap.get(DcMotor.class, "armMotor");
//        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        servoe3 = hardwaremap.get(Servo.class, "servoe3");
//        servoe4 = hardwaremap.get(Servo.class, "servoe4");
//        servoe5 = hardwaremap.get(Servo.class, "servoe5");
//        servoe2 = hardwaremap.get(Servo.class, "servoe2");
//        cliplock = false;
//        armup = false;
//        servo_position = 0.9;// default position when arm is down.calculated,useless
//
//        motorTime = 0;
//        motorLength = 540;
//        motorNowLength = 0;
//        motorPower = 0;
//
//        clipLockPos = 0.345;// 0123
//        clipUnlockPos = 0.625;// 0123
//        clipUpPos = 0.245;// 0,1
//        clipDownPos = 0.675;// calculated,useless
//        clipPosition = 0;// 0123
//        armUpPos = 0.1;// 0,1
//        armPosMax = 1;// useless
//        armPosMin = 0.7;// useless
//
//        if (MODE == HIGHBLANKET || MODE == LOWBLANKET) {
//            armUpSpendHalfMS = 1600;
//            if (MODE == HIGHBLANKET) {
//                armUpLength = 540;
//            } else {
//                armUpLength = 0;
//            }
//        } else if (MODE == HIGHRAILING || MODE == LOWRAILING) {
//            armUpSpendHalfMS = 800;
//            armUpLength = 540;
//            clipUpPos = 0.9;
//            if (MODE == HIGHRAILING) {
//                armUpPos = 0.4;
//            } else {
//                armUpPos = 0.5;
//            }
//
//        }
//
//        // armUp
//        servoe3.setPosition(armUpPos);// 一级舵机竖直
//        servoe4.setPosition(clipUpPos);// 二级舵机向后
//
//        armMotor.setPower(1);
//
//        while (System.currentTimeMillis() - t <= 2000) {
//            armMotor.setTargetPosition(560 * 2);
//            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            // telemetry.addData("Testing", armMotor.getCurrentPosition());
//            // telemetry.update();
//        }
//
//        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        // telemetry.addData("StartToBack", armMotor.getCurrentPosition());
//        // telemetry.update();
//        while (armMotor.getCurrentPosition() + motorLength > 5 || armMotor.getCurrentPosition() + motorLength < -5) {
//            armMotor.setTargetPosition(-(int) motorLength);
//            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            // telemetry.addData("Backing", armMotor.getCurrentPosition());
//            // telemetry.update();
//        }
//        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        // armLenthControl();
//        armCalculator();
//        armDown();
//        // armMotor.setMode(DcMotor.RunMode.Run_WITHOUT_ENCODER);
//        // telemetry.addData("Finished", armMotor.getCurrentPosition());
//        // telemetry.update();
//    }
//
//    void gampadArmReceiver() {
//        // clip
//        if (gamepad1.b || gamepad2.b) {
//            if (!bhasbeenpressed) {
//                if (!cliplock) {
//                    cliplock = true;
//                    clipLockTime = System.currentTimeMillis() + 500;
//                } else if (cliplock)
//                    cliplock = false;
//                bhasbeenpressed = true;
//            } else {
//                bhasbeenpressed = true;
//            }
//        } else {
//            bhasbeenpressed = false;
//        }
//        // 一级控制
//        /*
//         * if (gamepad1.left_bumper){
//         * servo_position +=0.005;
//         * }
//         * if (gamepad1.right_bumper){
//         * servo_position -=0.005;
//         * }
//         */
//        // 机械臂模式选择
//        if (gamepad1.a || gamepad2.a) {
//            if (!ahasbeenpressed) {
//                if (armup) {
//                    armup = false;
//                    clipPosition = 0;
//                    armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                } else {
//                    armup = true;
//                    clipPosition = 0;
//                    armUpTime = t + 1600;
//                    armMotor.setTargetPosition(0);
//                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    armMotor.setPower(1);
//                }
//                ahasbeenpressed = true;
//            }
//        } else {
//            ahasbeenpressed = false;
//        }
//
//        if (gamepad2.y) {
//            if (!y2hasbeenpressed) {
//                clipPosition += 0.25;
//                y2hasbeenpressed = true;
//            } else {
//                y2hasbeenpressed = true;
//            }
//        } else {
//            y2hasbeenpressed = false;
//        }
//        if (clipPosition >= 1)
//            clipPosition -= 1;
//    }
//
//    double armUpTime = 0;
//
//    void armUp() {
//
//        servoe3.setPosition(armUpPos);// 一级舵机竖直
//
//        if (armUpTime >= System.currentTimeMillis()) {
//            servoe4.setPosition(0.9);// 二级舵机收起
//            armMotor.setTargetPosition(0);
//            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            // armMotor.setPower(1);
//        } else {
//            servoe4.setPosition(clipUpPos);// 二级舵机向后
//            armMotor.setTargetPosition(armUpLength);
//            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            // armMotor.setPower(1);
//        }
//    }
//
//    void armDown() {
//        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        servoe3.setPosition(servo_position);// 一级舵机地面
//        servoe4.setPosition(clipDownPos);// 二级舵机向下
//    }
//
//    void angleRange() {
//        // arm角度范围限定
//        if (servo_position > armPosMax)
//            servo_position = armPosMax;
//        if (servo_position < armPosMin)
//            servo_position = armPosMin;
//        // arm长度范围限定
//        // motorNowLength = Math.max(0,Math.min(motorLength,motorNowLength));
//        // clip角度范围限定
//        clipDownPos = Math.max(0.29, Math.min(0.715, clipDownPos));
//
//    }
//
//    double clipLockTime = 0;
//
//    void clipControl() {
//        if (cliplock && clipLockTime <= System.currentTimeMillis())
//            servoe5.setPosition(clipLockPos);
//        else
//            servoe5.setPosition(clipUnlockPos);
//        servoe2.setPosition(clipPosition);
//    }
//
//    public void armLenthControl() {
//        if (!armup) {
//            if (gamepad1.left_trigger >= 0.5 && gamepad1.right_trigger < 0.5 && motorNowLength > 0)
//                motorPower = -0.5;
//            else if (gamepad1.left_trigger < 0.5 && gamepad1.right_trigger >= 0.5 && motorNowLength < motorLength)
//                motorPower = 0.5;
//            else if (gamepad2.left_trigger >= 0.5 && gamepad2.right_trigger < 0.5 && motorNowLength > 0)
//                motorPower = -0.5;
//            else if (gamepad2.left_trigger < 0.5 && gamepad2.right_trigger >= 0.5 && motorNowLength < motorLength)
//                motorPower = 0.5;
//            else
//                motorPower = 0;
//            armMotor.setPower(motorPower);
//        }
//        /*
//         * if (motorPower!=0){
//         * motorNowLength += motorPower*(System.currentTimeMillis()-motorTime);
//         * }
//         */
//        motorNowLength = armMotor.getCurrentPosition();
//
//        // telemetry.addData("armPos",armMotor.getCurrentPosition());
//        motorTime = System.currentTimeMillis();
//    }
//
//    double clipHeight = 10;// 12
//    double clipHeightError = 0;
//
//    void armCalculator() {
//        if (cliplock && clipLockTime + 500 >= System.currentTimeMillis())
//            clipHeightError = 10;// servo_position+=0.12;
//        else if (gamepad2.x)
//            clipHeightError = 10;// servo_position+=0.12;//8
//        else
//            clipHeightError = 0;
//        double L = 30.5 + (motorNowLength / motorLength) * 32.0;
//        double argument = -(clipHeight + clipHeightError) / L;
//        servo_position = Math.toDegrees(Math.acos(argument)) / 135.0 /*- 0.1*/;
//
//        clipDownPos = (3 * 0.3 + 2.2 - 1.5 * (servo_position - 0.1)) / 3;
//    }
//
//    public void armController() {
//        t = System.currentTimeMillis();// 获取当前时间
//        motorTime = t;
//        gampadArmReceiver();
//        armLenthControl();
//        armCalculator();
//        if (armup) {
//            armUp();
//        } else {
//            armDown();
//        }
//        clipControl();
//    }
//
//}

package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.hardware.Blinker;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;

public class ArmController {
    private Blinker control_Hub;
    private Blinker expansion_Hub_2;
    private HardwareDevice webcam_1;
    private DcMotor armMotor;
    private DcMotor armTest;
    private Gyroscope eimu;
    private IMU imu;
    private DcMotor leftBack;
    private DcMotor leftFront;
    private DcMotor rightBack;
    private DcMotor rightFront;
    private Servo servoe0;
    private Servo servoe1;
    private Servo servoe2;
    private Servo servoe3;
    private Servo servoe4;
    private Servo servoe5;
    private Servo servos0;
    private Servo servos1;
    private Servo servos2;
    private Servo servos3;
    private Servo servos4;
    private Servo servos5;
    private HardwareMap hardwaremap;
    private Gamepad gamepad1;
    private Gamepad gamepad2;

    private static final int HIGHBLANKET = 0;
    private static final int LOWBLANKET = 1;
    private static final int HIGHRAILING = 2;
    private static final int LOWRAILING = 3;

    public int MODE = 0;

    // todo: write your code here

    double t = 0;

    // ArmController

    boolean bhasbeenpressed = false, cliplock = false;
    boolean armup = false;
    boolean ahasbeenpressed = false;
    boolean y2hasbeenpressed = false;
    double servo_position = 0.9;

    double motorTime = 0;
    double motorLength = 100000;
    double motorNowLength = 0;
    double motorPower = 0;

    double clipLockPos = 0.7;
    double clipUnlockPos = 1;
    double clipPosition = 0;
    double clipUpPos = 0.3;
    double clipDownPos = 0.675;
    double armUpPos = 0.09;
    double armPosMax = 1;
    double armPosMin = 0.7;
    double[] armMotorPosition;

    int armUpSpendHalfMS;
    int armUpLength;
    Telemetry telemetry;
    void initArm(HardwareMap hardwaremaprc, Gamepad gamepadrc, Gamepad gamepadrc2, Telemetry telemetryrc) {

        t = System.currentTimeMillis();// 获取当前时间
        telemetry = telemetryrc;
        motorTime = t;
        gamepad1 = gamepadrc;
        gamepad2 = gamepadrc2;
        hardwaremap = hardwaremaprc;
        armMotor = hardwaremap.get(DcMotor.class, "armMotor");
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setDirection(DcMotor.Direction.REVERSE);
        servoe3 = hardwaremap.get(Servo.class, "servoe3");
        servoe4 = hardwaremap.get(Servo.class, "servoe4");
        servoe5 = hardwaremap.get(Servo.class, "servoe5");
        servoe2 = hardwaremap.get(Servo.class, "servoe2");
        cliplock = false;
        armup = false;
        servo_position = 0.9;// default position when arm is down.calculated,useless

        motorTime = 0;
        motorLength =430;
        motorNowLength = 0;
        motorPower = 0;

        clipLockPos = 0.345;// 0123
        clipUnlockPos = 0.625;// 0123
        clipUpPos = 0.3; //(0.245)// 0,1
        clipDownPos = 0.675;// calculated,useless
        clipPosition = 0;// 0123
        armUpPos = 0;// 0,1
        armPosMax = 1;// useless
        armPosMin = 0.7;// useless

        if (MODE == HIGHBLANKET || MODE == LOWBLANKET) {
            armUpSpendHalfMS = 1600;
            if (MODE == HIGHBLANKET) {
                armUpLength = 430;
            } else {
                armUpLength = 0;
            }
        } else if (MODE == HIGHRAILING || MODE == LOWRAILING) {
            armUpSpendHalfMS = 800;
            armUpLength = 430;//540
            clipUpPos = 0.9;
            if (MODE == HIGHRAILING) {
                armUpPos = 0.4;
            } else {
                armUpPos = 0.5;
            }

        }

        // armUp
        servoe3.setPosition(armUpPos);// 一级舵机竖直
        servoe4.setPosition(clipUpPos);// 二级舵机向后

        armMotor.setPower(1);

        while (System.currentTimeMillis() - t <= 2000) {
            armMotor.setTargetPosition(560 * 2);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // telemetry.addData("Testing", armMotor.getCurrentPosition());
            // telemetry.update();
        }

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("StartToBack", armMotor.getCurrentPosition());
        telemetry.update();
        while (armMotor.getCurrentPosition() + motorLength > 5 || armMotor.getCurrentPosition() + motorLength < -5) {
            armMotor.setTargetPosition(-(int) motorLength);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            telemetry.addData("Backing", armMotor.getCurrentPosition());
            telemetry.update();
        }
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // armLenthControl();
        armCalculator();
        armDown();
        //armMotor.setMode(DcMotor.RunMode.Run_WITHOUT_ENCODER);
        telemetry.addData("Finished", armMotor.getCurrentPosition());
        telemetry.update();
    }

    void gampadArmReceiver() {
        // clip
        if (gamepad1.b || gamepad2.b) {
            if (!bhasbeenpressed) {
                if (!cliplock) {
                    cliplock = true;
                    clipLockTime = System.currentTimeMillis() + 500;
                } else if (cliplock)
                    cliplock = false;
                bhasbeenpressed = true;
            } else {
                bhasbeenpressed = true;
            }
        } else {
            bhasbeenpressed = false;
        }
        // 一级控制
        /*
         * if (gamepad1.left_bumper){
         * servo_position +=0.005;
         * }
         * if (gamepad1.right_bumper){
         * servo_position -=0.005;
         * }
         */
        // 机械臂模式选择
        if (gamepad1.a || gamepad2.a) {
            if (!ahasbeenpressed) {
                if (armup) {
                    armup = false;
                    clipPosition = 0;
                    armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                } else {
                    armup = true;
                    clipPosition = 0;
                    armUpTime = t + 1600;
                    armMotor.setTargetPosition(0);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armMotor.setPower(1);
                }
                ahasbeenpressed = true;
            }
        } else {
            ahasbeenpressed = false;
        }

        if (gamepad2.y) {
            if (!y2hasbeenpressed) {
                clipPosition += 0.25;
                y2hasbeenpressed = true;
            } else {
                y2hasbeenpressed = true;
            }
        } else {
            y2hasbeenpressed = false;
        }
        if (clipPosition >= 1)
            clipPosition -= 1;
    }

    double armUpTime = 0;

    void armUp() {

        servoe3.setPosition(armUpPos);// 一级舵机竖直

        if (armUpTime >= System.currentTimeMillis()) {
            servoe4.setPosition(0.9);// 二级舵机收起
            armMotor.setTargetPosition(0);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // armMotor.setPower(1);
        } else {
            servoe4.setPosition(clipUpPos);// 二级舵机向后
            armMotor.setTargetPosition(armUpLength);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // armMotor.setPower(1);
        }
    }

    void armDown() {
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        servoe3.setPosition(servo_position);// 一级舵机地面
        servoe4.setPosition(clipDownPos);// 二级舵机向下
    }

    void angleRange() {
        // arm角度范围限定
        if (servo_position > armPosMax)
            servo_position = armPosMax;
        if (servo_position < armPosMin)
            servo_position = armPosMin;
        // arm长度范围限定
        // motorNowLength = Math.max(0,Math.min(motorLength,motorNowLength));
        // clip角度范围限定
        clipDownPos = Math.max(0.29, Math.min(0.715, clipDownPos));

    }

    double clipLockTime = 0;

    void clipControl() {
        if (cliplock && clipLockTime <= System.currentTimeMillis())
            servoe5.setPosition(clipLockPos);
        else
            servoe5.setPosition(clipUnlockPos);
        servoe2.setPosition(clipPosition);
    }

    public void armLenthControl() {
        if (!armup) {
            if (gamepad1.left_trigger >= 0.5 && gamepad1.right_trigger < 0.5 && motorNowLength > 0)
                motorPower = -0.5;
            else if (gamepad1.left_trigger < 0.5 && gamepad1.right_trigger >= 0.5 && motorNowLength < motorLength)
                motorPower = 0.5;
            else if (gamepad2.left_trigger >= 0.5 && gamepad2.right_trigger < 0.5 && motorNowLength > 0)
                motorPower = -0.5;
            else if (gamepad2.left_trigger < 0.5 && gamepad2.right_trigger >= 0.5 && motorNowLength < motorLength)
                motorPower = 0.5;
            else
                motorPower = 0;
            armMotor.setPower(motorPower);
        }
        /*
         * if (motorPower!=0){
         * motorNowLength += motorPower*(System.currentTimeMillis()-motorTime);
         * }
         */
        motorNowLength = armMotor.getCurrentPosition();

        // telemetry.addData("armPos",armMotor.getCurrentPosition());
        motorTime = System.currentTimeMillis();
    }

    double clipHeight = 6;// 12
    double clipHeightError = 0;

    void armCalculator() {
        // if (cliplock && clipLockTime + 500 >= System.currentTimeMillis())
        //     clipHeightError = 3;// servo_position+=0.12;
        // else if (gamepad2.x)
        //     clipHeightError = 3;// servo_position+=0.12;//8
        // else
        //     clipHeightError = 0;
        double L = 30.5 + (motorNowLength / motorLength) * 32.0;
        double argument = -(clipHeight + clipHeightError) / L;
        servo_position = Math.toDegrees(Math.acos(argument)) / 135.0 /*- 0.1*/;

        if (cliplock && clipLockTime + 500 >= System.currentTimeMillis())
            servo_position+=0.12;
        else if (gamepad2.x)
            servo_position+=0.12;//8

        clipDownPos = (3 * 0.3 + 2.2 - 1.5 * (servo_position - 0.1)) / 3;

    }

    public void armController() {
        t = System.currentTimeMillis();// 获取当前时间
        motorTime = t;
        gampadArmReceiver();
        armLenthControl();
        armCalculator();
        if (armup) {
            armUp();
        } else {
            armDown();
        }
        clipControl();
    }

}