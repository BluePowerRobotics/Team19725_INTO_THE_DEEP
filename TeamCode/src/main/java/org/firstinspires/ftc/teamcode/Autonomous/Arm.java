package org.firstinspires.ftc.teamcode.Autonomous;

//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Arm {

    private Blinker control_Hub;
    private Blinker expansion_Hub_2;
    private HardwareDevice webcam_1;
    public DcMotor armMotor;
    //private DcMotor armPuller;
    private  DcMotor armSpinner;
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
    private DistanceSensor distanceSensor;

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
    int armSpinnerLength = 112;
    int armSpinnerPosition = 0;
    int armSpinnerStartPosition;
    int armSpinnerStopPosition;
    int armUpSpendHalfMS;
    int armUpLength;
    Telemetry telemetry;
    double clipHeightErrorNumber = 8;

    public Arm(HardwareMap hardwaremaprc, Telemetry telemetryrc) {
        telemetry = telemetryrc;
        t = System.currentTimeMillis();// 获取当前时间
        motorTime = t;

        hardwaremap = hardwaremaprc;
        armMotor = hardwaremap.get(DcMotor.class, "armMotor");
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servoe3 = hardwaremap.get(Servo.class, "servoe3");
        servoe4 = hardwaremap.get(Servo.class, "servoe4");
        servoe5 = hardwaremap.get(Servo.class, "servoe5");
        cliplock = false;
        armup = false;

    }

    boolean startToBack = false;
    public class initArm implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            t = System.currentTimeMillis();// 获取当前时间
            motorTime = t;

            armMotor = hardwaremap.get(DcMotor.class, "armMotor");
            armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armMotor.setDirection(DcMotor.Direction.REVERSE);
            //armPuller = hardwaremap.get(DcMotor.class,"armPuller");
            //armPuller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //armPuller.setDirection(DcMotorSimple.Direction.FORWARD);
            armSpinner = hardwaremap.get(DcMotor.class, "armSpinner");
            armSpinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            armSpinner.setDirection(DcMotorSimple.Direction.FORWARD);
            distanceSensor = hardwaremap.get(DistanceSensor.class,"distanceSensor");
            Rev2mDistanceSensor sensorTimeOfFight = (Rev2mDistanceSensor) distanceSensor;
            //80
            //50
            servoe3 = hardwaremap.get(Servo.class, "servoe3");
            servoe4 = hardwaremap.get(Servo.class, "servoe4");
            servoe5 = hardwaremap.get(Servo.class, "servoe5");
            servoe2 = hardwaremap.get(Servo.class, "servoe2");
            cliplock = false;
            armup = false;
            servo_position = 0.9;// default position when arm is down.calculated,useless

            motorTime = 0;
            motorLength =600;
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

            armSpinnerLength = 120;
            armSpinnerPosition = (int)((120.0/135)*45);
            //armSpinnerStartPosition = ;
            //armSpinnerStopPosition = ;

            if (MODE == HIGHBLANKET || MODE == LOWBLANKET) {
                armUpSpendHalfMS = 1600;
                if (MODE == HIGHBLANKET) {
                    armUpLength = (int) motorLength;
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
            //armSpinner.setPower(-0.5);//一级电机向上拉
            servoe4.setPosition(clipUpPos);// 二级舵机向后

            //1.收到最短
            //2.用力抬升（armPuller拉满。setPower(1);）
            while (System.currentTimeMillis() - t <= 2000) {
                //armPuller.setPower(1);
                armMotor.setPower(-1);
                telemetry.addData("Backing", armMotor.getCurrentPosition());
                //telemetry.addData("TESTING", armPuller.getCurrentPosition());
                armSpinner.setPower(1);
                telemetry.addData("Uping", armSpinner.getCurrentPosition());
                telemetry.update();
            }
            armSpinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            armSpinner.setDirection(DcMotor.Direction.REVERSE);

            armSpinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            //armSpinner.set

            armSpinner.setPower(1);
            //armPuller.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            telemetry.addData("StartToTest", armMotor.getCurrentPosition());
            telemetry.update();
            armMotor.setTargetPosition(0);
            // armSpinner.setTargetPosition(60);

            int Place = 0;
            int Arm = 1;
            int uponArm = 2;
            int anywhere = 0;
            boolean armHasRun = false;
            while ((Place!=uponArm)&&(!((Place==anywhere)&&armHasRun))){
                if(Math.abs(distanceSensor.getDistance(DistanceUnit.MM) - 30) < 10){
                    telemetry.addData("status", "主臂");
                    telemetry.update();
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Place = Arm;
                    armHasRun = true;
                }
                else if(Math.abs(distanceSensor.getDistance(DistanceUnit.MM) - 60) < 10){
                    telemetry.addData("status", "主臂上");
                    telemetry.update();
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Place = uponArm;
                }
                else{
                    telemetry.addData("status", "无");
                    telemetry.update();
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Place = anywhere;
                }
            }
            armSpinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armSpinner.setDirection(DcMotor.Direction.FORWARD);
            armSpinner.setTargetPosition(0);
            armSpinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armSpinner.setPower(1);
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            // armLenthControl();
            //armCalculator();
            armDown();
            //armMotor.setMode(DcMotor.RunMode.Run_WITHOUT_ENCODER);
            telemetry.addData("Finished", armMotor.getCurrentPosition());
            telemetry.update();
            return false;
        }

    }

    public Action initArm() {
        return new initArm();
    }

    public class outPut implements Action {
        private boolean initialized = false;
        private boolean finished = false;
        private long finishtime = 0;
        long nowTime = System.currentTimeMillis() * 2;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            cliplock = false;
            armController();
            return false;
        }

    }

    public Action outPut() {
        return new outPut();
    }

    public int targetArmLength;

    public class armDownAction implements Action {

        private boolean initialized = false;
        private boolean startToInTake = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armup = false;
            armController();
            return false;
        }
    }

    public Action armDownAction(int targetInTakeLength, double targetClipPosition) {
        targetArmLength = targetInTakeLength;
        clipPosition = targetClipPosition;
        return new armDownAction();
    }
    public class armUpAction implements Action {

        private boolean initialized = false;
        private boolean startToInTake = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            armup = true;
            clipPosition = 0;
            armUpTime = t + 1600;
            armMotor.setTargetPosition(0);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(1);
            armController();
            return false;
        }
    }

    public Action armUpAction() {
        return new armUpAction();
    }

    public class inTake implements Action {

        private boolean initialized = false;
        private boolean startToInTake = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            cliplock = true;
            clipLockTime = System.currentTimeMillis() + 500;
            armController();
            return false;
        }
    }

    public Action inTake() {

        return new inTake();
    }


    double armUpTime = 0;

    void armUp() {
        if(armSpinner.getTargetPosition()!=120) armSpinner.setTargetPosition(120);
        servoe3.setPosition(armUpPos);// 一级舵机竖直
        armSpinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
        if(armSpinner.getTargetPosition()!=armSpinnerPosition) armSpinner.setTargetPosition(armSpinnerPosition);
        armSpinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
            armMotor.setTargetPosition(targetArmLength);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(1);
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

    //double armPullerLength = 0;
    double armPullerCycleEncoder =300;
    double armPullerCycleReal=1.3*2*Math.PI;
    void armCalculator() {
        armSpinnerPosition = (int)((120.0/135)*45);
        if (cliplock && clipLockTime + 500 >= System.currentTimeMillis())
            armSpinnerPosition-=30;//clipHeightError = 3;// servo_position+=0.12;
        else if (gamepad2.x)
            armSpinnerPosition-=30;//clipHeightError = 3;// servo_position+=0.12;//8
        else
            armSpinnerPosition-=0;//clipHeightError = 0;
        double L = 30.5 + (motorNowLength / motorLength) * 32.0;
        double argument = -(clipHeight + clipHeightError) / L;
        servo_position = Math.toDegrees(Math.acos(argument)) / 135.0 /*- 0.1*/;
        servo_position = Math.max(0, Math.min(1.0, servo_position));


//        if (cliplock && clipLockTime + 500 >= System.currentTimeMillis())
//            servo_position+=0.12;
//        else if (gamepad2.x)
//            servo_position+=0.12;//8

        clipDownPos = (3 * 0.3 + 2.2 - 1.5 * (servo_position - 0.1)) / 3;
    }

    public void armController() {
        t = System.currentTimeMillis();// 获取当前时间
        motorTime = t;
        armLenthControl();
        armCalculator();
        if (armup) {
            armUp();
        } else {
            armDown();

        }
//        armPuller.setPower(1);
//        armPuller.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        clipControl();
    }

}