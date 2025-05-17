package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ArmDirectController {
    DistanceSensor distanceSensor;
    public Gamepad gamepad2;
    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    public Servo servoe2,servoe3,servoe4,servoe5;
    public DcMotor armMotor,armSpinner;
    public void initArm(HardwareMap hardwareMapRC,Gamepad gamepad2RC, Telemetry telemetryRC){
        hardwareMap = hardwareMapRC;
        telemetry = telemetryRC;
        gamepad2 = gamepad2RC;
        servoe2 = hardwareMap.get(Servo.class,"servoe2");
        servoe3 = hardwareMap.get(Servo.class, "servoe3");
        servoe4 = hardwareMap.get(Servo.class,"servoe4");
        servoe5 = hardwareMap.get(Servo.class, "servoe5");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setDirection(DcMotor.Direction.REVERSE);
        armSpinner = hardwareMap.get(DcMotor.class, "armSpinner");
        armSpinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        armSpinner.setDirection(DcMotorSimple.Direction.FORWARD);
        distanceSensor = hardwareMap.get(DistanceSensor.class,"distanceSensor");
    }
    long t ;
    double armUpPos = 0;
    double clipUpPos = 0.3;
    double clipDownPos = 0.675;
    public void initArmAction(){
        t = System.currentTimeMillis();
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
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //servoe3.setPosition(servo_position);// 一级舵机地面
        servoe4.setPosition(clipDownPos);// 二级舵机向下
        //armMotor.setMode(DcMotor.RunMode.Run_WITHOUT_ENCODER);
        telemetry.addData("Finished", armMotor.getCurrentPosition());
        telemetry.update();

        armSpinner.setTargetPosition(0);
        armSpinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armSpinner.setPower(0.4);
        servoe4.setPosition(0.7);
        servoe5.setPosition(1);
        servoe2.setPosition(0);
    }

    int TargetArmSpinnerPosition = 0;
    long armSpinnerTime;
    boolean armSpinnerTargetPositionChanged = false;
    double armSpinnerPositionChangingSpeed = 6*2;
    public void armSpinnerController(double RECEIVE){
        if(RECEIVE!=0&&!armSpinnerTargetPositionChanged){
            TargetArmSpinnerPosition-=armSpinnerPositionChangingSpeed*RECEIVE;
            armSpinnerTime = System.currentTimeMillis();
            armSpinnerTargetPositionChanged = true;
        } else if (armSpinnerTargetPositionChanged) {
            if(System.currentTimeMillis()-armSpinnerTime>=50){
                armSpinnerTargetPositionChanged = false;
            }
        }
        if(TargetArmSpinnerPosition>120)TargetArmSpinnerPosition = 120;
        else if(TargetArmSpinnerPosition<0)TargetArmSpinnerPosition = 0;
        if(armSpinner.getTargetPosition()!=TargetArmSpinnerPosition)armSpinner.setTargetPosition(TargetArmSpinnerPosition);
        armSpinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armSpinner.setPower(1);
    }
    int TargetarmMotorPosition = 0;
    long armMotorTime;
    boolean armMotorTargetPositionChanged = false;
    double armMotorPositionChangingSpeed = 30*2;
    public void armMotorController(double RECEIVE){
        if(RECEIVE!=0&&!armMotorTargetPositionChanged){
            TargetarmMotorPosition-=armMotorPositionChangingSpeed*RECEIVE;
            armMotorTime = System.currentTimeMillis();
            armMotorTargetPositionChanged = true;
        } else if (armMotorTargetPositionChanged) {
            if(System.currentTimeMillis()-armMotorTime>=50){
                armMotorTargetPositionChanged = false;
            }
        }
        if(TargetarmMotorPosition>600)TargetarmMotorPosition = 600;
        else if(TargetarmMotorPosition<0)TargetarmMotorPosition = 0;
        if(armMotor.getTargetPosition()!=TargetarmMotorPosition)armMotor.setTargetPosition(TargetarmMotorPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1);
    }
    double Targetservoe4Position = 0;
    long servoe4Time;
    boolean servoe4TargetPositionChanged = false;
    double servoe4PositionChangingSpeed = 0.5;
    public void servoe4Controller(double RECEIVE){
//        if(RECEIVE!=0&&!servoe4TargetPositionChanged){
//            Targetservoe4Position-=servoe4PositionChangingSpeed*RECEIVE;
//            servoe4Time = System.currentTimeMillis();
//            servoe4TargetPositionChanged = true;
//        } else if (servoe4TargetPositionChanged) {
//            if(System.currentTimeMillis()-servoe4Time>=50){
//                servoe4TargetPositionChanged = false;
//            }
//        }
//        if(Targetservoe4Position>1)Targetservoe4Position = 1;
//        else if(Targetservoe4Position<0)Targetservoe4Position = 0;
//        if(servoe4.getPosition()!=Targetservoe4Position)servoe4.setPosition(Targetservoe4Position);
        servoe4.setPosition(0.65);
    }
    double clipLockPos=0.345;
    double clipUnLockPos=0.625;
    boolean clipLock = false;
    boolean clipHasBeenPressed;
    public void servoe5Controller(boolean RECEIVE){
        if(RECEIVE == true){
            if(!clipHasBeenPressed){
                clipLock = !clipLock;
                clipHasBeenPressed= true;
            }

        }else{
            clipHasBeenPressed = false;
        }
        if(clipLock){
            servoe5.setPosition(clipLockPos);
        }else{
            servoe5.setPosition(clipUnLockPos);
        }
    }
    public double clipPosition = 0;
    public boolean servoe2HasBeenPressed = false;
    public void servoe2Controller(boolean RECEIVE){
        if (RECEIVE) {
            if (!servoe2HasBeenPressed) {
                clipPosition += 0.25;
                servoe2HasBeenPressed = true;
            } else {
                servoe2HasBeenPressed = true;
            }
        } else {
            servoe2HasBeenPressed = false;
        }
        if (clipPosition >= 1)
            clipPosition -= 1;
    }

}
