package org.firstinspires.ftc.teamcode.Autonomous;

//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.sax.StartElementListener;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm {

    private HardwareDevice webcam_1;
    private DcMotor armMotor;
    private DcMotor armTest;
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
    // todo: write your code here
    
    double t = 0;
    
    //ArmController
    
    boolean bhasbeenpressed = false, cliplock = false;
    boolean armup = false;
    boolean ahasbeenpressed = false;
    double servo_position = 0.9;

    double motorTime = 0;
    double motorLength=100000;
    double motorNowLength = 0;
    double motorPower = 0;
    
    double clipLockPos=0.7;
    double clipUnlockPos=1;
    double clipUpPos=0.3;
    double clipDownPos=0.675;
    double armUpPos=0;
    double armPosMax=1;
    double armPosMin=0.7;
    double[] armMotorPosition;
    Telemetry telemetry;
    public Arm(HardwareMap hardwaremaprc, Telemetry telemetryrc){
        telemetry = telemetryrc;
        t=System.currentTimeMillis();//获取当前时间
        motorTime = t;

        hardwaremap = hardwaremaprc;
        armMotor = hardwaremap.get(DcMotor.class, "armMotor");
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        servoe3 = hardwaremap.get(Servo.class, "servoe3");
        servoe4 = hardwaremap.get(Servo.class, "servoe4");
        servoe5 = hardwaremap.get(Servo.class, "servoe5");
        cliplock = false;
        armup = false;
        servo_position = 0.9;//default position when arm is down.
        
        motorTime = 0;
        motorLength=600;
        motorNowLength = 0;
        motorPower = 0;
        
        clipLockPos=0.72;
        clipUnlockPos=1;
        clipUpPos=0.245;
        clipDownPos=0.715;
        armUpPos=0.1;
        armPosMax=1;
        armPosMin=0.7;
        
//        //armUp
//        servoe3.setPosition(armUpPos);//一级舵机竖直
//        servoe4.setPosition(clipUpPos);//二级舵机向后
//
//        armMotor.setPower(1);
//
//        while(System.currentTimeMillis()-t<=2000){
//            armMotor.setTargetPosition(560*2);
//            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            telemetry.addData("Testing", armMotor.getCurrentPosition());
//            telemetry.update();
//        }
//
//        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        //telemetry.addData("StartToBack", armMotor.getCurrentPosition());
//        //telemetry.update();
//        while(armMotor.getCurrentPosition()+motorLength>5 || armMotor.getCurrentPosition()+motorLength<-5){
//            armMotor.setTargetPosition(-(int)motorLength);
//            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            telemetry.addData("Backing", armMotor.getCurrentPosition());
//            telemetry.update();
//        }
//        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        armLenthControl();
//        armCalculator();
//        armDown();
        //armMotor.setMode(DcMotor.RunMode.Run_WITHOUT_ENCODER);
        //telemetry.addData("Finished", armMotor.getCurrentPosition());
        //telemetry.update();
    }
    
    
//    void gampadArmReceiver(){
//        //clip
//        if (gamepad1.b){
//            if (!bhasbeenpressed){
//                if (!cliplock){
//                    cliplock = true;
//                    clipLockTime = System.currentTimeMillis()+500;
//                }
//                else if (cliplock) cliplock = false;
//                bhasbeenpressed = true;
//            }else{
//                bhasbeenpressed = true;
//            }
//        }else{
//            bhasbeenpressed = false;
//        }
//        //一级控制
//        /*
//        if (gamepad1.left_bumper){
//            servo_position +=0.005;
//        }
//        if (gamepad1.right_bumper){
//            servo_position -=0.005;
//        }*/
//        //机械臂模式选择
//        if (gamepad1.a){
//            if (!ahasbeenpressed){
//                if(armup) {
//                    armup=false;
//                    armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                }else {
//                    armup = true;
//                    armUpTime = t +1500;
//                    armMotor.setTargetPosition(0);
//                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    armMotor.setPower(1);
//                }
//                ahasbeenpressed = true;
//            }
//        }else{
//            ahasbeenpressed = false;
//        }
//    }
    
    double armUpTime=0;

    public class initArm implements Action{
        private boolean initiatized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(!initiatized){
                t=System.currentTimeMillis();//获取当前时间
                motorTime = t;
                armMotor = hardwaremap.get(DcMotor.class, "armMotor");
                armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                servoe3 = hardwaremap.get(Servo.class, "servoe3");
                servoe4 = hardwaremap.get(Servo.class, "servoe4");
                servoe5 = hardwaremap.get(Servo.class, "servoe5");
                cliplock = false;
                armup = false;
                servo_position = 0.9;//default position when arm is down.

                motorTime = 0;
                motorLength=600;
                motorNowLength = 0;
                motorPower = 0;

                clipLockPos=0.72;
                clipUnlockPos=1;
                clipUpPos=0.245;
                clipDownPos=0.675;
                armUpPos=0.1;
                armPosMax=1;
                armPosMin=0.7;

                //armUp
                servoe3.setPosition(armUpPos);//一级舵机竖直
                servoe4.setPosition(clipUpPos);//二级舵机向后
                servoe5.setPosition(clipLockPos);
                armMotor.setPower(1);


                initiatized = true;
            }
            if(System.currentTimeMillis()-t<=2000){
                armMotor.setTargetPosition(560*2);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                telemetry.addData("Testing", armMotor.getCurrentPosition());
                telemetry.update();
                return false;
            }

            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            telemetry.addData("StartToBack", armMotor.getCurrentPosition());
            telemetry.update();
            if(armMotor.getCurrentPosition()+motorLength>5 || armMotor.getCurrentPosition()+motorLength<-5){
                armMotor.setTargetPosition(-(int)motorLength);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                telemetry.addData("Backing", armMotor.getCurrentPosition());
                telemetry.update();
                return false;
            }
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //armLenthControl();
            armCalculator();
            armDown();
            servoe5.setPosition(clipLockPos);
            //armMotor.setMode(DcMotor.RunMode.Run_WITHOUT_ENCODER);
            telemetry.addData("Finished", armMotor.getCurrentPosition());
            telemetry.update();
            return true;
        }

    }
    public Action initArm() {
        return new initArm();
    }
    public class outPut implements Action{
        private boolean initialized = false;
        private boolean finished = false;
        private long finishtime = 0;
        long nowTime=System.currentTimeMillis()*2;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                armUpTime = t +1500;
                armMotor.setTargetPosition(0);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(1);
                initialized = true;
            }
            servoe3.setPosition(armUpPos);//一级舵机竖直
            servoe4.setPosition(clipUpPos);//二级舵机向后
            //servoe5.setPosition(clipLockPos);
            if(armUpTime>=System.currentTimeMillis()){
                armMotor.setTargetPosition(0);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //armMotor.setPower(1);
                telemetry.addData("goingshorter",armMotor.getCurrentPosition());

                return true;
            }else{
                armMotor.setTargetPosition(540);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //armMotor.setPower(1);
                telemetry.addData("goinglonger",armMotor.getCurrentPosition());


            }
            if(armMotor.getCurrentPosition()-540 < 5 && armMotor.getCurrentPosition()-540 > -5){
                servoe5.setPosition(clipUnlockPos);

                if (!finished) {

                    finishtime = System.currentTimeMillis() + 1000;
                    finished = true;
                }

                telemetry.addData("unlocking",armMotor.getCurrentPosition());
                telemetry.update();
            }
            if (finishtime!=0&&System.currentTimeMillis()>=finishtime) {

                return false;
            } else {
                return true;
            }
        }


    }

    public Action outPut() {
        return new outPut();
    }

    void armDown(){
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        servoe3.setPosition(servo_position);//一级舵机地面
        servoe4.setPosition(clipDownPos);//二级舵机向下
    }
    
    
    void angleRange(){
        //arm角度范围限定
            if(servo_position>armPosMax) servo_position = armPosMax;
            if(servo_position<armPosMin) servo_position = armPosMin;
        //arm长度范围限定
            //motorNowLength = Math.max(0,Math.min(motorLength,motorNowLength));
        //clip角度范围限定
            clipDownPos = Math.max(0.29, Math.min(0.715, clipDownPos));
            
    }
    
    double clipLockTime=0;
    void clipControl(){
        if(cliplock && clipLockTime<=System.currentTimeMillis()) servoe5.setPosition(clipLockPos);
        else servoe5.setPosition(clipUnlockPos);
    }
    
    public int armLength = 0;
    public void armLenthControl(){
        if(!armup){
//            if(gamepad1.left_trigger >=0.5 && gamepad1.right_trigger <0.5 && motorNowLength>0) motorPower = -0.5;
//            else if(gamepad1.left_trigger < 0.5 && gamepad1.right_trigger >=0.5 && motorNowLength<motorLength) motorPower = 0.5;
//            else motorPower = 0;
//            armMotor.setPower(motorPower);
            armMotor.setTargetPosition(armLength);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        /*if (motorPower!=0){
            motorNowLength += motorPower*(System.currentTimeMillis()-motorTime);
        }*/
        motorNowLength = armMotor.getCurrentPosition();

        //telemetry.addData("armPos",armMotor.getCurrentPosition());
        motorTime = System.currentTimeMillis();
    }
    
    void armCalculator(){
        double L = 30.5 + (motorNowLength / motorLength) * 32.0;
        double argument = -18.0 / L;
        servo_position = Math.toDegrees(Math.acos(argument)) / 135.0 /*- 0.1*/;
        if(cliplock && clipLockTime+500>=System.currentTimeMillis()) servo_position+=0.12;
        clipDownPos = (3*0.29+2-1.5*(servo_position-0.2))/3;
    }
    
//    public void armController(){
//        t=System.currentTimeMillis();//获取当前时间
//        motorTime = t;
//        gampadArmReceiver();
//        armLenthControl();
//        armCalculator();
//        if (armup){
//            armUp();
//        }else{
//            armDown();
//        }
//        clipControl();
//    }
    
    
}