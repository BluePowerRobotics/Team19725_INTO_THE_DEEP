package org.firstinspires.ftc.teamcode.Controllers;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;

public class IntakeLength{

    HardwareMap hardwareMap;
    public IntakeLength(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        IntakeLengthController.getInstance(hardwareMap);
    }
    double targetPosition;

    public class IntakeRunToPosition implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet){
            IntakeLengthController.getInstance().setIntakeTargetPosition(targetPosition).update();
            return !(IntakeLengthController.getInstance().getIntakeLengthNowRadian() == IntakeLengthController.getInstance().getIntakeLengthTargetRadian());//true=continue , false = finish
        }

    }
    public Action intakeRunToPosition(double targetPosition){
        this.targetPosition=targetPosition;
        return new IntakeRunToPosition();
    }

    public class IntakeLengthInit implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet){
            IntakeLengthController.getInstance().setIntakeTargetPosition(0).update();
            return !(IntakeLengthController.getInstance().getIntakeLengthNowRadian() == IntakeLengthController.getInstance().getIntakeLengthTargetRadian());
        }

    }
    public Action intakeLengthInit(){

        return new IntakeLengthInit();
    }
}

class IntakeLengthController{
    static IntakeLengthController instance;

    private Servo inTakeLengthController;
    private HardwareMap hardwaremap;
    public IntakeLengthController(HardwareMap hardwaremap){
        this.hardwaremap = hardwaremap;
        inTakeLengthController = this.hardwaremap.get(Servo.class,"");
    }
    private double armALength=0;
    private double armBLength=0;
    private double inTakeMinLength=0;//收到最小时相对于默认值的位置（伸出为正）
    private double intakeTargetLength;
    private double intakeLengthTargetRadian;
    private double intakeLengthNowRadian;

    public IntakeLengthController setIntakeTargetPosition(double inTakeLength){
        intakeTargetLength = inTakeLength;
        intakeLengthTargetRadian = Math.acos((armALength*armALength+(intakeTargetLength +inTakeMinLength)*(intakeTargetLength +inTakeMinLength)-armBLength*armBLength)/(2*armALength*(intakeTargetLength +inTakeMinLength)));
        return instance;
    }
    public static synchronized IntakeLengthController getInstance(HardwareMap hardwareMap) {
        if (instance == null) {
            instance = new IntakeLengthController(hardwareMap);
        }
        return instance;
    }
    public static synchronized IntakeLengthController getInstance() {
        return instance;
    }

    public IntakeLengthController update(){
        inTakeLengthController.setPosition(intakeLengthTargetRadian /Math.PI);
        setPositionTime = System.currentTimeMillis();
        return instance;
    }
    long setPositionTime = 0;
    double servoSpeed = 0.24;//sec per 60 degree

    public double getIntakeLengthNowRadian(){
        boolean goBigger,goSmaller;
        goBigger = intakeLengthTargetRadian-intakeLengthNowRadian>0;
        goSmaller = intakeLengthTargetRadian-intakeLengthNowRadian<0;
        if(goBigger){
            intakeLengthNowRadian = Math.min(intakeLengthNowRadian+(System.currentTimeMillis()-setPositionTime)*Math.PI/(3*servoSpeed),intakeLengthTargetRadian);
        }
        else if(goSmaller){
            intakeLengthNowRadian = Math.max(intakeLengthNowRadian-(System.currentTimeMillis()-setPositionTime)*Math.PI/(3*servoSpeed),intakeLengthTargetRadian);
        }else{
            intakeLengthNowRadian = intakeLengthTargetRadian;
        }
        return intakeLengthNowRadian;
    }
    public double getIntakeLengthTargetRadian(){
        return intakeLengthTargetRadian;
    }
}
