package org.firstinspires.ftc.teamcode.Controllers;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class BasicArmController {
    private Servo inTakeLengthController;
    private Servo inTakePositionController;
    private Servo clipController;
    private double clipLockPos;
    private double clipUnlockPos;
    private HardwareMap hardwaremap;
    public void initBasicArm(HardwareMap hardwaremap){
        this.hardwaremap = hardwaremap;
        inTakeLengthController = this.hardwaremap.get(Servo.class,"");
        inTakePositionController = this.hardwaremap.get(Servo.class,"");
    }
    private double armALength=0;
    private double armBLength=0;
    private double inTakeMinLength=0;
    private double inTakeTargetLength;
    private double inTakeTargetLengthAngle;
    private double inTakeTargetAngle;
    public void setInTakePosition(double inTakeLength,double inTakeAngle){
        inTakeTargetLength = inTakeLength;
        inTakeTargetAngle = inTakeAngle;
    }
    public void inTakeLengthCalculator(){
        inTakeTargetLengthAngle = Math.acos((armALength*armALength+(inTakeTargetLength+inTakeMinLength)*(inTakeTargetLength+inTakeMinLength)-armBLength*armBLength)/(2*armALength*(inTakeTargetLength+inTakeMinLength)))/Math.PI;

    }
    public void setInTake(){
        inTakeLengthController.setPosition(inTakeTargetLengthAngle);
        inTakePositionController.setPosition(inTakeTargetAngle);
    }
}
