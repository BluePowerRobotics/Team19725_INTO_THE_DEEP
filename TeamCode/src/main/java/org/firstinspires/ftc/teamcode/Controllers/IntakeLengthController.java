package org.firstinspires.ftc.teamcode.Controllers;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeLengthController {
    private Servo inTakeLengthController;
    private HardwareMap hardwaremap;
    public void initBasicArm(HardwareMap hardwaremap){
        this.hardwaremap = hardwaremap;
        inTakeLengthController = this.hardwaremap.get(Servo.class,"");
    }
    private double armALength=0;
    private double armBLength=0;
    private double inTakeMinLength=0;
    private double inTakeTargetLength;
    private double inTakeTargetLengthAngle;
    public void setInTakePosition(double inTakeLength){
        inTakeTargetLength = inTakeLength;
        inTakeTargetLengthAngle = Math.acos((armALength*armALength+(inTakeTargetLength+inTakeMinLength)*(inTakeTargetLength+inTakeMinLength)-armBLength*armBLength)/(2*armALength*(inTakeTargetLength+inTakeMinLength)))/Math.PI;
    }
    public void setLength(){
        inTakeLengthController.setPosition(inTakeTargetLengthAngle);
    }
}
