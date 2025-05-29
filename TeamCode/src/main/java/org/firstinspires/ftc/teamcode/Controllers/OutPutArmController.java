package org.firstinspires.ftc.teamcode.Controllers;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class OutPutArmController {
    private Servo outPutPositionController,outPutClipController;
    private HardwareMap hardwareMap;
    private double outPutClipLockPos,outPutClipUnlockPos;
    public void initOutPutArm(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        outPutPositionController = hardwareMap.get(Servo.class,"");
        outPutClipController = hardwareMap.get(Servo.class,"");

    }


}
