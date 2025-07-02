package org.firstinspires.ftc.teamcode.Controllers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class OutPutArmController {
    private Servo outPutPositionController,outPutClipController;
    private DcMotor outPutArmLengthController;
    private HardwareMap hardwareMap;
    private final double outPutClipLockPos = 0,outPutClipUnlockPos = 0;
    private final double outPutLengthControllerNumberPerCycle=0,outPutLengthControllerMMPerCycle=0;
    public void initOutPutArm(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        outPutPositionController = hardwareMap.get(Servo.class,"");
        outPutClipController = hardwareMap.get(Servo.class,"");
        outPutArmLengthController = hardwareMap.get(DcMotor.class,"");

    }
    RobotStates.OUTPUT_RUNMODE outputStates = RobotStates.OUTPUT_RUNMODE.WAITING;
    public void setMode(RobotStates.OUTPUT_RUNMODE outputStates){
        this.outputStates = outputStates;
        switch (this.outputStates){
            case WAITING:
                break;
            case SCANNING:
                break;
            case DOWNING:
                break;
            case TAKING:
                break;
            case UPPING:
                break;
            case PUTTING:
                break;
        }
    }


}
