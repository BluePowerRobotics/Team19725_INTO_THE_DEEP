package org.firstinspires.ftc.teamcode.Controllers;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class OutputController {
    public OutputController(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        outputPositionController = hardwareMap.get(Servo.class,"");
        outputClipController = hardwareMap.get(Servo.class,"");
        outputLengthController = hardwareMap.get(DcMotor.class,"");
    }
    private Servo outputPositionController, outputClipController;
    private DcMotor outputLengthController;
    private HardwareMap hardwareMap;
    private final double outputClipLockPos = 0, outputClipUnlockPos = 0;
    private final double outputLengthControllerNumberPerCycle =0, outputLengthControllerMMPerCycle =0;

    RobotStates.OUTPUT_RUNMODE outputStates = RobotStates.OUTPUT_RUNMODE.WAITING;
    public void setMode(RobotStates.OUTPUT_RUNMODE outputStates){
        this.outputStates = outputStates;
        switch (this.outputStates){
            case WAITING:
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



    public static OutputController instance;
    public static synchronized OutputController getInstance(HardwareMap hardwareMap){
        if(instance == null){
            instance = new OutputController(hardwareMap);
        }
        return instance;
    }
    public static synchronized OutputController getInstance(){
        return instance;
    }

    public OutputController setClip(boolean isLocked){
        outputClipController.setPosition(isLocked? outputClipLockPos : outputClipUnlockPos);
        return instance;
    }
    int outputLengthControllerValue =0;
    public OutputController setTargetOutputHeight(double height){
        outputLengthControllerValue =(int)(outputLengthControllerNumberPerCycle *height/ outputLengthControllerMMPerCycle);
        outputLengthController.setTargetPosition(outputLengthControllerValue);
        return instance;
    }
    public OutputController setArmPosition(double servoValue){
        servoValue = Math.max(0.0,Math.min(1.0,servoValue));
        outputPositionController.setPosition(servoValue);
        return instance;
    }
    public boolean update(){
        outputLengthController.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        return !(Math.abs(outputLengthController.getCurrentPosition()- outputLengthControllerValue)<=2);
    }

}

