

package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmController {
    private Servo inTakeLengthController;
    private Servo inTakePositionController;
    private Servo clipController;
    private HardwareMap hardwaremap;
    private Gamepad gamepad1;
    private Gamepad gamepad2;

    private double clipLockPos;
    private double clipUnlockPos;





    public boolean AUTO = false;

    public MODE RUNMODE = MODE.HIGH_CHAMBER;
    public enum MODE {HIGH_CHAMBER,LOW_CHAMBER,HIGH_BASKET,LOW_BASKET}
    // todo: write your code here

    double t = 0;

    // ArmController
    public boolean initArmInitiated=false;
    public boolean initArm(HardwareMap hardwareMapRC, Gamepad gamepad1RC, Gamepad gamepad2RC, Telemetry telemetry){
        if(!initArmInitiated){
            hardwaremap = hardwareMapRC;
            gamepad1 = gamepad1RC;
            gamepad2 = gamepad2RC;
        }

        return false;
    }



    public void armController() {
        t = System.currentTimeMillis();// 获取当前时间

    }
    public OUTPUT_MODE outPutStatus;
    public void outPut(){
        if (RUNMODE == MODE.HIGH_CHAMBER){
            if(outPutStatus != OUTPUT_MODE.WAITING) outPutStatus = chamberOutPut();
        }
    }
    public enum OUTPUT_MODE {PUTTING, UPPING,DOWNING, WAITING}
    public OUTPUT_MODE OUTPUT_RUNMODE;
    public OUTPUT_MODE chamberOutPut(){
        if (OUTPUT_RUNMODE==OUTPUT_MODE.UPPING){

        } else if (OUTPUT_RUNMODE == OUTPUT_MODE.PUTTING) {

        }else if(OUTPUT_RUNMODE == OUTPUT_MODE.DOWNING){

        }
        return OUTPUT_RUNMODE;
    }
    public enum INTAKE_MODE{EXTENDING,SHORTENING,TAKING,PUTTING,WAITING}
    public INTAKE_MODE inTakeStatus;
    public INTAKE_MODE INTAKE_RUNMODE;
    public void inTake(){
        if(RUNMODE == MODE.HIGH_CHAMBER){
            if(inTakeStatus!= INTAKE_MODE.WAITING)inTakeStatus = chamberInTake();
        }
    }
    private boolean inTakeActionInitiated=false, inTakeActionFinished = false;

    public INTAKE_MODE chamberInTake(){
        if(INTAKE_RUNMODE==INTAKE_MODE.EXTENDING){
            if(!inTakeActionInitiated){

                inTakeActionInitiated = true;
            }

            if(AUTO&&inTakeActionFinished){
                INTAKE_RUNMODE = INTAKE_MODE.TAKING;
                inTakeActionInitiated = false;
            }
        } else if (INTAKE_RUNMODE==INTAKE_MODE.TAKING) {
            if(!inTakeActionInitiated){

                inTakeActionInitiated = true;
            }

            if(AUTO&&inTakeActionFinished){
                INTAKE_RUNMODE = INTAKE_MODE.SHORTENING;
                inTakeActionInitiated = false;
            }
        } else if(INTAKE_RUNMODE==INTAKE_MODE.SHORTENING){
            if(!inTakeActionInitiated){

                inTakeActionInitiated = true;
            }

            if(AUTO&&inTakeActionFinished){
                INTAKE_RUNMODE = INTAKE_MODE.PUTTING;
                inTakeActionInitiated = false;
            }
        } else if (INTAKE_RUNMODE==INTAKE_MODE.PUTTING) {
            if(!inTakeActionInitiated){

                inTakeActionInitiated = true;
            }

            if(AUTO&&inTakeActionFinished){
                INTAKE_RUNMODE = INTAKE_MODE.WAITING;
                inTakeActionInitiated = false;
            }
        }
        return INTAKE_RUNMODE;
    }
    public double armALength=0;
    public double armBLength=0;
    public double inTakeMinLength=0;
    public double inTakeTargetLength;
    public double inTakeTargetLengthAngle;
    public double inTakeTargetAngle;
    public void setInTakePosition(double inTakeLength,double inTakeAngle){
        inTakeTargetLength = inTakeLength;
        inTakeTargetAngle = inTakeAngle;
    }
    public void inTakeLengthCalculator(){
        inTakeTargetLengthAngle = Math.acos((armALength*armALength+(inTakeTargetLength+inTakeMinLength)*(inTakeTargetLength+inTakeMinLength)-armBLength*armBLength)/(2*armALength*(inTakeTargetLength+inTakeMinLength)))/Math.PI;
    }

}
