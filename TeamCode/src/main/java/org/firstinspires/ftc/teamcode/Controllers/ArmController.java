

package org.firstinspires.ftc.teamcode.Controllers;

import com.qualcomm.robotcore.hardware.HardwareMap;
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

    public enum ARM_MODE{BasicArm,FiveServoArm}
    public ARM_MODE USE_ARM_MODE = ARM_MODE.BasicArm;


    //public boolean AUTO = false;

//    public MODE RUNMODE = MODE.HIGH_CHAMBER;
//    public enum MODE {HIGH_CHAMBER,LOW_CHAMBER,HIGH_BASKET,LOW_BASKET}
    // todo: write your code here

    double t = 0;

    // ArmController

    Servo inTakeLength;

    public boolean initArmInitiated=false;
    public boolean initArmFinished = false;
    public boolean initArm(HardwareMap hardwareMapRC, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry){
        if(!initArmInitiated){
            hardwaremap = hardwareMapRC;
            if(!SharedStates.getInstance().isAUTO()) {
                this.gamepad1 = gamepad1;
                this.gamepad2 = gamepad2;
            }
            inTakeLength = hardwaremap.get(Servo.class,"");

            initArmInitiated = true;
        }
        if(1==1){
            initArmFinished = true;
        }
        return initArmFinished;
    }



    public void armController() {
        t = System.currentTimeMillis();// 获取当前时间

        if(!SharedStates.getInstance().isAUTO()){
            gamepadReceiver();
        }
        inTake();
        outPut();


    }
    boolean bHasBeenPressed = false, aHasBeenPressed = false;
    void gamepadReceiver(){
        if(gamepad1.b){
            if(!bHasBeenPressed){
                bHasBeenPressed = true;
                INTAKE_RUNMODE = INTAKE_RUNMODE.next();
            }
        }else{
            bHasBeenPressed = false;
        }
        if(gamepad1.a){
            if(!aHasBeenPressed){
                aHasBeenPressed = true;
                OUTPUT_RUNMODE = OUTPUT_RUNMODE.next();
            }
        }else{
            aHasBeenPressed = false;
        }
    }

    public OUTPUT_MODE outPutStatus;
    public void outPut(){
        if (SharedStates.getInstance().getRUNMODE() == SharedStates.MODE.HIGH_CHAMBER){
            if(outPutStatus != OUTPUT_MODE.WAITING) outPutStatus = chamberOutPut();
        }
    }
    public enum OUTPUT_MODE {
        PUTTING, UPPING,DOWNING, WAITING;
        private static final OUTPUT_MODE[] VALUES = values();

        public OUTPUT_MODE next() {
            return VALUES[(this.ordinal() + 1) % VALUES.length];
        }
    }
    private OUTPUT_MODE OUTPUT_RUNMODE;
    public OUTPUT_MODE chamberOutPut(){
        if (OUTPUT_RUNMODE==OUTPUT_MODE.UPPING){

        } else if (OUTPUT_RUNMODE == OUTPUT_MODE.PUTTING) {

        }else if(OUTPUT_RUNMODE == OUTPUT_MODE.DOWNING){

        }
        return OUTPUT_RUNMODE;
    }
    public enum INTAKE_MODE{
        EXTENDING,SHORTENING,TAKING,PUTTING,WAITING;
        private static final INTAKE_MODE[] VALUES = values();

        public INTAKE_MODE next() {
            return VALUES[(this.ordinal() + 1) % VALUES.length];
        }
    }
    public INTAKE_MODE inTakeStatus;
    private INTAKE_MODE INTAKE_RUNMODE;
    public void inTake(){
        if(SharedStates.getInstance().getRUNMODE() == SharedStates.MODE.HIGH_CHAMBER){
            if(inTakeStatus!= INTAKE_MODE.WAITING)inTakeStatus = chamberInTake();
        }
    }
    private boolean inTakeActionInitiated=false, inTakeActionFinished = false;

    public INTAKE_MODE chamberInTake(){
        if(INTAKE_RUNMODE==INTAKE_MODE.EXTENDING){
            if(!inTakeActionInitiated){

                inTakeActionInitiated = true;
            }

            if(SharedStates.getInstance().isAUTO()&&inTakeActionFinished){
                INTAKE_RUNMODE = INTAKE_MODE.TAKING;
                inTakeActionInitiated = false;
            }
        } else if (INTAKE_RUNMODE==INTAKE_MODE.TAKING) {
            if(!inTakeActionInitiated){

                inTakeActionInitiated = true;
            }

            if(SharedStates.getInstance().isAUTO()&&inTakeActionFinished){
                INTAKE_RUNMODE = INTAKE_MODE.SHORTENING;
                inTakeActionInitiated = false;
            }
        } else if(INTAKE_RUNMODE==INTAKE_MODE.SHORTENING){
            if(!inTakeActionInitiated){

                inTakeActionInitiated = true;
            }

            if(SharedStates.getInstance().isAUTO()&&inTakeActionFinished){
                INTAKE_RUNMODE = INTAKE_MODE.PUTTING;
                inTakeActionInitiated = false;
            }
        } else if (INTAKE_RUNMODE==INTAKE_MODE.PUTTING) {
            if(!inTakeActionInitiated){

                inTakeActionInitiated = true;
            }

            if(SharedStates.getInstance().isAUTO()&&inTakeActionFinished){
                INTAKE_RUNMODE = INTAKE_MODE.WAITING;
                inTakeActionInitiated = false;
            }
        }
        return INTAKE_RUNMODE;
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


}
