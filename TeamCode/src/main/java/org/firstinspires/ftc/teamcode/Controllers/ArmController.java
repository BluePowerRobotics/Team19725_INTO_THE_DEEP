

package org.firstinspires.ftc.teamcode.Controllers;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmController {

    private HardwareMap hardwaremap;
    private Gamepad gamepad1;
    private Gamepad gamepad2;



    public enum ARM_MODE{BasicArm,FiveServoArm}
    public ARM_MODE USE_ARM_MODE = ARM_MODE.BasicArm;



    // todo: write your code here

    double t = 0;

    // ArmController



    public boolean initArmInitiated=false;
    public boolean initArmFinished = false;
    public boolean initArm(HardwareMap hardwareMapRC, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry){
        if(!initArmInitiated){
            hardwaremap = hardwareMapRC;
            if(!SharedStates.getInstance().isAUTO()) {
                this.gamepad1 = gamepad1;
                this.gamepad2 = gamepad2;
            }


            initArmInitiated = true;
        }
        if(1==1){
            initArmFinished = true;
        }
        return !initArmFinished;
    }
    public class


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
                INTAKE_MODE = INTAKE_MODE.next();
            }
        }else{
            bHasBeenPressed = false;
        }
        if(gamepad1.a){
            if(!aHasBeenPressed){
                aHasBeenPressed = true;
                PUTPUT_MODE = PUTPUT_MODE.next();
            }
        }else{
            aHasBeenPressed = false;
        }
    }

    public OUTPUT_RUNMODE outPutStatus;
    public void outPut(){
        if (SharedStates.getInstance().getMODE() == SharedStates.RUNMODE.HIGH_CHAMBER){
            if(outPutStatus != OUTPUT_RUNMODE.WAITING) outPutStatus = chamberOutPut();
        }
    }
    public enum OUTPUT_RUNMODE {
        PUTTING, UPPING,DOWNING, WAITING;
        private static final OUTPUT_RUNMODE[] VALUES = values();

        public OUTPUT_RUNMODE next() {
            return VALUES[(this.ordinal() + 1) % VALUES.length];
        }
    }
    private OUTPUT_RUNMODE PUTPUT_MODE;
    public OUTPUT_RUNMODE chamberOutPut(){
        if (PUTPUT_MODE == OUTPUT_RUNMODE.UPPING){

        } else if (PUTPUT_MODE == OUTPUT_RUNMODE.PUTTING) {

        }else if(PUTPUT_MODE == OUTPUT_RUNMODE.DOWNING){

        }
        return PUTPUT_MODE;
    }
    public enum INTAKE_RUNMODE {
        EXTENDING,SHORTENING,TAKING,PUTTING,WAITING;
        private static final INTAKE_RUNMODE[] VALUES = values();

        public INTAKE_RUNMODE next() {
            return VALUES[(this.ordinal() + 1) % VALUES.length];
        }
    }
    public INTAKE_RUNMODE inTakeStatus;
    private INTAKE_RUNMODE INTAKE_MODE;
    public void inTake(){
        if(SharedStates.getInstance().getMODE() == SharedStates.RUNMODE.HIGH_CHAMBER){
            if(inTakeStatus!= INTAKE_RUNMODE.WAITING)inTakeStatus = chamberInTake();
        }
    }
    private boolean inTakeActionInitiated=false, inTakeActionFinished = false;

    public INTAKE_RUNMODE chamberInTake(){
        if(INTAKE_MODE == INTAKE_RUNMODE.EXTENDING){
            if(!inTakeActionInitiated){

                inTakeActionInitiated = true;
            }

            if(SharedStates.getInstance().isAUTO()&&inTakeActionFinished){
                INTAKE_MODE = INTAKE_RUNMODE.TAKING;
                inTakeActionInitiated = false;
            }
        } else if (INTAKE_MODE == INTAKE_RUNMODE.TAKING) {
            if(!inTakeActionInitiated){

                inTakeActionInitiated = true;
            }

            if(SharedStates.getInstance().isAUTO()&&inTakeActionFinished){
                INTAKE_MODE = INTAKE_RUNMODE.SHORTENING;
                inTakeActionInitiated = false;
            }
        } else if(INTAKE_MODE == INTAKE_RUNMODE.SHORTENING){
            if(!inTakeActionInitiated){

                inTakeActionInitiated = true;
            }

            if(SharedStates.getInstance().isAUTO()&&inTakeActionFinished){
                INTAKE_MODE = INTAKE_RUNMODE.PUTTING;
                inTakeActionInitiated = false;
            }
        } else if (INTAKE_MODE == INTAKE_RUNMODE.PUTTING) {
            if(!inTakeActionInitiated){

                inTakeActionInitiated = true;
            }

            if(SharedStates.getInstance().isAUTO()&&inTakeActionFinished){
                INTAKE_MODE = INTAKE_RUNMODE.WAITING;
                inTakeActionInitiated = false;
            }
        }
        return INTAKE_MODE;
    }



}
