

package org.firstinspires.ftc.teamcode.Controllers;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmController {

    private HardwareMap hardwaremap;
    private Telemetry telemetry;
    private Gamepad gamepad1;
    private Gamepad gamepad2;



    public enum ARM_MODE{BasicArm,SixServoArm}
    public ARM_MODE USE_ARM_MODE = ARM_MODE.BasicArm;



    // todo: write your code here

    double t = 0;

    // ArmController

    public ArmController(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwaremap = hardwareMap;
        this.telemetry = telemetry;
    }

    public boolean initArmInitiated=false;
    public boolean initArmFinished = false;
    public boolean initArm(Gamepad gamepad1, Gamepad gamepad2){
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        return initArm();
    }
    public boolean initArm(){
        if(!initArmInitiated){


            initArmInitiated = true;
        }
        if(1==1){
            initArmFinished = true;
        }
        return !initArmFinished;
    }

    //public class


    public void armController() {
        t = System.currentTimeMillis();// 获取当前时间

        if(!RobotStates.getInstance().isAUTO()){
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
                RobotStates.getInstance().INTAKE_MODE = RobotStates.getInstance().INTAKE_MODE.next();
            }
        }else{
            bHasBeenPressed = false;
        }
        if(gamepad1.a){
            if(!aHasBeenPressed){
                aHasBeenPressed = true;
                RobotStates.getInstance().OUTPUT_MODE = RobotStates.getInstance().OUTPUT_MODE.next();
            }
        }else{
            aHasBeenPressed = false;
        }
    }

    public RobotStates.OUTPUT_RUNMODE outPutStatus;
    public void outPut(){
        if (RobotStates.getInstance().getMODE() == RobotStates.RUNMODE.HIGH_CHAMBER){
            if(outPutStatus != RobotStates.OUTPUT_RUNMODE.WAITING) outPutStatus = chamberOutPut();
        }
    }

    public RobotStates.OUTPUT_RUNMODE chamberOutPut(){
        if (RobotStates.getInstance().OUTPUT_MODE == RobotStates.OUTPUT_RUNMODE.UPPING){

        } else if (RobotStates.getInstance().OUTPUT_MODE == RobotStates.OUTPUT_RUNMODE.PUTTING) {

        }else if(RobotStates.getInstance().OUTPUT_MODE == RobotStates.OUTPUT_RUNMODE.DOWNING){

        }
        return RobotStates.getInstance().OUTPUT_MODE;
    }

    public RobotStates.INTAKE_RUNMODE inTakeStatus;

    public void inTake(){
        if(RobotStates.getInstance().getMODE() == RobotStates.RUNMODE.HIGH_CHAMBER){
            if(inTakeStatus!= RobotStates.INTAKE_RUNMODE.WAITING)inTakeStatus = chamberInTake();
        }
    }
    private boolean inTakeActionInitiated=false, inTakeActionFinished = false;

    public RobotStates.INTAKE_RUNMODE chamberInTake(){
//        if(INTAKE_MODE == INTAKE_RUNMODE.EXTENDING){
//            if(!inTakeActionInitiated){
//
//                inTakeActionInitiated = true;
//            }
//
//            if(SharedStates.getInstance().isAUTO()&&inTakeActionFinished){
//                INTAKE_MODE = INTAKE_RUNMODE.TAKING;
//                inTakeActionInitiated = false;
//            }
//        } else if (INTAKE_MODE == INTAKE_RUNMODE.TAKING) {
//            if(!inTakeActionInitiated){
//
//                inTakeActionInitiated = true;
//            }
//
//            if(SharedStates.getInstance().isAUTO()&&inTakeActionFinished){
//                INTAKE_MODE = INTAKE_RUNMODE.SHORTENING;
//                inTakeActionInitiated = false;
//            }
//        } else if(INTAKE_MODE == INTAKE_RUNMODE.SHORTENING){
//            if(!inTakeActionInitiated){
//
//                inTakeActionInitiated = true;
//            }
//
//            if(SharedStates.getInstance().isAUTO()&&inTakeActionFinished){
//                INTAKE_MODE = INTAKE_RUNMODE.PUTTING;
//                inTakeActionInitiated = false;
//            }
//        } else if (INTAKE_MODE == INTAKE_RUNMODE.PUTTING) {
//            if(!inTakeActionInitiated){
//
//                inTakeActionInitiated = true;
//            }
//
//            if(SharedStates.getInstance().isAUTO()&&inTakeActionFinished){
//                INTAKE_MODE = INTAKE_RUNMODE.WAITING;
//                inTakeActionInitiated = false;
//            }
//        }
        return RobotStates.getInstance().INTAKE_MODE;
    }



}
