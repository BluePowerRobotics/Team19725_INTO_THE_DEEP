

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
    private Blinker control_Hub;
    private Blinker expansion_Hub_2;
    private HardwareDevice webcam_1;
    public DcMotor armMotor;
    //private DcMotor armPuller;
    private  DcMotor armSpinner;
    private Gyroscope eimu;
    private IMU imu;
    private DcMotor leftBack;
    private DcMotor leftFront;
    private DcMotor rightBack;
    private DcMotor rightFront;
    private Servo servoe0;
    private Servo servoe1;
    private Servo servoe2;
    private Servo servoe3;
    public Servo servoe4;
    private Servo servoe5;
    private Servo servos0;
    private Servo servos1;
    private Servo servos2;
    private Servo servos3;
    private Servo servos4;
    private Servo servos5;
    private HardwareMap hardwaremap;
    private Gamepad gamepad1;
    private Gamepad gamepad2;
    private DistanceSensor distanceSensor;



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

    public void inTake(){
        if(RUNMODE == MODE.HIGH_CHAMBER){
            if
        }
    }
}
