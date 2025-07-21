package org.firstinspires.ftc.teamcode.Controllers.IntakeLength;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.nio.channels.DatagramChannel;

public class MotorLineIntakeLengthController{
    HardwareMap hardwareMap;
    static MotorLineIntakeLengthController instance;
    DcMotor Motor;
    private final double intakeLengthControllerNumberPerCycle = 252, intakeLengthControllerMMPerCycle =30*Math.PI;
    public MotorLineIntakeLengthController(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        Motor=hardwareMap.get(DcMotor.class,"IntakeLengthMotor");
        Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public static synchronized MotorLineIntakeLengthController getInstance(HardwareMap hardwareMap){
        if(instance==null|| instance.hardwareMap != hardwareMap){
            instance = new MotorLineIntakeLengthController(hardwareMap);
        }
        return instance;
    }
    public static synchronized MotorLineIntakeLengthController getInstance(){
        return instance;
    }
    double IntakeLengthTargetPosition;
    //@Override
    public MotorLineIntakeLengthController setIntakeTargetPosition(double inTakeLength) {
        IntakeLengthTargetPosition = inTakeLength;
        Motor.setTargetPosition((int)(inTakeLength*intakeLengthControllerNumberPerCycle/intakeLengthControllerMMPerCycle));
        return instance;
    }

    //@Override
    public MotorLineIntakeLengthController update() {
        Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        return instance;
    }

    //@Override
    public double getIntakeLengthCurrentPosition() {
        return Motor.getCurrentPosition()*intakeLengthControllerMMPerCycle/intakeLengthControllerNumberPerCycle;
    }

    //@Override
    public double getIntakeLengthTargetPosition() {
        return IntakeLengthTargetPosition;
    }
    public long testZeroPositionTime=0;
    public boolean testZeroPositionInited=false;
    public int testZeroPositionLastPos=0;
    public boolean testZeroPosition(){
        if(testZeroPositionInited) {
            testZeroPositionTime = System.currentTimeMillis();
            testZeroPositionInited=true;
        }
        Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor.setPower(-0.5);
        if((Motor.getCurrentPosition()==testZeroPositionLastPos)&&(System.currentTimeMillis()-testZeroPositionTime>200)){
            testZeroPositionInited= false;
            return false;
        }else{
            return true;
        }
    }
    public void SingleMotorControl(double Power){
        Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor.setPower(Power);
    }
    public MotorLineIntakeLengthController setMotorEncoder(int number){
        Motor.setTargetPosition(number);
    }
    
}
