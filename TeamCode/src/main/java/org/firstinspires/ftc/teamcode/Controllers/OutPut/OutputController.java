package org.firstinspires.ftc.teamcode.Controllers.OutPut;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Controllers.RobotStates;
@Config

public class OutputController {
    public OutputController(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        outputPositionController = this.hardwareMap.get(Servo.class,"servoc5");
        outputClipController = this.hardwareMap.get(Servo.class,"servoc4");
        outputLengthController = this.hardwareMap.get(DcMotor.class,"OutputLengthMotor");
        outputLengthController.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    private Servo outputPositionController, outputClipController;
    private DcMotor outputLengthController;
    private HardwareMap hardwareMap;
    public static double ArmUpPos = 0.5,ArmDownPos = 0.68,ArmMid = 0.3;
    public static double outputClipLockPos = 0.25, outputClipUnlockPos = 0.55;
    public static double outputLengthControllerNumberPerCycle =147, outputLengthControllerMMPerCycle =20*Math.PI;

    public RobotStates.OUTPUT_RUNMODE outputStates = RobotStates.OUTPUT_RUNMODE.WAITING;
    public void setMode(RobotStates.OUTPUT_RUNMODE outputStates){
        this.outputStates = outputStates;
        switch (this.outputStates){
            case WAITING:
                ArmMiddle();
                setClip(false);
                break;
            case DOWNING:
                ArmDown();
                setClip(false);
                break;
            case TAKING:
                ArmDown();
                setClip(true);
                break;
            case UPPING:
                ArmUp();
                setClip(true);
                break;
            case PUTTING:
                ArmUp();
                setClip(false);
                break;
        }
    }



    public static OutputController instance;
    public static synchronized OutputController getInstance(HardwareMap hardwareMap){
        if(instance == null|| instance.hardwareMap != hardwareMap){
            instance = new OutputController(hardwareMap);
        }
        return instance;
    }
    //public static synchronized OutputController getInstance(){
    //    return instance;
    //}

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
    //向后为x轴，向上为y轴
    public static double armPositionControllerZeroPositionDegree=0;
    public OutputController ArmUp(){
        // double servoValue = (armDegree-armPositionControllerZeroPositionDegree)/255;
        // servoValue = Math.max(0.0,Math.min(1.0,servoValue));
        outputPositionController.setPosition(ArmUpPos);
        return instance;
    }

    public OutputController ArmMiddle(){
        outputPositionController.setPosition(ArmMid);
        return getInstance(hardwareMap);
    }
    public OutputController ArmDown(){
        // double servoValue = (armDegree-armPositionControllerZeroPositionDegree)/255;
        // servoValue = Math.max(0.0,Math.min(1.0,servoValue));
        outputPositionController.setPosition(ArmDownPos);
        return instance;
    }
    
    public boolean update(){
        outputLengthController.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        return !(Math.abs(outputLengthController.getCurrentPosition()- outputLengthControllerValue)<=2);
    }
    boolean[] eatIntakeCheckPointInitialized = {false,false};
    long eatIntakeCheckPointStartTime;
    boolean[] eatIntakeCheckPointPassed=eatIntakeCheckPointInitialized;
    //吃掉intake送过来的sample
    public boolean eatIntake(){
        if(eatIntakeCheckPointInitialized[0]){
            eatIntakeCheckPointStartTime =System.currentTimeMillis();

            setTargetOutputHeight(0);
            //action 1
            eatIntakeCheckPointInitialized[0] =true;
        }
        if(!eatIntakeCheckPointPassed[0]){
            if(update()&&(System.currentTimeMillis()- eatIntakeCheckPointStartTime)>1000){//完成条件判断
                eatIntakeCheckPointPassed[0]=true;
            }
        }
        if(eatIntakeCheckPointInitialized[1]&&eatIntakeCheckPointPassed[0]){
            eatIntakeCheckPointStartTime =System.currentTimeMillis();
            ArmUp();
            setTargetOutputHeight(0);
            //action 2







            eatIntakeCheckPointInitialized[1] =true;
        }
        if(!eatIntakeCheckPointPassed[1]){
            if(update()&&(System.currentTimeMillis()- eatIntakeCheckPointStartTime)>1000){//完成条件判断
                eatIntakeCheckPointPassed[1]=true;
            }
        }
        boolean allCheckPointPassed=true;
        for(int i=0;i< eatIntakeCheckPointPassed.length;i++){
            if (eatIntakeCheckPointPassed[i] == false)
                allCheckPointPassed = false;
        }
        if(allCheckPointPassed){
            for(int i=0;i< eatIntakeCheckPointPassed.length;i++){
                eatIntakeCheckPointPassed[i] = false;
            }
            eatIntakeCheckPointInitialized = eatIntakeCheckPointPassed;
        }
        return !allCheckPointPassed;
    }



    boolean[] vomitInstallerCheckPointInitialized = {false,false};
    long vomitInstallerCheckPointStartTime;
    boolean[] vomitInstallerCheckPointPassed=vomitInstallerCheckPointInitialized;
    //把吃到的sample吐给installer
    public boolean vomitInstaller(){
        if(vomitInstallerCheckPointInitialized[0]){
            vomitInstallerCheckPointStartTime =System.currentTimeMillis();
            ArmUp();
            setTargetOutputHeight(0);
            //action 1







            vomitInstallerCheckPointInitialized[0] =true;
        }
        if(!vomitInstallerCheckPointPassed[0]){
            if(update()&&(System.currentTimeMillis()- vomitInstallerCheckPointStartTime)>1000){//完成条件判断
                vomitInstallerCheckPointPassed[0]=true;
            }
        }
        if(vomitInstallerCheckPointInitialized[1]&&vomitInstallerCheckPointPassed[0]){
            vomitInstallerCheckPointStartTime =System.currentTimeMillis();
            ArmUp();
            setTargetOutputHeight(0);
            //action 2






            vomitInstallerCheckPointInitialized[1] =true;
        }
        if(!vomitInstallerCheckPointPassed[1]){
            if(update()&&(System.currentTimeMillis()- vomitInstallerCheckPointStartTime)>1000){//完成条件判断
                vomitInstallerCheckPointPassed[1]=true;
            }
        }
        boolean allCheckPointPassed=true;
        for(int i=0;i< vomitInstallerCheckPointPassed.length;i++){
            if (vomitInstallerCheckPointPassed[i] == false)
                allCheckPointPassed = false;
        }
        if(allCheckPointPassed){
            for(int i=0;i< vomitInstallerCheckPointPassed.length;i++){
                vomitInstallerCheckPointPassed[i] = false;
            }
            vomitInstallerCheckPointInitialized = vomitInstallerCheckPointPassed;
        }
        return !allCheckPointPassed;
    }




    boolean[] eatInstallerCheckPointInitialized = {false,false};
    long eatInstallerCheckPointStartTime;
    boolean[] eatInstallerCheckPointPassed=eatInstallerCheckPointInitialized;
    //吃掉installer送过来的sample
    public boolean eatInstaller(){
        if(eatInstallerCheckPointInitialized[0]){
            eatInstallerCheckPointStartTime =System.currentTimeMillis();
            ArmUp();
            setTargetOutputHeight(100);
            //action 1





            eatInstallerCheckPointInitialized[0] =true;
        }
        if(!eatInstallerCheckPointPassed[0]){
            if(update()&&(System.currentTimeMillis()- eatInstallerCheckPointStartTime)>1000){//完成条件判断
                eatInstallerCheckPointPassed[0]=true;
            }
        }
        if(eatInstallerCheckPointInitialized[1]&&eatInstallerCheckPointPassed[0]){
            eatInstallerCheckPointStartTime =System.currentTimeMillis();
            ArmUp();
            setTargetOutputHeight(30);
            //action 2





            eatInstallerCheckPointInitialized[1] =true;
        }
        if(!eatInstallerCheckPointPassed[1]){
            if(update()&&(System.currentTimeMillis()- eatInstallerCheckPointStartTime)>1000){//完成条件判断
                eatInstallerCheckPointPassed[1]=true;
            }
        }
        boolean allCheckPointPassed=true;
        for(int i=0;i< eatInstallerCheckPointPassed.length;i++){
            if (eatInstallerCheckPointPassed[i] == false)
                allCheckPointPassed = false;
        }
        if(allCheckPointPassed){
            for(int i=0;i< eatInstallerCheckPointPassed.length;i++){
                eatInstallerCheckPointPassed[i] = false;
            }
            eatInstallerCheckPointInitialized = eatInstallerCheckPointPassed;
        }
        return !allCheckPointPassed;
    }
    boolean[] throwAwaySampleCheckPointInitialized = {false,false};
    long throwAwaySampleCheckPointStartTime;
    boolean[] throwAwaySampleCheckPointPassed=throwAwaySampleCheckPointInitialized;
    //把吃掉的sample丢掉
    public boolean throwAwaySample(){
        if(throwAwaySampleCheckPointInitialized[0]){
            throwAwaySampleCheckPointStartTime =System.currentTimeMillis();
            ArmUp();
            setTargetOutputHeight(0);
            //action 1





            throwAwaySampleCheckPointInitialized[0] =true;
        }
        if(!throwAwaySampleCheckPointPassed[0]){
            if(update()&&(System.currentTimeMillis()- throwAwaySampleCheckPointStartTime)>1000){//完成条件判断
                throwAwaySampleCheckPointPassed[0]=true;
            }
        }
        if(throwAwaySampleCheckPointInitialized[1]&&throwAwaySampleCheckPointPassed[0]){
            throwAwaySampleCheckPointStartTime =System.currentTimeMillis();
            ArmUp();
            setTargetOutputHeight(0);
            //action 2





            throwAwaySampleCheckPointInitialized[1] =true;
        }
        if(!throwAwaySampleCheckPointPassed[1]){
            if(update()&&(System.currentTimeMillis()- throwAwaySampleCheckPointStartTime)>1000){//完成条件判断
                throwAwaySampleCheckPointPassed[1]=true;
            }
        }
        boolean allCheckPointPassed=true;
        for(int i=0;i< throwAwaySampleCheckPointPassed.length;i++){
            if (throwAwaySampleCheckPointPassed[i] == false)
                allCheckPointPassed = false;
        }
        if(allCheckPointPassed){
            for(int i=0;i< throwAwaySampleCheckPointPassed.length;i++){
                throwAwaySampleCheckPointPassed[i] = false;
            }
            throwAwaySampleCheckPointInitialized = throwAwaySampleCheckPointPassed;
        }
        return !allCheckPointPassed;
    }
}

