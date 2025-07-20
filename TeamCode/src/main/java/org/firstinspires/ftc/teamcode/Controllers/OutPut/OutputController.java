package org.firstinspires.ftc.teamcode.Controllers.OutPut;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Controllers.RobotStates;

public class OutputController {
    public OutputController(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        outputPositionController = this.hardwareMap.get(Servo.class,"");
        outputClipController = this.hardwareMap.get(Servo.class,"");
        outputLengthController = this.hardwareMap.get(DcMotor.class,"");
    }
    private Servo outputPositionController, outputClipController;
    private DcMotor outputLengthController;
    private HardwareMap hardwareMap;
    private final double outputClipLockPos = 0.28, outputClipUnlockPos = 1;
    private final double outputLengthControllerNumberPerCycle =147, outputLengthControllerMMPerCycle =20*Math.PI;

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
    public OutputController setArmDegree(double armDegree){
        double servoValue = (armDegree-armPositionControllerZeroPositionDegree)/255;
        servoValue = Math.max(0.0,Math.min(1.0,servoValue));
        outputPositionController.setPosition(servoValue);
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
            setArmDegree(0);
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
            setArmDegree(0);
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
            setArmDegree(0);
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
            setArmDegree(0);
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
            setArmDegree(0);
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
            setArmDegree(-45);
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
            setArmDegree(0);
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
            setArmDegree(0);
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

