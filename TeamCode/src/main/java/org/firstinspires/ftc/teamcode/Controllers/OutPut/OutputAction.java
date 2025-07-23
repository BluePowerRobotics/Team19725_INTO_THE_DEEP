package org.firstinspires.ftc.teamcode.Controllers.OutPut;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
@Config
public class OutputAction {
    HardwareMap hardwareMap;
    public OutputAction(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
    }
    class OutputArmRunToPosition implements Action {
        double targetHeight;
        OutputController outputController = OutputController.getInstance(hardwareMap);
        public OutputArmRunToPosition(double targetHeight){
            this.targetHeight = targetHeight;

        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return outputController.setTargetOutputHeight(targetHeight).update();
        }
    }
    public Action OutputArmRunToPosition(double targetHeight){
        return new OutputArmRunToPosition(targetHeight);
    }
    class OutputEatIntake implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return OutputController.getInstance(hardwareMap).eatIntake();
        }
    }
    public Action OutputEatIntake(){
        return new OutputEatIntake();
    }
    class OutputVomitInstaller implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return OutputController.getInstance(hardwareMap).vomitInstaller();
        }
    }
    public Action OutputVomitInstaller(){
        return new OutputVomitInstaller();
    }
    class OutputEatInstaller implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return OutputController.getInstance(hardwareMap).eatInstaller();
        }
    }
    public Action OutputEatInstaller(){
        return new OutputEatInstaller();
    }
    class OutputThrowAwaySample implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return OutputController.getInstance(hardwareMap).throwAwaySample();
        }
    }
    public Action OutputThrowAwaySample(){
        return new OutputThrowAwaySample();
    }

    public static double ArmUpRequireTimeMS=2000;
    boolean upInited=false;
    long upInitTime;
    class OutputArmUp implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(!upInited) {
                OutputController.getInstance(hardwareMap).ArmUp();
                upInitTime=System.currentTimeMillis();
                return false;
//                upInited = false;
            }
            if(System.currentTimeMillis()-upInitTime>ArmUpRequireTimeMS){
                return false;
            }
            return true;
        }
    }
    public Action OutPutArmUp(){
        return new OutputArmUp();
    }
    public static double ArmMidRequireTimeMS=2000;
    boolean midInited=false;
    long midInitTime;
    class OutputArmMid implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(!midInited) {
                OutputController.getInstance(hardwareMap).ArmMiddle();
                return false;
//                midInitTime=System.currentTimeMillis();
//                midInited = false;
            }
            if(System.currentTimeMillis()-midInitTime>ArmMidRequireTimeMS){
                return false;
            }
            return true;
        }
    }
    public Action OutPutArmMid(){
        return new OutputArmMid();
    }
    public static double ArmDownRequireTimeMS=2000;
    boolean downInited=false;
    long downInitTime;
    class OutputArmDown implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(!downInited) {
                OutputController.getInstance(hardwareMap).ArmDown();
                downInitTime=System.currentTimeMillis();
                return false;
//                downInited = false;
            }
            if(System.currentTimeMillis()-downInitTime>ArmDownRequireTimeMS){
                return false;
            }
            return true;
        }
    }
    public Action OutPutArmDown(){
        return new OutputArmDown();
    }

    class setClip implements Action{
        boolean clipLock;
        public setClip(boolean clipLock){
            this.clipLock=clipLock;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            OutputController.getInstance(hardwareMap).setClip(clipLock);
            return false;
        }
    }
    public Action setClip(boolean clipLock){
        return new setClip(clipLock);
    }
}
