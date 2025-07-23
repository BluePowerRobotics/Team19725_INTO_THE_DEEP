package org.firstinspires.ftc.teamcode.Controllers.SixServoArm;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Vision.model.ArmAction;
@Config
public class SixServoArmEasyAction {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    Gamepad gamepad2;
    public ServoRadianEasyCalculator servoRadianCalculator;
    public ServoValueEasyOutputter servoValueOutputter;
    public SixServoArmEasyController sixServoArmController;
    public SixServoArmEasyAction(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad2){
        this.hardwareMap= hardwareMap;
        this.telemetry= telemetry;
        this.gamepad2 = gamepad2;
        sixServoArmController = SixServoArmEasyController.getInstance(hardwareMap,telemetry);
        servoValueOutputter = sixServoArmController.servoValueOutputter;
        servoRadianCalculator = sixServoArmController.servoRadianCalculator;
    }
    public class SixServoArmInit implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet){
            sixServoArmController.initArm();
            return sixServoArmController.checkIfFinished();
        }
    }
    public Action SixServoArmInit(){
        return new SixServoArmInit();
    }

    public class SixServoArmRunToPosition implements Action{
        double x,y,z,ArmThreeRadian,ClipRadian;
        public SixServoArmRunToPosition(double x,double y,double z,double ArmThreeRadian,double ClipRadian){
            this.x=x;
            this.y=y;
            this.z=z;
            this.ArmThreeRadian = ArmThreeRadian;
            this.ClipRadian = ClipRadian;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet){
            sixServoArmController.setTargetPosition(x,y,ArmThreeRadian,ClipRadian);
            if(gamepad2.right_bumper){
                return false;
            }
            return sixServoArmController.update();
        }
    }
    public Action SixServoArmRunToPosition(double x,double y,double z,double ArmThreeRadian,double ClipRadian){
        return new SixServoArmRunToPosition(x,y,z,ArmThreeRadian,ClipRadian);
    }
    public Action SixServoArmRunToPosition(@NonNull ArmAction armAction){
        return SixServoArmRunToPosition(armAction.GoToX,armAction.GoToY,-5,Math.PI, armAction.ClipAngle);
    };
    ServoValueEasyOutputter.ClipPosition clipPosition;
    public class SixServoArmSetClip implements Action{
        boolean initiated = false;
        long setClipTime = 0;
        double ClipLockSpendSec = 0.5;
        @Override
        public boolean run(@NonNull TelemetryPacket packet){
            if(!initiated){
                servoValueOutputter.setClip(clipPosition);
                setClipTime = System.currentTimeMillis();
                initiated = true;
            }
            return System.currentTimeMillis()-setClipTime<=1000*ClipLockSpendSec;
        }
    }
    public Action SixServoArmSetClip(ServoValueEasyOutputter.ClipPosition clipPosition){
        this.clipPosition = clipPosition;
        return new SixServoArmSetClip();
    }
    long SixServoArmVomitOutputStartTime;
    class SixServoArmVomitOutput implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            SixServoArmVomitOutputStartTime=System.currentTimeMillis();
            servoValueOutputter.setRadians(new double[]{0.5 * Math.PI, Math.PI, Math.PI, 0.5 * Math.PI},0,false);
            if(System.currentTimeMillis()-SixServoArmVomitOutputStartTime>1500)
                return false;
            return true;
        }
    }
    public Action SixServoArmVomitOutput (){
        return new SixServoArmVomitOutput();
    }
    public class SixServoArmGiveTheSample implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){

            boolean states = sixServoArmController.giveTheSample();
            if(!states){
                for(int i=0;i<=1;i++){
                    sixServoArmController.giveTheSampleCheckPointInited[i]=false;
                    sixServoArmController.giveTheSampleCheckPointPassed[i]=false;
                }
            }
            return states;
        }
        
    }
    public Action SixServoArmGiveTheSample(){
        return new SixServoArmGiveTheSample();
    }
    public static double dropTheSampleRequireTimeMS=1000;
    public class SixServoArmDropTheSample implements Action{
        public boolean dtsinited=false;
        public long startTime;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            if(!dtsinited) {
                sixServoArmController.dropTheSample();
                dtsinited=true;
                startTime=System.currentTimeMillis();
            }
            if(System.currentTimeMillis()-startTime>dropTheSampleRequireTimeMS){
                return false;
            }
            return true;
        }

    }
    public Action SixServoArmDropTheSample(){
        return new SixServoArmDropTheSample();
    }

    public static double testIfTheSampleIsEatenRequireTimeMS=500;
    public class SixServoArmtestIfTheSampleIsEaten implements Action{
        public boolean titsieInited =false;
        public long startTime;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            if(!titsieInited) {
                sixServoArmController.testIfTheSampleIsEaten();
                titsieInited =true;
                startTime=System.currentTimeMillis();
            }
            if(System.currentTimeMillis()-startTime>dropTheSampleRequireTimeMS){
                return false;
            }
            return true;
        }

    }
    public Action SixServoArmtestIfTheSampleIsEaten(){
        return new SixServoArmtestIfTheSampleIsEaten();
    }


    public static double prepareForTakingRequireTimeMS=500;
    public class SixServoArmprepareForTaking implements Action{
        public boolean titsieInited =false;
        public long startTime;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            if(!titsieInited) {
                sixServoArmController.prepareForTaking();
                titsieInited =true;
                startTime=System.currentTimeMillis();
            }
            if(System.currentTimeMillis()-startTime>dropTheSampleRequireTimeMS){
                return false;
            }
            return true;
        }

    }
    public Action SixServoArmprepareForTaking(){
        return new SixServoArmprepareForTaking();
    }

}
