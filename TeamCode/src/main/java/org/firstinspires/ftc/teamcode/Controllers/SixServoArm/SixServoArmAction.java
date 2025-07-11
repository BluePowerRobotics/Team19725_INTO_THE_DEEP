package org.firstinspires.ftc.teamcode.Controllers.SixServoArm;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.VisualColor.model.ArmAction;

public class SixServoArmAction{
    HardwareMap hardwareMap;
    Telemetry telemetry;
    Gamepad gamepad2;
    ServoRadianCalculator servoRadianCalculator;
    ServoValueOutputter servoValueOutputter;
    SixServoArmController sixServoArmController;
    public SixServoArmAction(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad2){
        this.hardwareMap= hardwareMap;
        this.telemetry= telemetry;
        this.gamepad2 = gamepad2;
        sixServoArmController = SixServoArmController.getInstance(hardwareMap,telemetry);
        servoValueOutputter = sixServoArmController.servoValueOutputter;
        servoRadianCalculator = sixServoArmController.servoRadianCalculator;
    }
    public class SixServoArmInit implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet){
            sixServoArmController.setTargetPosition(-100,1,100,0.1,0.1);
            return sixServoArmController.setMode();
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
            sixServoArmController.setTargetPosition(x,y,z,ArmThreeRadian,ClipRadian);
            if(gamepad2.right_bumper){
                return false;
            }
            return sixServoArmController.setMode();
        }
    }
    public Action SixServoArmRunToPosition(double x,double y,double z,double ArmThreeRadian,double ClipRadian){
        return new SixServoArmRunToPosition(x,y,z,ArmThreeRadian,ClipRadian);
    }
    public Action SixServoArmRunToPosition(@NonNull ArmAction armAction){
        return SixServoArmRunToPosition(armAction.GoToX,armAction.GoToY,-5,Math.PI, armAction.ClipAngle);
    };
    ServoValueOutputter.ClipPosition clipPosition;
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
    public Action SixServoArmSetClip(ServoValueOutputter.ClipPosition clipPosition){
        this.clipPosition = clipPosition;
        return new SixServoArmSetClip();
    }
}
