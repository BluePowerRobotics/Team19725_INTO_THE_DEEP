package org.firstinspires.ftc.teamcode.Controllers;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.VisualColor.model.ArmAction;

import java.util.Arrays;

public class SixServoArmController {
    private static SixServoArmController instance;
    public static synchronized SixServoArmController getInstance(HardwareMap hardwareMap,Telemetry telemetry){
        if(instance == null){
            instance = new SixServoArmController(hardwareMap,telemetry);
        }
        return instance;
    }
    public ServoRadianCalculator servoRadianCalculator;
    public ServoValueOutputter servoValueOutputter;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    public enum SIX_SERVO_ARM_RUNMODE{
        RUN_TO_POSITION,STOP_AND_RESET, RUN_WITHOUT_LOCATOR
    }
    private SIX_SERVO_ARM_RUNMODE SIX_SERVO_ARM_MODE;
    private long setLocationTime;

    public SixServoArmController(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        servoRadianCalculator = ServoRadianCalculator.getInstance();
        servoValueOutputter = ServoValueOutputter.getInstance(hardwareMap, telemetry, servoRadianCalculator);
    }
    public void initArm(){
        setMode(SIX_SERVO_ARM_RUNMODE.RUN_WITHOUT_LOCATOR);
        setTargetPosition(resetX, resetY, resetZ, 0 * Math.PI, 0.5 * Math.PI);
    }

    final double resetX = 100,resetY = 0,resetZ = 10;
    double targetX = resetX,targetY = resetY,targetZ = resetZ;
    double nowX=resetX,nowY=resetY,nowZ=resetZ;
    double recentX = resetX,recentY = resetY,recentZ = resetZ;
    double Distance;
    double servoMoveTime = 0.0;//单位秒
    private double[] servoSpeed={0.36,0.24,0.24,0.24,0.24};//sec per 60 degree
    //均为向上正方向，向右正方向，向前正方向
    //D为顺时针
    double targetClipRadian = 0.5 * Math.PI;

    public void setTargetPosition(@NonNull ArmAction armAction){
        setTargetPosition(armAction.GoToX,armAction.GoToY,-5,Math.PI, armAction.ClipAngle);
    };
    public void setTargetPosition(double X,double Y,double Z,double alpha4,double clipRadian) {
        switch (SIX_SERVO_ARM_MODE) {
            case RUN_TO_POSITION:
                targetX = X;
                targetY = Y;
                targetZ = Z;
                targetClipRadian = clipRadian;
                setLocationTime = System.currentTimeMillis();
                Distance = Math.sqrt((X-nowX)*(X-nowX)+(Y-nowY)*(Y-nowY)+(Z-nowZ)*(Z-nowZ));
                double[] targetPosition = servoRadianCalculator.calculate(targetX, targetY, targetZ, alpha4);
                double[] nowPosition = servoRadianCalculator.calculate(nowX, nowY, nowZ, targetClipRadian);
                double[] servoMoveTime = new double[4];
                for(int i = 0; i <= 3; i++) {
                    servoMoveTime[i] = Math.toDegrees(targetPosition[i] - nowPosition[i])*(servoSpeed[i]/60.0);
                }
                double servoMoveTimeMax = Arrays.stream(servoMoveTime).max().getAsDouble();
                this.servoMoveTime = servoMoveTimeMax;
                recentX = nowX;
                recentY = nowY;
                recentZ = nowZ;
                break;
            case RUN_WITHOUT_LOCATOR:
                servoValueOutputter.setRadians(servoRadianCalculator.calculate(targetX, targetY, targetZ, targetClipRadian), targetClipRadian, true);
                break;
        }

    }

    public boolean setMode(){
        boolean states;
        this.SIX_SERVO_ARM_MODE = SIX_SERVO_ARM_RUNMODE.RUN_TO_POSITION;
        telemetry.addData("SixServoArmController", "RUN_TO_POSITION");
        if (System.currentTimeMillis() - setLocationTime >= servoMoveTime * 1000) {
            nowX = targetX;
            nowY = targetY;
            nowZ = targetZ;
            states = false;
        } else {
            double ratio = (System.currentTimeMillis() - setLocationTime) / (servoMoveTime * 1000);
            nowX = recentX + (targetX - recentX) * ratio;
            nowY = recentY + (targetY - recentY) * ratio;
            nowZ = recentZ + (targetZ - recentZ) * ratio;
            states = true;
        }
        servoValueOutputter.setRadians(servoRadianCalculator.calculate(nowX, nowY, nowZ, targetClipRadian), targetClipRadian, true);
        return states;
    }

    public void setMode(SIX_SERVO_ARM_RUNMODE mode) {
        this.SIX_SERVO_ARM_MODE = mode;
        switch(SIX_SERVO_ARM_MODE){
            case RUN_TO_POSITION:
                telemetry.addData("SixServoArmController","RUN_TO_POSITION");
                if (System.currentTimeMillis() - setLocationTime >= servoMoveTime * 1000) {
                    nowX = targetX;
                    nowY = targetY;
                    nowZ = targetZ;
                } else {
                    double ratio = (System.currentTimeMillis() - setLocationTime) / (servoMoveTime * 1000);
                    nowX = recentX + (targetX - recentX) * ratio;
                    nowY = recentY + (targetY - recentY) * ratio;
                    nowZ = recentZ + (targetZ - recentZ) * ratio;
                }
                servoValueOutputter.setRadians(servoRadianCalculator.calculate(nowX, nowY, nowZ, targetClipRadian), targetClipRadian, true);
                break;
            case STOP_AND_RESET:
                telemetry.addData("SixServoArmController","STOP_AND_RESET");
                servoValueOutputter.setRadians(servoRadianCalculator.calculate(resetX, resetY, resetZ, 0.5 * Math.PI),0.5*Math.PI, true);
                break;
            case RUN_WITHOUT_LOCATOR:
                telemetry.addData("SixServoArmController","RUN_WITHOUT_PREDICTOR");
                break;
        }
    }
}
