package org.firstinspires.ftc.teamcode.Controllers.SixServoArm;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.VisualColor.model.ArmAction;

import java.util.Arrays;

public class SixServoArmEasyController {
    private static SixServoArmEasyController instance;
    public static synchronized SixServoArmEasyController getInstance(HardwareMap hardwareMap, Telemetry telemetry){
        if(instance == null){
            instance = new SixServoArmEasyController(hardwareMap,telemetry);
        }
        return instance;
    }
    public ServoRadianEasyCalculator servoRadianCalculator;
    public ServoValueEasyOutputter servoValueOutputter;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    public enum SIX_SERVO_ARM_RUNMODE{
        RUN_TO_POSITION,STOP_AND_RESET, RUN_WITHOUT_LOCATOR
    }
    private SIX_SERVO_ARM_RUNMODE SIX_SERVO_ARM_MODE;
    private long setLocationTime;

    public SixServoArmEasyController(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        servoRadianCalculator = ServoRadianEasyCalculator.getInstance();
        servoValueOutputter = ServoValueEasyOutputter.getInstance(hardwareMap, telemetry,servoRadianCalculator);
    }
    public void initArm(){
        SixServoArmEasyController.getInstance(hardwareMap,telemetry);
        setTargetPosition(resetX, resetY, 0 * Math.PI, 0.5 * Math.PI).update();
    }

    final double resetX = 100,resetY = 0,resetZ = 10;
    double targetX = resetX,targetY = resetY,targetZ = resetZ;
    double nowX=resetX,nowY=resetY,nowZ=resetZ;
    double recentX = resetX,recentY = resetY,recentZ = resetZ;
    double Distance;
    double servoMoveTime = 0.0;//单位秒
    double[] servoMoveNeedTime=new double[5];
    private double[] servoSpeed={0.36,0.24,0.24,0.24,0.24};//sec per 60 degree
    //均为向上正方向，向右正方向，向前正方向
    //D为顺时针
    double targetClipRadian = 0.5 * Math.PI;

    public SixServoArmEasyController setTargetPosition(@NonNull ArmAction armAction){
        setTargetPosition(armAction.GoToX,armAction.GoToY,Math.PI, armAction.ClipAngle);
        return instance;
    }
    public SixServoArmEasyController setTargetPosition(double X,double Y,double alpha4,double clipRadian) {
        targetX = X;
        targetY = Y;
        targetClipRadian = clipRadian;
        setLocationTime = System.currentTimeMillis();
        Distance = Math.sqrt((X-nowX)*(X-nowX)+(Y-nowY)*(Y-nowY));
        double[] targetPosition = servoRadianCalculator.calculate(targetX, targetY, alpha4);
        double[] nowPosition = servoRadianCalculator.calculate(nowX, nowY, targetClipRadian);
        servoMoveNeedTime = new double[5];
        for(int i = 0; i <= 3; i++) {
            servoMoveNeedTime[i] = Math.toDegrees(targetPosition[i] - nowPosition[i])*(servoSpeed[i]/60.0);
        }
        this.servoMoveTime = Arrays.stream(servoMoveNeedTime).max().getAsDouble();
        recentX = nowX;
        recentY = nowY;
        return instance;
    }
    public boolean update(){
        boolean states;
        if (System.currentTimeMillis() - setLocationTime >= servoMoveTime * 1000) {
            nowX = targetX;
            nowY = targetY;
            states = false;
        } else {
            double ratio = (System.currentTimeMillis() - setLocationTime) / (servoMoveTime * 1000);
            nowX = recentX + (targetX - recentX) * ratio;
            nowY = recentY + (targetY - recentY) * ratio;
            states = true;
        }
        servoValueOutputter.setRadians(servoRadianCalculator.calculate(targetX, targetY, targetClipRadian), targetClipRadian, true);

        return states;//ifRequireRepeat
    }
}
