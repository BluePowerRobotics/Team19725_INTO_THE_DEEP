package org.firstinspires.ftc.teamcode.Controllers.SixServoArm;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Vision.model.ArmAction;

import java.util.Arrays;

public class SixServoArmEasyController {
    private static SixServoArmEasyController instance;
    public static synchronized SixServoArmEasyController getInstance(HardwareMap hardwareMap, Telemetry telemetry){
        if(instance == null|| instance.hardwareMap != hardwareMap || instance.telemetry != telemetry){
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

    final double resetX = 0,resetY = 100,resetZ = 10;
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
        return getInstance(hardwareMap,telemetry);
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
        return getInstance(hardwareMap,telemetry);
    }


    public boolean update(){
        servoValueOutputter.setRadians(servoRadianCalculator.calculate(targetX, targetY, targetClipRadian), targetClipRadian, true);
        boolean states=false;
        boolean[] servoStates = new boolean[5];
        for(int i=0;i<=4;i++){
            if(servoValueOutputter.servoSetDegree[i]-servoValueOutputter.servoNowDegree[i]>0){
                servoValueOutputter.servoNowDegree[i] += (60/servoSpeed[i])*((System.currentTimeMillis()-servoValueOutputter.servoNowDegreeTime[i])/1000.0);
                if(servoValueOutputter.servoNowDegree[i]>servoValueOutputter.servoSetDegree[i]){
                    servoValueOutputter.servoNowDegree[i] = servoValueOutputter.servoSetDegree[i];
                    servoStates[i]= false;
                }
                servoValueOutputter.servoNowDegreeTime[i] = System.currentTimeMillis();
            }else{
                servoValueOutputter.servoNowDegree[i] -= (60/servoSpeed[i])*((System.currentTimeMillis()-servoValueOutputter.servoNowDegreeTime[i])/1000.0);
                if(servoValueOutputter.servoNowDegree[i]<servoValueOutputter.servoSetDegree[i]){
                    servoValueOutputter.servoNowDegree[i] = servoValueOutputter.servoSetDegree[i];
                    servoStates[i]= false;
                }
                servoValueOutputter.servoNowDegreeTime[i] = System.currentTimeMillis();
            }
        }
        //保证所有servoStates为false时states才为false
        for(int i=0;i<=4;i++){
            if(servoStates[i]){
                states = true;
            }else{
                states = states || false;
            }
        }
        return states;
    }
    public boolean[] giveTheSampleCheckPointInited={false,false,false};
    public boolean[] giveTheSampleCheckPointPassed={false,false,false};
    public long[] giveTheSampleStartTime = {0,0};
    public static int[] giveTheSampleRequireTimeMS={1500,500};
    public static double InstallerRequireErrorX=-30;
    public boolean giveTheSample(){
        if(!giveTheSampleCheckPointInited[0]){
            servoValueEasyOutputter.giveTheSample(servoValueOutputter.InstallerLocationX,servoValueOutputter.InstallerLocationY,servoValueOutputter.InstallerLocationZ);
            giveTheSampleStartTime[0]=System.currentTimeMillis();
        }
        if(!giveTheSampleCheckPointPassed[0]){
            if(System.currentTimeMillis()-giveTheSampleStartTime[0]>giveTheSampleRequireTimeMS[0]){
                giveTheSampleCheckPointPassed[0]=true;
            }
            return true;
        }
        if(!giveTheSampleCheckPointInited[1]){
            servoValueEasyOutputter.giveTheSample(servoValueOutputter.InstallerLocationX-InstallerRequireErrorX,servoValueOutputter.InstallerLocationY,servoValueOutputter.InstallerLocationZ);
            giveTheSampleStartTime[1]=System.currentTimeMillis();
        }
        if(!giveTheSampleCheckPointPassed[1]){
            if(System.currentTimeMillis()-giveTheSampleStartTime[1]>giveTheSampleRequireTimeMS[1]){
                giveTheSampleCheckPointPassed[1]=true;
            }
            return true;
        }
        for(int i=0;i<=1;i++){
            giveTheSampleCheckPointInited[i]=false;
            giveTheSampleCheckPointPassed[i]=false;
        }
        return false;
    }







    public boolean update_mini(){
        boolean states;
        if(servoMoveTime <= 0) {
            nowX = targetX;
            nowY = targetY;
            states = false;

        } else if (System.currentTimeMillis() - setLocationTime >= servoMoveTime * 1000) {
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
