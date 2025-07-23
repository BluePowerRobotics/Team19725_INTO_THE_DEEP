package org.firstinspires.ftc.teamcode.Controllers.SixServoArm;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Controllers.ArmColorSensor;
import org.firstinspires.ftc.teamcode.Vision.model.ArmAction;

import java.util.Arrays;
@Config
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
        //todo:fixthis
        //setTargetPosition(resetX, resetY, 0 * Math.PI, 0.5 * Math.PI).update();
        servoValueOutputter.DegreeServoControl(0,90);
        servoValueOutputter.DegreeServoControl(1,140);
        servoValueOutputter.DegreeServoControl(2,80);
        servoValueOutputter.DegreeServoControl(3,50);
        servoValueOutputter.DegreeServoControl(4,0);
        servoValueOutputter.setClip(ServoValueEasyOutputter.ClipPosition.LOCKED);
    }
    long PFTStartTime;
    public boolean PFTinited=false;
    public void inToTheDeep(){

            servoValueOutputter.DegreeServoControl(0,90);
            servoValueOutputter.DegreeServoControl(1,45);
            servoValueOutputter.DegreeServoControl(2,135);
            servoValueOutputter.DegreeServoControl(3,90);
            servoValueOutputter.DegreeServoControl(4,0);
            servoValueOutputter.setClip(ServoValueEasyOutputter.ClipPosition.UNLOCKED);


    }
    public void scanTheSample(){
        servoValueOutputter.DegreeServoControl(0,90);
        servoValueOutputter.DegreeServoControl(1,45);
        servoValueOutputter.DegreeServoControl(2,170);
        servoValueOutputter.DegreeServoControl(3,45);
        servoValueOutputter.DegreeServoControl(4,0);
        servoValueOutputter.setClip(ServoValueEasyOutputter.ClipPosition.UNLOCKED);
    }

    //todo finish the following void
    public boolean testIfTheSampleIsEaten(){
        servoValueOutputter.DegreeServoControl(0, 90);
        servoValueOutputter.DegreeServoControl(1, 90);
        servoValueOutputter.DegreeServoControl(2, 90);
        servoValueOutputter.DegreeServoControl(3, 180);
        servoValueOutputter.SingleServoControl(4, 1);
        servoValueOutputter.setClip(ServoValueEasyOutputter.ClipPosition.HALF_LOCKED);
        return new ArmColorSensor(hardwareMap).ifheld();
    }
    public void eatHuman(){
        servoValueOutputter.DegreeServoControl(0,90);
        servoValueOutputter.DegreeServoControl(1,45);
        servoValueOutputter.DegreeServoControl(2,150);
        servoValueOutputter.DegreeServoControl(3,180);
        servoValueOutputter.DegreeServoControl(4,180);
        servoValueOutputter.setClip(ServoValueEasyOutputter.ClipPosition.UNLOCKED);
    }
    public void dropTheSample(){
        servoValueOutputter.DegreeServoControl(0, 90);
        servoValueOutputter.DegreeServoControl(1, 90);
        servoValueOutputter.DegreeServoControl(2, 90);
        servoValueOutputter.DegreeServoControl(3, 90);
        servoValueOutputter.SingleServoControl(4, 1);
        servoValueOutputter.setClip(ServoValueEasyOutputter.ClipPosition.HALF_LOCKED);
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


    public static double deltax = 5;
    public static double deltay = -260;
    public SixServoArmEasyController setTargetPosition(@NonNull ArmAction armAction){
        telemetry.addData("ArmActionX", armAction.GoToX - deltax);
        telemetry.addData("ArmActionY",-armAction.GoToY - deltay);
        telemetry.addData("Angle", armAction.ClipAngle);
        setTargetPosition(armAction.GoToX - deltax,-armAction.GoToY - deltay,Math.PI, armAction.ClipAngle);
        return getInstance(hardwareMap,telemetry);
    }
    public SixServoArmEasyController setTargetPosition(double X,double Y,double alpha4,double clipRadian) {
        targetX = X;
        targetY = Y;
        targetClipRadian = -clipRadian;
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
    public boolean checkIfFinished(){
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
    public static double[] giveTheSample={70,90,90,260};
    public boolean giveTheSample(){
        servoValueOutputter.DegreeServoControl(0,giveTheSample[0]);
        servoValueOutputter.DegreeServoControl(1,giveTheSample[1]);
        servoValueOutputter.DegreeServoControl(2,giveTheSample[2]);
        servoValueOutputter.DegreeServoControl(3,giveTheSample[3]);
        servoValueOutputter.DegreeServoControl(4,0);

        return false;
//        if(!giveTheSampleCheckPointInited[0]){
//            servoValueOutputter.giveTheSample(servoValueOutputter.InstallerLocationX,servoValueOutputter.InstallerLocationY,servoValueOutputter.InstallerLocationZ);
//            giveTheSampleStartTime[0]=System.currentTimeMillis();
//            giveTheSampleCheckPointInited[0]=true;
//        }
//        if(!giveTheSampleCheckPointPassed[0]){
//            if(System.currentTimeMillis()-giveTheSampleStartTime[0]>giveTheSampleRequireTimeMS[0]){
//                giveTheSampleCheckPointPassed[0]=true;
//            }
//            return true;
//        }
//        if(!giveTheSampleCheckPointInited[1]){
//            servoValueOutputter.giveTheSample(servoValueOutputter.InstallerLocationX-InstallerRequireErrorX,servoValueOutputter.InstallerLocationY,servoValueOutputter.InstallerLocationZ);
//            giveTheSampleStartTime[1]=System.currentTimeMillis();
//            giveTheSampleCheckPointInited[1]=true;
//        }
//        if(!giveTheSampleCheckPointPassed[1]){
//            if(System.currentTimeMillis()-giveTheSampleStartTime[1]>giveTheSampleRequireTimeMS[1]){
//                giveTheSampleCheckPointPassed[1]=true;
//            }
//            return true;
//        }
//
//        return false;
    }

}
