package org.firstinspires.ftc.teamcode.Controllers;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;


class ServoRadianCalculator {
    double a = 153;
    double b = 145;
    double c = 10;
    private double[] result = new double[4];
    public double[] calculate(double x,double y,double z,double alpha4){
        double theta = 0;
        if (x>0 && y>0){theta = Math.atan(y/x);}
        else if (x<0 && y>0){theta = Math.PI-Math.atan(y/x);}
        else if (x<0 && y<0){theta = Math.PI+Math.atan(y/x);}
        else if (x>0 && y<0){theta = Math.PI*2-Math.atan(y/x);}
        else if (x==0 && y>0){theta = Math.PI*0.5;}
        else if (x==0 && y<0){theta = Math.PI*1.5;}
        double length1 = x-c*Math.sin(alpha4)*Math.cos(theta);
        double length2 = y-c*Math.sin(alpha4)*Math.sin(theta);
        double length3 = z+c*Math.cos(alpha4);
        double length = Math.sqrt(length1*length1+length2*length2+length3*length3);
        double alpha1 = Math.acos((a*a+length*length-b*b)/(2*a*length))+Math.asin(length3/length);
        double alpha2 = Math.acos((a*a+b*b-length*length)/(2*a*b));
        double alpha3 = Math.acos((b*b+length*length-a*a)/(2*b*length))+Math.acos(length3/length)+alpha4;
        result = new double[]{theta,alpha1,alpha2,alpha3};
        return result;
    }
    public double getTheta() {
        return result[0];
    }
    public double getAlpha1() {
        return result[1];
    }
    public double getAlpha2() {
        return result[2];
    }
    public double getAlpha3() {
        return result[3];
    }
}
class ServoValueOutputter{
    private ServoRadianCalculator servoRadianCalculator;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private Servo[] servo = new Servo[6];
    
    private double[] servoZeroPositionDegree = {0, 0, 0, 0, 0, 0};
    private int[] servoDegree = {360, 180, 180, 270, 180, 180};//舵机总旋转角度

    public ServoValueOutputter(HardwareMap hardwareMap,Telemetry telemetry,ServoRadianCalculator servoRadianCalculator) {
        this.servoRadianCalculator = servoRadianCalculator;
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        servo[0] = this.hardwareMap.get(Servo.class,"servos0");
        servo[1] = this.hardwareMap.get(Servo.class,"servos1");
        servo[2] = this.hardwareMap.get(Servo.class,"servos2");
        servo[3] = this.hardwareMap.get(Servo.class,"servos3");
        servo[4] = this.hardwareMap.get(Servo.class,"servos4");
        servo[5] = this.hardwareMap.get(Servo.class,"servos5");
        servo[0].setDirection(Servo.Direction.REVERSE);//逆时针
        servo[1].setDirection(Servo.Direction.REVERSE);//逆时针
        servo[2].setDirection(Servo.Direction.FORWARD);//顺时针
        servo[3].setDirection(Servo.Direction.FORWARD);//顺时针
        servo[4].setDirection(Servo.Direction.FORWARD);
        servo[5].setDirection(Servo.Direction.FORWARD);//夹取
    }


    private double[] servoPosition = new double[6];//当前角度
    public void setRadians(double[] Radians,double clipRadian,boolean useAutoCalculator) {//控制机械臂
        for (int i = 0; i <= 3; i++) {
            servoPosition[i] = Math.toDegrees(Radians[i]) + servoZeroPositionDegree[i];
            telemetry.addData("Servo " + i + " Degree", Math.toDegrees(Radians[i]));
            telemetry.addData("Servo " + i + " Position", servoPosition[i]);
            servo[i].setPosition(Math.min(1, Math.max(0, servoPosition[i] / servoDegree[i])));
            servo[i].setPosition((servoPosition[i] / servoDegree[i]));
            Range.clip((servoPosition[i] / servoDegree[i]),0, 1);
        }
        setClipPosition(clipRadian,useAutoCalculator);
    }
    enum ClipPosition {
        LOCKED,
        UNLOCKED,
        HALF_LOCKED
    }
    public void setClip(ClipPosition clipPosition) {
        switch (clipPosition) {
            case LOCKED:
                servo[5].setPosition(0); // Assuming 0 is the locked position
                break;
            case UNLOCKED:
                servo[5].setPosition(1); // Assuming 1 is the unlocked position
                break;
            case HALF_LOCKED:
                servo[5].setPosition(0.5); // Assuming 0.5 is the half-locked position
                break;
        }
        telemetry.addData("Clip Position", clipPosition);
    }
    public void setClipPosition(double Radian){
        while(Radian>Math.PI) Radian-=Math.PI;
        while(Radian<0) Radian+=Math.PI;
        servo[4].setPosition(Radian/(Math.toRadians(servoDegree[4])));
        telemetry.addData("Clip Position", Radian);
    }
    public void setClipPosition(double radian,boolean useAutoCalculator){
        if(useAutoCalculator){
            setClipPosition(radian-servoRadianCalculator.getTheta());
        }else{
            setClipPosition(radian);
        }
    }
}

public class SixServoArmController {
    ServoRadianCalculator servoRadianCalculator = new ServoRadianCalculator();
    ServoValueOutputter servoValueOutputter;
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
        servoValueOutputter = new ServoValueOutputter(hardwareMap, telemetry, servoRadianCalculator);
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
