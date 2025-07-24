package org.firstinspires.ftc.teamcode.Controllers.SixServoArm;

import static java.lang.Double.isNaN;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Config
public class ServoValueEasyOutputter {
    private static ServoValueEasyOutputter instance;
    public static synchronized ServoValueEasyOutputter getInstance(HardwareMap hardwareMap, Telemetry telemetry, ServoRadianEasyCalculator servoRadianCalculator) {
        if(instance == null|| instance.hardwareMap != hardwareMap || instance.telemetry != telemetry || instance.servoRadianCalculator != servoRadianCalculator){
            instance = new ServoValueEasyOutputter(hardwareMap,telemetry,servoRadianCalculator);
        }
        return instance;
    }
    //public static synchronized ServoValueEasyOutputter getInstance() {
    //    return instance;
    //}

    private ServoRadianEasyCalculator servoRadianCalculator;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private Servo[] servo = new Servo[6];
    public static double[] servoZeroPositionDegree = {-28.92857,-63.529411,-43.71428,55.5882 , -77.83783783783785, 0};
    public static double[] servoDegree = {321.42857, 264.70588, 257.142847142857142857, 264.70588, 243.24324324324328, 170};//舵机总旋转角度
    public static double[] x1 ={0.09,0.24,0.87,0.47,0.32};
    public static double[] y1 ={0,0,180,180,0};
    public static double[] x2 ={0.37,0.58,0.52,0.13,0.69};
    public static double[] y2 ={90,90,90,90,90};
    public static boolean reverse = false;

    public double[] servoSetDegree = {0,0,0,0,0,0};
    public long[] servoSetDegreeTime = {0,0,0,0,0,0};//设置舵机角度的时间
    public double[] servoNowDegree = {0,0,0,0,0,0};//当前舵机角度
    public long[] servoNowDegreeTime = {0,0,0,0,0,0};//计算当前舵机角度的时间

    public ServoValueEasyOutputter(HardwareMap hardwareMap, Telemetry telemetry, ServoRadianEasyCalculator servoRadianCalculator) {
        this.servoRadianCalculator = servoRadianCalculator;
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        servo[0] = this.hardwareMap.get(Servo.class,"servoe0");
        servo[1] = this.hardwareMap.get(Servo.class,"servoe1");
        servo[2] = this.hardwareMap.get(Servo.class,"servoe2");
        servo[3] = this.hardwareMap.get(Servo.class,"servoe3");
        servo[4] = this.hardwareMap.get(Servo.class,"servoe4");
        servo[5] = this.hardwareMap.get(Servo.class,"servoe5");
        servo[0].setDirection(Servo.Direction.FORWARD);//逆时针
        servo[1].setDirection(Servo.Direction.FORWARD);//逆时针
        servo[2].setDirection(Servo.Direction.REVERSE);//顺时针
        servo[3].setDirection(Servo.Direction.REVERSE);//顺时针
        servo[4].setDirection(Servo.Direction.REVERSE);
        servo[5].setDirection(Servo.Direction.FORWARD);//夹取
    }



    private double[] servoPosition = new double[6];//当前角度
    double clipRadian=0;
    public void setRadians(double[] Radians,double clipRadian,boolean useAutoCalculator) {//控制机械臂
        this.clipRadian = clipRadian;
        for (int i = 0; i <= 3; i++) {
            if(x1[i]!=0&& x2[i]!=0) {
                servoDegree[i] = (y1[i] - y2[i]) / (x1[i] - x2[i]);
                servoZeroPositionDegree[i]=y1[i] - servoDegree[i] * x1[i];
            }

            servoPosition[i] = Math.toDegrees(Radians[i]) - servoZeroPositionDegree[i];
            while(servoPosition[i]>2*Math.PI)servoPosition[i]-=360;
            while(servoPosition[i]<0)servoPosition[i]+=360;
            telemetry.addData("Servo " + i + " Degree", Math.toDegrees(Radians[i]));
            servoSetDegree[i] = Math.toDegrees(Radians[i]);
            servoSetDegreeTime[i] = System.currentTimeMillis();
            if(Double.isNaN(servoPosition[i])){
                servoPosition[i]=0;
                if(i==1) servoPosition[i]=0- servoZeroPositionDegree[i];
                if(i==2) servoPosition[i]=180- servoZeroPositionDegree[i];
                if(i==3) servoPosition[i]=90- servoZeroPositionDegree[i];
            }
            if(reverse){
                servoPosition[i]=1-servoPosition[i];
            }
            telemetry.addData("Servo " + i + " Position", servoPosition[i]);
            telemetry.addData("Servo " + i + " Position",Math.min(1, Math.max(0, servoPosition[i] / servoDegree[i])));
            servo[i].setPosition(Math.min(1, Math.max(0, servoPosition[i] / servoDegree[i])));

        }
        setClipPosition(clipRadian,useAutoCalculator);
    }
    public enum ClipPosition {
        LOCKED,
        UNLOCKED,
        HALF_LOCKED
    }
    public static double clipLockPos=0.8,clipHalfLockPos=0.69,clipUnlockPos=0.3;
    public static double HeightError= 10;
    public ServoValueEasyOutputter setClip(ClipPosition clipPosition) {
        switch (clipPosition) {
            case LOCKED:
                servo[5].setPosition(clipLockPos);
                servoRadianCalculator.setClipHeight(-HeightError);//夹取时高度为-5
                break;
            case UNLOCKED:
                servo[5].setPosition(clipUnlockPos);
                servoRadianCalculator.setClipHeight(0);//夹取时高度为0
                break;
            case HALF_LOCKED:
                servo[5].setPosition(clipHalfLockPos);
                servoRadianCalculator.setClipHeight(0);//半夹取时高度为0
                break;
        }
        telemetry.addData("Clip Position", clipPosition);
        return getInstance(hardwareMap,telemetry,servoRadianCalculator);
    }
    public void setClipPosition(double Radian){
        while(Radian>Math.PI) Radian-=Math.PI;
        while(Radian<0) Radian+=Math.PI;
        servo[4].setPosition(Radian/(Math.toRadians(servoDegree[4])));
        telemetry.addData("Clip Position", Radian);
    }
    public void setClipPosition(double radian,boolean useAutoCalculator){
        if(useAutoCalculator){
            setClipPosition(radian-servoRadianCalculator.getRadian0());
        }else{
            setClipPosition(radian);
        }
    }
    public void SingleServoControl(int servoIndex, double position) {
        if(isNaN(position)){
            return;
        }
        position = Range.clip(position,0,1);
        servo[servoIndex].setPosition(position);
        servoSetDegree[servoIndex]=position*servoDegree[servoIndex]+servoZeroPositionDegree[servoIndex];
    }
    public double getServoPosition(int servoIndex){
        return (servoSetDegree[servoIndex]-servoZeroPositionDegree[servoIndex])/servoDegree[servoIndex];
    }
    public void DegreeServoControl(int servoIndex, double degree) {
        if(isNaN(degree)) degree=0;
        degree-=servoZeroPositionDegree[servoIndex];
        double servoValue = degree/servoDegree[servoIndex];
        servo[servoIndex].setPosition(Range.clip((servoValue), 0, 1));
        servoSetDegree[servoIndex] = degree;
        servoSetDegreeTime[servoIndex] = System.currentTimeMillis();
        telemetry.addLine("Servo " + servoIndex + " Degree: " + (degree+ servoZeroPositionDegree[servoIndex]) + " Position: " + servoValue);
    }
    public static double servo0leftDegree = 180, servo1leftDegree = 45, servo2leftDegree = 180, servo3leftDegree = 45, servo4leftDegree = 0;
    public void moveToLeft(){
        DegreeServoControl(0,servo0leftDegree);
        DegreeServoControl(1,servo1leftDegree);
        DegreeServoControl(2,servo2leftDegree);
        DegreeServoControl(3,servo3leftDegree);
        DegreeServoControl(4,servo4leftDegree);
    }

    public static double InstallerLocationX=0;
    public static double InstallerLocationY=273;
    public static double InstallerLocationZ=154.5;

    //return the intake Length
    public static double pos_0 = 0.37;
    public static double pos_1 = 1;
    public static double pos_2 = 0.74;
    public static double pos_3 = 0.93;
    public static double pos_4 = 0.33;
    public double giveTheSample(double InstallerLocationX,double InstallerLocationY,double InstallerLocationZ){
        double intakeLength = 0;
        SingleServoControl(0, pos_0);
        SingleServoControl(1, pos_1);
        SingleServoControl(2, pos_2);
        SingleServoControl(3, pos_3);
        SingleServoControl(4, pos_4);
        setClip(ClipPosition.LOCKED);
        return intakeLength;
    }
}
