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
        if(instance == null){
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
    public static double[] servoZeroPositionDegree = {-10,-48,-58.7,18, 0, 0};
    public static int[] servoDegree = {315, 257, 230, 255, 255, 170};//舵机总旋转角度
    public static boolean reverse = false;

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
            servoPosition[i] = Math.toDegrees(Radians[i]) - servoZeroPositionDegree[i];
            while(Radians[i]>2*Math.PI)Radians[i]-=2*Math.PI;
            while(Radians[i]<0)Radians[i]+=2*Math.PI;
            telemetry.addData("Servo " + i + " Degree", Math.toDegrees(Radians[i]));
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
    public void setClip(ClipPosition clipPosition) {
        switch (clipPosition) {
            case LOCKED:
                servo[5].setPosition(clipLockPos);
                servoRadianCalculator.setClipHeight(-5);//夹取时高度为-5
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
        servo[servoIndex].setPosition(position);
    }
    public void DegreeServoControl(int servoIndex, double degree) {
        if(isNaN(degree)) degree=0;
        while(degree>servoDegree[servoIndex]) degree-=servoDegree[servoIndex];
        while(degree<0) degree+=servoDegree[servoIndex];
        servo[servoIndex].setPosition(Range.clip((degree / servoDegree[servoIndex]), 0, 1));
        telemetry.addData("Servo " + servoIndex + " Degree", degree);
    }
    public static double servo0leftDegree = 180, servo1leftDegree = 45, servo2leftDegree = 180, servo3leftDegree = 45, servo4leftDegree = 0;
    public void moveToLeft(){
        DegreeServoControl(0,servo0leftDegree);
        DegreeServoControl(1,servo1leftDegree);
        DegreeServoControl(2,servo2leftDegree);
        DegreeServoControl(3,servo3leftDegree);
        DegreeServoControl(4,servo4leftDegree);
    }
}
