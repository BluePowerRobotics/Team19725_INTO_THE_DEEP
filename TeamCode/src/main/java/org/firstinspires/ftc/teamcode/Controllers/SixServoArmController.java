package org.firstinspires.ftc.teamcode.Controllers;

import com.qualcomm.robotcore.hardware.HardwareMap;
import java.text.MessageFormat;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.text.MessageFormat;
import java.util.Objects;

public class SixServoArmController {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private Servo[] servo = new Servo[6];
    public enum SIX_SERVO_ARM_RUNMODE{
        RUN_TO_POSITION,STOP_AND_RESET,RUN_WITHOUT_PREDICTOR
    }
    private SIX_SERVO_ARM_RUNMODE SIX_SERVO_ARM_MODE;
    private long setLocationTime;
    private static final double lengthA=153,lengthB=145,lengthC=0;//mm
    private static final double clipLockPos=0,clipUnlockPos=0;
    private final int[] servoDegree={270,270,270,270,180,180};
    private double[] servoTargetDegree=new double[5];
    private double[] servoNowDegree={0,0,0,0,0};
    private double[] servoDegreeError = new double[5];
    private double[] servoSpeed={0.24,0.24,0.24,0.24,0.24};//sec per 60 degree
    private double servoRunTimeMax =0;
    private final double[] servoZeroPositionDegree={0,0,0,0,0};

    //private final int servoADegree = 270,servoBDegree = 180,servoCDegree = 180,servoDDegree = 270,servoEDegree = 180,servoFDegree = 180;
    private double[] servoPosition={0,0,0,0,0};
    public void initArm(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
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

        setMode(SIX_SERVO_ARM_RUNMODE.RUN_WITHOUT_PREDICTOR);
        setTargetLocation(resetX, resetY, resetZ, 0.5 * Math.PI, 0.5 * Math.PI);
    }

    final double resetX = 100,resetY = 0,resetZ = 10;
    double x=resetX,y=resetY,z=resetZ;
    double recentX=resetX,recentY=resetY,recentZ=resetZ;
    double nowX=resetX,nowY=resetY,nowZ=resetZ;
    double Distance;
    double radianArmPosition;
    double radianClipPosition;
    //均为向上正方向，向右正方向，向前正方向
    //D为顺时针
    public void setTargetLocation(double x, double y, double z, double radianArmPosition, double radianClipPosition) {
        Distance = Math.sqrt(Math.pow((this.x-x),2)+Math.pow((this.y-y),2)+Math.pow((this.y-y),2));
        recentX= this.x;
        recentY= this.y;
        recentZ= this.z;
        this.x = x;
        this.y = y;
        this.z = z;
        this.radianArmPosition = radianArmPosition;
        this.radianClipPosition = radianClipPosition;
        setLocationTime=System.currentTimeMillis();
        double lengthAC;
        double length;
        double alphaDegree=0;

        if (x == 0 && y > 0)
            alphaDegree = 90;
        else if (y <= 0 && x == 0)
            alphaDegree = 270;
        else {
            double degrees = Math.toDegrees(Math.atan(Math.abs(y / x)));
            if (y <= 0 && x > 0)
                alphaDegree = 360 - degrees;
            else if (y <= 0 && x < 0)
                alphaDegree = 180 + degrees;
            else if (y > 0 && x < 0)
                alphaDegree = 180 - degrees;
        }


        double r = Math.sqrt(x*x+y*y);
        length = Math.sqrt(r*r+z*z);
        lengthAC = Math.sqrt(length * length + lengthC * lengthC - 2 * length * lengthC * Math.cos(Math.toRadians(Math.abs(alphaDegree))-0.5*Math.PI+radianArmPosition));
        double radianA = Math.acos((lengthA*lengthA+lengthAC*lengthAC-lengthB*lengthB)/(2*lengthA*lengthAC));
        double radianB = Math.acos((lengthA*lengthA+lengthB*lengthB-lengthAC*lengthAC)/(2*lengthA*lengthB));
        double radianC = Math.PI-radianA-radianB;
        double radianCRest = Math.acos((lengthAC*lengthAC+lengthC*lengthC-length*length)/(2*lengthAC*lengthC));
        double radianARest = Math.PI*0.5-radianArmPosition-radianC-radianCRest;
        servoTargetDegree[0]=alphaDegree;
        servoTargetDegree[1]=Math.toDegrees(radianA+radianARest);
        servoTargetDegree[2]=Math.toDegrees(radianB);
        servoTargetDegree[3]=Math.toDegrees(radianC+radianCRest);
        servoTargetDegree[4]=Math.toDegrees(radianClipPosition);
        for(int i = 0;i<=4;i++){
            servoDegreeError[i] = servoTargetDegree[i]-servoNowDegree[i];
            servoRunTimeMax = Math.max(servoRunTimeMax,servoSpeed[i]*60*servoDegreeError[i]);
        }
        if (Objects.requireNonNull(SIX_SERVO_ARM_MODE) == SIX_SERVO_ARM_RUNMODE.RUN_WITHOUT_PREDICTOR) {
            for (int i = 0; i <= 4; i++) {
                servoPosition[i] = (servoTargetDegree[i] - servoZeroPositionDegree[i]) / servoDegree[i];
                servoPosition[i] = Math.min(1, Math.max(0, servoPosition[i]));
                if(!Double.isNaN(servoPosition[i])) {
                    servo[i].setPosition(servoPosition[i]);
                    telemetry.addData(MessageFormat.format("servo{0}", i),servoPosition[i]);
                }
            }
        }


    }
    public boolean runToPositionFinished=false;



    double lengthAC;
    double length;
    double alphaDegree=0;
    double r;
    double radianA;
    double radianB;
    double radianC;
    double radianARest;
    double radianCRest;


    public void setMode(SIX_SERVO_ARM_RUNMODE SIX_SERVO_ARM_MODE){
        this.SIX_SERVO_ARM_MODE = SIX_SERVO_ARM_MODE;
        switch (this.SIX_SERVO_ARM_MODE) {
            case RUN_TO_POSITION: {
                double x,y,z;
                if(System.currentTimeMillis()-setLocationTime>servoRunTimeMax*1000){
                    x = this.x;
                    y = this.y;
                    z = this.z;
                    runToPositionFinished = true;
                }else{
                    double ratio = (System.currentTimeMillis()-setLocationTime)/(servoRunTimeMax*1000);
                    x = (this.x - recentX) * ratio;
                    y = (this.y - recentY) * ratio;
                    z = (this.z - recentZ) * ratio;
                    runToPositionFinished = false;
                }


                if (x == 0 && y > 0)
                    alphaDegree = 90;
                else if (y <= 0 && x == 0)
                    alphaDegree = 270;
                else {
                    double degrees = Math.toDegrees(Math.atan(Math.abs(y / x)));
                    if (y <= 0 && x > 0)
                        alphaDegree = 360 - degrees;
                    else if (y <= 0 && x < 0)
                        alphaDegree = 180 + degrees;
                    else if (y > 0 && x < 0)
                        alphaDegree = 180 - degrees;
                }


                r = Math.sqrt(x*x+y*y);
                length = Math.sqrt(r*r+z*z);
                lengthAC = Math.sqrt(length * length + lengthC * lengthC - 2 * length * lengthC * Math.cos(Math.toRadians(Math.abs(alphaDegree))-0.5*Math.PI+radianArmPosition));
                radianA = Math.acos((lengthA*lengthA+lengthAC*lengthAC-lengthB*lengthB)/(2*lengthA*lengthAC));
                radianB = Math.acos((lengthA*lengthA+lengthB*lengthB-lengthAC*lengthAC)/(2*lengthA*lengthB));
                radianC = 2*Math.PI-radianA-radianB;
                radianCRest = Math.acos((lengthAC*lengthAC+lengthC*lengthC-length*length)/(2*lengthAC*lengthC));
                radianARest = Math.PI*0.5-radianArmPosition-radianC-radianCRest;
                servoTargetDegree[0]=alphaDegree;
                servoTargetDegree[1]=Math.toDegrees(radianA+radianARest);
                servoTargetDegree[2]=Math.toDegrees(radianB);
                servoTargetDegree[3]=Math.toDegrees(radianC+radianCRest);
                servoTargetDegree[4]=Math.toDegrees(radianClipPosition);

                for(int i = 0;i<=4;i++){
                    servoNowDegree[i] = servoTargetDegree[i];
                    servoPosition[i] = (servoTargetDegree[i]-servoZeroPositionDegree[i])/servoDegree[i];
                    servoPosition[i]=Math.min(1,Math.max(0,servoPosition[i]));
                    if(!Double.isNaN(servoPosition[i])) {
                        servo[i].setPosition(servoPosition[i]);
                        telemetry.addData(MessageFormat.format("servo{0}", i),servoPosition[i]);
                    }
                }
                break;
            }case STOP_AND_RESET: {
                setTargetLocation(resetX,resetY,resetZ,0.5*Math.PI,0.5*Math.PI);
                for(int i = 0;i<=4;i++){
                    servoPosition[i] = (servoTargetDegree[i]-servoZeroPositionDegree[i])/servoDegree[i];
                    servoPosition[i]=Math.min(1,Math.max(0,servoPosition[i]));
                    if(!Double.isNaN(servoPosition[i])) {
                        servo[i].setPosition(servoPosition[i]);
                        telemetry.addData(MessageFormat.format("servo{0}", i),servoPosition[i]);
                    }
                }
                break;
            }
        }
        telemetry.addLine(MessageFormat.format("lengthAC:{0}",lengthAC));
        telemetry.addLine(MessageFormat.format("length:{0}",length));
        telemetry.addLine(MessageFormat.format("alphaDegree:{0}",alphaDegree));
        telemetry.addLine(MessageFormat.format("r:{0}",r));
        telemetry.addLine(MessageFormat.format("radianA:{0}",radianA));
        telemetry.addLine(MessageFormat.format("radianB:{0}",radianB));
        telemetry.addLine(MessageFormat.format("radianC:{0}",radianC));
        telemetry.addLine(MessageFormat.format("radianARest:{0}",radianARest));
        telemetry.addLine(MessageFormat.format("radianCRest:{0}",radianCRest));
        telemetry.update();
    }
    public void setClip(boolean lock){
        if(lock) servo[5].setPosition(clipLockPos);
        else servo[5].setPosition(clipUnlockPos);
    }
}
