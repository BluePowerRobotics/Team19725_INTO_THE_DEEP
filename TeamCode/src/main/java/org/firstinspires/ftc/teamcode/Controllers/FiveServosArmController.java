package org.firstinspires.ftc.teamcode.Controllers;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FiveServosArmController {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private Servo[] servo;
    private double lengthA,lengthB,lengthC;
    private double clipLockPos,clipUnlockPos;
    private final int[] servoDegree={270,180,180,270,180,180};

    private final int servoADegree = 270,servoBDegree = 180,servoCDegree = 180,servoDDegree = 270,servoEDegree = 180,servoFDegree = 180;
    private double[] servoPosition;
    public void initArm(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        servo[0] = this.hardwareMap.get(Servo.class,"");
        servo[1] = this.hardwareMap.get(Servo.class,"");
        servo[2] = this.hardwareMap.get(Servo.class,"");
        servo[3] = this.hardwareMap.get(Servo.class,"");
        servo[4] = this.hardwareMap.get(Servo.class,"");
        servo[5] = this.hardwareMap.get(Servo.class,"");
        servo[0].setDirection(Servo.Direction.REVERSE);//逆时针
        servo[1].setDirection(Servo.Direction.REVERSE);//逆时针
        servo[2].setDirection(Servo.Direction.FORWARD);//顺时针
        servo[3].setDirection(Servo.Direction.FORWARD);//顺时针
        servo[4].setDirection(Servo.Direction.FORWARD);
        servo[5].setDirection(Servo.Direction.FORWARD);//夹取
    }

    //均为向上正方向，向右正方向，向前正方向
    //D为顺时针
    public void setLocation(double x,double y,double z,double radianArmPosition,double radianClipPosition) {
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
        double radianC = 2*Math.PI-radianA-radianB;
        double radianCRest = Math.acos((lengthAC*lengthAC+lengthC*lengthC-length*length)/(2*lengthAC*lengthC));
        double radianARest = Math.PI*0.5-radianArmPosition-radianC-radianCRest;

        servoPosition[0]=alphaDegree/servoDegree[0];
        servoPosition[1]=(radianA+radianARest)/Math.toRadians(servoDegree[1]);
        servoPosition[2]=radianB/Math.toRadians(servoDegree[2]);
        servoPosition[3]=(radianC+radianCRest)/Math.toRadians(servoDegree[3]);
        servoPosition[4]=radianClipPosition/Math.toRadians(servoDegree[4]);

        for(int i = 0;i<=4;i++){
            servoPosition[i]=Math.min(1,Math.max(0,servoPosition[i]));
            servo[i].setPosition(servoPosition[i]);
        }


    }
    public void setClip(boolean lock){
        if(lock) servo[5].setPosition(clipLockPos);
        else servo[5].setPosition(clipUnlockPos);
    }
}
