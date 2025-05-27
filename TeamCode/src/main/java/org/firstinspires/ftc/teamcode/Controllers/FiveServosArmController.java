package org.firstinspires.ftc.teamcode.Controllers;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FiveServosArmController {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private Servo servoA,servoB,servoC,servoD,servoE,servoF;
    private double lengthA,lengthB,lengthC;
    private double clipLockPos,clipUnlockPos;
    public void initArm(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        servoA = this.hardwareMap.get(Servo.class,"");
        servoB = this.hardwareMap.get(Servo.class,"");
        servoC = this.hardwareMap.get(Servo.class,"");
        servoD = this.hardwareMap.get(Servo.class,"");
        servoE = this.hardwareMap.get(Servo.class,"");
        servoF = this.hardwareMap.get(Servo.class,"");
        servoA.setDirection(Servo.Direction.REVERSE);//逆时针
        servoB.setDirection(Servo.Direction.REVERSE);//逆时针
        servoC.setDirection(Servo.Direction.FORWARD);//顺时针
        servoD.setDirection(Servo.Direction.FORWARD);//顺时针
        servoE.setDirection(Servo.Direction.FORWARD);
        servoF.setDirection(Servo.Direction.FORWARD);//夹取
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

        if(alphaDegree/270>=1){
            servoA.setPosition((alphaDegree-180)/270);
            servoD.setPosition((2*Math.PI-radianC-radianCRest)/(1.5*Math.PI));
            servoC.setPosition((2*Math.PI-radianB)/(1.5*Math.PI));
            servoB.setPosition((2*Math.PI-radianA-radianARest)/(1.5*Math.PI));
            servoE.setPosition((Math.PI-radianClipPosition)/Math.PI);
        }else{
            servoA.setPosition(alphaDegree/270);
            servoD.setPosition((radianC+radianCRest)/(1.5*Math.PI));
            servoC.setPosition(radianB/(1.5*Math.PI));
            servoB.setPosition((radianA+radianARest)/(1.5*Math.PI));
            servoE.setPosition(radianClipPosition/Math.PI);
        }

    }
    public void setClip(boolean lock){
        if(lock) servoF.setPosition(clipLockPos);
        else servoF.setPosition(clipUnlockPos);
    }
}
