package org.firstinspires.ftc.teamcode.Controllers;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FiveServosArmController {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    Servo servoA,servoB,servoC,servoD,servoE,servoF;
    double lengthA,lengthB,lengthC,positionA,positionB;
    double clipLockPos,clipUnlockPos;
    public void initArm(HardwareMap hardwareMapRC, Telemetry telemetryRC){
        hardwareMap = hardwareMapRC;
        telemetry = telemetryRC;
        servoA = hardwareMap.get(Servo.class,"");
        servoB = hardwareMap.get(Servo.class,"");
        servoC = hardwareMap.get(Servo.class,"");
        servoD = hardwareMap.get(Servo.class,"");
        servoE = hardwareMap.get(Servo.class,"");
        servoF = hardwareMap.get(Servo.class,"");
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
        alphaDegree = Math.toDegrees(Math.atan(Math.abs(y / x)));
        if (y <= 0 && x > 0)
            alphaDegree = 360 - alphaDegree;
        if (y <= 0 && x <= 0)
            alphaDegree = 180 + alphaDegree;
        if (y > 0 && x <= 0)
            alphaDegree = 180 - alphaDegree;
        servoA.setPosition(alphaDegree/270);
        double r = Math.sqrt(x*x+y*y);
        length = Math.sqrt(r*r+z*z);
        lengthAC = Math.sqrt(length * length + lengthC * lengthC - 2 * length * lengthC * Math.cos(Math.toRadians(Math.abs(alphaDegree))-0.5*Math.PI+radianArmPosition));
        double radianA = Math.acos((lengthA*lengthA+lengthAC*lengthAC-lengthB*lengthB)/(2*lengthA*lengthAC));
        double radianB = Math.acos((lengthA*lengthA+lengthB*lengthB-lengthAC*lengthAC)/(2*lengthA*lengthB));
        double radianC = 2*Math.PI-radianA-radianB;
        double radianCRest = Math.acos((lengthAC*lengthAC+lengthC*lengthC-length*length)/(2*lengthAC*lengthC));
        servoD.setPosition((radianC+radianCRest)/(0.75*Math.PI));
        servoC.setPosition(radianB/(0.75*Math.PI));
        double radianARest = Math.PI*0.5-radianArmPosition-radianC-radianCRest;
        servoB.setPosition((radianA+radianARest)/(0.75*Math.PI));
        servoE.setPosition(radianClipPosition/Math.PI);
    }
    public void setClip(boolean lock){
        if(lock) servoF.setPosition(clipLockPos);
        else servoF.setPosition(clipUnlockPos);
    }
}
