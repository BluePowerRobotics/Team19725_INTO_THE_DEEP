package org.firstinspires.ftc.teamcode.Controllers.SixServoArm;

import static java.lang.Double.NaN;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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

    private double[] servoZeroPositionDegree = {0, 0, 56.7, 90, 0, 0};
    private int[] servoDegree = {315, 270, 270, 270, 270, 180};//舵机总旋转角度

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
        servo[1].setDirection(Servo.Direction.REVERSE);//逆时针
        servo[2].setDirection(Servo.Direction.FORWARD);//顺时针
        servo[3].setDirection(Servo.Direction.FORWARD);//顺时针
        servo[4].setDirection(Servo.Direction.REVERSE);
        servo[5].setDirection(Servo.Direction.FORWARD);//夹取
    }


    private double[] servoPosition = new double[6];//当前角度
    public void setRadians(double[] Radians,double clipRadian,boolean useAutoCalculator) {//控制机械臂
        for (int i = 0; i <= 3; i++) {
            servoPosition[i] = Math.toDegrees(Radians[i]) + servoZeroPositionDegree[i];
            telemetry.addData("Servo " + i + " Degree", Math.toDegrees(Radians[i]));
            telemetry.addData("Servo " + i + " Position", servoPosition[i]);
            telemetry.update();
            if(servoPosition[i]==NaN) servoPosition[i]=0;
            servo[i].setPosition(Math.min(1, Math.max(0, servoPosition[i] / servoDegree[i])));
            servo[i].setPosition((servoPosition[i] / servoDegree[i]));
            Range.clip((servoPosition[i] / servoDegree[i]),0, 1);
        }
        setClipPosition(clipRadian,useAutoCalculator);
    }
    public enum ClipPosition {
        LOCKED,
        UNLOCKED,
        HALF_LOCKED
    }
    public void setClip(ClipPosition clipPosition) {
        switch (clipPosition) {
            case LOCKED:
                servo[5].setPosition(0.8); // Assuming 0 is the locked position
                break;
            case UNLOCKED:
                servo[5].setPosition(0.3); // Assuming 1 is the unlocked position
                break;
            case HALF_LOCKED:
                servo[5].setPosition(0.69); // Assuming 0.5 is the half-locked position
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
}
