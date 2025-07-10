package org.firstinspires.ftc.teamcode.Controllers.IntakeLength;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class TrigonometricIntakeLengthController implements IntakeLengthControllerInterface {
    static TrigonometricIntakeLengthController instance;

    private Servo inTakeLengthController;
    private HardwareMap hardwaremap;
    public TrigonometricIntakeLengthController(HardwareMap hardwaremap){
        this.hardwaremap = hardwaremap;
        inTakeLengthController = this.hardwaremap.get(Servo.class,"");
    }
    private double armALength=0;
    private double armBLength=0;
    private double inTakeMinLength=0;//收到最小时(重叠时)相对于默认值的位置（伸出为正）
    private double intakeTargetLength;
    private double intakeLengthTargetRadian;
    private double intakeLengthNowRadian;
    @Override
    public TrigonometricIntakeLengthController setIntakeTargetPosition(double inTakeLength){
        intakeTargetLength = inTakeLength;
        intakeLengthTargetRadian = Math.acos((armALength*armALength+(intakeTargetLength +inTakeMinLength)*(intakeTargetLength +inTakeMinLength)-armBLength*armBLength)/(2*armALength*(intakeTargetLength +inTakeMinLength)));
        return instance;
    }
    public static synchronized TrigonometricIntakeLengthController getInstance(HardwareMap hardwareMap) {
        if (instance == null) {
            instance = new TrigonometricIntakeLengthController(hardwareMap);
        }
        return instance;
    }
    public static synchronized TrigonometricIntakeLengthController getInstance() {
        return instance;
    }
    @Override
    public TrigonometricIntakeLengthController update(){
        inTakeLengthController.setPosition(intakeLengthTargetRadian /Math.PI);
        setPositionTime = System.currentTimeMillis();
        return instance;
    }
    long setPositionTime = 0;
    double servoSpeed = 0.24;//sec per 60 degree
    @Override
    public double getIntakeLengthCurrentPosition(){
        boolean goBigger,goSmaller;
        goBigger = intakeLengthTargetRadian-intakeLengthNowRadian>0;
        goSmaller = intakeLengthTargetRadian-intakeLengthNowRadian<0;
        if(goBigger){
            intakeLengthNowRadian = Math.min(intakeLengthNowRadian+(System.currentTimeMillis()-setPositionTime)*Math.PI/(3*servoSpeed),intakeLengthTargetRadian);
        }
        else if(goSmaller){
            intakeLengthNowRadian = Math.max(intakeLengthNowRadian-(System.currentTimeMillis()-setPositionTime)*Math.PI/(3*servoSpeed),intakeLengthTargetRadian);
        }else{
            intakeLengthNowRadian = intakeLengthTargetRadian;
        }
        return intakeLengthNowRadian;
    }
    @Override
    public double getIntakeLengthTargetPosition(){
        return intakeLengthTargetRadian;
    }
}
