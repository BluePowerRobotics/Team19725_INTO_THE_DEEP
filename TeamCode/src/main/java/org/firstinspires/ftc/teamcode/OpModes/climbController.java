package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class climbController {
    double climbLeftPower;
    double climbRightPower;
    HardwareMap hardwareMap;
    DcMotor climbLeft, climbRight;
    public void initClimb(HardwareMap hardwareMaprc){
        hardwareMap = hardwareMaprc;
        climbLeft = hardwareMap.get(DcMotor.class,"climbLeft");
        climbRight = hardwareMap.get(DcMotor.class,"climbRight");
    }
}
