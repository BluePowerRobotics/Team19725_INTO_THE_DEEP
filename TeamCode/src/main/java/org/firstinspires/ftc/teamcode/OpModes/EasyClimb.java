package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class EasyClimb {
    double climbLeftLength;
    double climbRightLength;
    boolean climbUp;
    double t =System.currentTimeMillis();
    HardwareMap hardwareMap;
    DcMotor climbLeft, climbRight;
    Gamepad gamepad2;
    public void initClimb(HardwareMap hardwareMaprc, Gamepad gamepad2rc){
        hardwareMap = hardwareMaprc;
        climbLeft = hardwareMap.get(DcMotor.class,"climbLeft");
        climbRight = hardwareMap.get(DcMotor.class,"climbRight");
        climbLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        climbRight.setDirection(DcMotorSimple.Direction.FORWARD);
        t = System.currentTimeMillis();
        gamepad2 =gamepad2rc;
        climbLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        climbRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        while (System.currentTimeMillis() - t <= 2000) {
            //climbLeft.setPower(1);
            climbRight.setPower(0.1);
        }
        climbLeft.setPower(0);
        climbRight.setPower(0);
    }
    void climb() {
        if (gamepad2.left_bumper) {
            //climbLeft.setPower(-1);
            climbRight.setPower(-0.5);
        } else if(gamepad2.right_bumper){
            //climbLeft.setPower(1);
            climbRight.setPower(0.5);
        }
    }
}
