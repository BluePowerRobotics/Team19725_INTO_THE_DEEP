package org.firstinspires.ftc.teamcode.Controllers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ClimbController {
    double climbLeftLength;
    double climbRightLength;
    boolean climbUp;
    double t =System.currentTimeMillis();
    HardwareMap hardwareMap;
    DcMotor climbLeft, climbRight;
    Gamepad gamepad2;
    Telemetry telemetry;
    public void initClimb(HardwareMap hardwareMapRC, Gamepad gamepad2RC, Telemetry telemetryRC){
        hardwareMap = hardwareMapRC;
        climbLeft = hardwareMap.get(DcMotor.class,"climbLeft");
        climbRight = hardwareMap.get(DcMotor.class,"climbRight");
        climbLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        climbRight.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry = telemetryRC;
        t = System.currentTimeMillis();
        gamepad2 =gamepad2RC;
        climbLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climbRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climbLeft.setTargetPosition(560 * 2);
        climbRight.setTargetPosition(560 * 2);
        climbLeft.setPower(1);
        climbRight.setPower(1);
        while (System.currentTimeMillis() - t <= 2000) {

            climbLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            climbRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            telemetry.addData("ClimbController:Testing", climbLeft.getCurrentPosition());
            telemetry.update();
        }
        climbLeftLength=climbLeft.getCurrentPosition();
        climbRightLength=climbRight.getCurrentPosition();
        climbLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climbRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("ClimbController:StartToBack", climbLeft.getCurrentPosition());
        telemetry.update();
        climbLeft.setTargetPosition(-(int) climbLeftLength);
        climbRight.setTargetPosition(-(int) climbRightLength);
        while ((climbLeft.getCurrentPosition() + climbLeftLength > 5 || climbLeft.getCurrentPosition() + climbLeftLength < -5)&&(climbRight.getCurrentPosition() + climbRightLength > 5 || climbRight.getCurrentPosition() + climbRightLength < -5)&&System.currentTimeMillis() - t <= 4000) {
            climbLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            climbRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            telemetry.addData("ClimbController:Backing", climbRight.getCurrentPosition());
            telemetry.update();
        }
        climbRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climbLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void climb(){
        telemetry.addData("climbLeftLength",climbLeftLength);
        telemetry.addData("climbRightLength",climbRightLength);
        if(climbUp){
            climbLeft.setTargetPosition(0);
            climbRight.setTargetPosition(0);
        }else{
            climbLeft.setTargetPosition((int) climbLeftLength);
            climbRight.setTargetPosition((int) climbRightLength);
        }
        climbLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        climbRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (gamepad2.right_bumper) {
            if (!rbHasBeenPressed) {
                climbUp = !climbUp;
                }
            rbHasBeenPressed = true;
        } else {
            rbHasBeenPressed = false;
        }
    }
    boolean rbHasBeenPressed =false;

}
