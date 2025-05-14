package org.firstinspires.ftc.teamcode.OpModes;

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
    public void initClimb(HardwareMap hardwareMaprc, Gamepad gamepad2rc, Telemetry telemetryrc){
        hardwareMap = hardwareMaprc;
        climbLeft = hardwareMap.get(DcMotor.class,"climbLeft");
        climbRight = hardwareMap.get(DcMotor.class,"climbRight");
        climbLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        climbRight.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry = telemetryrc;
        t = System.currentTimeMillis();
        gamepad2 =gamepad2rc;
        climbLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climbRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climbLeft.setTargetPosition(560 * 2);
        climbRight.setTargetPosition(560 * 2);
        climbLeft.setPower(1);
        climbRight.setPower(1);
        while (System.currentTimeMillis() - t <= 2000) {

            climbLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            climbRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            telemetry.addData("Testing", climbLeft.getCurrentPosition());
            telemetry.update();
        }
        climbLeftLength=climbLeft.getCurrentPosition();
        climbRightLength=climbRight.getCurrentPosition();
        climbLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climbRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("StartToBack", climbLeft.getCurrentPosition());
        telemetry.update();
        climbLeft.setTargetPosition(-(int) climbLeftLength);
        climbRight.setTargetPosition(-(int) climbRightLength);
        while ((climbLeft.getCurrentPosition() + climbLeftLength > 5 || climbLeft.getCurrentPosition() + climbLeftLength < -5)&&(climbRight.getCurrentPosition() + climbRightLength > 5 || climbRight.getCurrentPosition() + climbRightLength < -5)) {
            climbLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            climbRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            telemetry.addData("Backing", climbRight.getCurrentPosition());
            telemetry.update();
        }
        climbRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climbLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    void climb(){
        if(climbUp){
            climbLeft.setTargetPosition((int) climbLeftLength);
            climbRight.setTargetPosition((int) climbRightLength);
            climbLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            climbRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }else{
            climbLeft.setTargetPosition(0);
            climbRight.setTargetPosition(0);
            climbLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            climbRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (gamepad2.right_bumper) {
            if (!rbhasbeenpressed) {
                if (!climbUp)
                    climbUp = true;
                else
                    climbUp = false;
                rbhasbeenpressed = true;
            } else {
                rbhasbeenpressed = true;
            }
        } else {
            rbhasbeenpressed = false;
        }
    }
    boolean rbhasbeenpressed=false;
}
