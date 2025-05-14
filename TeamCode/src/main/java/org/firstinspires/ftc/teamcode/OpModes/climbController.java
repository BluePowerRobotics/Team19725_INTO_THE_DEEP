package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class climbController {
    double climbLeftLength;
    double climbRightLength;
    double t =System.currentTimeMillis();
    HardwareMap hardwareMap;
    DcMotor climbLeft, climbRight;
    Telemetry telemetry;
    public void initClimb(HardwareMap hardwareMaprc, Telemetry telemetryrc){
        hardwareMap = hardwareMaprc;
        climbLeft = hardwareMap.get(DcMotor.class,"climbLeft");
        climbRight = hardwareMap.get(DcMotor.class,"climbRight");
        telemetry = telemetryrc;
        t = System.currentTimeMillis();
        climbLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climbRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climbLeft.setTargetPosition(560 * 2);
        climbRight.setTargetPosition(560 * 2);
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
        armMotor.setTargetPosition(-(int) motorLength);.setTargetPosition(-(int) motorLength);
        while (climbLeft.getCurrentPosition() + climbLeftLength > 5 || climbLeft.getCurrentPosition() + climbLeftLength < -5) {

            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            telemetry.addData("Backing", armMotor.getCurrentPosition());
            telemetry.update();
        }

    }
}
