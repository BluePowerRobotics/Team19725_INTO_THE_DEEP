package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ClimbController {

    double climbLength;

    boolean climbUp;
    double t =System.currentTimeMillis();
    HardwareMap hardwareMap;
    DcMotor climb;
    Gamepad gamepad2;
    Telemetry telemetry;
    public void initClimb(HardwareMap hardwareMaprc, Gamepad gamepad2rc, Telemetry telemetryrc){
        hardwareMap = hardwareMaprc;
        climb = hardwareMap.get(DcMotor.class,"climb");

        climb.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry = telemetryrc;
        t = System.currentTimeMillis();
        gamepad2 =gamepad2rc;
        climb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //拉到底，获得encoder读数
        climb.setTargetPosition(9999);
        climb.setPower(0.6);
        while (System.currentTimeMillis() - t <= 2000) {

            climb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            telemetry.addData("climb_backing", climb.getCurrentPosition());
            telemetry.update();
        }
        climbLength=climb.getCurrentPosition();
        climb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        telemetry.addData("StartToBack", climb.getCurrentPosition());
//        telemetry.update();
//
//        climb.setTargetPosition(-(int) climbLength);
//        while ((climb.getCurrentPosition() + climbLength > 5 || climb.getCurrentPosition() + climbLength < -5)) {
//            climb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            telemetry.addData("Backing", climb.getCurrentPosition());
//            telemetry.update();
//        }
//        climb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    void climb(){
        telemetry.addData("climbLength",climbLength);
        if(climbUp){
            climb.setTargetPosition(-(int)climbLength);
            climb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }else{
            climb.setTargetPosition(0);
            climb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
