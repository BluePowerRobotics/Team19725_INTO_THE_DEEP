package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Controllers.SixServoArm.SixServoArmController;

@TeleOp

public class OP0709 extends LinearOpMode {
    SixServoArmController sixServoArmController ;

    Servo servos3, servos4, servos5;


    long t;
    public boolean aHasBeenPressed = false;
    public int servo_select = 3;
    public double servo_position = 0.5;


    private void initHardware() {
        sixServoArmController= new SixServoArmController(hardwareMap,telemetry);
        sixServoArmController.initArm();

        t = System.currentTimeMillis();// 获取当前时间


    }

    public void fps_and_telemetry() {

        //telemetry.addData("armPuller",armPuller.getCurrentPosition());
        telemetry.addData("fps", 1000 / (System.currentTimeMillis() - t));// fps


        telemetry.update();
        t = System.currentTimeMillis();// 获取当前时间
    }


    public void runOpMode() {
        initHardware();

        waitForStart();
        while (opModeIsActive()) {
            fps_and_telemetry();

            sixServoArmController.setMode(SixServoArmController.SIX_SERVO_ARM_RUNMODE.RUN_WITHOUT_LOCATOR);
                    sleep(200);
            sixServoArmController.setTargetPosition(200 * gamepad1.left_stick_x, 200 * gamepad1.left_stick_y, 20,
                    Math.PI, 0.5 * Math.PI);
            sleep(2000);
        }
    }

}