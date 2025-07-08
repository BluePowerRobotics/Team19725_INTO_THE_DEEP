package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class OP12123 extends LinearOpMode {


    Servo servos3, servos4, servos5;


    long t;
    public boolean aHasBeenPressed = false;
    public int servo_select = 3;
    public double servo_position = 0.5;


    private void initHardware() {

        servos3 = hardwareMap.get(Servo.class, "servos3");
        servos4 = hardwareMap.get(Servo.class, "servos4");
        servos5 = hardwareMap.get(Servo.class, "servos5");


        t = System.currentTimeMillis();// 获取当前时间


    }

    public void fps_and_telemetry() {

        //telemetry.addData("armPuller",armPuller.getCurrentPosition());
        telemetry.addData("fps", 1000 / (System.currentTimeMillis() - t));// fps


        telemetry.addData("servo_select", servo_select);
        telemetry.addData("servo_position", servo_position);

        telemetry.update();
        t = System.currentTimeMillis();// 获取当前时间
    }


    public void runOpMode() {
        initHardware();

        waitForStart();
        while (opModeIsActive()) {
            servoControl();
            fps_and_telemetry();

        }
    }


    public void servoControl() {



        if (gamepad1.a) {
            if (!aHasBeenPressed) {
                servo_select++;
                aHasBeenPressed = true;
            }
        } else {
            aHasBeenPressed = false;
        }
        if (servo_select > 5) {
            servo_select = 3;
        }

        if (gamepad1.left_bumper) {

            servo_position += 0.005;

        }
        if (gamepad1.right_bumper) {

            servo_position -= 0.005;

        }

        if (servo_position > 1)
            servo_position = 1;
        if (servo_position < 0)
            servo_position = 0;

        if (servo_select == 3) {
            servos3.setPosition(servo_position);
        } else if (servo_select == 4) {
            servos4.setPosition(servo_position);
        } else {
            servos5.setPosition(servo_position);
        }
    }
}