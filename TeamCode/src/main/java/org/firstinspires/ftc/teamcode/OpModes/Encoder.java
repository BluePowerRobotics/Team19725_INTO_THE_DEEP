package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name = "Encoder Test ")
public class Encoder extends OpMode {
    boolean aispressed = false;
    DcMotor motor;
    double ticks = 530;
    double newTarget;
    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "motor");
        telemetry.addData("Hardware: ", "Initialized");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        if(gamepad1.a){
                if(!aispressed){
                    encoder(2);
                    aispressed = true;
                }
            }else{
                aispressed = false;

        }
        telemetry.addData("Motor Ticks: ", motor.getCurrentPosition());
        if(gamepad1.b){
            tracker();
        }

    }
    public void encoder(int turnage){
        newTarget = ticks/turnage;
        motor.setTargetPosition(motor.getCurrentPosition()+(int)newTarget);
        motor.setPower(0.3);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void tracker(){
        motor.setTargetPosition(0);
        motor.setPower(0.8);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

}