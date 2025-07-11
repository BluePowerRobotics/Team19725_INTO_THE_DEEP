package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Controllers.SixServoArm.SixServoArmController;
import org.firstinspires.ftc.teamcode.Controllers.SixServoArm.SixServoArmEasyAction;

@TeleOp(name = "SixServoArmTest",group = "Test")
public class SixServoArmTest extends LinearOpMode {
    SixServoArmEasyAction sixServoArmController = new SixServoArmEasyAction(hardwareMap, telemetry, gamepad2);
    void initHardwareMap() {
        // 初始化硬件映射
        // 这里可以添加初始化代码，例如获取硬件组件等
        Actions.runBlocking(sixServoArmController.SixServoArmInit());


    }
    @Override
    public void runOpMode() throws InterruptedException {
        initHardwareMap();
        waitForStart();
        while (opModeIsActive()) {
//            sixServoArmController.setTargetLocation(200 * gamepad1.left_stick_x, 200 * gamepad1.left_stick_y, 0,
//                    Math.PI, 0.5 * Math.PI);
//            sixServoArmController.setMode(SixServoArmController.SIX_SERVO_ARM_RUNMODE.RUN_TO_POSITION);
            sleep(200);
        }
    }
}
