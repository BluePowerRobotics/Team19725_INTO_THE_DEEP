package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Controllers.SixServoArm.*;
import org.firstinspires.ftc.teamcode.Controllers.SixServoArm.SixServoArmEasyAction;
@Config
@TeleOp(name = "SixServoArmTest",group = "Test")
public class SixServoArmTest extends LinearOpMode {
    SixServoArmEasyController sixServoArmController;
    public static double TGX = 100;
    public static double TGY = 100;
    void initHardwareMap() {
        // 初始化硬件映射
        // 这里可以添加初始化代码，例如获取硬件组件等
        sixServoArmController  =SixServoArmEasyController.getInstance(hardwareMap, telemetry);
        //sixServoArmController.initArm();


    }
    @Override
    public void runOpMode() throws InterruptedException {
        initHardwareMap();
        waitForStart();
        while (opModeIsActive()) {
            sixServoArmController.setTargetPosition(TGX, TGY,
                    Math.PI, 0.5 * Math.PI);
            sixServoArmController.update();
            sleep(200);
        }
    }
}
