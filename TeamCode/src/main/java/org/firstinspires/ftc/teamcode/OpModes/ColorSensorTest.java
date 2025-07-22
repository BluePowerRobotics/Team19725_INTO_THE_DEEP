package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import org.firstinspires.ftc.teamcode.Vision.ColorSensorDetector;
@TeleOp(name = "Color Sensor Test")
public class ColorSensorTest extends LinearOpMode {
    private ColorSensorDetector colorDetector;

    @Override
    public void runOpMode() {
        // 初始化传感器
        NormalizedColorSensor sensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        colorDetector = new ColorSensorDetector(sensor);

        waitForStart();

        while (opModeIsActive()) {
            // 获取检测到的颜色
            int color = colorDetector.getDetectedColor();

            // 获取HSV值（可选）
            float[] hsv = colorDetector.getHSVValues();

            // 输出结果
            telemetry.addData("Detected Color", color);
            telemetry.addData("Hue", hsv[0]);
            telemetry.addData("Saturation", hsv[1]);
            telemetry.addData("Value", hsv[2]);
            telemetry.update();
        }
    }
}