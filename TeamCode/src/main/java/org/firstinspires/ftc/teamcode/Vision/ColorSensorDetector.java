package org.firstinspires.ftc.teamcode.Vision;

import android.graphics.Color;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public class ColorSensorDetector {
    private NormalizedColorSensor colorSensor;

    public ColorSensorDetector(NormalizedColorSensor sensor) {
        this.colorSensor = sensor;
    }

    public int getDetectedColor() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        float[] hsvValues = new float[3];
        Color.colorToHSV(colors.toColor(), hsvValues);

        double h = hsvValues[0];
        double s = hsvValues[1];
        double v = hsvValues[2];

        if (s <= 0.35 || v <= 0.1 || v >= 0.7) {
            return 0; // 灰色
        } else if (h >= 20 && h < 80) {
            return 2; // 黄色
        } else if (h >= 180 && h < 280) {
            return 3; // 蓝色
        } else if (h >= 280 || h < 20) {
            return 1; // 红色
        }
        return -1; // 未知
    }

    public float[] getHSVValues() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        float[] hsvValues = new float[3];
        Color.colorToHSV(colors.toColor(), hsvValues);
        return hsvValues;
    }
}