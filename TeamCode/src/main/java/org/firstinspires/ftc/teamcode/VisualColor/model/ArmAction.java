package org.firstinspires.ftc.teamcode.VisualColor.model;

public class ArmAction {
    public double ArmAngle;
    public double ClipAngle;
    public double length;
    public double GoToX;
    public double GoToY;
    public int suggestion;
    //1: 需要车辆左移
    //2: 需要车辆右移
    //3: 需要滑轨前移
    public ArmAction(double armAngle, double clipAngle, double length, double goToX, double goToY, int suggestion) {
        this.ArmAngle = armAngle;
        this.ClipAngle = clipAngle;
        this.length = length;
        this.GoToX = goToX;
        this.GoToY = goToY;
        this.suggestion = suggestion; // 默认不需要移动
    }
}
