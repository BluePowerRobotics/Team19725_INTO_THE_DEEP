package org.firstinspires.ftc.teamcode.VisualColor.model;

import org.opencv.core.Point;

public class CubeInfo {
    Point centerpoint;
    public final double size;
    public final double density;
    public final double angleDeg;
    public final double rectWidthPx;
    public final double rectHeightPx;
    public  final double DisToCamInMM;

    public CubeInfo(Point centerpoint, double size, double density, double angle, double widthPx, double heightPx) {
        this.centerpoint = centerpoint;
        this.size = size;
        this.density = density;
        this.angleDeg = angle;
        this.rectWidthPx = widthPx;
        this.rectHeightPx = heightPx;
        this.DisToCamInMM = Math.sqrt(centerpoint.x * centerpoint.x + centerpoint.y * centerpoint.y);
    }
}