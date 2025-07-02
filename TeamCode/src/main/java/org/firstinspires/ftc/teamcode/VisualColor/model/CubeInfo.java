package org.firstinspires.ftc.teamcode.VisualColor.model;

import org.opencv.core.Point;

public class CubeInfo {
    public int index;
    public Point centerpoint;
    public final double size;
    public final double density;
    public final double angleDeg;
    public final double rectWidthPx;
    public final double rectHeightPx;
    public  final double DisToCamInMM;

    public CubeInfo(int index, Point centerpoint, double size, double density, double angle, double widthPx, double heightPx) {
        this.index = index;
        this.centerpoint = centerpoint;
        this.size = size;
        this.density = density;
        this.angleDeg = angle;
        this.rectWidthPx = widthPx;
        this.rectHeightPx = heightPx;
        this.DisToCamInMM = Math.sqrt(centerpoint.x * centerpoint.x + centerpoint.y * centerpoint.y);
    }
}