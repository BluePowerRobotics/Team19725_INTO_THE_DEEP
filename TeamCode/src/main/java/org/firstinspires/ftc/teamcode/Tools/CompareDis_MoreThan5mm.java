package org.firstinspires.ftc.teamcode.Tools;

public class CompareDis_MoreThan5mm {
    public boolean compare(double a, double b) {
        if (Math.abs(a - b) >= 5.0) {
            return a < b;
        }
    }
}
