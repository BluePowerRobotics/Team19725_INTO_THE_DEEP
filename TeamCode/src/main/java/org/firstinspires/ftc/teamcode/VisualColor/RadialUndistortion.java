package org.firstinspires.ftc.teamcode.VisualColor;


import org.opencv.core.Point;

public class RadialUndistortion {


    // ==== 输入参数（可修改） ====
    double Xdr;// ▶ Xdr：畸变后的图像 X 坐标
    double Ydr; // ▶ Ydr：畸变后的图像 Y 坐标





    // 反畸变主函数，返回矫正后的理想坐标
    public static Point undistortPoint(double Xdr, double Ydr) {
        double R2 = Xdr * Xdr + Ydr * Ydr;
        double alpha = 1.0; // 初始猜测值
        double tol = 1e-10;
        int maxIter = 100;
        double k1 = -0.0864112; // ▶ k1：径向畸变系数第1项
        double k2 = 0.09922028; // ▶ k2：径向畸变系数第2项
        double k3 = -0.04736412; // ▶ k3：径向畸变系数第3项
        for (int i = 0; i < maxIter; i++) {
            double r2 = alpha * alpha * R2;
            double distortion = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
            double f = 1.0 / alpha - distortion;

            // 计算导数 f'(alpha)
            double dDistortion_dAlpha = 2 * alpha * R2 * (k1 + 2 * k2 * r2 + 3 * k3 * r2 * r2);
            double df = -1.0 / (alpha * alpha) - dDistortion_dAlpha;

            double delta = -f / df;
            alpha += delta;

            if (Math.abs(delta) < tol) break;
        }

        double x = alpha * Xdr;
        double y = alpha * Ydr;
        return new Point(x, y);
    }
}

