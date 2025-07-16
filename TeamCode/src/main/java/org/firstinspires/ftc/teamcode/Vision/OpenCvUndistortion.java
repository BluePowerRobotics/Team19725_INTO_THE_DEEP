package org.firstinspires.ftc.teamcode.Vision;

import org.opencv.core.*;
import org.opencv.calib3d.Calib3d;

public class OpenCvUndistortion{
//    static {
//        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
//    }
    Mat cameraMatrix = null;
    Mat distCoeffs = null;

    public OpenCvUndistortion(){
        // 1. 定义相机内参矩阵 (3x3)
        Mat cameraMatrix = new Mat(3, 3, CvType.CV_64F);
        cameraMatrix.put(0, 0, 367); // fx
        cameraMatrix.put(0, 1, 0);
        cameraMatrix.put(0, 2, 400); // cx
        cameraMatrix.put(1, 0, 0);
        cameraMatrix.put(1, 1, 330); // fy
        cameraMatrix.put(1, 2, 300); // cy
        cameraMatrix.put(2, 0, 0);
        cameraMatrix.put(2, 1, 0);
        cameraMatrix.put(2, 2, 1);

        // 2. 定义畸变系数 [k1, k2, p1, p2, k3]
        Mat distCoeffs = new Mat(1, 5, CvType.CV_64F);
        distCoeffs.put(0, 0, -0.0864112);  // k1
        distCoeffs.put(0, 1, 0.09922028);   // k2
        distCoeffs.put(0, 2, 0);      // p1 (切向畸变)
        distCoeffs.put(0, 3, 0);      // p2 (切向畸变)
        distCoeffs.put(0, 4, -0.04736412);      // k3
    }

    public Point openCvUndistortion(double PreX, double PreY){


        // 3. 准备输入点（畸变点坐标）
        Point distortedPoint = new Point((int)PreX, (int)PreY); // 畸变点
        MatOfPoint2f srcPoints = new MatOfPoint2f(distortedPoint);
        MatOfPoint2f dstPoints = new MatOfPoint2f();

            // 4. 执行畸变校正

        Mat R = Mat.eye(3, 3, CvType.CV_64F);
            Calib3d.undistortPoints(
                    srcPoints,
                    dstPoints,
                    cameraMatrix,
                    distCoeffs,
                    R,   // 用单位矩阵表示不旋转
                    cameraMatrix // 使用原始内参输出到图像坐标
            );
            // 5. 获取校正后的点
        return dstPoints.toArray()[0];


    }
}