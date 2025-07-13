package org.firstinspires.ftc.teamcode.VisualColor;

import android.util.Pair;

import org.firstinspires.ftc.teamcode.VisualColor.model.CubeInfo;
import org.opencv.core.Point;

import java.util.Arrays;

public class CubeProcessor {



    //todo 视摄像头方向调整这里和下面的正负、大于小于
    static int ellipseX = 264; // 不可夹取区椭圆中心点X坐标
    static int ellipseY = 149; // 不可夹取区椭圆中心点Y坐标
    static int ellipseA = 194; // 不可夹取区椭圆半长轴
    static int ellipseB = 134; // 不可夹取区椭圆半短轴

    static int XTolerance = 264; // 可夹取区X轴max
    static int YTolerance = 149; // 可夹取区Y轴max

    boolean ifInArea = false;



    //return:     -1  不符合要求
    //            0: 在可夹取区内
    //            1: 需要车辆左移
    //            2: 需要车辆右移
    //            3: 需要滑轨前移

    public static int ProcessCube(CubeInfo cubeInfo) {
        double X = cubeInfo.centerpoint.x;
        double Y = cubeInfo.centerpoint.y;
        if(Y >= 10){
            return -1;
        }
        double m1 = (Math.abs(X) - ellipseX) * (Math.abs(X) - ellipseX) / (ellipseA * ellipseA);
        double m2 = (Math.abs(Y) - ellipseY) * (Math.abs(Y) - ellipseY) / (ellipseB * ellipseB);
        if (m1 + m2 > 1 && Math.abs(X) < XTolerance && Math.abs(Y) < YTolerance) {
            // 在可夹取区内
            boolean ifInArea = true;
            return 0;
        }
        if(X < 0){
            return 1; // 需要车辆左移
        }
        else{
            return 2; // 需要车辆右移
        }
    }

    public static double CalculateRadians(Point[] points) {
        int MinIndex = 0; // 最小点的索引
        double minY = 10000; // 初始化一个很大的值作为最小Y坐标
        for(int i = 0; i < points.length; i++) {
            if(points[i].y < minY) {
                minY = points[i].y;
                MinIndex = i; // 更新最小点的索引
            }
        }
        Pair<Integer, Double>[] Index_Dis = new Pair[3];
        int cnt = 0;
        for(int i = 0; i < points.length; i++) {
            if(i == MinIndex) continue; // 跳过最小点
            double Distance = Math.sqrt(Math.pow(points[i].x - points[MinIndex].x, 2) + Math.pow(points[i].y - points[MinIndex].y, 2));
            Index_Dis[cnt] = new Pair<>(i, Distance);
            cnt++;
        }

        Arrays.sort(Index_Dis, (a, b) -> Double.compare(a.second, b.second));
        //取出短边顶点与Y最小点，计算斜率
        double deltax = points[Index_Dis[0].first].x - points[MinIndex].x;
        double deltay = points[Index_Dis[0].first].y - points[MinIndex].y;
        double ClipRadians = Math.atan2(deltay, deltax);

        return ClipRadians;
    }

}
