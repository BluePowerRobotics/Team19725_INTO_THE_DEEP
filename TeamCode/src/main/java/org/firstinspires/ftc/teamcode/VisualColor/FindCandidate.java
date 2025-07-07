/*
 * Copyright (c) 2024 Phil Malone
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.VisualColor;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.VisualColor.model.ArmAction;
import org.firstinspires.ftc.teamcode.VisualColor.model.CubeInfo;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;


import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

@Config
public class FindCandidate{

    public static double MMtoPixel = 1.486;
    public static double PixeltoMM = 1 / MMtoPixel;

    public static int MidX = 400;
    public static int MidY = 300;
    public static  int blurSize = 10;
    public static  int erodeSize = 7;
    public static  int dilateSize = 12;
    public static int resolutionwidth = 800;
    public static int resolutionheight= 600;

    public static  int minarea = 2000;
    public static  int maxarea = 20000;
    public static  int BR = 51;
    public static  int BG = 35;
    public static  int BB = 190;

    public CubeInfo[] InsideCandidates = new CubeInfo[100];
    public int InsideCnt = 0;
    public CubeInfo[] LeftCandidates = new CubeInfo[100];
    public int LeftCnt = 0;
    public CubeInfo[] RightCandidates = new CubeInfo[100];
    public int RightCnt = 0;
    public CubeInfo[] FrontCandidates = new CubeInfo[100];
    public int FrontCnt = 0;
    final ColorRange BLUE = new ColorRange(
            ColorSpace.YCrCb,
            new Scalar( 26,   10, 165),
            new Scalar(255, 127, 255)
    );
    public CubeInfo[] candidates = new CubeInfo[1000];

    int CandidateLength = 0;

    Telemetry Mytelemetry;
    CameraStreamProcessor processor = new CameraStreamProcessor();
    ColorBlobLocatorProcessor colorLocator;
    VisionPortal portal;

    public void init(HardwareMap hardWareMap, Telemetry telerc) {
        Mytelemetry = telerc;
        //telemetry = new MultipleTelemetry(telerc, FtcDashboard.getInstance().getTelemetry());
        colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(BLUE)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))  // search central 1/4 of camera view
                .setDrawContours(true)// Show contours on the Stream Preview
                .setBlurSize(blurSize)// Smooth the transitions between different colors in image
                .setErodeSize(erodeSize)
                .setDilateSize(dilateSize)
                .build();
       portal = new VisionPortal.Builder()
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(colorLocator)
                //.addProcessor(colorLocatorGreen)
                .addProcessor(processor)
                .setCameraResolution(new Size(resolutionwidth, resolutionheight))

                .setCamera(hardWareMap.get(WebcamName.class, "Webcam 1"))
                .build();
    }

    public static class CameraStreamProcessor implements VisionProcessor, CameraStreamSource {
        private final AtomicReference<Bitmap> lastFrame =
                new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            drawXYlines(frame, 100, new Scalar(255, 0, 0), 1); // 在Mat上画网格
            Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
            Utils.matToBitmap(frame, b);
            lastFrame.set(b);
            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                                Object userContext) {

        }

        @Override
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
            continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
        }
        //在Mat上画网格线
        public static void drawXYlines(Mat frame, int size, Scalar color, int thickness) {
            if (size <= 0 || color == null || thickness < 1) return; // 参数检查
            for (int x = 0; x < frame.cols(); x += size) Imgproc.line(frame, new Point(x, 0), new Point(x, frame.rows()), color, thickness);
            for (int y = 0; y < frame.rows(); y += size) Imgproc.line(frame, new Point(0, y), new Point(frame.cols(), y), color, thickness);
        }
    }

    public ArmAction findCandidate()
    {
        processor.drawXYlines(new Mat(), 50, new Scalar(255, 0, 0), 1); // Draw grid lines on the camera preview
        Mytelemetry.setMsTransmissionInterval(50);   // Speed up telemetry updates, Just use for debugging.
        Mytelemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

            // Read the current list
        List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();



        //todo:change this range when reset resolution-----zhz
        ColorBlobLocatorProcessor.Util.filterByArea(minarea, maxarea, blobs);  // filter out very small blobs.


        Mytelemetry.addData("Blur", blurSize);
        Mytelemetry.addData("Erode", erodeSize);
        Mytelemetry.addData("Dilate", dilateSize);
        Mytelemetry.addData("FPS_Camera", portal.getFps());
        Mytelemetry.addData("State_Camera", portal.getCameraState().toString());

        // Display the size (area) and center location for each Blob.
        int i = 0;
        for(ColorBlobLocatorProcessor.Blob b : blobs) {

            RotatedRect boxFit = b.getBoxFit();
            CubeInfo cubeInfo = new CubeInfo(
                    i + 1,
                    boxFit.center,
                    boxFit.size.area(),
                    b.getDensity(),
                    boxFit.angle,
                    boxFit.size.width,
                    boxFit.size.height
            );

            candidates[i] = cubeInfo;
            i++;
        }

        CandidateLength = i;
        Arrays.sort(candidates, (p1, p2) -> {
            if (p1 == null || p2 == null) {
                return 0; // Handle null cases
            }
            if (Math.abs(p1.DisToCamInMM - p2.DisToCamInMM) >= 0.0001) {        //距离差大于5MM则按距离排序
                if(p2.DisToCamInMM - p1.DisToCamInMM < 0) {
                    return -1;
                }
                return 1;
            }
            return (int)(p2.size - p1.size);                 //否则按面积排序
        });
            //按距离和面积递增

        Mytelemetry.addData("length:", CandidateLength);
        for(int j = 0; j < CandidateLength; j++){
            candidates[j].centerpoint = OpenCvUndistortion.openCvUndistortion(candidates[j].centerpoint.x - MidX, candidates[j].centerpoint.y - MidY);
            candidates[j].centerpoint = new Point(candidates[j].centerpoint.x * PixeltoMM, candidates[j].centerpoint.y * PixeltoMM);
            int Status = CubeProcessor.ProcessCube(candidates[j]);
            if(Status == 0){
                InsideCandidates[j] = candidates[j];
                InsideCnt++;
                Mytelemetry.addData("Candidate " + j, "(inside) index: %d, Size: %.2f, Density: %.2f, Angle: %.2f, Center: (%.2f, %.2f), Distance: %.2f mm",
                        candidates[j].index,
                        candidates[j].size, candidates[j].density, candidates[j].angleDeg,
                        candidates[j].centerpoint.x, candidates[j].centerpoint.y, candidates[j].DisToCamInMM);
            }
            else if(Status == 1){
                LeftCandidates[j] = candidates[j];
                LeftCnt++;
            }
            else if(Status == 2){
                RightCandidates[j] = candidates[j];
                RightCnt++;
            }
            else if(Status == 3){
                FrontCandidates[j] = candidates[j];
                FrontCnt++;
            }
            else{
                //int a = 1/0; // 触发异常，测试异常处理
                continue;
            }
        }


//        if(InsideCnt > 0 && !(InsideCandidates[0] == null)) {
//            Mytelemetry.addData("Running to", "index: %d, Size: %.2f, Density: %.2f, Angle: %.2f, Center: (%.2f, %.2f), Distance: %.2f mm",
//                    InsideCandidates[0].index,
//                    InsideCandidates[0].size, InsideCandidates[0].density, InsideCandidates[0].angleDeg,
//                    InsideCandidates[0].centerpoint.x, InsideCandidates[0].centerpoint.y, InsideCandidates[0].DisToCamInMM);
//        }


        int Suggestion = 0;
        if(LeftCnt > RightCnt){
            Suggestion = 1; // 左移
        }
        else if(RightCnt > LeftCnt){
            Suggestion = 2; // 右移
        }
        else{
            Suggestion = 0; // 不移动
        }
        Mytelemetry.addData("not inside","left: %d Right: %d", LeftCnt, RightCnt);

        InsideCnt = 0;
        LeftCnt = 0;
        RightCnt = 0;
        FtcDashboard.getInstance().startCameraStream(processor, 0);
        //telemetry.update();       已在主程序中统一更新
        if(!(InsideCandidates[0] == null)){
            return new ArmAction(Math.atan((InsideCandidates[0].centerpoint.x) / (InsideCandidates[0].centerpoint.y) * 180 / Math.PI),InsideCandidates[0].angleDeg, InsideCandidates[0].DisToCamInMM, InsideCandidates[0].centerpoint.x, InsideCandidates[0].centerpoint.y, Suggestion);
        }
        return new ArmAction(-1,-1,-1,-1,-1,-1);
    }
}
