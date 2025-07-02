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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
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

/*
 * This OpMode illustrates how to use a video source (camera) to locate specifically colored regions
 *
 * Unlike a "color sensor" which determines the color of an object in the field of view, this "color locator"
 * will search the Region Of Interest (ROI) in a camera image, and find any "blobs" of color that match the requested color range.
 * These blobs can be further filtered and sorted to find the one most likely to be the item the user is looking for.
 *
 * To perform this function, a VisionPortal runs a ColorBlobLocatorProcessor process.
 *   The ColorBlobLocatorProcessor process is created first, and then the VisionPortal is built to use this process.
 *   The ColorBlobLocatorProcessor analyses the ROI and locates pixels that match the ColorRange to form a "mask".
 *   The matching pixels are then collected into contiguous "blobs" of pixels.  The outer boundaries of these blobs are called its "contour".
 *   For each blob, the process then creates the smallest possible rectangle "boxFit" that will fully encase the contour.
 *   The user can then call getBlobs() to retrieve the list of Blobs, where each Blob contains the contour and the boxFit data.
 *   Note: The default sort order for Blobs is ContourArea, in descending order, so the biggest contours are listed first.
 *
 * To aid the user, a colored boxFit rectangle is drawn on the camera preview to show the location of each Blob
 * The original Blob contour can also be added to the preview.  This is helpful when configuring the ColorBlobLocatorProcessor parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Config
@TeleOp(name = "Find_Candidate_FINISH", group = "Test")
public class FindCandidate extends LinearOpMode
{

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








    int CandidateLength = 0;

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
    @Override
    public void runOpMode()
    {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        CameraStreamProcessor processor = new CameraStreamProcessor();
        processor.drawXYlines(new Mat(), 50, new Scalar(255, 0, 0), 1); // Draw grid lines on the camera preview




        final ColorRange BLUE = new ColorRange(
                ColorSpace.YCrCb,
                new Scalar( 26,   10, 165),
                new Scalar(255, 127, 255)
        );

        CubeInfo[] candidates = new CubeInfo[1000];


        /* Build a "Color Locator" vision processor based on the ColorBlobLocatorProcessor class.
         * - Specify the color range you are looking for.  You can use a predefined color, or create you own color range
         *     .setTargetColorRange(ColorRange.BLUE)                      // use a predefined color match
         *       Available predefined colors are: RED, BLUE YELLOW GREEN
         *     .setTargetColorRange(new ColorRange(ColorSpace.YCrCb,      // or define your own color match
         *                                           new Scalar( 32, 176,  0),
         *                                           new Scalar(255, 255, 132)))
         *
         * - Focus the color locator by defining a RegionOfInterest (ROI) which you want to search.
         *     This can be the entire frame, or a sub-region defined using:
         *     1) standard image coordinates or 2) a normalized +/- 1.0 coordinate system.
         *     Use one form of the ImageRegion class to define the ROI.
         *         ImageRegion.entireFrame()
         *         ImageRegion.asImageCoordinates(50, 50,  150, 150)  100x100 pixel square near the upper left corner
         *         ImageRegion.asUnityCenterCoordinates(-0.5, 0.5, 0.5, -0.5)  50% width/height square centered on screen
         *
         * - Define which contours are included.
         *     You can get ALL the contours, or you can skip any contours that are completely inside another contour.
         *        .setContourMode(ColorBlobLocatorProcessor.ContourMode.ALL_FLATTENED_HIERARCHY)  // return all contours
         *        .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)            // exclude contours inside other contours
         *        note: EXTERNAL_ONLY helps to avoid bright reflection spots from breaking up areas of solid color.
         *
         * - turn the display of contours ON or OFF.  Turning this on helps debugging but takes up valuable CPU time.
         *        .setDrawContours(true)
         *
         * - include any pre-processing of the image or mask before looking for Blobs.
         *     There are some extra processing you can include to improve the formation of blobs.  Using these features requires
         *     an understanding of how they may effect the final blobs.  The "pixels" argument sets the NxN kernel size.
         *        .setBlurSize(int pixels)    Blurring an image helps to provide a smooth color transition between objects, and smoother contours.
         *                                    The higher the number of pixels, the more blurred the image becomes.
         *                                    Note:  Even "pixels" values will be incremented to satisfy the "odd number" requirement.
         *                                    Blurring too much may hide smaller features.  A "pixels" size of 5 is good for a 320x240 image.
         *        .setErodeSize(int pixels)   Erosion removes floating pixels and thin lines so that only substantive objects remain.
         *                                    Erosion can grow holes inside regions, and also shrink objects.
         *                                    "pixels" in the range of 2-4 are suitable for low res images.
         *        .setDilateSize(int pixels)  Dilation makes objects more visible by filling in small holes, making lines appear thicker,
         *                                    and making filled shapes appear larger. Dilation is useful for joining broken parts of an
         *                                    object, such as when removing noise from an image.
         *                                    "pixels" in the range of 2-4 are suitable for low res images.
         */
        ColorBlobLocatorProcessor colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(BLUE)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))  // search central 1/4 of camera view
                .setDrawContours(true)// Show contours on the Stream Preview
                .setBlurSize(blurSize)// Smooth the transitions between different colors in image
                .setErodeSize(erodeSize)
                .setDilateSize(dilateSize)
                .build();
//        ColorBlobLocatorProcessor colorLocatorGreen = new ColorBlobLocatorProcessor.Builder()
//                .setTargetColorRange(ColorRange.GREEN)         // use a predefined color match
//                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
//                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))  // search central 1/4 of camera view
//                .setDrawContours(true)// Show contours on the Stream Preview
//                .build();

        /*
         * Build a vision portal to run the Color Locator process.
         *
         *  - Add the colorLocator process created above.
         *  - Set the desired video resolution.
         *      Since a high resolution will not improve this process, choose a lower resolution that is
         *      supported by your camera.  This will improve overall performance and reduce latency.
         *  - Choose your video source.  This may be
         *      .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))  .....   for a webcam
         *  or
         *      .setCamera(BuiltinCameraDirection.BACK)    ... for a Phone Camera
         */
        VisionPortal portal = new VisionPortal.Builder()
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(colorLocator)
                //.addProcessor(colorLocatorGreen)
                .addProcessor(processor)
                .setCameraResolution(new Size(resolutionwidth, resolutionheight))

                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        telemetry.setMsTransmissionInterval(50);   // Speed up telemetry updates, Just use for debugging.
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        // WARNING:  To be able to view the stream preview on the Driver Station, this code runs in INIT mode.
        while (opModeIsActive() || opModeInInit())
        {
            telemetry.addData("preview on/off", "... Camera Stream\n");

            // Read the current list
            List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();

            /*
             * The list of Blobs can be filtered to remove unwanted Blobs.
             *   Note:  All contours will be still displayed on the Stream Preview, but only those that satisfy the filter
             *          conditions will remain in the current list of "blobs".  Multiple filters may be used.
             *
             * Use any of the following filters.
             *
             * ColorBlobLocatorProcessor.Util.filterByArea(minArea, maxArea, blobs);
             *   A Blob's area is the number of pixels contained within the Contour.  Filter out any that are too big or small.
             *   Start with a large range and then refine the range based on the likely size of the desired object in the viewfinder.
             *
             * ColorBlobLocatorProcessor.Util.filterByDensity(minDensity, maxDensity, blobs);
             *   A blob's density is an indication of how "full" the contour is.
             *   If you put a rubber band around the contour you would get the "Convex Hull" of the contour.
             *   The density is the ratio of Contour-area to Convex Hull-area.
             *
             * ColorBlobLocatorProcessor.Util.filterByAspectRatio(minAspect, maxAspect, blobs);
             *   A blob's Aspect ratio is the ratio of boxFit long side to short side.
             *   A perfect Square has an aspect ratio of 1.  All others are > 1
             */

            //todo:change this range when reset resolution-----zhz
            ColorBlobLocatorProcessor.Util.filterByArea(minarea, maxarea, blobs);  // filter out very small blobs.

            /*
             * The list of Blobs can be sorted using the same Blob attributes as listed above.
             * No more than one sort call should be made.  Sorting can use ascending or descending order.
             *     ColorBlobLocatorProcessor.Util.sortByArea(SortOrder.DESCENDING, blobs);      // Default
             *     ColorBlobLocatorProcessor.Util.sortByDensity(SortOrder.DESCENDING, blobs);
             *     ColorBlobLocatorProcessor.Util.sortByAspectRatio(SortOrder.DESCENDING, blobs);
             */
            telemetry.addData("Blur", blurSize);
            telemetry.addData("Erode", erodeSize);
            telemetry.addData("Dilate", dilateSize);
            telemetry.addData("FPS_Camera", portal.getFps());
            telemetry.addData("State_Camera", portal.getCameraState().toString());
            if(!blobs.isEmpty()){

                RotatedRect current_running_to =  blobs.get(0).getBoxFit();
                telemetry.addLine(" Area Density Aspect  Center");
                telemetry.addData("running to x:", current_running_to.center.x / resolutionwidth);
                telemetry.addData("running to y:", current_running_to.center.y / resolutionheight);
                telemetry.addData("running to angle:", current_running_to.angle);
                telemetry.addData("running to (size):", current_running_to.size.toString());
            }

            // Display the size (area) and center location for each Blob.
            int i = 0;
            for(ColorBlobLocatorProcessor.Blob b : blobs)
            {

                RotatedRect boxFit = b.getBoxFit();
                Point tmpp = new Point((boxFit.center.x - MidX) * PixeltoMM, (boxFit.center.y - MidY) * PixeltoMM);
                telemetry.addData("pre_candidate" + i+ 1, tmpp.toString());
                CubeInfo cubeInfo = new CubeInfo(
                        i + 1,
                        //new Point(0,0),
                        boxFit.center,
                        //OpenCvUndistortion.openCvUndistortion(boxFit.center.x - MidX, boxFit.center.y - MidY),
                        boxFit.size.area(),
                        b.getDensity(),
                        boxFit.angle,
                        boxFit.size.width,
                        boxFit.size.height
                );

                candidates[i] = cubeInfo;
                i++;
//                telemetry.addData("", boxFit.size.area());
//                telemetry.addLine(String.format("%5d  %4.2f   %5.2f  (%3d,%3d)",
//                          b.getContourArea(), b.getDensity(), b.getAspectRatio(), (int) boxFit.center.x, (int) boxFit.center.y));
            }



            CandidateLength = i;
            Arrays.sort(candidates, (p1, p2) -> {
                if (p1 == null || p2 == null) {
                    return 0; // Handle null cases
                }
                if (Math.abs(p1.DisToCamInMM - p2.DisToCamInMM) >= 0.0001) {        //距离差大于5MM则按距离排序
                    if(p2.DisToCamInMM - p1.DisToCamInMM < 0) {
                        return 1;
                    }
                    return -1;
                }
                return (int)(p1.size - p2.size);                 //否则按面积排序
            });
            //按距离和面积递增

            telemetry.addData("length:", CandidateLength);
            for(int j = 0; j < candidates.length&&candidates[j] != null; j++){



                candidates[j].centerpoint = OpenCvUndistortion.openCvUndistortion(candidates[j].centerpoint.x - MidX, candidates[j].centerpoint.y - MidY);
                candidates[j].centerpoint = new Point(candidates[j].centerpoint.x * PixeltoMM, candidates[j].centerpoint.y * PixeltoMM);
                int Status = CubeProcessor.ProcessCube(candidates[j]);
                telemetry.addData("Candidate " + j, "index: %d, Size: %.2f, Density: %.2f, Angle: %.2f, Center: (%.2f, %.2f), Distance: %.2f mm",
                        candidates[j].index,
                        candidates[j].size, candidates[j].density, candidates[j].angleDeg,
                        candidates[j].centerpoint.x, candidates[j].centerpoint.y, candidates[j].DisToCamInMM);
                //candidates[j].centerpoint = new Point((candidates[j].centerpoint.x - MidX) * PixeltoMM, (candidates[j].centerpoint.y - MidY) * PixeltoMM);
                //telemetry.addData("New Center " + j + " (mm)", "Center: (%.2f, %.2f)", candidates[j].centerpoint.x, candidates[j].centerpoint.y);
            }

            //int tmp = 1/0; // 触发异常，测试异常处理
            FtcDashboard.getInstance().startCameraStream(processor, 0);
            telemetry.update();
            //sleep(50);
        }
    }
}
