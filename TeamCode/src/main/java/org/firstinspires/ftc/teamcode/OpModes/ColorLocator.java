package org.firstinspires.ftc.teamcode.OpModes;
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
//12-19已修改

import android.util.Size;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.opencv.core.RotatedRect;

import java.util.List;

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

public class ColorLocator{
    public double bluex;
    public double bluey;
    public ColorBlobLocatorProcessor colorLocator;
    public VisionPortal portal;




    //////////
    public ColorLocator(WebcamName cam1, boolean ifblue){
        if(ifblue){
            colorLocator = new ColorBlobLocatorProcessor.Builder()
                    .setTargetColorRange(ColorRange.BLUE)         // use a predefined color match
                    .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                    .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))  // ROI:全屏
                    .setDrawContours(true)                        // Show contours on the Stream Preview
                    .setBlurSize(5)                               // Smooth the transitions between different colors in image
                    .build();
        }
        else{
            colorLocator = new ColorBlobLocatorProcessor.Builder()
                    .setTargetColorRange(ColorRange.RED)
                    .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                    .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))
                    .setDrawContours(true)
                    .setBlurSize(5)
                    .build();
        }
        portal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                .setCameraResolution(new Size(320, 240))
                .setCamera(cam1)
                .build();


    }
    ////////




    public ColorReturn LocateAll()
    {
        List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();
        ColorBlobLocatorProcessor.Util.filterByArea(50, 20000, blobs);
        org.opencv.core.Size myBoxFitSize;
        for(ColorBlobLocatorProcessor.Blob b : blobs){
            RotatedRect boxFit = b.getBoxFit();
            myBoxFitSize = boxFit.size;
            ColorReturn LocateAll = new ColorReturn(boxFit.center.x,boxFit.center.y,myBoxFitSize.width,myBoxFitSize.height,boxFit.angle);
            //telemetry.addData("x:",boxFit.center.x);
            //telemetry.addData("y:",boxFit.center.y);
            //telemetry.addData("######################################");
            //telemetry.update();
            return (LocateAll);
        }
        //
        //telemetry.update();
        ColorReturn LocateError = new ColorReturn(999999,999999,999999,999999,999999);
        return(LocateError);
        //sleep(50);
    }
}
