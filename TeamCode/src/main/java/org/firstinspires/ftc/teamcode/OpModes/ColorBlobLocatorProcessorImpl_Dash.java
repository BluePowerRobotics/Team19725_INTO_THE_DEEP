package org.firstinspires.ftc.teamcode.OpModes;
import android.graphics.Canvas;
import android.graphics.Paint;

import androidx.annotation.ColorInt;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

class ColorBlobLocatorProcessorImpl_Dash extends ColorBlobLocatorProcessor implements VisionProcessor {
    private ColorRange colorRange;
    private ImageRegion roiImg;
    private Rect roi;
    private int frameWidth;
    private int frameHeight;
    private Mat roiMat;
    private Mat roiMat_userColorSpace;
    private final int contourCode;

    private Mat mask = new Mat();

    private final Paint boundingRectPaint;
    private final Paint roiPaint;
    private final Paint contourPaint;
    private final boolean drawContours;
    private final @ColorInt int boundingBoxColor;
    private final @ColorInt int roiColor;
    private final @ColorInt int contourColor;

    private final Mat erodeElement;
    private final Mat dilateElement;
    private final Size blurElement;

    private final Object lockFilters = new Object();
    private final List<BlobFilter> filters = new ArrayList<>();
    private volatile BlobSort sort;

    private volatile ArrayList<Blob> userBlobs = new ArrayList<>();

    ColorBlobLocatorProcessorImpl_Dash(ColorRange colorRange, ImageRegion roiImg,
                                       //ContourMode contourMode,
                                       int erodeSize, int dilateSize, boolean drawContours, int blurSize,
                                       @ColorInt int boundingBoxColor, @ColorInt int roiColor, @ColorInt int contourColor) {
        this.colorRange = colorRange;
        this.roiImg = roiImg;
        this.drawContours = drawContours;
        this.boundingBoxColor = boundingBoxColor;
        this.roiColor = roiColor;
        this.contourColor = contourColor;

        if (blurSize > 0) {
            // enforce Odd blurSize
            blurElement = new Size(blurSize | 0x01, blurSize | 0x01);
        } else {
            blurElement = null;
        }

//        if (contourMode == ContourMode.EXTERNAL_ONLY) {
//            contourCode = Imgproc.RETR_EXTERNAL;
//        } else {
//            contourCode = Imgproc.RETR_LIST;
//        }
        contourCode = Imgproc.RETR_EXTERNAL;
        if (erodeSize > 0) {
            erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(erodeSize, erodeSize));
        } else {
            erodeElement = null;
        }

        if (dilateSize > 0) {
            dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(dilateSize, dilateSize));
        } else {
            dilateElement = null;
        }

        boundingRectPaint = new Paint();
        boundingRectPaint.setAntiAlias(true);
        boundingRectPaint.setStrokeCap(Paint.Cap.BUTT);
        boundingRectPaint.setColor(boundingBoxColor);

        roiPaint = new Paint();
        roiPaint.setAntiAlias(true);
        roiPaint.setStrokeCap(Paint.Cap.BUTT);
        roiPaint.setColor(roiColor);

        contourPaint = new Paint();
        contourPaint.setStyle(Paint.Style.STROKE);
        contourPaint.setColor(contourColor);
    }

    @Override
    public void addFilter(BlobFilter filter) {

    }

    @Override
    public void removeFilter(BlobFilter filter) {

    }

    @Override
    public void removeAllFilters() {

    }

    @Override
    public void setSort(BlobSort sort) {

    }

    @Override
    public List<Blob> getBlobs() {
        return Collections.emptyList();
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}
