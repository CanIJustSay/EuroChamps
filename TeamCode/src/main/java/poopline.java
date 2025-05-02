
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class poopline extends OpenCvPipeline {

    public Scalar lowerYCrCb = new Scalar(171.4, 0, 0, 0.0);
    public Scalar upperYCrCb = new Scalar(255.0, 255.0, 255.0, 0.0);
    public double dx = 0;
    public double dy = 0;
    public String orient = "none";


    Mat ycrcb = new Mat();
    Mat mask = new Mat();
    Mat blurred = new Mat();
    Mat morphed = new Mat();
    Mat hierarchy = new Mat();
    List<MatOfPoint> contours = new ArrayList<>();

    @Override
    public Mat processFrame(Mat input) {
        // Blur makes it easier to read so we dont get freaky double box
        Imgproc.GaussianBlur(input, blurred, new Size(5, 5), 0);

        // Convert to YCrCb
        Imgproc.cvtColor(blurred, ycrcb, Imgproc.COLOR_RGB2YCrCb);

        // Threshold
        Core.inRange(ycrcb, lowerYCrCb, upperYCrCb, mask);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(2, 2));
        Imgproc.morphologyEx(mask, morphed, Imgproc.MORPH_CLOSE, kernel);
        Imgproc.morphologyEx(morphed, morphed, Imgproc.MORPH_OPEN, kernel);

        // Find contours
        contours.clear();
        Point frameCenter = new Point(input.cols() / 2.0, input.rows() / 2.0);
        RotatedRect closestRotatedRect = null;
        Point closestSampleCenter = null;
        double closestDistance = Double.MAX_VALUE;
        Imgproc.findContours(morphed, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint contour : contours) {
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
            RotatedRect rotatedRect = Imgproc.minAreaRect(contour2f);
            
            
            double rawAngle = rotatedRect.angle;
            double width = rotatedRect.size.width;
            double height = rotatedRect.size.height;

            double correctedAngle;
            if (width > height) {
                correctedAngle = rawAngle;
            } else {
                correctedAngle = rawAngle + 90;
            }
            correctedAngle = -(correctedAngle - 360) % 180;
            String orientationLabel;
            if (correctedAngle <= 22 || correctedAngle >= 161) {
                orientationLabel = "horizontal";
            } else if (correctedAngle <= 68) {
                orientationLabel = "diagonal right";
            } else if (correctedAngle <= 114) {
                orientationLabel = "vertical";
            } else if (correctedAngle <= 160) {
                orientationLabel = "diagonal left";
            } else {
                orientationLabel = "unknown"; // for oopsie doopsies
            }


            double area = Imgproc.contourArea(contour);
            double aspectRatio = Math.max(rotatedRect.size.width, rotatedRect.size.height) /
                                 Math.min(rotatedRect.size.width, rotatedRect.size.height);

            double dist = Math.hypot(rotatedRect.center.x - frameCenter.x, rotatedRect.center.y - frameCenter.y);
            if (dist < closestDistance) {
                closestDistance = dist;
                closestSampleCenter = rotatedRect.center;
                closestRotatedRect = rotatedRect;
                dx = closestSampleCenter.x - frameCenter.x;
                dy = closestSampleCenter.y - frameCenter.y;
                orient = orientationLabel;
            }
           

            // Loosen area threshold to catch smaller ones
            if (area > 250 && aspectRatio > 0.3 && aspectRatio < 3.0) {
                Point[] rectPoints = new Point[4];
                rotatedRect.points(rectPoints);
                for (int i = 0; i < 4; i++) {
                    Imgproc.line(input, rectPoints[i], rectPoints[(i + 1) % 4], new Scalar(0, 255, 0), 2);
                }

                String label = "blue sample (" + orientationLabel + ")";
                Imgproc.putText(input, label, rotatedRect.center,
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255), 1);


            }
        }
        if (closestSampleCenter != null && closestRotatedRect != null) {
            Imgproc.line(input, frameCenter, closestSampleCenter, new Scalar(0, 0, 255), 2);
            Imgproc.circle(input, frameCenter, 5, new Scalar(255, 255, 255), -1);
        
            // Displacement
            double dx = closestSampleCenter.x - frameCenter.x;
            double dy = closestSampleCenter.y - frameCenter.y;
        
            // Orientation classification
            double rawAngle = closestRotatedRect.angle;
            double width = closestRotatedRect.size.width;
            double height = closestRotatedRect.size.height;
            double correctedAngle = (width > height) ? rawAngle : rawAngle + 90;
            correctedAngle = (360 - correctedAngle) % 180;
        
            String orientationLabel;
            if (correctedAngle <= 22 || correctedAngle >= 161) {
                orientationLabel = "horizontal";
            } else if (correctedAngle <= 68) {
                orientationLabel = "diagonal right";
            } else if (correctedAngle <= 114) {
                orientationLabel = "vertical";
            } else if (correctedAngle <= 160) {
                orientationLabel = "diagonal left";
            } else {
                orientationLabel = "unknown";
            }
        
            // Combined label
            String label = String.format("dx: %.0f, dy: %.0f, %s", dx, dy, orientationLabel);
            Imgproc.putText(input, label, new Point(frameCenter.x + 10, frameCenter.y - 10),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 255), 1);
        }
        return input;
    }
}