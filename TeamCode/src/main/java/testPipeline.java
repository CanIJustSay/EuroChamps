import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
@Config
public class testPipeline extends OpenCvPipeline {

    private Mat hsv = new Mat();
    private Mat mask = new Mat();
    private Mat hierarchy = new Mat();
    private Mat homography;

    // === HSV YELLOW RANGE ===
    public static Scalar lowerYellow = new Scalar(20, 100, 100);
    private static Scalar upperYellow = new Scalar(30, 255, 255);

    private Point imageCenter;
    public double xDisplacement = 0;
    public double yDisplacement = 0;
    public double orientationAngle = 0;
    public Point targetCenter = null;
    public Point fieldPosition = null;

    public static int imgPointTLX = 100;
    public static int imgPointTLY = 100;
    public static int imgPointTRX = 500;
    public static int imgPointTRY = 100;
    public static int imgPointBLX = 100;
    public static int imgPointBLY = 400;
    public static int imgPointBRX = 500;
    public static int imgPointBRY = 400;





    public static int worPointTLX = 0;
    public static int worPointTLY = 0;
    public static int worPointTRX = 24;
    public static int worPointTRY = 0;
    public static int worPointBLX = 0;
    public static int worPointBLY = 24;
    public static int worPointBRX = 24;
    public static int worPointBRY = 24;



    // After homography

    //ARBITRARY points — tune these based on camera angle and four known field points
    private static MatOfPoint2f imagePoints = new MatOfPoint2f(
            new Point(imgPointTLX, imgPointTLY),  // top-left in image
            new Point(imgPointTRX, imgPointTRY),  // top-right
            new Point(imgPointBLX, imgPointBLY),  // bottom-left
            new Point(imgPointBRX, imgPointBRY)   // bottom-right
    );


    //Real-world coordinates in inches/cm matching imagePoints
    public static MatOfPoint2f fieldPoints = new MatOfPoint2f(
            new Point(worPointTLX, worPointTLY),      // top-left of calibration square
            new Point(worPointTRX, worPointTRY),     // top-right
            new Point(worPointBLX, worPointBLY),     // bottom-left
            new Point(worPointBRX, worPointBRY)     // bottom-right
    );


    Mat morphed = new Mat();


    @Override
    public void init(Mat firstFrame) {
        // Called once to calculate homography
        homography = Imgproc.getPerspectiveTransform(imagePoints, fieldPoints);
    }

    @Override
    public Mat processFrame(Mat input) {
        int width = input.cols();
        int height = input.rows();
        imageCenter = new Point(width / 2.0, height / 2.0);

        // Convert to HSV
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        // Apply yellow mask
        Core.inRange(hsv, lowerYellow, upperYellow, mask);

        // Gaussian blur for noise reduction
        Imgproc.GaussianBlur(mask, mask, new Size(5, 5), 0);

        // Morphological cleanup
        Imgproc.erode(mask, mask, new Mat(), new Point(-1, -1), 2);
        Imgproc.dilate(mask, mask, new Mat(), new Point(-1, -1), 2);


        // Find contours
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        double closestDist = Double.MAX_VALUE;
        RotatedRect bestRect = null;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);

            // 🔧 Adjust this based on expected block size in image
            if (area > 500) {
                MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
                RotatedRect rect = Imgproc.minAreaRect(contour2f);
                Point center = rect.center;
                double dist = Math.hypot(center.x - imageCenter.x, center.y - imageCenter.y);

                if (dist < closestDist) {
                    closestDist = dist;
                    bestRect = rect;
                }
            }
        }

        if (bestRect != null) {
            Point[] box = new Point[4];
            bestRect.points(box);
            for (int i = 0; i < 4; i++) {
                Imgproc.line(input, box[i], box[(i + 1) % 4], new Scalar(0, 255, 0), 2);
            }

            targetCenter = bestRect.center;
            orientationAngle = bestRect.angle;

            // Raw pixel displacement
            xDisplacement = targetCenter.x - imageCenter.x;
            yDisplacement = targetCenter.y - imageCenter.y;

            if (homography == null) {
                homography = Imgproc.getPerspectiveTransform(imagePoints, fieldPoints);
            }
            // === Apply homography to get real-world position ===
            if (targetCenter != null && homography != null) {
                MatOfPoint2f inputPoints = new MatOfPoint2f(targetCenter);
                MatOfPoint2f outputPoints = new MatOfPoint2f();
                Core.perspectiveTransform(inputPoints, outputPoints, homography);
                fieldPosition = outputPoints.toArray()[0];
            }

            // Draw target
            Imgproc.circle(input, targetCenter, 5, new Scalar(255, 0, 0), -1);
            Imgproc.putText(input,
                    String.format("x: %.0f y: %.0f a: %.1f", xDisplacement, yDisplacement, orientationAngle),
                    new Point(10, 30), Imgproc.FONT_HERSHEY_SIMPLEX, 0.7, new Scalar(255, 255, 0), 2);

            Imgproc.putText(input,
                    String.format("Field: (%.1f, %.1f)", fieldPosition.x, fieldPosition.y),
                    new Point(10, 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.7, new Scalar(255, 255, 0), 2);
        }

        // Optional: show center of camera
        Imgproc.circle(input, imageCenter, 5, new Scalar(0, 255, 255), -1);
        return input;
    }
}
