package frc.robot.subsystems.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.features2d.Features2d;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSource;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ImageProcessor {
  private static final double HATCH_FOCAL_LENGTH = 736.0;
  private static final double HATCH_WIDTH = 18.0;

  public static double getHatchDistance(double perceived_width) {
    return (HATCH_WIDTH * HATCH_FOCAL_LENGTH) / perceived_width;
  }

  public static Mat annotate(Mat img, MatOfPoint cont) {
    Rect r = Imgproc.boundingRect(cont);
    
    Scalar sc = new Scalar(2, 254, 255);

    Point p1 = new Point(r.x, r.y);
    Point p2 = new Point(r.x + r.width, r.y + r.height);
    Imgproc.rectangle(img, p1, p2, sc);

    return img;
  }

  public static void displayImageToSmartDashboard(Mat img) {
    CvSource outputStream = CameraServer.getInstance().putVideo("Hatch Detect", img.width(), img.height());

    outputStream.putFrame(img);
  }

  public static double getHatchAngle(Mat img, MatOfPoint contour, boolean debug) {
    if(blob.empty()) return 420;

    System.out.println(contour);
    Rect rect = Imgproc.boundingRect(contour);
    SmartDashboard.putNumber("Hatch Width", rect.width);
    
    double ppi = rect.width / HATCH_WIDTH;

    if(debug) {
      displayImageToSmartDashboard(annotate(img, contour));
    }

    double ppi = rect.width / HATCH_WIDTH;
    double yDist = getHatchDistance(rect.width);
    double xDist = ppi * ((imgWidth / 2) - (rect.width / 2));

    SmartDashboard.putNumber("yDist of Hatch", yDist);
    SmartDashboard.putNumber("xDist of Hatch", xDist);

    return Math.toDegrees(Math.atan(yDist / xDist));
  }
}
