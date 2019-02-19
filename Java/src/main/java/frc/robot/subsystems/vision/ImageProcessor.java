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

  public static Mat annotate(Mat img, MatOfKeyPoint blob) {
    Mat outputImage = new Mat();

    Scalar sc = new Scalar(2, 254, 255);
    Features2d.drawKeypoints(img, blob, outputImage, sc, Features2d.DRAW_RICH_KEYPOINTS); 

    return outputImage;
  }

  public static void displayImageToSmartDashboard(Mat img) {
    CvSource outputStream = CameraServer.getInstance().putVideo("Hatch Detect", img.width(), img.height());

    outputStream.putFrame(img);
  }

  public static double getHatchAngle(Mat img, MatOfKeyPoint blob, boolean debug) {
    if(blob.empty()) return 420;

    System.out.println(blob);
    Rect rect = Imgproc.boundingRect(blob);
    SmartDashboard.putNumber("Hatch Width", rect.width);
    
    double ppi = rect.width / HATCH_WIDTH;

    if(debug) {
      displayImageToSmartDashboard(annotate(img, blob));
    }

    double yDist = getHatchDistance(rect.width);
    double xDist = ppi * Math.abs(img.width() / 2 - rect.width / 2);
    SmartDashboard.putNumber("yDist of Hatch", yDist);
    SmartDashboard.putNumber("xDist of Hatch", xDist);

    return Math.atan(yDist / xDist);
  }
}
