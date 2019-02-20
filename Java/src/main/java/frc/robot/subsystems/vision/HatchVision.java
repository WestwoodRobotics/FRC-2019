package frc.robot.subsystems.vision;

import org.opencv.core.Mat;

import edu.wpi.cscore.CvSource;
import edu.wpi.first.cameraserver.CameraServer;
import frc.robot.RobotMap;

public class HatchVision {
  private HatchPipeline pipeline;
  private String cName = "USB Camera " + RobotMap.armCameraPort;
  private Mat hatchImg;

  public HatchVision() {
    this.pipeline = new HatchPipeline();
    serveHatchImage();
  }

  public void serveHatchImage() {
    new Thread(() -> {
      CameraServer.getInstance().getVideo(cName).grabFrame(hatchImg);
      CvSource outputStream = CameraServer.getInstance().putVideo("Hatch Img", hatchImg.width(), hatchImg.height());
      
      while(!Thread.interrupted()) {
        System.out.println("Putting frame");
        outputStream.putFrame(hatchImg);
      }
    }).start();
  }

  public double getAngleFromHatch() {
    Mat img = new Mat();
    CameraServer.getInstance().getVideo(cName).grabFrame(img);

    pipeline.process(img);

    this.hatchImg = ImageProcessor.annotate(img, pipeline.largestContourOutput());
    double angle = ImageProcessor.getHatchAngle(img, pipeline.largestContourOutput(), true);

    return angle;
  }

  private static HatchVision instance;
  public static HatchVision getInstance() {
    if(instance == null) {
      instance = new HatchVision();
    }
      
    return instance;
  }
}
