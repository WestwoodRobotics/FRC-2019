package frc.robot.subsystems.vision;

import org.opencv.core.Mat;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import frc.robot.RobotMap;

public class HatchVision {
  private HatchPipeline pipeline;

  public HatchVision() {
    this.pipeline = new HatchPipeline();
  }

  public double getAngleFromHatch() {
    Mat img = new Mat();
    String cName = "USB Camera " + RobotMap.armCameraPort;
    CameraServer.getInstance().getVideo(cName).grabFrame(img);

    double ang = ImageProcessor.getHatchAngle(img, pipeline.largestContourOutput(), true);

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
