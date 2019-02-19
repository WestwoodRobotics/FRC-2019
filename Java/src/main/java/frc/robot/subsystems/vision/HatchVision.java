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
    /*Mat m = new Mat();
    String cName = "USB Camera " + RobotMap.armCameraPort;
    CameraServer.getInstance().getVideo(cName).grabFrame(m);

    this.pipeline.process(m);
    double angle = ImageProcessor.getHatchAngle(m, this.pipeline.findBlobsOutput(), true);

    return angle;*/
    return 0;
  }

  private static HatchVision instance;
  public static HatchVision getInstance() {
    if(instance == null) {
      instance = new HatchVision();
    }
      
    return instance;
  }
}
