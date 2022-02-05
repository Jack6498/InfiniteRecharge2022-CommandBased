// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VisionSystem extends SubsystemBase {
  PhotonCamera limelight = new PhotonCamera(Constants.Vision.LimelightCameraName);
  PhotonPipelineResult currentResult;
  boolean active = true;
  /** Creates a new VisionSystem. */
  public VisionSystem() {
    limelight.setDriverMode(false);
    limelight.setPipelineIndex(Constants.Vision.AutoPipelineID);
    limelight.setLED(VisionLEDMode.kOn);
  }

  /**
   * 
   * @param target The target you want data for. Pass in directly from getTarget().
   * @return A double[] with the yaw, pitch, and area values of the target in that order
   */
  public Double[] getTargetData()
  {
    PhotonTrackedTarget target = currentResult.getBestTarget();
    Double[] data = new Double[]
    {
      target.getYaw(),
      target.getPitch(),
      target.getArea()

    };
    return data;
  }

  public void setLED(VisionLEDMode ledMode)
  {
    limelight.setLED(ledMode);
  }

  public boolean hasTargets() {
    return currentResult.hasTargets();
  }
  /**
   * 
   * @return Distance to the current pipeline's best target, for input to a PID controller (for shooting)
   */
  public double getTargetDistance()
  {
    return PhotonUtils.calculateDistanceToTargetMeters(
      Constants.Vision.LimelightHeightFromField, 
      Constants.Vision.UpperHubTargetHeight, 
      Units.degreesToRadians(Constants.Vision.LimelightPitch), 
      Units.degreesToRadians(getTargetData()[1])
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per robot loop; before triggered commands are scheduled and before any commands are run
    if (active) {
      currentResult = limelight.getLatestResult();
    }
  }
}
