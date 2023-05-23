// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.Constants;
public class Vision extends SubsystemBase {
  private PhotonCamera camera;
  private PhotonPoseEstimator estimator;
  private AprilTagFieldLayout layout;

  private PhotonPipelineResult result;
  private PhotonTrackedTarget bestTarget;

  private boolean haveTargets = false;
  private double lastYaw = 0.0;
  private double lastDistance = 0.0;
  private Pose2d lastRobotPose = new Pose2d();
  private Pose2d _currentTagPose = new Pose2d();


  public Vision() {
    
    camera = new PhotonCamera(Constants.VisionConstants.cameraName);
    PhotonCamera.setVersionCheckEnabled(false);

    try {
      layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile);
    } catch (IOException e) {
      e.printStackTrace();
    }

    if (DriverStation.getAlliance() == Alliance.Blue) {
      layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    } else if (DriverStation.getAlliance() == Alliance.Red) {
      layout.setOrigin(OriginPosition.kRedAllianceWallRightSide);
    }

    estimator =
        new PhotonPoseEstimator(
            layout,
            PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
            camera,
            Constants.VisionConstants.robotToCam);

  }

  public void setRobotPose(Pose2d robotPose) {
    if (robotPose != null) {
      lastRobotPose = robotPose;
    } else {
      lastRobotPose = new Pose2d();
    }
  }
  public Pose2d getCurrentRobotPose(){
    return lastRobotPose;
  }

  public boolean hasTargets() {
    return haveTargets;
  }

  public double getYaw() {
    if (haveTargets) {
      return bestTarget.getYaw();
    } else {
      return lastYaw;
    }
  }
  
  public double getDistanceFromTarget() {
    if (haveTargets) {
      Optional<Pose3d> _currentTagPoseOps = layout.getTagPose(bestTarget.getFiducialId());
      if(_currentTagPoseOps.isPresent()){
        _currentTagPose = _currentTagPoseOps.get().toPose2d();
      }
      return PhotonUtils.getDistanceToPose(
          lastRobotPose, layout.getTagPose(bestTarget.getFiducialId()).get().toPose2d());
    } else {
      return lastDistance;
    }
  }

  public Pose2d getCurrentTagPose() {
    return _currentTagPose;
  }
  
  
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    estimator.setReferencePose(prevEstimatedRobotPose);
    Optional<EstimatedRobotPose> result = estimator.update();
    if (result.isPresent()) {
      return result;
    } else {
      return Optional.ofNullable(null);
    }
  }
  
  @Override
  public void periodic() {
    if (RobotBase.isReal()) {
      result = camera.getLatestResult();
      bestTarget = result.getBestTarget();
      haveTargets = result.hasTargets();

      Optional<EstimatedRobotPose> currentRobotPose = getEstimatedGlobalPose(lastRobotPose);
      
      if (currentRobotPose.isPresent()){
        setRobotPose(currentRobotPose.get().estimatedPose.toPose2d());
      }

      if (haveTargets) {
        lastYaw = bestTarget.getYaw();
        lastDistance = getDistanceFromTarget();
      }
    }
  }
}