// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstruction;

public class Vision extends SubsystemBase {
  AprilTagFieldLayout fieldLayout;

  PhotonCamera cam = new PhotonCamera("photonvision");
  PhotonPoseEstimator poseEstimator;
  ShuffleboardTab camera, pose;
  /** Creates a new Vision. */
  public Vision() { 
    try{
      fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    }catch(IOException ex){
      DriverStation.reportError("Unable to open trajectory: " + "paths/taxipassscaleblue.wpilib.json", ex.getStackTrace());
    }
    var camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
    camList.add(new Pair<PhotonCamera, Transform3d>(cam, RobotConstruction.kRobotToCam));
    poseEstimator = new PhotonPoseEstimator(
      fieldLayout, 
      PoseStrategy.CLOSEST_TO_REFERENCE_POSE, 
      cam, 
      RobotConstruction.kRobotToCam
    );
  }

  /** 
   * @param pipe sets pipe 0 = april, 1 = cone, 2 = cube
   */
  public void setPipeline(int pipe){cam.setPipelineIndex(pipe);}

  public void setReferencePose(Pose3d refrencePose){} //TODO figure this out

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    poseEstimator.setReferencePose(prevEstimatedRobotPose);
    return poseEstimator.update();
  }

  public PhotonPipelineResult getResults(){
    return cam.getLatestResult();
  }

  public int getPipelineIndex(){
    return cam.getPipelineIndex();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
