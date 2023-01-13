// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstruction;

public class Vision extends SubsystemBase {
  AprilTagFieldLayout aprilTagFieldLayout; //= new AprilTagFieldLayout(AprilTagFieldLayout.loadFromResource(AprilTagFields.k2022RapidReact.m_resourceFile));
  PhotonCamera cam = new PhotonCamera("photonvision");
  ArrayList camList;
  RobotPoseEstimator poseEstimator;
  ShuffleboardTab camera, pose;
  /** Creates a new Vision. */
  public Vision() {
    camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
    camList.add(new Pair<PhotonCamera, Transform3d>(cam, RobotConstruction.kRobotToCam));
    poseEstimator = new RobotPoseEstimator(
      aprilTagFieldLayout, 
      PoseStrategy.CLOSEST_TO_REFERENCE_POSE, 
      camList
    );
  }

  /**
   * @param pipe sets pipe 0 = april, 1 = cone, 2 = cube
   */
  public void setPipeline(int pipe){cam.setPipelineIndex(pipe);}

  public void setReferencePose(Pose3d refrencePose){
    
  }

  public Pair<Pose2d, Double> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    poseEstimator.setReferencePose(prevEstimatedRobotPose);

    double currentTime = Timer.getFPGATimestamp();
    Optional<Pair<Pose3d, Double>> result = poseEstimator.update();
    if (result.isPresent()) {
        return new Pair<Pose2d, Double>(result.get().getFirst().toPose2d(), currentTime - result.get().getSecond());
    } else {
        return new Pair<Pose2d, Double>(null, 0.0);
    }
  }

  public boolean hasTarget(){return false;}
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
