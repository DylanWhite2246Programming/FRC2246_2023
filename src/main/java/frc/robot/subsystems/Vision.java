// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  AprilTagFieldLayout aprilTagFieldLayout; //= new AprilTagFieldLayout(AprilTagFieldLayout.loadFromResource(AprilTagFields.k2022RapidReact.m_resourceFile));
  PhotonCamera cam = new PhotonCamera("photonvision");
  ShuffleboardTab camera, pose;
  /** Creates a new Vision. */
  public Vision() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
