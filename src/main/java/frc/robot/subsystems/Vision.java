// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;


public class Vision extends SubsystemBase {
  public Vision() {
    LimelightHelpers.SetRobotOrientation("limelightName", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    
  }
  



public double getX (){
  return LimelightHelpers.getTX("limelight-april");
}

public double getY (){
  return LimelightHelpers.getTY("limelight-april");
}

public double getZ (){
  return LimelightHelpers.getTA("limelight-april");
}

 public double AprilNumber (){
    return LimelightHelpers.getFiducialID("limelight-april");
}


// vision botpose
/*
public LimelightHelpers.PoseEstimate getPose(){
  return LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-april");
}
*/



public LimelightHelpers.PoseEstimate getPose(){
  return LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-april");
}

public Pose2d test() {
  LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-april");
  return poseEstimate.pose;  
}



/*public Pose2d getPose() {
  LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-april");
  return poseEstimate.pose;  // Return the Pose2d object
}
*/

public Command print(){
  return runOnce(()-> System.out.println(AprilNumber()));
}


public boolean HasTargets (){
  return LimelightHelpers.getTV("april");
 }


// Not needed do not add subsytems to other subsystems. If two subsystems need to interact they do so in RobotContainer

// public Command Aprilthingy (){
//   return runOnce(()-> drivetrain.Angles.get(AprilNumber()));
// }


  @Override
  public void periodic() {

  }
}
