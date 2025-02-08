// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.ObjectInputStream.GetField;
import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Barcode;
import frc.robot.LimelightHelpers.LimelightTarget_Classifier;
import frc.robot.LimelightHelpers.LimelightTarget_Detector;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.LimelightHelpers.LimelightTarget_Retro;
import frc.robot.LimelightHelpers.RawDetection;
import frc.robot.LimelightHelpers.RawFiducial;

public class Vision extends SubsystemBase {
  
  public Vision() {

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




public Command print(){
  return runOnce(()-> System.out.println(AprilNumber()));
}


public boolean HasTargets (){
  return LimelightHelpers.getTV("april");
 }

  @Override
  public void periodic() {

  }
}
