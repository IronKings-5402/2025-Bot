// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;


public class Vision extends SubsystemBase {
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
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

public Command Aprilthingy (){
  return runOnce(()-> drivetrain.Angles.get(AprilNumber()));
}

  @Override
  public void periodic() {

  }
}
