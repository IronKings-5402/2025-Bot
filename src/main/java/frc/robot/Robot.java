// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    // boolean doRejectUpdate1 = false;
    // boolean doRejectUpdate2 = false;
    // // adds vision measurement to pose
    // LimelightHelpers.PoseEstimate mt21 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-back");
    // LimelightHelpers.PoseEstimate mt22 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-front");
    // if(Math.abs(m_robotContainer.drivetrain.getState().Speeds.omegaRadiansPerSecond) > 2*Math.PI) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
    // {
    //   doRejectUpdate1 = true;
    //   doRejectUpdate2 = true;
    // }
    // if(mt21 == null||mt21.tagCount == 0)
    // {
    //   doRejectUpdate1 = true;
    // }
    // if(mt22==null||mt22.tagCount == 0)
    // {
    //   doRejectUpdate2 = true;
    // }
    // if(!doRejectUpdate1)
    // {
    //   m_robotContainer.drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.99,.99,9999999));
    //   m_robotContainer.drivetrain.addVisionMeasurement(
    //       mt21.pose,
    //       mt21.timestampSeconds);
    // }
    // if(!doRejectUpdate2)
    // {
    //   m_robotContainer.drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.99,.99,9999999));
    //   m_robotContainer.drivetrain.addVisionMeasurement(
    //       mt22.pose,
    //       mt22.timestampSeconds);
    // }
    boolean doRejectUpdate = false;
    System.out.println(m_robotContainer.drivetrain.getState().Pose.getRotation().getDegrees());
    LimelightHelpers.SetRobotOrientation("limelight-front", m_robotContainer.drivetrain.getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-front");
      if(Math.abs(m_robotContainer.drivetrain.getPigeon2().getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
      {
        doRejectUpdate = true;
      }
      if(mt2.tagCount == 0)
      {
        doRejectUpdate = true;
      }
      if(!doRejectUpdate)
      {
       m_robotContainer.drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        m_robotContainer.drivetrain.addVisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds);
      }



  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
