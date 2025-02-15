// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BallGrabber extends SubsystemBase {
  /** Creates a new BallGrabber. */
  public BallGrabber() {}
  PIDController BallPID = new PIDController(1, 0, 0);
  private final TalonFX BallGrabber = new TalonFX(0);

  void BallMover (double speeds){
    BallGrabber.set(speeds);
  }

  public Command BallGrabbything(double speed){
   return run(()->BallMover(speed));
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
