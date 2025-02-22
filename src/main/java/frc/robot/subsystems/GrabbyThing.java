// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GrabbyThing extends SubsystemBase {
  /** Creates a new GrabbyThing. */
  private final TalonFX grabbything = new TalonFX(51);
  public GrabbyThing() {
    TalonFXConfiguration grabbythingConfig = new TalonFXConfiguration();
    grabbythingConfig.Slot0.kP = .1;
    grabbythingConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    grabbythingConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    grabbything.getConfigurator().apply(grabbythingConfig);
  }


 void setSpeed(double speed){
  grabbything.set(speed);
 }

   public Command GrabbythingSpeed(double speed){
    return runOnce(()-> setSpeed(speed));
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
