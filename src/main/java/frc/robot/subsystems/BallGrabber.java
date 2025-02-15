// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class BallGrabber extends SubsystemBase {
  /** Creates a new BallGrabber. */
  public BallGrabber() {}
    DigitalInput input = new DigitalInput(0);
  PIDController BallPID = new PIDController(1, 0, 0);
  private final TalonFX BallGrabber = new TalonFX(0);
  private final TalonFX BallGrabberTurn = new TalonFX(0);
  private final Encoder encoder = new Encoder(0, 1);
  PIDController PID = new PIDController(1, 0, 0);
  void BallMover (double speeds){
    BallGrabber.set(speeds);
  }

  void BallMoverTurning (double speeds){
    BallGrabberTurn.set(speeds);
  }

  public Command BallGrabbyThingTurning(double Pos){
    return run(()->BallMover(Pos));
}
 Command posBallGrabber(double setpoint){
 return runOnce(()-> BallGrabberTurn.set(PID.calculate(encoder.getDistance(), setpoint)));
 }

  public Command BallGrabbyThing(double speed){
   return run(()->BallMover(speed));
}

public Command BallOutputThing(double speed){
  return run(()->BallMover(speed *-1));
}

public BooleanSupplier lineBreakSupplier(){
  return () -> !input.get();
}

public Trigger linebreak (){
 return new Trigger(lineBreakSupplier());
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
