// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private final PIDController pidController = new PIDController(0.1, 0.01, 0.0);
    private final TalonFX elevatorLiftleft = new TalonFX(52);
    private final TalonFX elevatorLiftRight = new TalonFX(50);
    private final Encoder encoder = new Encoder(0, 1);

    public Elevator() {
         TalonFXConfiguration elevatorConfigleft = new TalonFXConfiguration();
         TalonFXConfiguration elevatorConfigright = new TalonFXConfiguration();
         elevatorConfigleft.Slot0.kP = 1;
         elevatorConfigleft.MotorOutput.NeutralMode = NeutralModeValue.Coast;
         elevatorConfigright.Slot0.kP = 1;
         elevatorConfigright.MotorOutput.NeutralMode = NeutralModeValue.Coast;
         elevatorConfigright.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
         //elevatorLiftleft.getConfigurator().apply(elevatorConfigleft);
         elevatorLiftRight.getConfigurator().apply(elevatorConfigright);
         //5.5 in for 8 rotations

    }
    void setPos (double height){
        elevatorLiftRight.setControl(new PositionVoltage((height-6)*8/5.5));
        elevatorLiftleft.setControl(new StrictFollower(elevatorLiftleft.getDeviceID()));
    }

        

    public Command elevatorLift(DoubleSupplier liftPosition) {
        return runOnce(()->this.setPos(liftPosition.getAsDouble()));     
    }



    @Override
    public void periodic() {
    }

    public void resetEncoder() {
        encoder.reset();
    }
}
