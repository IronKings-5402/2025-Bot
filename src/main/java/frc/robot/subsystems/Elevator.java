// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import java.util.HashMap;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Elevator extends SubsystemBase {
    private final TalonFX elevatorLiftleft = new TalonFX(52);
    private final TalonFX elevatorLiftRight = new TalonFX(50);
    private final Encoder encoder = new Encoder(0, 1);
    DutyCycleEncoder armEncoder = new DutyCycleEncoder(new DigitalInput(0));
    TalonFX angleMotor = new TalonFX(55);
    TalonFX intakeMotor = new TalonFX(56);
    DigitalInput linebreak = new DigitalInput(1);
    double currentDegree = 110;
    HashMap<String,Double> angles = new HashMap<>();

    PIDController anglePIDController = new PIDController(1, 0, 0);

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

        angles.put("Level1", 110.0);
        angles.put("Level2-3", 120.0);
        angles.put("Intake", 40.0);
        angles.put("Level4", 100.0);

    }

    void intakeMotor(double speed){
        intakeMotor.set(speed);
    }

    public Command intakeMotorCommand(DoubleSupplier speed){
        return runOnce(() -> intakeMotor(speed.getAsDouble()));
    }

    void goToAngle(double degree){
        double speed = anglePIDController.calculate(armEncoder.get(), degree);
        speed = MathUtil.clamp(speed, -.25, .25);
        angleMotor.set(speed);
    }

    void setAngle(double degrees){
        this.currentDegree = degrees;
    }

    void setAngle(String level){
        this.currentDegree = angles.get(level);
    }

    public Command setAngleCommand(DoubleSupplier angle){
        return runOnce(() -> goToAngle(angle.getAsDouble()));
    }

    public Trigger sensor(){
        return new Trigger(() -> !linebreak.get());
    }
    
    
    void setPos (double height){
        elevatorLiftRight.setControl(new PositionVoltage((height-6)*8/5.5));
        elevatorLiftleft.setControl(new StrictFollower(elevatorLiftleft.getDeviceID()));
    }

        

    public Command elevatorLift(DoubleSupplier liftPosition) {
        return runOnce(()->this.setPos(liftPosition.getAsDouble()));     
    }

    public void resetEncoder() {
        encoder.reset();
    }

    @Override
    public void periodic() {
        goToAngle(currentDegree);
    }
}
