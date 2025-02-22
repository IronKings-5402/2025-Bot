// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import java.io.ObjectInputFilter.Config;
import java.util.HashMap;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constant;

public class Elevator extends SubsystemBase {
    private final TalonFX elevatorLiftleft = new TalonFX(52);
    private final TalonFX elevatorLiftRight = new TalonFX(50);
    DutyCycleEncoder armEncoder = new DutyCycleEncoder(new DigitalInput(1));
    TalonFX angleMotor = new TalonFX(53);
    TalonFX intakeMotor = new TalonFX(54);
    TalonSRX coaralGuide = new TalonSRX(8);
    DigitalInput linebreak = new DigitalInput(2);
    double currentDegree = Constant.AllLevels;
    HashMap<String,Double> angles = new HashMap<>();
    Servo servo = new Servo(0);

    PIDController anglePIDController = new PIDController(.01, 0, 0);
    PIDController coralGuidePID = new PIDController(.01, 0, 0);

    public Elevator() {
        TalonFXConfiguration elevatorConfigleft = new TalonFXConfiguration();
        TalonFXConfiguration elevatorConfigright = new TalonFXConfiguration();
        TalonFXConfiguration angleMotorConfig = new TalonFXConfiguration();
        TalonFXConfiguration intakeMotorConfig = new TalonFXConfiguration();
        elevatorConfigleft.Slot0.kP = 1;
        elevatorConfigleft.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        elevatorConfigright.Slot0.kP = 1;
        elevatorConfigright.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        elevatorConfigright.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        elevatorConfigleft.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        intakeMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        //angle motor Config
        angleMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        elevatorLiftleft.getConfigurator().apply(elevatorConfigleft);
        elevatorLiftRight.getConfigurator().apply(elevatorConfigright);
         //5.5 in for 8 rotations

        angles.put("Level1", 110.0);
        angles.put("Level2-3", 120.0);
        angles.put("Intake", 40.0);
        angles.put("Level4", 100.0);

    }


    // BALL intake
    void intakeMotor(double speed){
        intakeMotor.set(speed);
    }
    // Command for BALL intake
    public Command intakeMotorCommand(DoubleSupplier speed){
        return runOnce(() -> intakeMotor(speed.getAsDouble()));
    }

    void servoMove(double Position){
        servo.setAngle(Position);
    }

    public Command ServoThing(double Position){
        return runOnce(()-> servoMove(Position));
    }

    void coaralGuideing(double speed){
        coaralGuide.set(ControlMode.PercentOutput, speed);
    }

    public Command coaralGuideCommand(DoubleSupplier speed){
        return runOnce(()-> coaralGuideing(speed.getAsDouble()));
    }

    // private internal subsystem command to go to angle
    private void goToAngle(double degree){
        double speed = anglePIDController.calculate(armEncoder.get()*360, degree);
        speed = MathUtil.clamp(speed, -.25, .25);
        angleMotor.set(speed);
    }
    
    


    // sets angle setpoint
    void setAngle(double degrees){
        this.currentDegree = degrees;
    }
    // alternate way to set angle setpoint
    void setAngle(String level){
        this.currentDegree = angles.get(level);
    }
    // command to set angle
    public Command setAngleCommand(DoubleSupplier angle){
        return runOnce(() -> setAngle(angle.getAsDouble()));
    }

    // Linebreak sensor
    public Trigger sensor(){
        return new Trigger(() -> !linebreak.get());
    }
    
    // Sets elevator position
    void setPos (double height){
        elevatorLiftRight.setControl(new PositionVoltage((height-6)*8/5.5));
        elevatorLiftleft.setControl(new Follower(elevatorLiftleft.getDeviceID(), false));
    }

        
    // command to set elevator posotion
    public Command elevatorLift(DoubleSupplier liftPosition) {
        return runOnce(()->this.setPos(liftPosition.getAsDouble()));     
    }


    // Makes go to angle
    @Override
    public void periodic() {
        goToAngle(currentDegree);
        SmartDashboard.putBoolean("Linebreak", sensor().getAsBoolean());
        SmartDashboard.putNumber("armEncoder", armEncoder.get()*360);
        SmartDashboard.putNumber("armLocation", elevatorLiftRight.getPosition().getValueAsDouble()*5.5/8+6);
    }
}
