// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.BallGrabber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.GrabbyThing;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.CommandSwerveDrivetrain.paths;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    SendableChooser<String> chooser = new SendableChooser<>();
    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController joystick2 = new CommandXboxController(1);

    private final CommandXboxController Board = new CommandXboxController(1);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    Elevator elevator = new Elevator();
    Vision vision = new Vision();
    GrabbyThing Grabbything = new GrabbyThing();
   // BallGrabber ballGrabber = new BallGrabber();

  public RobotContainer() {
    configureBindings();
    chooser.setDefaultOption("TestAuto", "GoForward");
    chooser.addOption("TestAuto2", "New Auto");
    SmartDashboard.putData("AutoOptions", chooser);
  }


    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        

       /*joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        )); */ 
        //joystick.y().onTrue(drivetrain.Test(()->Vision.AprilNumber()));

        /*
         Run SysId routines when holding back/start and X/Y.
         Note that each routine should be run exactly once in a single log.
         joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
         joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
         joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
         joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        */
       joystick.x().whileTrue(elevator.elevatorLift(()-> 10));
       joystick.y().whileTrue(elevator.elevatorLift(()-> 20));
       joystick.b().whileTrue(elevator.elevatorLift(()-> 55));
       //joystick.a().whileTrue(elevator.elevatorLift(()-> 6));
       joystick.a().whileTrue(Grabbything.GrabbythingSpeed(.5));
       joystick.a().whileFalse(Grabbything.GrabbythingSpeed(0.0));
       joystick.rightBumper().whileTrue(drivetrain.StrafeApril(()-> vision.getX(),() -> joystick.getLeftY() * MaxSpeed,() -> false));
       joystick.leftBumper().whileTrue(drivetrain.StrafeApril(()-> vision.getX(),() -> joystick.getLeftY() * MaxSpeed,() -> true));

        
        joystick2.y().onTrue(elevator.setAngleCommand(() ->110));
        joystick2.b().onTrue(elevator.setAngleCommand(() ->80));
        joystick2.a().onTrue(elevator.setAngleCommand(() ->70));
        joystick2.leftBumper().onTrue(elevator.intakeMotorCommand(() -> 1));
        elevator.sensor().onTrue(elevator.intakeMotorCommand(() -> 0));
        joystick2.rightBumper().onTrue(drivetrain.goToPoint(paths.station));
        joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    

        joystick.povDown().onTrue(drivetrain.turnToAngle(() -> 180, () -> joystick.getLeftX() * MaxSpeed,() ->joystick.getLeftY() * MaxSpeed));
        joystick.povUp().onTrue(drivetrain.turnToAngle(() -> 0, () -> joystick.getLeftX() * MaxSpeed, () ->joystick.getLeftY() * MaxSpeed));
        joystick.povLeft().onTrue(drivetrain.turnToAngle(() -> 270, () -> joystick.getLeftX() * MaxSpeed ,() -> joystick.getLeftY() * MaxSpeed));
        joystick.povRight().onTrue(drivetrain.turnToAngle(() -> 90, ()->joystick.getLeftX() * MaxSpeed , () ->joystick.getLeftY() * MaxSpeed ));
        joystick.povDownLeft().onTrue(drivetrain.turnToAngle(() -> 225, () ->joystick.getLeftX() * MaxSpeed, () ->joystick.getLeftY() * MaxSpeed));
        joystick.povDownRight().onTrue(drivetrain.turnToAngle(() -> 135, () ->joystick.getLeftX() * MaxSpeed,() ->joystick.getLeftY() * MaxSpeed));
        joystick.povUpLeft().onTrue(drivetrain.turnToAngle(() -> 315, ()->joystick.getLeftX() * MaxSpeed , () ->joystick.getLeftY() * MaxSpeed));
        joystick.povUpRight().onTrue(drivetrain.turnToAngle(() -> 45, ()->joystick.getLeftX() * MaxSpeed, ()->joystick.getLeftY() * MaxSpeed));
        joystick.axisGreaterThan(4, .05).onTrue(drivetrain.getDefaultCommand());
        joystick.axisGreaterThan(4, -.05).onTrue(drivetrain.getDefaultCommand());

        // Board.x().onTrue(ballGrabber.BallGrabbyThing(0.5));
        // Board.x().onFalse(ballGrabber.BallGrabbyThing(0.0));
        // Board.y().onTrue(ballGrabber.BallOutputThing(0.5));
        // Board.y().onFalse(ballGrabber.BallOutputThing(0.0));
        // Board.rightBumper().onTrue(ballGrabber.BallGrabbyThingTurning(0.1));
        // Board.b().onTrue(ballGrabber.BallGrabbyThingTurning(0.0));

        //might work
        //ballGrabber.linebreak().onTrue(ballGrabber.BallGrabbyThing(0.1).andThen(elevator.elevatorLift(()->130)));

        //Board.povDown().onTrue((ballGrabber.BallGrabbyThingTurning(0.1)).andThen(new WaitCommand(0.3)).andThen(elevator.elevatorLift(()-> 55).andThen(new WaitCommand(0.2).andThen((ballGrabber.BallGrabbyThingTurning(0.1))))));
        //Board.povUp().onTrue((ballGrabber.BallGrabbyThingTurning(0.1)).andThen(new WaitCommand(0.3)).andThen(elevator.elevatorLift(()-> 20).andThen(new WaitCommand(0.2).andThen((ballGrabber.BallGrabbyThingTurning(0.1))))));
        //Board.povRight().onTrue(elevator.elevatorLift(()-> 55).andThen(new WaitCommand(0.5)).andThen(ballGrabber.BallGrabbyThingTurning(0.0).alongWith(ballGrabber.BallOutputThing(1.0)).andThen(new WaitCommand(0.1).andThen(ballGrabber.BallGrabbyThingTurning(0.0)))));
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {  
        SmartDashboard.putData(chooser);
        return new PathPlannerAuto(chooser.getSelected());
    }
}
