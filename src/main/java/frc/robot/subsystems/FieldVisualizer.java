package frc.robot.subsystems;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FieldVisualizer extends SubsystemBase {
  private final Field2d field = new Field2d(); 
  private final Vision visionSubsystem; 
  Vision vision = new Vision();
  public FieldVisualizer(Vision visionSubsystem) {
      this.visionSubsystem = visionSubsystem;
      SmartDashboard.putData("Field", field);
  }

//just testing something I found

  @Override
  public void periodic() {
      if (visionSubsystem.HasTargets()) {
          Pose2d robotPose = vision.test(); 
          field.setRobotPose(robotPose); 
      }
  }
}