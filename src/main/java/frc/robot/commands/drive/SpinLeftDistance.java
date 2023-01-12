package frc.robot.commands.drive;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SpinLeftDistance extends CommandBase {
    
  private final DriveSubsystem driveTrain;
  private double distanceDesired;

  /**
   * Creates a new DriveForward command.
   *
   * @param driveSubsystem The subsystem used by this command.
   * @param leftWheelDistanceInches How far to go forward, in inches.
   */
  public SpinLeftDistance(DriveSubsystem driveSubsystem, double leftWheelDistanceInches) {
    driveTrain = driveSubsystem;
    distanceDesired = Units.inchesToMeters(leftWheelDistanceInches);

    // If this command is called, we want to interupt any other commands running
    // on the driving subystem
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    distanceDesired = driveTrain.getLeftDistance() - distanceDesired;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Turn at half the auto speed so that ramp and inertia has less of an effect
   // driveTrain.arcadeDrive(0, -Constants.AUTO_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //driveTrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return driveTrain.getLeftDistance() <= distanceDesired;
  }
}
