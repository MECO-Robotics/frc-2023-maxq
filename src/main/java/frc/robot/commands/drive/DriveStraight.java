// Copyright (c) MECO Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Drive a requested distance, forward. */
public class DriveStraight extends CommandBase {
  
  private final DriveSubsystem driveTrain;
  private double distanceDesired;
  private final boolean forward;
  private double initialHeading;
  private double speed;
  /**
   * Creates a new DriveStraight command.
   *
   * @param driveSubsystem The subsystem used by this command.
   * @param distanceInches How far to go forward (negative backwards), in inches.
   */
  public DriveStraight(DriveSubsystem driveSubsystem, double distanceInches) {
    driveTrain = driveSubsystem;
    distanceDesired = Units.inchesToMeters(distanceInches);
    forward = distanceDesired > 0;
    speed =  Constants.AUTO_SPEED;
    // If this command is called, we want to interupt any other commands running
    // on the driving subystem
    addRequirements(driveSubsystem);
  }

  public DriveStraight(DriveSubsystem driveSubsystem, double distanceInches, double desiredSpeed) {
    driveTrain = driveSubsystem;
    distanceDesired = Units.inchesToMeters(distanceInches);
    forward = distanceDesired > 0;
    speed = desiredSpeed;
    // If this command is called, we want to interupt any other commands running
    // on the driving subystem
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // TODO: Setup distance (or use path planning)\
    MecanumDriveWheelPositions d = driveTrain.getWheelPositions();
    distanceDesired = d.frontRightMeters + distanceDesired;
    initialHeading = driveTrain.getHeadingDegrees();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double throttle;
    if(forward) {
      throttle = speed;
    } else {
      throttle = -speed;
    }

    // To make sure we're continuing to drive straight, make slight adjustments to the
    // turn based on Yaw error (Propotional only control). This adjustment basically says
    // for every 1 degree of error, adjust the turn value by 0.1 input in the opposite direction
    // If our current heading matches the initial heading, then the turn comes out to zero.
    //double turn = -(driveTrain.getHeadingDegrees() - initialHeading)/10f;

    driveTrain.arcadeDrive(throttle, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("CMD: DriveStraight: end");

    // Use tank drive, which doesn't have speed ramping because we need the motors to
    // stop immediately.
    driveTrain.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    MecanumDriveWheelPositions positions = driveTrain.getWheelPositions();
    
    //TODO
     if(forward) {
       return positions.frontRightMeters  >= distanceDesired;
     } else {
       return positions.frontRightMeters <= distanceDesired;
     }
  }
}
