// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Turn right for an angle. */
public class SpinRightAngle extends CommandBase {

  private final DriveSubsystem driveTrain;
  private double angleDesired;

  // True if we're turning clock-wise (CW), false if we're turning counter clock-wise (CCW)
  private final boolean cw;

  /**
   * Creates a new command.
   *
   * @param driveSubsystem The subsystem used by this command.
   * @param turnDegrees    How far to go forward, in inches.
   */
  public SpinRightAngle(DriveSubsystem driveSubsystem, double turnDegrees) {
    driveTrain = driveSubsystem;
    angleDesired = turnDegrees;

    if(turnDegrees >= 0) {
      cw = true;
    } else {
      cw = false;
    }

    // If this command is called, we want to interupt any other commands running
    // on the driving subystem
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Update the desired angle here because we need the current heading
    // when the command is scheduled, not during construction.
    angleDesired = driveTrain.getHeadingDegrees() + angleDesired;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double s = 0.5f;
    double a = 0f;

    if(cw) {
      a = angleDesired - driveTrain.getHeadingDegrees();
    } else {
      a = driveTrain.getHeadingDegrees() - angleDesired;
    }

    // if (a < 25) {
    //   s = 0.5;
    // } else if (a < 35) {
    //   s = 0.5;
    // }
    
    // If we're CCW, flip the motor direction
    if(!cw) {
      s = -s;
    }

    driveTrain.tankDrive(s, -s);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(cw) {
      return driveTrain.getHeadingDegrees() >= angleDesired;
    } else {
      return driveTrain.getHeadingDegrees() <= angleDesired;
    }
  }
}
