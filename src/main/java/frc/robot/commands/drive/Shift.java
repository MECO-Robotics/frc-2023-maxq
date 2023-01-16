// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Switch between full speed / slow speed */
public class Shift extends CommandBase {
  
  private final DriveSubsystem driveTrain;
  private final boolean shiftUp;

  /**
   * Creates a new command.
   *
   * @param driveSubsystem The subsystem used by this command.
   */
  public Shift(DriveSubsystem driveSubsystem, boolean up) {
    driveTrain = driveSubsystem;
    shiftUp = up;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //driveTrain.shift(shiftUp);
  }

  // Returns true when the command should end. (this command never finishes)
  @Override
  public boolean isFinished() {
    return true;
  }
}
