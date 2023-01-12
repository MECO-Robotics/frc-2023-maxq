// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Stop the robot from moving. */
public class Stop extends CommandBase {
  
  private final DriveSubsystem driveTrain;

  /**
   * Creates a new Ecommand.
   *
   * @param driveSubsystem The subsystem used by this command.
   */
  public Stop(DriveSubsystem driveSubsystem) {
    driveTrain = driveSubsystem;

    // Since this is used as a default command, it must specify a required subsystem
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  //  driveTrain.tankDrive(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end. (this command never finishes)
  @Override
  public boolean isFinished() {
    return false;
  }
}
