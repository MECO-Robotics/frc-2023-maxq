// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Drives the robot autonomously in a plus sign pattern. */
public class AutoShootCollect extends SequentialCommandGroup {

  /**
   * Create a new command.
   *
   * @param driveSubsystem The subsystem used by this command.
   */
  public AutoShootCollect(DriveSubsystem driveSubsystem) {

    // Don't need to add the DriveSubsystem as a required subsystem because the
    // commands
    // scheduled will do that.

    addCommands(
        //shoot
        new DriveStraight(driveSubsystem, -20),
        new SpinRightAngle(driveSubsystem, 180),
        new DriveStraight(driveSubsystem, 20),
        //intake
        new DriveStraight(driveSubsystem, -20),
        new SpinRightAngle(driveSubsystem, 180),
        new DriveStraight(driveSubsystem, 20)
        //shoot

    );

  }
}
