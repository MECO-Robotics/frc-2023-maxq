// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * Drives the robot autonomously in a prescribed pattern.
 * See
 * https://docs.google.com/drawings/d/17LFevv80pT293MpkuqUrxbYcjTK-lv5cHN731H0MZ54/edit?usp=sharing
 */
public class AutoShoot3 extends SequentialCommandGroup {

  /**
   * Create a new command.
   *
   * @param driveSubsystem The subsystem used by this command.
   */
  public AutoShoot3(DriveSubsystem driveSubsystem) {

    // Don't need to add the DriveSubsystem as a required subsystem because the
    // commands
    // scheduled will do that.

    addCommands(  
        // shoot
        new DriveBackward(driveSubsystem, 20),
        new SpinRightDistance(driveSubsystem, 20),
        new DriveForward(driveSubsystem, 20),
        // intake
        new DriveBackward(driveSubsystem, 40),
        new SpinLeftDistance(driveSubsystem, 20),
        // intake
        new SpinLeftDistance(driveSubsystem, 20),
        new DriveForward(driveSubsystem, 20),
        new SpinLeftDistance(driveSubsystem, 20),
        new DriveForward(driveSubsystem, 20),
        // shoot 2 balls
        new SpinRightDistance(driveSubsystem, 20));

  }
}
