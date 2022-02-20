// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.cargo;

import frc.robot.subsystems.CargoSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** This safely raises elbow */
public class LowerWristRaiseElbow extends SequentialCommandGroup {

  /**
   * Create a new command.
   *
   * @param driveSubsystem The subsystem used by this command.
   */
  public LowerWristRaiseElbow(CargoSubsystem cargoSubsystem) {

    // Don't need to add the DriveSubsystem as a required subsystem because the
    // commands
    // scheduled will do that.

    addCommands(new LowerCargoWrist(cargoSubsystem),
        new RaiseCargoElbow(cargoSubsystem));
  }
}
