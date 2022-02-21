// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import frc.robot.commands.cargo.Intake;
import frc.robot.commands.cargo.LowerCargoElbow;
import frc.robot.commands.cargo.LowerCargoWrist;
import frc.robot.commands.cargo.Outtake;
import frc.robot.commands.cargo.RaiseCargoElbow;
import frc.robot.commands.cargo.RaiseCargoWrist;
import frc.robot.commands.drive.DriveStraight;
import frc.robot.commands.drive.SpinRightAngle;
import frc.robot.subsystems.CargoSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CargoSubsystem;

/** Drives the robot autonomously in a plus sign pattern. */
public class BallAuto extends SequentialCommandGroup {

  /**
   * Create a new command.
   *
   * @param driveSubsystem The subsystem used by this command.
   */
  public BallAuto(DriveSubsystem driveSubsystem, CargoSubsystem cargoSubsystem) {

    // Don't need to add the DriveSubsystem as a required subsystem because the
    // commands
    // scheduled will do that.

    addCommands(
        // already have 1 ball preloaded
        new DriveStraight(driveSubsystem, -50),
        new SpinRightAngle(driveSubsystem, 178),
        new ParallelCommandGroup(
            new LowerCargoWrist(cargoSubsystem),
            new DriveStraight(driveSubsystem, 30)),
        new LowerCargoWrist(cargoSubsystem),
        new DriveStraight(driveSubsystem, 30),
        new Intake(cargoSubsystem).withTimeout(5),
        new RaiseCargoWrist(cargoSubsystem),
        new SpinRightAngle(driveSubsystem, -178),
        new LowerCargoWrist(cargoSubsystem),
        new RaiseCargoElbow(cargoSubsystem),
        new DriveStraight(driveSubsystem, 70),
        new Outtake(cargoSubsystem).withTimeout(5),
        new DriveStraight(driveSubsystem, -50),
        new SpinRightAngle(driveSubsystem, 90),
        new DriveStraight(driveSubsystem, 50),
        new LowerCargoElbow(cargoSubsystem),
        new RaiseCargoWrist(cargoSubsystem)

    );
  }
}
