package frc.robot.commands.auto;

import frc.robot.commands.cargo.Intake;
import frc.robot.commands.cargo.LowerCargoElbow;
import frc.robot.commands.cargo.LowerCargoWrist;
import frc.robot.commands.cargo.LowerWristRaiseElbow;
import frc.robot.commands.cargo.Outtake;
import frc.robot.commands.cargo.RaiseCargoElbow;
import frc.robot.commands.cargo.RaiseCargoWrist;
import frc.robot.commands.drive.DriveStraight;
import frc.robot.commands.drive.SpinRightAngle;
import frc.robot.subsystems.CargoSubsystem;
import frc.robot.subsystems.DriveSubsystem;

import java.sql.DriverPropertyInfo;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Autonomously collect one ball and score 2 balls in the lower hub. */
public class ThreeBallAuto extends SequentialCommandGroup {

  /**
   * Create a new command.
   */
  public ThreeBallAuto(DriveSubsystem driveSubsystem, CargoSubsystem cargoSubsystem) {

    addCommands(
        // Starting conditions:
        // 1) Pointed toward the hub, bumper touching
        // 2) 1 ball preloaded

        // Drive backward and turn towards the ball to our back right
      new Outtake(cargoSubsystem).withTimeout(2),
      new DriveStraight(driveSubsystem, -30),
      new SpinRightAngle(driveSubsystem, 170),
      new ParallelCommandGroup(
        new DriveStraight(driveSubsystem, 5),
        new Intake(cargoSubsystem).withTimeout(5)
      ),
      new SpinRightAngle(driveSubsystem, 95),
      new DriveStraight(driveSubsystem, 30),
      new ParallelCommandGroup(
        new DriveStraight(driveSubsystem, 5),
        new Intake(cargoSubsystem).withTimeout(5)
      )
      
    );
  }
}
// [Insert secret message here]