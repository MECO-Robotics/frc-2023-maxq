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
        new DriveStraight(driveSubsystem, -50),
        new ParallelCommandGroup(
            new SpinRightAngle(driveSubsystem, 178),
            new LowerCargoWrist(cargoSubsystem)

        ),

        // Grab the ball

        new DriveStraight(driveSubsystem, 30).deadlineWith(new Intake(cargoSubsystem)),

        // Return to the hub
        new SpinRightAngle(driveSubsystem, -178),
        new ParallelCommandGroup(
          new RaiseCargoElbow(cargoSubsystem),
          new DriveStraight(driveSubsystem, 70)),

        // Deposit 2 balls in the hub
        new Outtake(cargoSubsystem).withTimeout(1),

        new DriveStraight(driveSubsystem, -50),
        new SpinRightAngle(driveSubsystem, 175),
        new LowerCargoElbow(cargoSubsystem),
        new LowerCargoWrist(cargoSubsystem),
        new DriveStraight(driveSubsystem, 30).deadlineWith(new Intake(cargoSubsystem)),
        new DriveStraight(driveSubsystem, -30),
        new SpinRightAngle(driveSubsystem, -170),
         new RaiseCargoElbow(cargoSubsystem).deadlineWith(new RaiseCargoWrist(cargoSubsystem)),
         new DriveStraight(driveSubsystem, 30).deadlineWith(new Intake(cargoSubsystem))
        

        

    );
  }
}
// [Insert secret message here]