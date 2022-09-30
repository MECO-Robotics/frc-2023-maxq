package frc.robot.commands.auto;

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

import java.sql.DriverPropertyInfo;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/** Autonomously collect one ball and score 2 balls in the lower hub. */
public class HalfBall extends SequentialCommandGroup {

  /**
   * Create a new command.
   */
  public HalfBall(DriveSubsystem driveSubsystem, CargoSubsystem cargoSubsystem) {

    addCommands(
        // score preloaded ball
        // face the hub
        new RaiseCargoElbow(cargoSubsystem),
        new WaitCommand(6),
        new Outtake(cargoSubsystem).withTimeout(.5),
        new DriveStraight(driveSubsystem, -4 * 12),
        new LowerCargoElbow(cargoSubsystem),
        // drive to get new ball
        new SpinRightAngle(driveSubsystem, 145),
        new DriveStraight(driveSubsystem, 3.0 * 12,1).deadlineWith(new Intake(cargoSubsystem))
      



    );
  }
}
// [Insert secret message here]