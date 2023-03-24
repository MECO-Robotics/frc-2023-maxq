package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.GripperPosition;
import frc.robot.commands.arm.GoNodeHighCube;
import frc.robot.commands.arm.SetGripperPosition;
import frc.robot.commands.brakes.RaiseBrakes;
import frc.robot.commands.drive.DriveStraightByTime;
import frc.robot.commands.drive.ResetSensors;
import frc.robot.commands.drive.SpinRightAngle;
import frc.robot.commands.drive.Stop;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.BrakesSubsystem;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Travel in a square pattern
 */
public class LowNodeCube extends SequentialCommandGroup {

    public LowNodeCube(DriveSubsystem drive, ArmSubsystem arm, BrakesSubsystem brakes) {

        addCommands(
                new RaiseBrakes(brakes),
                new DriveStraightByTime(drive, 1.3, -.5),
                new ParallelRaceGroup(new WaitCommand(10), new GoNodeHighCube(arm)),
                new DriveStraightByTime(drive, 1.9, .25),
                new WaitCommand(.5),
                new SetGripperPosition(arm, GripperPosition.GripOpen),
                new WaitCommand(.5),
                new DriveStraightByTime(drive, 4.0, -.25),
                new SpinRightAngle(drive, 180)

        /*
         * new DriveStraightByTime(drive, .2, .5),
         * new DriveStraightByTime(drive, 1.65, -.8),
         * new WaitCommand(1.5),
         * new SetGripperPosition(arm, GripperPosition.GripClose),
         * new WaitCommand(1),
         * new DriveStraightByTime(drive, 2, .5),
         * new SpinRightAngle(drive, 180),
         * new DriveStraightByTime(drive, .5, -.5),
         * new SetGripperPosition(arm, GripperPosition.GripOpen),
         * new Stop(drive)
         */

        // new SetGripperPosition(arm, GripperPosition.GripOpen),
        // new DriveStraightByTime(drive, 2, .4),
        // new SpinRightAngle(drive, 180),
        // new DriveStraightByTime(drive, 1, .4)

        );

    }
}
