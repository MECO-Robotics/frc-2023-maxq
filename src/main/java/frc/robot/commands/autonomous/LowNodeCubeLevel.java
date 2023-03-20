package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.GripperPosition;
import frc.robot.commands.arm.ArmIntake;
import frc.robot.commands.arm.GoNodeHighCube;
import frc.robot.commands.arm.GripperManualControl;
import frc.robot.commands.arm.SetGripperPosition;
import frc.robot.commands.brakes.LowerBrakes;
import frc.robot.commands.brakes.RaiseBrakes;
import frc.robot.commands.drive.AutoLevelOnChargeStation;
import frc.robot.commands.drive.DriveStraight;
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
public class LowNodeCubeLevel extends SequentialCommandGroup {

    public LowNodeCubeLevel(DriveSubsystem drive, ArmSubsystem arm, BrakesSubsystem brakes) {

        addCommands(
        // new RaiseBrakes(brakes),
        // new ParallelDeadlineGroup(new WaitCommand(9), new GoNodeHigh(arm)),
        // new DriveStraightByTime(drive, 1, .5),
        // new WaitCommand(.5),
        // new SetGripperPosition(arm, GripperPosition.GripOpen),
        // new DriveStraightByTime(drive, 2, -.5)
        new DriveStraightByTime(drive, 3, 1.0),
        new AutoLevelOnChargeStation(drive)

        );

    }
}
