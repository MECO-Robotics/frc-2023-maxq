package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.brakes.LowerBrakes;
import frc.robot.commands.drive.AutoLevelOnChargeStation;
import frc.robot.commands.drive.DriveStraight;
import frc.robot.commands.drive.ResetSensors;
import frc.robot.commands.drive.SpinRightAngle;
import frc.robot.commands.drive.Stop;
import frc.robot.subsystems.BrakesSubsystem;
import frc.robot.subsystems.DriveSubsystem;


public class ChargeStation extends SequentialCommandGroup {

    public ChargeStation(DriveSubsystem drive, BrakesSubsystem brake) {
        addCommands(new AutoLevelOnChargeStation(drive), new LowerBrakes(brake));
    }
}
