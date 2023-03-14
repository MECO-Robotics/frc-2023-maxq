package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveStraight;
import frc.robot.commands.drive.DriveStraightByTime;
import frc.robot.commands.drive.ResetSensors;
import frc.robot.commands.drive.SpinRightAngle;
import frc.robot.commands.drive.Stop;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Travel in a square pattern
 */
public class PlaceBackOutFar extends SequentialCommandGroup {

    public PlaceBackOutFar(DriveSubsystem drive) {

        addCommands(
        // start up against cone node farthest to substation side
        // back up
        new DriveStraightByTime(drive, 3, .25)
        // pull up arm
        //new GoNodeHigh(),
        // move forward

        // drop cone

        // move back and pull arm down

        );

        


    }
}
