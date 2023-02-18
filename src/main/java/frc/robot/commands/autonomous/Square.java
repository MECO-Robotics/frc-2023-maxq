package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.ResetSensors;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Travel in a square pattern
 */
public class Square extends SequentialCommandGroup {

    public Square(DriveSubsystem drive) {

        // This will load the file "test Path.path" and generate it with a max
        // velocity of 4 m/s and a max acceleration of 3 m/s^2
        PathPlannerTrajectory examplePath = PathPlanner.loadPath("test Path", new PathConstraints(4, 3));

        addCommands(
                new ResetSensors(drive),
                new FollowPath(drive, examplePath));
    }
}
