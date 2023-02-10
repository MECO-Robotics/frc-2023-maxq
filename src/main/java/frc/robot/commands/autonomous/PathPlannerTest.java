package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class PathPlannerTest extends CommandBase {

    private final DriveSubsystem driveTrain;

    /**
     * Creates a new ExampleCommand.
     *
     * @param driveSubsystem The subsystem used by this command.
     */
    public PathPlannerTest(DriveSubsystem driveSubsystem) {
        driveTrain = driveSubsystem;

        // Cancel any other currently running drive subsystem commands before we run.
        // this includes the StopCommand, which is the default command always running
        // for the drive subsystem.
        addRequirements(driveSubsystem);
    }

    public void execute() {
        // This will load the file "Example Path.path" and generate it with a max
        // velocity of 4 m/s and a max acceleration of 3 m/s^2
        PathPlannerTrajectory examplePath = PathPlanner.loadPath("test Path", new PathConstraints(4, 3));

        // This trajectory can then be passed to a path follower such as a
        // PPSwerveControllerCommand
        // Or the path can be sampled at a given point in time for custom path following

        // Sample the state of the path at 1.2 seconds
        PathPlannerState exampleState = (PathPlannerState) examplePath.sample(1.2);
        exampleState.poseMeters.getX();
        exampleState.poseMeters.getY();
        exampleState.poseMeters.getRotation();


       // driveTrain.fieldDrive(exampleState.poseMeters.getX(), exampleState.poseMeters.getY(), exampleState.poseMeters.getRotation());


        // Print the velocity at the sampled time
        System.out.println(exampleState.velocityMetersPerSecond);

    }
}
