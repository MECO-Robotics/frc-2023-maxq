package frc.robot.commands.autonomous;

import java.util.function.Consumer;
import java.util.function.Supplier;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPMecanumControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

/**
 * 
 */
public class FollowPath extends SequentialCommandGroup {

    private final DriveSubsystem driveTrain;

    /**
     * Create a new command.
     *
     * @param driveSubsystem The subsystem used by this command.
     * @param path The path to follow
     */
    public FollowPath(DriveSubsystem driveSubsystem, PathPlannerTrajectory pathToFollow) {
        driveTrain = driveSubsystem;

        // X controller. Tune these values for your robot. Leaving them 0
        // will only use feedforwards.
        PIDController xPidController = new PIDController(0, 0, 0);

        // Y controller (usually the same values as X controller)
        PIDController yPidController = new PIDController(0, 0, 0);

        // Rotation controller. Tune these values for your robot. Leaving
        // them 0 will only use feedforwards.
        PIDController rotationPidController = new PIDController(0, 0, 0);

        // Cancel any other currently running drive subsystem commands before we run.
        // this includes the StopCommand, which is the default command always running
        // for the drive subsystem.
        addRequirements(driveSubsystem);

        Supplier<Pose2d> poseGetter = () -> {
            return driveTrain.getPoseMeters();
        };
        Consumer<ChassisSpeeds> chassisSpeedSetter = (ChassisSpeeds speeds) -> {
            driveTrain.setChassisSpeeds(speeds);
        };

        super.addCommands(
                new PPMecanumControllerCommand(
                    pathToFollow,
                        poseGetter,
                        xPidController,
                        yPidController,
                        rotationPidController,
                        chassisSpeedSetter,
                        driveTrain));
    }
}
