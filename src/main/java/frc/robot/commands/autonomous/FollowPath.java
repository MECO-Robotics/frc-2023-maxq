package frc.robot.commands.autonomous;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPMecanumControllerCommand;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.commands.drive.ResetSensors;
import frc.robot.subsystems.DriveSubsystem;

public class FollowPath extends CommandBase {

    private final DriveSubsystem driveTrain;

    /**
     * Creates a new ExampleCommand.
     *
     * @param driveSubsystem The subsystem used by this command.
     */
    public FollowPath(DriveSubsystem driveSubsystem) {
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

        Supplier<Pose2d> poseGetter = () -> {
            return driveTrain.getPoseMeters();
        };
        Consumer<ChassisSpeeds> chassisSpeedSetter = (ChassisSpeeds speeds) -> {
            driveTrain.setChassisSpeeds(speeds);
        };
        Subsystem[] requirments = new Subsystem[] { driveTrain };

        // Assuming this method is part of a drivetrain subsystem that provides the
        // necessary methods
        Command command = new SequentialCommandGroup(
                new ResetSensors(driveTrain),
                new PPMecanumControllerCommand(
                        examplePath,
                        poseGetter, // Pose supplier
                        new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0
                                                    // will only use feedforwards.
                        new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
                        new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving
                                                    // them 0 will only use feedforwards.
                        chassisSpeedSetter,
                        requirments));
    }
}
