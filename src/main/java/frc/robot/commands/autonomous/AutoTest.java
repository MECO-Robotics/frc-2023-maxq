package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveStraight;
import frc.robot.commands.drive.ResetSensors;
import frc.robot.commands.drive.SpinRightAngle;
import frc.robot.commands.drive.Stop;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Travel in a square pattern
 */
public class AutoTest extends SequentialCommandGroup {

    public AutoTest(DriveSubsystem drive) {

       addCommands(
<<<<<<< HEAD
        new DriveStraight(drive, 12, .5)
    
=======
        new DriveStraight(drive, 24),
       new Stop(drive)
>>>>>>> 62cc219d92d4b52be8176816bc811da0a7c67970

       );

        


    }
}
