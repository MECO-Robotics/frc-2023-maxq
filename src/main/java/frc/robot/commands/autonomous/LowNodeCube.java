package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.GripperPosition;
import frc.robot.commands.arm.ArmIntake;
import frc.robot.commands.arm.GoNodeHigh;
import frc.robot.commands.arm.GripperManualControl;
import frc.robot.commands.arm.SetGripperPosition;
import frc.robot.commands.brakes.LowerBrakes;
import frc.robot.commands.brakes.RaiseBrakes;
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
public class LowNodeCube extends SequentialCommandGroup {

    //THIS ONE private BrakesSubsystem brakesSubsystem;

    public LowNodeCube(DriveSubsystem drive, ArmSubsystem arm) {

        addCommands(
        //THIS ONE new RaiseBrakes(brakesSubsystem),
        new ParallelDeadlineGroup(new WaitCommand(9), new GoNodeHigh(arm)),
        new DriveStraightByTime(drive, 1, .5),
        new WaitCommand(.5),
        new SetGripperPosition(arm, GripperPosition.GripOpen),
        new DriveStraightByTime(drive, 2, -.5)
        
        
        
        
        
        /*   new DriveStraightByTime(drive, .2, .5),
               new DriveStraightByTime(drive, 1.65, -.8),
               new WaitCommand(1.5),
               new SetGripperPosition(arm, GripperPosition.GripClose),
               new WaitCommand(1),
               new DriveStraightByTime(drive, 2, .5),
               new SpinRightAngle(drive, 180),
               new DriveStraightByTime(drive, .5, -.5),
               new SetGripperPosition(arm, GripperPosition.GripOpen),
               new Stop(drive) */
        
        
        
        
        
        //new SetGripperPosition(arm, GripperPosition.GripOpen),
               // new DriveStraightByTime(drive, 2, .4),
                //new SpinRightAngle(drive, 180),
                //new DriveStraightByTime(drive, 1, .4)
                
                );
                
    }
}
