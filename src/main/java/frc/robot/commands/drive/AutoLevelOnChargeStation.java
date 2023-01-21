package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoLevelOnChargeStation extends CommandBase {

    private final DriveSubsystem driveTrain;

    private final PIDController pidController = new PIDController(1, 0, 0);
    

    public AutoLevelOnChargeStation(DriveSubsystem driveSubsystem) {

        driveTrain = driveSubsystem;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {

        pidController.setSetpoint(0);
        pidController.setTolerance(1.5);        // Allow up to +/- 1.5 degrees from 0 for the set point
        pidController.enableContinuousInput(-15.0, 15.0);           // The robot pitch should be between -15 and +15 degrees.

        // TODO

        // Record the initial state
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // TODO

        double forwardLevel = pidController.calculate(driveTrain.getPitch());

        driveTrain.robotDrive(0, forwardLevel, 0);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

        driveTrain.robotDrive(0, 0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {

        return pidController.atSetpoint();
    }

};
