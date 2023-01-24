package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoLevelOnChargeStation extends CommandBase {

    private final DriveSubsystem driveTrain;

    // Input range: -15 to 15
    // Output range: -1.0 to 1.0
    private final PIDController pidController = new PIDController(1.0 / 15.0, 0, 0);

    // The number of degrees from 0 that is considered good enough for level
    private static final double TOLERANCE = 3.0;

    // True if we're on the floor, false if we're on the ramp
    private boolean onFloor = false;

    public AutoLevelOnChargeStation(DriveSubsystem driveSubsystem) {

        driveTrain = driveSubsystem;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {

        pidController.setSetpoint(0);
        pidController.setTolerance(TOLERANCE);

        onFloor = Math.abs(driveTrain.getPitch()) <= TOLERANCE;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        if (onFloor) {

            // Just drive forward. Assume teleop has lined us up to the ramp properly.
            // Ideally, we'd use automatic field positioning to make sure we're going up
            // perfectly straight
            driveTrain.robotDrive(0.5, 0, 0);

        } else {

            double forwardLevel = pidController.calculate(driveTrain.getPitch());

            if (forwardLevel == 0.0) {
                driveTrain.stop();

                // TODO drop the brake system
                
            } else {
                driveTrain.robotDrive(forwardLevel, 0, 0);
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

        driveTrain.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {

        return pidController.atSetpoint();
    }

};
