package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ResetSensors extends CommandBase {
    private final DriveSubsystem driveTrain;

    /**
     * Creates a new Ecommand.
     *
     * @param driveSubsystem The subsystem used by this command.
     */
    public ResetSensors(DriveSubsystem driveSubsystem) {
        driveTrain = driveSubsystem;

        // Since this is used as a default command, it must specify a required subsystem
        addRequirements(driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        driveTrain.resetSensors();
    }

    // Returns true when the command should end. (this command never finishes)
    @Override
    public boolean isFinished() {
        return true;
    }
}
