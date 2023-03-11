package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.ElbowPosition;

public class ArmStow extends CommandBase {

    private final ArmSubsystem armSubsystem;

    public ArmStow(ArmSubsystem arm) {
        armSubsystem = arm;

        addRequirements(arm);
    }

    @Override
    public void execute() {
        armSubsystem.move(Constants.ElbowPosition.allIn);
        armSubsystem.move(Constants.ShoulderPosition.allBackStow);
    }

    @Override
    public boolean isFinished() {
        return true;

    }

}