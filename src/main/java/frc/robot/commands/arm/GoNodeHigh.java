package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.ElbowPosition;

public class GoNodeHigh extends CommandBase {

    private final ArmSubsystem armSubsystem;

    public GoNodeHigh(ArmSubsystem arm) {
        armSubsystem = arm;

        addRequirements(arm);
    }

    @Override
    public void execute() {
        armSubsystem.move(Constants.ElbowPosition.middle_HighNode);
        armSubsystem.move(Constants.ShoulderPosition.middle_HighNode);
    }

    @Override
    public boolean isFinished() {
        return true;

    }

}