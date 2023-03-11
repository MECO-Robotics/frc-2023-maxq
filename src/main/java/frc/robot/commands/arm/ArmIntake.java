package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.ElbowPosition;

public class ArmIntake extends CommandBase {

    private final ArmSubsystem armSubsystem;

    public ArmIntake(ArmSubsystem arm) {
        armSubsystem = arm;

        addRequirements(arm);
    }

    @Override
    public void execute() {
        armSubsystem.move(Constants.ElbowPosition.middle_LowNode);
        armSubsystem.move(Constants.ShoulderPosition.middle_LowNode);
    }

    @Override
    public boolean isFinished() {
        return true;

    }

}