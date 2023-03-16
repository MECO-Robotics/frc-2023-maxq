package frc.robot.commands.lights;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.LightSubsystem;

public class Signal extends SequentialCommandGroup {

    public Signal(LightSubsystem lightSubsystem, Color colour) {
        addCommands(
                new ParallelDeadlineGroup(
                        new WaitCommand(5),
                        new Blink(lightSubsystem, colour)));
    }

}
