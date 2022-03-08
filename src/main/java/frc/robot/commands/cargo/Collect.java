package frc.robot.commands.cargo;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CargoSubsystem;

public class Collect extends SequentialCommandGroup {
    public Collect(CargoSubsystem cargoSubsystem) {

        addCommands(
                new LowerCargoWrist(cargoSubsystem),
                new LowerCargoElbow(cargoSubsystem)

        );
    }
}
