package frc.robot.commands.cargo;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CargoSubsystem;

public class Stow extends SequentialCommandGroup {
    public Stow(CargoSubsystem cargoSubsystem) {

        addCommands(
                new RaiseCargoWrist(cargoSubsystem),
                new LowerCargoElbow(cargoSubsystem)
        );
    }
}
