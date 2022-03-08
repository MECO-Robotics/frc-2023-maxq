package frc.robot.commands.cargo;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CargoSubsystem;

public class Shoot extends SequentialCommandGroup {

    public Shoot(CargoSubsystem cargoSubsystem) {

        addCommands(
                new LowerCargoWrist(cargoSubsystem),
                new RaiseCargoElbow(cargoSubsystem)

        );
    }

}
