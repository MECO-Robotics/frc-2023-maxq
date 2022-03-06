// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.cargo;

import frc.robot.subsystems.CargoSubsystem;
import frc.robot.subsystems.ControllerSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Spins the intake roller on demand from the pilot's triggers */
public class PilotTriggersControlIntake extends CommandBase {

  private final CargoSubsystem cargo;
  private final ControllerSubsystem control;

  /**
   * Creates a new Command.
   *
   * @param cargoSubsystem The subsystem used by this command.
   */
  public PilotTriggersControlIntake(CargoSubsystem cargoSubsystem, ControllerSubsystem controllerSubystem) {
    cargo = cargoSubsystem;
    control = controllerSubystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // So if left is 1 and right is 0:  1 = intake
    // if left is 0 and right is 1:    -1 = outtake
    cargo.setIntakeRoller(
      control.getPilotController().getLeftTriggerAxis() - control.getPilotController().getRightTriggerAxis());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    cargo.setIntakeRoller(0);
   }

  // Returns true when the command should end. (this command never finishes)
  @Override
  public boolean isFinished() {
    return false;
  }
}
