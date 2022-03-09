// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.cargo;

import frc.robot.subsystems.CargoSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Spins the intake roller to pull in balls */
public class Outtake extends CommandBase {

  private final CargoSubsystem cargo;

  /**
   * Creates a new Command.
   *
   * @param cargoSubsystem The subsystem used by this command.
   */
  public Outtake(CargoSubsystem cargoSubsystem) {
    cargo = cargoSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    cargo.setIntakeRoller(1);
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
