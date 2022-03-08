// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.util.sendable.SendableBuilder;

public class ControllerSubsystem extends SubsystemBase {

  private final XboxController pilotController = new XboxController(0);
  private final XboxController copilotController = new XboxController(1);
  private final Joystick joystick = new Joystick(0);

  double throttle;
  double turn;
  double liftExtenderSpeed;
  double intakeThrottle;
  double outtakeThrottle;
  boolean armUpButton, armDownButton;
  double manualArm, manualIntake;
  double direction = 1;

  /** Creates a new ExampleSubsystem. */
  public ControllerSubsystem() {
  }

  
  public XboxController getPilotController() {
    return pilotController;
  }

  public XboxController getCopilotController() {
    return copilotController;
  }

  public double getThrottle() {
    return throttle * direction;
  }

  public double getTurn() {
    return turn;
  }

  public void toggleDirection() {
    direction = -direction;
  }

  public double getTankLeft() {
    return pilotController.getLeftY();
  }

  public double getTankRight() {
    return pilotController.getRightY();
  }

  public double getJoystickY() {
    return -joystick.getY();
  }

  public double getJoystickX() {
    return joystick.getX();
  }

  public boolean getArmUpButton() {
    return armUpButton;
  }

  public boolean getArmDownButton() {
    return armDownButton;
  }

  public double getIntakeThrottle() {
    return intakeThrottle;
  }

  public double getOuttakeThrottle() {
    return outtakeThrottle;
  }

  public double getManualArm() {
    return manualArm;
  }

  public double getManualIntake() {
    return manualIntake;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Pilot controls - dead band handled by the DifferentialDrive class within the DriveSubsystem

    // Gamepads use negative values for up, and positive for down
    throttle = -pilotController.getLeftY();
    // Gamepads use positive values for right, negative for left
    turn = pilotController.getRightX();

    // Copilot controls
    outtakeThrottle = deadzone(copilotController.getRightTriggerAxis(), 0.05);
    intakeThrottle = deadzone(copilotController.getLeftTriggerAxis(), 0.05);
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  /**
   * Return value unless it is within deadzone of zero, then just return zero.
   */
  public static double deadzone(double value, double deadzone) {
    if (Math.abs(value) < deadzone)
      return 0;
    return value;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }
}
