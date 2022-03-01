// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * 2022 Robot:
 * Actuators
 * 2 pneumatic cylinders to telescoping/raise secondary arm
 * 1 motor for telescoping arm winch
 * 1 motor for rotating arm winch
 * Sensors
 * 1 Encoder for each winch
 */
public class ClimbingSubsystem extends SubsystemBase {

  // This solenoid controls the forward and reverse movement of the pneumatic
  // cylinder that raises (forward) or telescopings (contracts) the rotating arm
  // Using the CTRE Pneumatics Control Module (PCM).
  // Need to move these numbers to Constants.
  private final DoubleSolenoid rotatingArmSolenoid = new DoubleSolenoid(
      PneumaticsModuleType.CTREPCM,
      Constants.ROTATING_ARM_DOUBLE_SOLENOID_FWD_PCM,
      Constants.ROTATING_ARM_DOUBLE_SOLENOID_REV_PCM);
  private final AnalogInput pressureSensor = new AnalogInput(Constants.PRESSURE_SENSOR_ANLG);


  private final TalonSRX telescopingLeftArmWinch = new TalonSRX(Constants.TELESCOPING_LEFT_WINCH_CAN);
  private final TalonSRX telescopingRightArmWinch = new TalonSRX(Constants.TELESCOPING_RIGHT_WINCH_CAN);
  private final TalonSRX rotatingLeftArmWinch = new TalonSRX(Constants.ROTATING_LEFT_WINCH_CAN);
  private final TalonSRX rotatingRightArmWinch = new TalonSRX(Constants.ROTATING_RIGHT_WINCH_CAN);

  // private MotorStallMonitor telescopingRightArmWinchStallMonitor = new MotorStallMonitor(telescopingRightArmWinch);
  // private MotorStallMonitor telescopingLeftArmWinchStallMonitor = new MotorStallMonitor(telescopingLeftArmWinch);
  // private MotorStallMonitor rotatingLeftArmWinchStallMonitor = new MotorStallMonitor(rotatingLeftArmWinch);
  // private MotorStallMonitor rotatingRightArmWinchStallMonitor = new MotorStallMonitor(rotatingRightArmWinch);

  // Need to remove & test. Docs indicate this isn't needed

  /** Creates a new subsystem. */
  public ClimbingSubsystem() {

    // Parameters
    // feedbackDevice
    // We're using the VersaPlanetary Integrated Encoder
    // (https://www.vexrobotics.com/217-5046.html) that is directly
    // connected to the Talon SRX Motor Controller
    // (https://store.ctr-electronics.com/talon-srx/).
    // The Mag Encoder can do either relative (quadrature) or absolute (PWM). The
    // relative approach can sample at a
    // higher rate, so using that.
    // pidIdx - 0 for Primary closed-loop. 1 for auxiliary closed-loop.
    //          *** Selected 0 ***
    // timeoutMs - Time to wait for the configuration to complete, and report if an error occurred.
    telescopingLeftArmWinch.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 250);
    telescopingRightArmWinch.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 250);
    rotatingLeftArmWinch.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 250);
    rotatingRightArmWinch.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 250);

    // We want the winches to run in brake mode so that when we're not winding or
    // unwinding it stays put.
    telescopingLeftArmWinch.setNeutralMode(NeutralMode.Brake);
    telescopingRightArmWinch.setNeutralMode(NeutralMode.Brake);
    rotatingLeftArmWinch.setNeutralMode(NeutralMode.Brake);
    rotatingRightArmWinch.setNeutralMode(NeutralMode.Brake);
    addChild("pressure", pressureSensor);
    addChild("rotating Arm Solenoid", rotatingArmSolenoid);
  }

  @Override
  public void periodic() {
/*
    // Check if either telescoping motor has stalled. If it has, stop the motors and
    // set the winch state
    // accordingly.
    telescopingLeftArmWinchStallMonitor.periodic();
    telescopingRightArmWinchStallMonitor.periodic();
    if (telescopingLeftArmWinchStallMonitor.isStalled() || telescopingRightArmWinchStallMonitor.isStalled()) {
      telescopingLeftArmWinch.set(ControlMode.PercentOutput, 0);
      telescopingRightArmWinch.set(ControlMode.PercentOutput, 0);

      // Assume if we're stalled then it's because we're all the way winched in, so reset the encoders.
      telescopingLeftArmWinch.setSelectedSensorPosition(0);
      telescopingRightArmWinch.setSelectedSensorPosition(0);
    }

    // Check if either telescoping motor has stalled. If it has, stop the motors and
    // set the winch state
    // accordingly.
    rotatingLeftArmWinchStallMonitor.periodic();
    rotatingRightArmWinchStallMonitor.periodic();
    if (rotatingLeftArmWinchStallMonitor.isStalled() || rotatingRightArmWinchStallMonitor.isStalled()) {
      rotatingLeftArmWinch.set(ControlMode.PercentOutput, 0);
      rotatingRightArmWinch.set(ControlMode.PercentOutput, 0);
      
      // Assume if we're stalled then it's because we're all the way winched in, so reset the encoders.
      rotatingLeftArmWinch.setSelectedSensorPosition(0);
      rotatingRightArmWinch.setSelectedSensorPosition(0);
    }

    */
  }

  // --------------------------------------------------------------------------
  //
  // Rotating Arm
  //

  /**
   * Set the pneumatic cylinder to extend (forward), and let out the
   * rotating arm winch.
   */
  public void rotatingArmPneumaticOut() {
    System.out.println("CLIMB: Rotating arm solenoid: FWD");
    rotatingArmSolenoid.set(Value.kForward);
  }

  public void rotatingArmPneumaticIn() {
    System.out.println("CLIMB: Rotating arm solenoid: REV");
    rotatingArmSolenoid.set(Value.kReverse);
  }

  public void rotatingArmPneumaticOff() {
    System.out.println("CLIMB: Rotating arm solenoid: OFF");
    rotatingArmSolenoid.set(Value.kOff);
  }

  /**
   * Set the winch state. 0 brings the winch in all the way. 1 lets the winch out all the way.
   * The winch automatically stops when it reaches the desired position.
   * @param position 0 to 1, indicating how far out the winch is desired.
   */
  public void rotatingArmSetWinch(double position) {
    double desiredPosition = Constants.ROTATING_ARM_WINCH_LENGTH_TICKS * position;
    rotatingLeftArmWinch.set(TalonSRXControlMode.Position, desiredPosition);
    rotatingRightArmWinch.set(TalonSRXControlMode.Position, desiredPosition);
  }

  /**
   * Get the position of the left rotating winch.
   * @return 0 if all the way in, 1 if all the way out.
   */
  public double getRotatingArmLeftWinchPosition() {
    return rotatingLeftArmWinch.getSelectedSensorPosition() / Constants.ROTATING_ARM_WINCH_LENGTH_TICKS;
  }

  /**
   * Get the position of the left rotating winch.
   * @return 0 if all the way in, 1 if all the way out.
   */
  public double getRotatingArmRightWinchPosition() {
    return rotatingRightArmWinch.getSelectedSensorPosition() / Constants.ROTATING_ARM_WINCH_LENGTH_TICKS;
  }

  /**
   * Move the rotating arm winch in or out and reset the encoder to zero while doing so.
   * @param speed
   */
  public void rotatingArmWinchMove(double leftSpeed, double rightSpeed) {
    rotatingLeftArmWinch.set(ControlMode.PercentOutput, Constants.ROTATING_DIR * leftSpeed);
    rotatingRightArmWinch.set(ControlMode.PercentOutput, Constants.ROTATING_DIR * rightSpeed);
  }

  // --------------------------------------------------------------------------
  //
  // Telescoping Arm
  //

  public void telescopingArmSetWinch(double position) {
    // Use the TalonSRX builtin Position control mode, which allows us to simply
    // tell
    // the motor controller, using the attached encoder, how many ticks to move the
    // motor
    double desiredPosition = Constants.TELESCOPING_ARM_WINCH_LENGTH_TICKS * position;

    telescopingLeftArmWinch.set(TalonSRXControlMode.Position, desiredPosition);
    telescopingRightArmWinch.set(TalonSRXControlMode.Position, desiredPosition);
  }

  /**
   * Get the position of the left telescoping winch.
   * @return 0 if all the way in, 1 if all the way out.
   */
  public double getTelescopingArmLeftWinchPosition() {
    return telescopingLeftArmWinch.getSelectedSensorPosition() / Constants.TELESCOPING_ARM_WINCH_LENGTH_TICKS;
  }

  /**
   * Get the position of the left telescoping winch.
   * @return 0 if all the way in, 1 if all the way out.
   */
  public double getTelescopingArmRightWinchPosition() {
    return telescopingRightArmWinch.getSelectedSensorPosition() / Constants.TELESCOPING_ARM_WINCH_LENGTH_TICKS;
  }

  /**
   * Move the rotating arm winch in or out and reset the encoder to zero while doing so.
   * @param leftSpeed
   */
  public void telescopingArmWinchMove(double leftSpeed, double rightSpeed) {
    telescopingLeftArmWinch.set(ControlMode.PercentOutput, Constants.TELESCOPING_DIR * leftSpeed);
    telescopingRightArmWinch.set(ControlMode.PercentOutput, Constants.TELESCOPING_DIR * rightSpeed);
  }


  // ------------------------------------------------------------------------------------

  /**
   * Reset the encoders for the winches to zero.
   * @param speed
   */
  public void winchResetZero() {
    rotatingLeftArmWinch.setSelectedSensorPosition(0);
    rotatingRightArmWinch.setSelectedSensorPosition(0);
    telescopingLeftArmWinch.setSelectedSensorPosition(0);
    telescopingRightArmWinch.setSelectedSensorPosition(0);
  }


  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}
