// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import frc.robot.Constants;

/**
 * An arm with a roller to grab and shoot balls.
 */
public class BallCollectionSubsystem extends SubsystemBase {

  double currentArmMotorSpeed = 0.0;
  Victor armLift = new Victor(Constants.ARM_LIFT);
  Victor intake = new Victor(Constants.INTAKE);
  DigitalInput bottomLimitSwitch = new DigitalInput(Constants.TOP_LIMIT_SWITCH);
  DigitalInput topLimitSwitch = new DigitalInput(Constants.BOTTOM_LIMIT_SWITCH);

  // Timer to track the time spent raising or lowering
  private Timer armTimer = new Timer();

  /** Creates a new subsystem. */
  public BallCollectionSubsystem() {
    armLift.set(0);
    intake.set(0);

    addChild("Arm motor", armLift);
    addChild("Intake motor", intake);
    addChild("Bottom limit", bottomLimitSwitch);
    addChild("Upper limit", topLimitSwitch);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public double getCurrentArmMotorSpeed() {
    return currentArmMotorSpeed;
  }

  /**
   * Additional method of controlling the ball collection subsystem that
   * ignores the limit switches or any time limits. The copilot has complete
   * control over both motor actuators.
   */
  public void manualControl(double armSpeed, double intakeSpeed) {
      armLift.set(armSpeed);
      intake.set(intakeSpeed);
  }
  /**
   * Determine the arm motor speed given the game pad inputs. Only one button should 
   * be pressed at a time, but the condition is possible, so it needs to be
   * handled. If neither button is currently pressed, it may mean the arm
   * needs to keep moving or it could mean it's time to stop it.
   *
   * @param upButton         Move the arm all the way up
   * @param downButton       Move the arm all the way down
   */
  public void moveArm(
      boolean upButton, 
      boolean downButton) {
            
    boolean topLimit = topLimitSwitch.get();
    boolean bottomLimit = bottomLimitSwitch.get();

      // IF up button pressed THEN
      //   IF up limit NOT reached AND the motor is moving down or is stopped THEN
      //     Set motor speed to 0.5
      //   ELSE IF up limit switch reached
      //     Set motor speed to 0
      
      if (upButton == true) {
          if (topLimit == true && currentArmMotorSpeed <= 0.0 ) {
              // start moving up
              currentArmMotorSpeed = 0.5;

              // track when we started moving
              armTimer.reset();
              armTimer.start();

              System.out.println("COLLECTION ARM: Up button pressed. Moving up");

          } else if(topLimit == false) {

              System.out.printf("COLLECTION ARM: Upper limit reached in %.3f seconds. Speed was %f. Now stopped.\n", armTimer.get(), currentArmMotorSpeed);

              // We're at the top, but the user pressed the button, so just 
              // make sure the motor is off (already should be)
              currentArmMotorSpeed = 0;
          }
      }

      // ELSE IF down button pressed THEN
      //   IF down limit NOT reached THEN
      //     Set motor speed to 0.5
      //   ELSE
      //     Set motor speed to 0.0
      
    
      else if (downButton == true  ) {
          if ( bottomLimit == true && currentArmMotorSpeed >= 0.0 ) {
              // start moving down
              currentArmMotorSpeed = -0.5;

              // track when we started moving
              armTimer.reset();
              armTimer.start();

              System.out.println("COLLECTION ARM: Down button pressed. Moving up");

          } else if (bottomLimit== false) {

              System.out.printf("COLLECTION ARM: Down limit reached in %.3f seconds. Speed was %f. Now stopped.\n", armTimer.get(), currentArmMotorSpeed);

              // We're at the bottom, but the user pressed the down button, so just
              // make sure the motor is off (already should be)
              currentArmMotorSpeed = 0;
          }
      }
      
      // ELSE IF the arm is moving up and the user hasn't pressed any buttons THEN
      //   IF up limit switch reached THEN
      //     Set set motor speed to 0

      else if ( currentArmMotorSpeed > 0.0) {
          if ( topLimit == false) {
              System.out.printf("COLLECTION ARM: Upper limit reached in %.3f seconds. Speed was %f. Now stopped.\n", armTimer.get(), currentArmMotorSpeed);
              currentArmMotorSpeed = 0.0;
          } else {
              // We're moving up, but haven't hit the limit switch yet. If we've traveled for 
              // 0.7 seconds, move the remaining distance to the limit switch very slowly
              if (armTimer.hasElapsed(0.7)) {
                  currentArmMotorSpeed = 0.1;
                  System.out.printf("COLLECTION ARM: %.3f seconds elapsed while raising arm. Switching to slow speed. \n", armTimer.get());
              }
          }
      }
      
      // ELSE IF motor speed LESS THAN 0 THEN
      //   IF down limit switch reached THEN
      //     Set motor speed to 0

      else if (currentArmMotorSpeed < 0) {
          if (bottomLimit == false) {
              System.out.printf("COLLECTION ARM: Down limit reached in %.3f seconds. Speed was %f. Now stopped.\n", armTimer.get(), currentArmMotorSpeed);
              currentArmMotorSpeed = 0.0;
          } else {
              // We're moving down, but haven't hit the limit switch yet. If we've traveled for 
              // 0.7 seconds, move the remaining distance to the limit switch very slowly
              if (armTimer.hasElapsed(0.7)) {
                  System.out.printf("COLLECTION ARM: %.3f seconds elapsed while lowering arm. Switching to slow speed. \n", armTimer.get());
                  currentArmMotorSpeed = -0.1;
              }
          }
      }
      
      armLift.set(currentArmMotorSpeed);
  }        

  /**
   * Determine the roller motor speed given the intakeTrigger level and
   * the outTakeTrigger level. If both triggers are pulled, or neither are pulled,
   * the motor speed should be 0.
   */
  public void moveIntake(double intakeTriggerLevel, double outTakeTriggerLevel) {
      
      double intakeMotorSpeed = 0;
      
      // IF intake level GREATER THAN 0 THEN
      //   Set intake motor speed to the negative of the intake level
      // ELSE IF outtake level GREATER THAN 0 THEN
      //   Set the intake motor speed to the outtake trigger level
      
      if ( intakeTriggerLevel > 0 )  {
          intakeMotorSpeed = -intakeTriggerLevel;            
      } else if ( outTakeTriggerLevel > 0 ) {
          intakeMotorSpeed = outTakeTriggerLevel;
      }

      intake.set(intakeMotorSpeed);
  }
}
