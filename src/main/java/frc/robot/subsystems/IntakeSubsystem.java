// Copyright (c) MECO Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Control the pneumatically and motor controlled intake.
 */

public class IntakeSubsystem extends SubsystemBase {

    public IntakeSubsystem() {
        setName("INTAKE");

        SmartDashboard.putData(this);
    }
    
    DoubleSolenoid intakeSolenoid = new DoubleSolenoid(
            PneumaticsModuleType.CTREPCM,
            Constants.INTAKE_EXTEND_PCM,
            Constants.INTAKE_CONTRACT_PCM);

    CANSparkMax intakeController = new CANSparkMax(
            Constants.INTAKE_CONTROLLER_CAN,
            MotorType.kBrushed);

    public void intakeExtend() {
        intakeSolenoid.set(Value.kForward);
    }

    public void intakeContract() {
        intakeSolenoid.set(Value.kReverse);
    }

    public void intakeSpin(double intakeSpeed) {
        intakeController.set(intakeSpeed);
    }

}