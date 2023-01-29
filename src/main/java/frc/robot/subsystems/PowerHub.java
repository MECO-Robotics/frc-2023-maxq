// Copyright (c) MECO Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

public class PowerHub extends SubsystemBase {

  private static final int PDH_CAN_ID = 1;
  private static final int CYCLES_PER_UPDATE = 100;

  private long cycleCount = 0;

  PowerDistribution powerHub = new PowerDistribution(PDH_CAN_ID, ModuleType.kRev);

  public PowerHub() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the susystem scheduled.
  @Override
  public void periodic() {

    if (++cycleCount % CYCLES_PER_UPDATE == 0) {
      SmartDashboard.putNumber("Voltage", powerHub.getVoltage());
      SmartDashboard.putNumber("Total Current", powerHub.getTotalCurrent());

      /**
       * Get the currents of each channel of the PDH and display them on
       * Shuffleboard.
       */
      for (int channel = 0; channel < 6; channel++) {
        double current = powerHub.getCurrent(channel);
        if (current > 0.1) {
          SmartDashboard.putNumber(("Ch" + String.valueOf(channel) + " Current"), current);
        }
      }
    }
  }
}
