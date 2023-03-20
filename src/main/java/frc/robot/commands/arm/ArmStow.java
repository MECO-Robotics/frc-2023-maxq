// Copyright (c) MECO Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.ElbowPosition;
import frc.robot.Constants.ShoulderPosition;

public class ArmStow extends CommandBase {

    private final ArmSubsystem armSubsystem;
    boolean doneElbow = false;
    boolean doneShoulder = false;
    private boolean almostDone = false;

    public ArmStow(ArmSubsystem arm) {
        armSubsystem = arm;
    }

    @Override
    public void initialize() {
        doneElbow = false;
        doneShoulder = false;
        System.out.println("ArmStow: STARTED");
    }

    @Override
    public void execute() {
        if (!doneElbow) {
            doneElbow = armSubsystem.move(ElbowPosition.stow);
        }

        if (!doneShoulder) {
            doneShoulder = armSubsystem.move(ShoulderPosition.allBackStow);
        }
    }

    @Override
    public boolean isFinished() {
        if (doneElbow || doneShoulder) {
            if (!almostDone || (doneElbow && doneShoulder)) {
                System.out.println("ArmStow: ELBOW:" + doneElbow + "; SHOULDER:" + doneShoulder);
                almostDone = true;
            }
        }
        return doneElbow && doneShoulder;

    }

}