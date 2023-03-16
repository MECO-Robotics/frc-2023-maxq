// Copyright (c) MECO Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.ElbowPosition;
import frc.robot.Constants.ShoulderPosition;

public class GoNodeMid extends CommandBase {

    private final ArmSubsystem armSubsystem;
    boolean doneElbow = false;
    boolean doneShoulder = false;
    int logger = 0;
    private boolean almostDone = false;



    public GoNodeMid(ArmSubsystem arm) {
        armSubsystem = arm;

        addRequirements(arm);
    }

    
    @Override
    public void initialize() {
        System.out.println("GoNodeMid: STARTED");
    }
    
    @Override
    public void execute() {
        
        doneElbow = armSubsystem.move(ElbowPosition.middle_MiddleNode);

        doneShoulder = armSubsystem.move(ShoulderPosition.middle_MiddleNode);
    }

    @Override
    public boolean isFinished() {
        if(logger++ % 10 == 0) System.out.println("GoNodeMid: ELBOW:" + doneElbow + "; SHOULDER:" + doneShoulder);

        if(doneElbow && doneShoulder) {
            System.out.println("GoNodeMid: DONE");
        }
        return doneElbow && doneShoulder;
    }

}