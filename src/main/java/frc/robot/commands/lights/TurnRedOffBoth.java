package frc.robot.commands.lights;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LightSubsystem;

public class TurnRedOffBoth extends CommandBase {
    
    private final LightSubsystem lights;




    public TurnRedOffBoth(LightSubsystem lightSubsystem) {
        lights = lightSubsystem;
    }

    

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    lights.turnBothOffRedFull();
  }

  

  // Returns true when the command should end. (this command never finishes)
  @Override
  public boolean isFinished() {
    return false;
  }





}
