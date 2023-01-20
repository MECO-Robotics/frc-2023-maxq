package frc.robot.subsystems;

import javax.sound.sampled.Port;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

    // Uses this color sensor: https://www.revrobotics.com/rev-31-1557/
    ColorSensorV3 colorSensor;

    @Override
    public void periodic() {

    }

    /**
     * Use the color sensor to determine if the game piece is a cone or cube, then
     * grab the game piece using an appropriate amount of force.
     */
    public void grabGamePiece() {

    }

    @Override
    public void initSendable(SendableBuilder builder) {
    }
}
