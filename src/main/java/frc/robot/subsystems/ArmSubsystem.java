package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GamePiece;

public class ArmSubsystem extends SubsystemBase {

    @Override
    public void periodic() {

    }

    /**
     * Use the color sensor to determine if the game piece is a cone or cube, then
     * grab the game piece using an appropriate amount of force.
     * @return The type of game piece grabbed
     */
    public GamePiece grabGamePiece() {
        return GamePiece.Unknown;
    }

    public void openGrip() {

    }

    public void lowerArm() {

    }

    public void raiseArm() {

    }


    @Override
    public void initSendable(SendableBuilder builder) {
    }
}
