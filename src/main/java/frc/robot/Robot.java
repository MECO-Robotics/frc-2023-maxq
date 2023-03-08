// Copyright (c) MECO Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.VideoException;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private Command autonomousCommand;

    private RobotContainer robotContainer;

     private final Compressor compressor = new
     Compressor(PneumaticsModuleType.CTREPCM);

    /***************************************************************************/
    /* COMMON - Runs in all modes */

    /**
     * Robot-wide initialization code should go here.
     *
     * <p>
     * Users should override this method for default Robot-wide initialization which
     * will be called when the robot is first powered on. It will be called exactly
     * one time.
     *
     * <p>
     * Warning: the Driver Station "Robot Code" light and FMS "Robot Ready"
     * indicators will be off until RobotInit() exits. Code in RobotInit() that
     * waits for enable will cause the robot to never indicate that the code is
     * ready, causing the robot to be bypassed in a match.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our autonomous chooser on the dashboard.
        robotContainer = new RobotContainer();

        try {
            if (!RobotBase.isSimulation()) {
                CameraServer.startAutomaticCapture();
            }
        } catch (VideoException videoEx) {
            videoEx.printStackTrace();
        }
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for
     * items like diagnostics that you want ran during disabled, autonomous,
     * teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled

        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();

        robotContainer.robotPeriodic();
    }

    /***************************************************************************/
    /* DISABLED */

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void disabledPeriodic() {
    }

    /***************************************************************************/
    /* AUTONOMOUS */

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {

        CommandScheduler.getInstance().cancelAll();

        autonomousCommand = robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
    }

    /***************************************************************************/
    /* TELEOP */

    /** Called once when switching to teleop mode. */
    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }

        // Reset the position of the bot (in meters x,y, degrees)

        // Left center field
        // TODO: Update with initial pose
        // robotContainer.getDriveSubsystem().setPoseMeters(new Pose2d(1.5, 4.0,
        // Rotation2d.fromDegrees(0)));

        // Lower left blue tarmac, facing hub
        // robotContainer.getDriveSubsystem()
        // .setPoseMeters(new Pose2d(7.532176, 2.963651,
        // Rotation2d.fromDegrees(-69.349228)));

        // Start up the teleop commands

        // Put DriveSubsystem on Shuffleboard
        SmartDashboard.putData(robotContainer.getDriveSubsystem());

        robotContainer.getTeleopCommand().schedule();
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
    }

    /***************************************************************************/
    /* TEST */

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();

        // By default, before testInit() is called the scheduler is disabled, so that
        // default commands are not run and you can test the system.
        // Re-enable here because we're using it for testing.
        CommandScheduler.getInstance().enable();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
        robotContainer.testMode();
    }

    /***************************************************************************/
    /* SIMULATION */

    /**
     * Robot-wide simulation initialization code should go here.
     *
     * <p>
     * Users should override this method for default Robot-wide simulation related
     * initialization
     * which will be called when the robot is first started. It will be called
     * exactly one time after
     * RobotInit is called only when the robot is in simulation.
     */
    public void simulationInit() {
    }

    /**
     * Periodic simulation code should go here.
     *
     * <p>
     * This function is called in a simulated robot after user code executes.
     */
    public void simulationPeriodic() {
    }

}
