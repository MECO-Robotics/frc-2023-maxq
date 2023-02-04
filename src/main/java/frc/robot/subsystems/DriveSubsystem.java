// Copyright (c) MECO Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.math.util.Units;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import frc.robot.Constants;

/**
 * A mecanum drive train. The setup is as follows:
 * Motors are setup with a single stage of pinions of 12 to 72 from motor to
 * wheel (1:6), so:
 * 
 * Left motors: Not inverted
 * Right motors: Inverted
 */

public class DriveSubsystem extends SubsystemBase {

    // We're using this encoder: https://www.revrobotics.com/rev-11-1271/

    private static final int ENCODER_RESOLUTION = 8192;
    private static final double WHEEL_DIAMETER_INCHES = 6.0; // inches
    private static final double WHEEL_CIRCUM_METERS = Units.inchesToMeters(WHEEL_DIAMETER_INCHES) * Math.PI;
    // TODO Add a constant that is the maximum wheel speed. We'll need this in the
    // fieldDriveRelativeSpeed function

    private static final Translation2d FRONT_LEFT_POS = new Translation2d(9.25, 24.23);
    private static final Translation2d FRONT_RIGHT_POS = new Translation2d(-9.25, -24.23);
    private static final Translation2d BACK_LEFT_POS = new Translation2d(9.25, 24.23);
    private static final Translation2d BACK_RIGHT_POS = new Translation2d(-9.25, -24.23);

    // This object defines the properties of how the robot turns
    private final MecanumDriveKinematics driveKinematics = new MecanumDriveKinematics(FRONT_LEFT_POS, FRONT_RIGHT_POS,
            BACK_LEFT_POS, BACK_RIGHT_POS);

    // The Inertial Measurement Unit (IMU) connects to the RoboRio through the MXP
    // port, which is the wide female pin header in the middle of the Rio. Through
    // the MXP, the Serial Port Interface (SPI) is used.

    private final AHRS imu = new AHRS(SPI.Port.kMXP);

    // Converts tank or arcade drive speed requests to voltage requests to the motor
    // controllers
    private MecanumDrive drive;

    // Keeps track of where we are on the field based on gyro and encoder inputs.
    private final MecanumDriveOdometry odometry;

    // Sensor simulations
    private final Field2d field2d = new Field2d();

    CANSparkMax frontLeftController;
    CANSparkMax frontRightController;
    CANSparkMax backLeftController;
    CANSparkMax backRightController;

    RelativeEncoder frontLeftEncoderRev;
    RelativeEncoder backRightEncoderRev;
    RelativeEncoder frontRightEncoderRev;
    RelativeEncoder backLeftEncoderRev;

    /* Creates a new Subsystem. */
    public DriveSubsystem() {

        configureUsingSparkMaxMotorControllers();

        // Setting this conversion factor allows us to get a distance traveled when we
        // call RelativeEncoder.getPosition() from the getWheelPositions() method.
        frontLeftEncoderRev.setPositionConversionFactor(WHEEL_CIRCUM_METERS / ENCODER_RESOLUTION);
        frontRightEncoderRev.setPositionConversionFactor(WHEEL_CIRCUM_METERS / ENCODER_RESOLUTION);
        backLeftEncoderRev.setPositionConversionFactor(WHEEL_CIRCUM_METERS / ENCODER_RESOLUTION);
        backRightEncoderRev.setPositionConversionFactor(WHEEL_CIRCUM_METERS / ENCODER_RESOLUTION);

        // Set the initial position (0,0) and heading (whatever it is) of the robot on
        // the field
        odometry = new MecanumDriveOdometry(driveKinematics,
                imu.getRotation2d(), getWheelPositions());

        addChild("Drive", drive);
        addChild("IMU", imu);
        addChild("Field", field2d);
    }

    public void configureUsingSparkMaxMotorControllers() {

        frontLeftController = new CANSparkMax(Constants.FRONT_LEFT_CAN, MotorType.kBrushed);
        frontLeftController.setIdleMode(IdleMode.kBrake);
        frontLeftController.setInverted(true);

        frontRightController = new CANSparkMax(Constants.FRONT_RIGHT_CAN, MotorType.kBrushed);
        frontRightController.setIdleMode(IdleMode.kBrake);
        frontRightController.setInverted(false);

        backLeftController = new CANSparkMax(Constants.BACK_LEFT_CAN, MotorType.kBrushed);
        backLeftController.setIdleMode(IdleMode.kBrake);
        backLeftController.setInverted(true);

        backRightController = new CANSparkMax(Constants.BACK_RIGHT_CAN, MotorType.kBrushed);
        backRightController.setIdleMode(IdleMode.kBrake);
        backRightController.setInverted(false);

        // TODO Verify we're using the Encoder and not the AlternateEncoder.
        frontLeftEncoderRev = frontLeftController.getEncoder(Type.kQuadrature, ENCODER_RESOLUTION);
        backRightEncoderRev = backRightController.getEncoder(Type.kQuadrature, ENCODER_RESOLUTION);
        frontRightEncoderRev = frontRightController.getEncoder(Type.kQuadrature, ENCODER_RESOLUTION);
        backLeftEncoderRev = backLeftController.getEncoder(Type.kQuadrature, ENCODER_RESOLUTION);

        drive = new MecanumDrive(frontLeftController, backLeftController, frontRightController, backRightController);
    }

    /**
     * Get the wheel positions
     * 
     * @return
     */
    private MecanumDriveWheelPositions getWheelPositions() {

        // TODO Verify that the getPosition is return distance
        return new MecanumDriveWheelPositions(
                frontLeftEncoderRev.getPosition(),
                frontRightEncoderRev.getPosition(),
                backLeftEncoderRev.getPosition(),
                backRightEncoderRev.getPosition());

    }

    /**
     * This method will be called once per scheduler run
     */
    @Override
    public void periodic() {

        odometry.update(imu.getRotation2d(), getWheelPositions());

        // This allows us to see the robot's position and orientation on the
        // Shuffleboard
        field2d.setRobotPose(getPoseMeters());

        // Update the motor safety. This makes sure the system knows we're in control of
        // the motors.
        // If the code crashes, or somehow gets stuck in a loop, and this method isn't
        // called then the motors will automaitcally stop.
        drive.feed();
    }

    /***********************************************************************/
    /* ROBOT RELATIVE DRIVE */
    /***********************************************************************/

    /**
     * Arcade drive. Inputs are directly fed to motor controllers as-is.
     * 
     * @param throttle
     * @param turn
     */
    public void arcadeDrive(double throttle, double turn) {
        double left = throttle - turn;
        double right = throttle + turn;
        frontLeftController.set(left);
        backLeftController.set(left);
        frontRightController.set(right);
        backRightController.set(right);
    }

    /**
     * Tank drive. Inputs are directly fed to motor controllers as-is.
     * 
     * @param left
     * @param right
     */
    public void tankDrive(double left, double right) {
        frontLeftController.set(left);
        backLeftController.set(left);
        frontRightController.set(right);
        backRightController.set(right);
    }

    public void runWheel(int wheel, double level) {
        switch (wheel) {
            case Constants.FRONT_LEFT_CAN:
                frontLeftController.set(level);
                break;
            case Constants.FRONT_RIGHT_CAN:
                frontRightController.set(level);
                break;
            case Constants.BACK_LEFT_CAN:
                backLeftController.set(level);
                break;
            case Constants.BACK_RIGHT_CAN:
                backRightController.set(level);
                break;
        }

    }

    /**
     * Move relative to the robot's orientation
     * 
     * @param forwardX Forward speed demand, [-1.0 .. 1.0]
     * @param leftY    Left strafe speed demand, [-1.0 .. 1.0]
     * @param twist    CCW rotation rate demand, [-1.0 .. 1.0]
     */
    public void robotDrive(double forwardX, double leftY, double twist) {
        drive.driveCartesian(forwardX, leftY, twist);
    }

    /***********************************************************************/
    /* FIELD RELATIVE DRIVE */
    /***********************************************************************/

    /**
     * Move relative to the orientation on the field. This relies on the robot being
     * in initial, field oriented position
     * 
     * @param forwardX Forward speed demand, [-1.0 .. 1.0]
     * @param leftY    Left strafe speed demand, [-1.0 .. 1.0]
     * @param twist    CCW rotation rate demand, [-1.0 .. 1.0]
     */
    public void fieldDrive(double forwardX, double leftY, double twist) {
        drive.driveCartesian(forwardX, leftY, twist, imu.getRotation2d());
    }

    // TODO: Move these three constants to Constants.java

    // Feet per second 
    private static final double FPS_TO_MPS = 0.308;

    // Allow travel up  to 15 feet per second
    private static final double MAX_SPEED_MPS = 15 * FPS_TO_MPS;

    // Allow twist up to 90 degrees per second
    private static final double MAX_TWIST_RADS_PER_SECOND = Math.toRadians(90);

    /**
     * Drive at specifc forward (x), left (y), and angular speeds
     * 
     * @param forwardX Forward speed demand, [-1.0 .. 1.0]
     * @param leftY    Left strafe speed demand, [-1.0 .. 1.0]
     * @param twist    CCW rotation rate demand, [-1.0 .. 1.0]
     */
    public void fieldDriveVelocity(double forwardX, double leftY, double twist) {

        double vx = forwardX * MAX_SPEED_MPS;
        double vy = leftY * MAX_SPEED_MPS;
        double rot = twist * MAX_TWIST_RADS_PER_SECOND;

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(vx, vy, rot);

        // or using Field relative:
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds,
                imu.getRotation2d());

        MecanumDriveWheelSpeeds wheelSpeeds = driveKinematics.toWheelSpeeds(chassisSpeeds);

        
        // Use PID control to get to the desired speeds (set point) by measuring
        // the current wheel speed using encoders (process variable)

        frontLeftController.getPIDController().setReference(wheelSpeeds.frontLeftMetersPerSecond,
                ControlType.kVelocity);
        // TODO Update other motor controllers


        // TODO Find and call a function on the wheelSpeeds object that resets the speed
        // of all wheels based on a max allowable speed.
    }

    /***********************************************************************/
    /* AUTONOMOUS DRIVING */
    /***********************************************************************/

    /**
     * Automatically level the charge station by moving the robot forward and
     * backward. The robot must either be at least partially on one of the inclines.
     * If the robot is flat on the field floor, this routine will think it's on
     * top of the charge station and already leveled.
     * 
     * @return true if the robot is level on the charge station (energized) and
     *         false if not
     */
    public boolean chargeStationEnergize() {

        // TODO The basic algorithm is: 
        // 1. Get the roll
        // 2. Convert roll to a roll-slope
        // 3. Calculate a change in twist to counteract a non-zero roll
        // 4. Get the pitch 
        // 5. Convert the pitch to a pitch-slope
        // 6. Calculate the change in robot relative, forward velocity needed to achieve 0 pitch 
        // 7. drive the motor using robot relative drive passing in forward and twist
        // 8. Return true if the pitch is within 2 degrees of zero and the roll is within 2 degrees of 0.
        return false;

        // HINTS:
        // - Slope is rise over run, so it's the tan(angle). 
        // - Assume the pitch and roll can be -12 to +12 degrees
        // - To calculate motor input from slope, scale by a single value.
    }

    /***********************************************************************/
    /* CONTROL */
    /***********************************************************************/

    /*
     * Stop all motors
     */
    public void stop() {
        drive.stopMotor();
    }

    /**
     * Normally, this should not be called, except form the setPose() method.
     * Using setPose() instead also updates the drive train, odometry, and drive
     * sim.bb
     */
    public void resetSensors() {
        imu.zeroYaw();
        frontLeftEncoderRev.setPosition(0);
        frontRightEncoderRev.setPosition(0);
        backLeftEncoderRev.setPosition(0);
        backRightEncoderRev.setPosition(0);
    }

    /**
     * Set the initial condition of the robot (where it is on the field)
     */
    public void setPoseMeters(Pose2d pose) {
        resetSensors();

        odometry.resetPosition(imu.getRotation2d(), getWheelPositions(),
                getPoseMeters());

        field2d.setRobotPose(pose);
    }

    /**
     * Add a vision sample to adjust robot pose
     * 
     * @param visionRobotPoseMeters
     * @param timeStamp
     */
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timeStamp) {
        // TODO First switch from MecanumDriveOdometry to MecanumDrivePoseEstimator,
        // then call this
        // odometry.addVisionMeasurement(visionRobotPoseMeters, timeStamp);
    }

    public Pose2d getPoseMeters() {
        MecanumDrivePoseEstimator e;
        return odometry.getPoseMeters();
        // TODO switch to this if we switch odometry to use MecanumDrivePoseEstimator
        // return odometry.getEstimatedPosition();
    }

    public double getHeadingDegrees() {
        // TODO: If the rio mounts vertically, you have to call something on the AHRS class I believe.
        return imu.getAngle();
    }

    public double getPitch() {
        // TODO: Verify rio is mounted properly to return pitch
        return imu.getPitch();
    }
    
    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Pitch", this::getPitch, null);
        
        // This syntax might look a little weird. Here's what it means:
        //
        // This part:
        //      () -> { return frontLeftEncoderRev.getPosition(); }
        //
        // is called a lambda function. It's away of defining a function where it's being used.
        // In the Pitch line above, we use the this::getPitch argument instead because we have
        // an actual function already defined.

        builder.addDoubleProperty("FL Encoder", () -> { return frontLeftEncoderRev.getPosition(); }, null);
        builder.addDoubleProperty("FR Encoder", () -> { return frontRightEncoderRev.getPosition(); }, null);
        builder.addDoubleProperty("BL Encoder", () -> { return backLeftEncoderRev.getPosition(); }, null);
        builder.addDoubleProperty("BR Encoder", () -> { return backRightEncoderRev.getPosition(); }, null);
    }

}
