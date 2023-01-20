// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.math.util.Units;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.motorcontrol.IMotorController;

import frc.robot.Constants;

/**
 * A mecanum drive train.
 */
public class DriveSubsystem extends SubsystemBase {

    // We're using this encoder: https://www.revrobotics.com/rev-11-1271/
    private static final int ENCODER_RESOLUTION = 8192;
    private static final double WHEEL_DIAMETER_INCHES = 6.0; // inches
    private static final double WHEEL_CIRCUM_METERS = Units.inchesToMeters(WHEEL_DIAMETER_INCHES) * Math.PI;

    // TODO: Define Wheel positions. Repeat the following for each wheel x andd y
    // are in meters
    private static final Translation2d FRONT_LEFT_POS = new Translation2d(0.0, 0.0);

    // This object defines the properties of how the robot turns
    private static final MecanumDriveKinematics DRIVE_KINEMATICS = new MecanumDriveKinematics(null, null, null, null);

    // Encoders - front left, front right, rear left, rear right
    private Encoder frontLeftEncoder, frontRightEncoder, backLeftEncoder, backRightEncoder;

    // The Inertial Measurement Unit (IMU) connects to the RoboRio through the MXP
    // port, which is the wide female pin header in the middle of the Rio. Through
    // the MXP, the Serial Port Interface (SPI) is used.
    private final AHRS imu = new AHRS(SPI.Port.kMXP);

    // Converts tank or arcade drive speed requests to voltage requests to the motor
    // controllers
    private final MecanumDrive drive;

    // Keeps track of where we are on the field based on gyro and encoder inputs.
    private final MecanumDrivePoseEstimator odometry;

    // Sensor simulations
    private final Field2d field2d = new Field2d();

    /** Creates a new Subsystem. */
    public DriveSubsystem() {

        // Create a VictorSPX for each motor.
        WPI_VictorSPX frontLeft = new WPI_VictorSPX(Constants.RIGHT_DRIVE_1_CAN);
        WPI_VictorSPX backLeft = new WPI_VictorSPX(Constants.RIGHT_DRIVE_2_CAN);
        WPI_VictorSPX frontRight = new WPI_VictorSPX(Constants.RIGHT_DRIVE_3_CAN);
        WPI_VictorSPX backRight = new WPI_VictorSPX(Constants.LEFT_DRIVE_1_CAN);

        frontLeft.setNeutralMode(NeutralMode.Brake);
        backLeft.setNeutralMode(NeutralMode.Brake);
        frontRight.setNeutralMode(NeutralMode.Brake);
        backRight.setNeutralMode(NeutralMode.Brake);

        // TODO: Setup the left side or right side motors for inverted, depending on the
        // gearing.
        frontLeft.setInverted(false);

        drive = new MecanumDrive(backLeft, frontLeft, backRight, frontRight);

        // TODO: Setup encoders
        // Left: reverse direction (decreasing values go forward)
        frontLeftEncoder = new Encoder(Constants.FRONT_LEFT_ENCODER_A_DIO, Constants.FRONT_LEFT_ENCODER_A_DIO + 1,
                true);

        // leftEncoder.setDistancePerPulse(WHEEL_CIRCUM_METERS / ENCODER_RESOLUTION);
        // rightEncoder.setDistancePerPulse(WHEEL_CIRCUM_METERS / ENCODER_RESOLUTION);

        // TODO: Create the following object using the distances from each of the
        // encoders
        // MecanumDriveWheelPositions wheelPositions = new MecanumDriveWheelPositions(
        // frontLeftEncoder.getDistance(),
        // ...
        // )

        // Set the initial position (0,0) and heading (whatever it is) of the robot on
        // the field
        // TODO - initialize the odometry with initial states
        odometry = new MecanumDrivePoseEstimator(DRIVE_KINEMATICS, imu.getRotation2d(), getWheelPositions(), getPoseMeters(), null, null );

        addChild("Drive", drive);
        addChild("IMU", imu);
        addChild("Field", field2d);

    }

    private MecanumDriveWheelPositions getWheelPositions() {
        return new MecanumDriveWheelPositions(
                frontLeftEncoder.getDistance(),
                frontRightEncoder.getDistance(),
                backLeftEncoder.getDistance(),
                backRightEncoder.getDistance());
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

    /**
     * Move relative to the robot's orientation
     */
    public void robotDrive(double strafe, double forward, double twist) {
        drive.driveCartesian(strafe, forward, twist);
    }

    /**
     * Move relative to the orientation on the field
     */
    public void fieldDrive(double strafe, double forward, double twist) {

        drive.driveCartesian(strafe, forward, twist, imu.getRotation2d());
    }

    /**
     * Normally, this should not be called, except form the setPose() method.
     * Using setPose() instead also updates the drive train, odometry, and drive
     * sim.bb
     */
    public void resetSensors() {
        imu.reset();
        frontLeftEncoder.reset();
        frontRightEncoder.reset();
        backLeftEncoder.reset();
        backRightEncoder.reset();
    }

    /**
     * Set the initial condition of the robot (where it is on the field)
     */
    public void setPoseMeters(Pose2d pose) {
        field2d.setRobotPose(pose);
        imu.reset();
        odometry.resetPosition(imu.getRotation2d(), getWheelPositions(), getPoseMeters());
        resetSensors();
    }

    public Pose2d getPoseMeters() {
        return odometry.getEstimatedPosition();
    }

    public double getHeadingDegrees() {
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
    }

}
