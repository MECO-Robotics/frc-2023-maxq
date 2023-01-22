// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.math.util.Units;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

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
    private static final Translation2d FRONT_RIGHT_POS = new Translation2d(0.0, 0.0);
    private static final Translation2d BACK_LEFT_POS = new Translation2d(0.0, 0.0);
    private static final Translation2d BACK_RIGHT_POS = new Translation2d(0.0, 0.0);

    // This object defines the properties of how the robot turns
    private final MecanumDriveKinematics driveKinematics = new MecanumDriveKinematics(null, null, null, null);

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
        frontLeftEncoder = new Encoder(
                Constants.FRONT_LEFT_ENCODER_A_DIO,
                Constants.FRONT_LEFT_ENCODER_A_DIO + 1, // Assuming B channel is the next DIO port
                true);

        frontLeftEncoder.setDistancePerPulse(WHEEL_CIRCUM_METERS / ENCODER_RESOLUTION);
        frontRightEncoder.setDistancePerPulse(WHEEL_CIRCUM_METERS / ENCODER_RESOLUTION);
        backLeftEncoder.setDistancePerPulse(WHEEL_CIRCUM_METERS / ENCODER_RESOLUTION);
        backRightEncoder.setDistancePerPulse(WHEEL_CIRCUM_METERS / ENCODER_RESOLUTION);
        // TODO: Repeat for other encoders

        // Set the initial position (0,0) and heading (whatever it is) of the robot on
        // the field
        // TODO - initialize the odometry with initial states
        odometry = new MecanumDrivePoseEstimator(driveKinematics, imu.getRotation2d(), getWheelPositions(),
                getPoseMeters(), null, null);

        addChild("Drive", drive);
        addChild("IMU", imu);
        addChild("Field", field2d);

    }

    /**
     * Get the wheel positions
     * 
     * @return
     */
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
     * @param forwardX Forward speed demand, [-1.0 .. 1.0]
     * @param leftY Left strafe speed demand, [-1.0 .. 1.0]
     * @param twist CCW rotation rate demand, [-1.0 .. 1.0]
     */
    public void robotDrive(double forwardX, double leftY, double twist) {

        drive.driveCartesian(forwardX, leftY, twist);
    }


    /**
     * Move relative to the orientation on the field. This relies on the robot being
     * in initial, field oriented position
     * @param forwardX Forward speed demand, [-1.0 .. 1.0]
     * @param leftY Left strafe speed demand, [-1.0 .. 1.0]
     * @param twist CCW rotation rate demand, [-1.0 .. 1.0]
     */
    public void fieldDrive(double forwardX, double leftY, double twist) {

        drive.driveCartesian(forwardX, leftY, twist, imu.getRotation2d());
    }

    /**
     * Drive at specifc forward (x), left (y), and angular speeds
     * @param vx Forward speed, in m/s
     * @param vy Left strafe speed, in m/s
     * @param rot Angular speed CCW, in rad/s
     */
    public void fieldDriveVelocity(double vx, double vy, double rot) {
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(vx, vy, rot);

        // or using Field relative:
        // chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, null)

        MecanumDriveWheelSpeeds wheelSpeeds = driveKinematics.toWheelSpeeds(chassisSpeeds);

        // TODO Use PID control to get to the desired speeds (set point) by measuring
        // the current wheel speed using encoders (process variable)
        // For example:

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

    /**
     * Add a vision sample to adjust robot pose
     * 
     * @param visionRobotPoseMeters
     * @param timeStamp
     */
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timeStamp) {
        odometry.addVisionMeasurement(visionRobotPoseMeters, timeStamp);
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
