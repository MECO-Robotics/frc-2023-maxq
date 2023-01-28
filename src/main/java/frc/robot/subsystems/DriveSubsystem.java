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
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;

import frc.robot.Constants;
import frc.robot.commands.lights.TurnBlueBothOn;

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
    private MecanumDrivePoseEstimator odometry;

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

        frontLeftEncoderRev.setPositionConversionFactor(WHEEL_CIRCUM_METERS / ENCODER_RESOLUTION);
        frontRightEncoderRev.setPositionConversionFactor(WHEEL_CIRCUM_METERS / ENCODER_RESOLUTION);
        backLeftEncoderRev.setPositionConversionFactor(WHEEL_CIRCUM_METERS / ENCODER_RESOLUTION);
        backRightEncoderRev.setPositionConversionFactor(WHEEL_CIRCUM_METERS / ENCODER_RESOLUTION);

        // Set the initial position (0,0) and heading (whatever it is) of the robot on
        // the field
        // odometry = new MecanumDrivePoseEstimator(driveKinematics,
        //         imu.getRotation2d(), getWheelPositions(),
        //         getPoseMeters(), null, null);

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

        frontLeftEncoderRev = frontLeftController.getAlternateEncoder(Type.kQuadrature, ENCODER_RESOLUTION);

        backRightEncoderRev = backRightController.getAlternateEncoder(Type.kQuadrature, ENCODER_RESOLUTION);

        frontRightEncoderRev = frontRightController.getAlternateEncoder(Type.kQuadrature, ENCODER_RESOLUTION);

        backLeftEncoderRev = backLeftController.getAlternateEncoder(Type.kQuadrature, ENCODER_RESOLUTION);

        drive = new MecanumDrive(frontLeftController, backLeftController, frontRightController, backRightController);
    }

    /**
     * Get the wheel positions
     * 
     * @return
     */
    private MecanumDriveWheelPositions getWheelPositions() {
        /*
         * return new MecanumDriveWheelPositions(
         * frontLeftEncoder.getDistance(),
         * frontRightEncoder.getDistance(),
         * backLeftEncoder.getDistance(),
         * backRightEncoder.getDistance());
         */

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

        // odometry.update(imu.getRotation2d(), getWheelPositions());

        // This allows us to see the robot's position and orientation on the
        // Shuffleboard
        // field2d.setRobotPose(getPoseMeters());

        // Update the motor safety. This makes sure the system knows we're in control of
        // the motors.
        // If the code crashes, or somehow gets stuck in a loop, and this method isn't
        // called then the motors will automaitcally stop.
        drive.feed();
    }

    /**
     * Arcade drive. Inputs are directly fed to motor controllers as-is.
     * 
     * @param throttle
     * @param turn
     */
    public void arcadeDrive(double throttle, double turn) {
        System.out.println(String.format("Arcade: %f, %f", throttle, turn));
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

        /**
         * Test an individual wheel
         */
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

        // System.out.println(String.format("X,Y,Z: %f, %f, %f", forwardX, leftY,
        // twist));
        drive.driveCartesian(forwardX, leftY, twist);
    }

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

    /**
     * Drive at specifc forward (x), left (y), and angular speeds
     * 
     * @param vx  Forward speed away from alliance wall, in m/s
     * @param vy  Left strafe speed, in m/s
     * @param rot Angular speed CCW, in rad/s
     */
    public void fieldDriveVelocity(double vx, double vy, double rot) {
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(vx, vy, rot);

        // or using Field relative:
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds,
                imu.getRotation2d());

        MecanumDriveWheelSpeeds wheelSpeeds = driveKinematics.toWheelSpeeds(chassisSpeeds);

        // TODO Use PID control to get to the desired speeds (set point) by measuring
        // the current wheel speed using encoders (process variable)
        // For example:

        frontLeftController.getPIDController().setReference(wheelSpeeds.frontLeftMetersPerSecond,
                ControlType.kVelocity);
        // TODO Update other motor controllers
    }

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
        imu.reset();
        frontLeftEncoderRev.setPosition(0);
        frontRightEncoderRev.setPosition(0);
        backLeftEncoderRev.setPosition(0);
        backRightEncoderRev.setPosition(0);
    }

    /**
     * Set the initial condition of the robot (where it is on the field)
     */
    public void setPoseMeters(Pose2d pose) {
        field2d.setRobotPose(pose);
        imu.reset();
        odometry.resetPosition(imu.getRotation2d(), getWheelPositions(),
                getPoseMeters());
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
