// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

import frc.robot.Constants;

/**
 * A tank drive train driven by dual motors for each side.
 */
public class DriveSubsystem extends SubsystemBase {

    // The type of encoder indicates the resolution - such as the Grayhill 63R128
    private static final int ENCODER_RESOLUTION = 128;
    private static final double WHEEL_DIAMETER_INCHES = 6.0; // inches
    private static final double WHEEL_CIRCUM_METERS = Units.inchesToMeters(WHEEL_DIAMETER_INCHES) * Math.PI;

    //TODO: Define Wheel positions. Repeat the following for each wheel x andd y are in meters
    private static final Translation2d FRONT_LEFT_POS = new Translation2d(0.0,0.0);

    // This object defines the properties of how the robot turns
    private static final MecanumDriveKinematics DRIVE_KINEMATICS = new MecanumDriveKinematics(null, null, null, null);

    // Encoders - front left, front right, rear left, rear right
    private Encoder frontLeftEncoder, frontRightEncoder, rearLeftEncoder, rearRightEncoder;

    // The Inertial Measurement Unit (IMU) connects to the RoboRio through the MXP
    // port, which is the wide female pin header in the middle of the Rio. Through
    // the MXP, the Serial Port Interface (SPI) is used.
    private final AHRS imu = new AHRS(SPI.Port.kMXP);

    // Converts tank or arcade drive speed requests to voltage requests to the motor
    // controllers
    private final MecanumDrive drive;

    // Keeps track of where we are on the field based on gyro and encoder inputs.
    private final MecanumDriveOdometry odometry;

    // Sensor simulations
    private final Field2d field2d = new Field2d();
    private final SimDeviceSim imuSim;

    /** Creates a new Subsystem. */
    public DriveSubsystem() {

        // Create the WPI_VictorSPX individually and call configOpenLoopRamp on it.
        WPI_VictorSPX frontLeft = new WPI_VictorSPX(Constants.RIGHT_DRIVE_1_CAN);
        WPI_VictorSPX backLeft = new WPI_VictorSPX(Constants.RIGHT_DRIVE_2_CAN);
        WPI_VictorSPX frontRight = new WPI_VictorSPX(Constants.RIGHT_DRIVE_3_CAN);
        WPI_VictorSPX backRight = new WPI_VictorSPX(Constants.LEFT_DRIVE_1_CAN);

        frontLeft.setNeutralMode(NeutralMode.Brake);
        backLeft.setNeutralMode(NeutralMode.Brake);
        frontRight.setNeutralMode(NeutralMode.Brake);
        backRight.setNeutralMode(NeutralMode.Brake);

        // TODO: Setup the left side or right side motors for inverted, depending on the gearing.
        frontLeft.setInverted(false);

        drive = new MecanumDrive(backLeft, frontLeft, backRight, frontRight);

        // TODO: Setup encoders
        // Left: reverse direction (decreasing values go forward)
        // leftEncoder = new Encoder(Constants.LEFT_DRIVE_ENCODER_1_DIO, Constants.LEFT_DRIVE_ENCODER_2_DIO, true);
        // // Right
        // rightEncoder = new Encoder(Constants.RIGHT_DRIVE_ENCODER_1_DIO, Constants.RIGHT_DRIVE_ENCODER_2_DIO, false);

        // leftEncoder.setDistancePerPulse(WHEEL_CIRCUM_METERS / ENCODER_RESOLUTION);
        // rightEncoder.setDistancePerPulse(WHEEL_CIRCUM_METERS / ENCODER_RESOLUTION);

        if (RobotBase.isSimulation()) {
            // At some point, we'll need to reconfigure the drive sim from the defaults
            // since we're using a custom drive train
            // driveSim = new DifferentialDrivetrainSim(DCMotor.getCIM(3), 10f,
            // jKgMetersSquared, massKg, wheelRadiusMeters, trackWidthMeters,
            // measurementStdDevs)

            imuSim = new SimDeviceSim("navX-Sensor[0]");

        } else {
            imuSim = null;
        }

        // TODO: Create the following object using the distances from each of the encoders
        // MecanumDriveWheelPositions wheelPositions = new MecanumDriveWheelPositions(
        //     frontLeftEncoder.getDistance(),
        //     ...
        // )

        // Set the initial position (0,0) and heading (whatever it is) of the robot on
        // the field
        // TODO - initialize the odometry with initial states
        odometry = new MecanumDriveOdometry(null, null, null);

        addChild("Drive", drive);
        addChild("IMU", imu);
        addChild("Field", field2d);

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
//        odometry.update(imu.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
        field2d.setRobotPose(odometry.getPoseMeters());

        // Update the motor safety. This makes sure the system knows we're in control of
        // the motors.
        // If the code crashes, or somehow gets stuck in a loop, and this method isn't
        // called then
        // the motors will automaitcally stop.
        drive.feed();
    }

    /**
     * Normally, this should not be called, except form the setPose() method.
     * Using setPose() instead also updates the drive train, odometry, and drive
     * sim.bb
     */
    public void resetSensors() {
        // TODO: Call reset on all encoders
        imu.reset();
    }

    /**
     * Drive using arcade controls with speed ramping. Do not use for autonomous
     * routines
     * unless speed ramping is desired.
     */

    public Field2d getField2d() {
        return field2d;
    }

    public void setPoseMeters(Pose2d pose) {
        field2d.setRobotPose(pose);
        imu.reset();
        // odometry.resetPosition(pose, imu.getRotation2d());
        resetSensors();
    }

    public Pose2d getPoseMeters() {
        return odometry.getPoseMeters();
    }

    public double getHeadingDegrees() {
        return imu.getAngle();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        // TODO: Define getPitch()
        //builder.addDoubleProperty("Pitch", this::getPitch, null);
    }

}
