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

    // Allow travel up to 15 feet per second
    private static final double MAX_SPEED_MPS = 15 * Constants.FPS_TO_MPS;

    // Allow twist up to 90 degrees per second
    private static final double MAX_TWIST_RADS_PER_SECOND = Math.toRadians(90);

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

        // Set the initial position (0,0) and heading (whatever it is) of the robot on
        // the field
        odometry = new MecanumDriveOdometry(driveKinematics,
                imu.getRotation2d(), getWheelPositions());

        resetSensors();

        addChild("Drive", drive);
        addChild("IMU", imu);
        addChild("Field", field2d);
    }

    private CANSparkMax setupMotorController(int canID, boolean inverted) {
        CANSparkMax controller = new CANSparkMax(canID, MotorType.kBrushed);
        controller.setIdleMode(IdleMode.kBrake);
        controller.setInverted(inverted);
        controller.getPIDController().setFF(0.1);
        controller.getPIDController().setP(0.1);
        controller.getPIDController().setI(0);
        controller.getPIDController().setD(0);
        controller.getPIDController().setIZone(0);
        return controller;
    }

    private RelativeEncoder setupRelativeEncoder(CANSparkMax controller) {
        RelativeEncoder encoder = controller.getEncoder(Type.kQuadrature, ENCODER_RESOLUTION);
        encoder.setPositionConversionFactor(WHEEL_CIRCUM_METERS);
        encoder.setInverted(controller.getInverted());
        return encoder;
    }

    public void configureUsingSparkMaxMotorControllers() {

        frontLeftController = setupMotorController(Constants.FRONT_LEFT_CAN, true);
        backLeftController = setupMotorController(Constants.BACK_LEFT_CAN, true);
        frontRightController = setupMotorController(Constants.FRONT_RIGHT_CAN, false);
        backRightController = setupMotorController(Constants.BACK_RIGHT_CAN, false);

        frontLeftEncoderRev = setupRelativeEncoder(frontLeftController);
        backLeftEncoderRev = setupRelativeEncoder(backLeftController);
        frontRightEncoderRev = setupRelativeEncoder(frontRightController);
        backRightEncoderRev = setupRelativeEncoder(backRightController);

        drive = new MecanumDrive(frontLeftController, backLeftController, frontRightController, backRightController);
    }

    /**
     * Get the wheel positions in terms of distance traveled by each wheel.
     * 
     * @return
     */
    private MecanumDriveWheelPositions getWheelPositions() {
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
        drive.driveCartesian(forwardX / 2.0, leftY / 2.0, twist / 2.0);
    }

    /**
     * Move relative to the robot's orientation using closed loop control
     * 
     * @param forwardX Forward speed demand, [-1.0 .. 1.0]
     * @param leftY    Left strafe speed demand, [-1.0 .. 1.0]
     * @param twist    CCW rotation rate demand, [-1.0 .. 1.0]
     */
    public void robotDriveClosed(double forwardX, double leftY, double twist) {
        double vx = forwardX * MAX_SPEED_MPS;
        double vy = leftY * MAX_SPEED_MPS;
        double rot = twist * MAX_TWIST_RADS_PER_SECOND;

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(vx, vy, rot);
        MecanumDriveWheelSpeeds wheelSpeeds = driveKinematics.toWheelSpeeds(chassisSpeeds);

        setWheelSpeeds(wheelSpeeds);
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

        setWheelSpeeds(wheelSpeeds);
    }

    /**
     * Given field relative chassis speeds, drive the wheels
     * 
     * @param chassisSpeeds
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        MecanumDriveWheelSpeeds wheelSpeeds = driveKinematics.toWheelSpeeds(chassisSpeeds);
        setWheelSpeeds(wheelSpeeds);
    }

    /**
     * Run the motors using PID control to achieve specific wheel speeds
     * 
     * @param wheelSpeeds The desired speeds of each of the wheels
     */
    public void setWheelSpeeds(MecanumDriveWheelSpeeds wheelSpeeds) {

        // Make sure none of the wheels tries to go faster than our max allowed.
        wheelSpeeds.desaturate(MAX_SPEED_MPS);

        // Use PID control to get to the desired speeds (set point) by measuring
        // the current wheel speed using encoders (process variable)

        frontLeftController.getPIDController().setReference(wheelSpeeds.frontLeftMetersPerSecond,
                CANSparkMax.ControlType.kVelocity);
        frontRightController.getPIDController().setReference(wheelSpeeds.frontRightMetersPerSecond,
                CANSparkMax.ControlType.kVelocity);
        backLeftController.getPIDController().setReference(wheelSpeeds.rearLeftMetersPerSecond,
                CANSparkMax.ControlType.kVelocity);
        backRightController.getPIDController().setReference(wheelSpeeds.rearRightMetersPerSecond,
                CANSparkMax.ControlType.kVelocity);
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
    public boolean chargeStationEnergize(boolean strafeClimb) {

        // NOTES
        /*
         * The pitch is currently negated, but that looks backwards now, relative to the
         * code here. Need to switch back
         * 
         * Algorithm approach for strafe climb:
         * 1. Orient field left or right
         * 2. Pitch -> Twist right, or use heading to maintain field left orientation
         * 3. Roll -> Left
         * 
         * Algorithm approah for straight climb:
         * 1. Orient field forward or backward
         * 2. Pitch -> Forward
         * 3. Roll -> Twist left
         * 
         */

        double MAX_ANGLE = 12.0;
        double MAX_MOTOR = 0.7;
        if (strafeClimb) {
            double twist = getPitch() / MAX_ANGLE * MAX_MOTOR;
            double left = getRoll() / MAX_ANGLE * MAX_MOTOR;
            if (Math.abs(twist) > 0.1 || Math.abs(left) > 0.1) {
                robotDrive(0, left, twist);
            }
        } else {
            double twist = getRoll() / MAX_ANGLE * MAX_MOTOR;
            double forward = getPitch() / MAX_ANGLE * MAX_MOTOR;
            if (Math.abs(twist) > 0.1 || Math.abs(forward) > 0.1) {
                robotDrive(forward, 0, twist);
            }
        }

        return false;
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

        // TODO For now, leave out until we can research if this should be done and if
        // it's necessary
        // try {
        // imu.calibrate();

        // while (imu.isCalibrating())
        // Thread.sleep(100);
        // } catch (InterruptedException e) {
        // }

        imu.zeroYaw();
        frontLeftEncoderRev.setPosition(0);
        frontRightEncoderRev.setPosition(0);
        backLeftEncoderRev.setPosition(0);
        backRightEncoderRev.setPosition(0);
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

    /**
     * Get the field position and orientation of the robot.
     * 
     * @return
     */
    public Pose2d getPoseMeters() {
        MecanumDrivePoseEstimator e;
        return odometry.getPoseMeters();
        // TODO switch to this if we switch odometry to use MecanumDrivePoseEstimator
        // return odometry.getEstimatedPosition();
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
     * Get the drive kinematics.
     * 
     * @return
     */
    public MecanumDriveKinematics getKinematics() {
        return this.driveKinematics;
    }

    public double getHeadingDegrees() {
        // TODO: If the rio mounts vertically, you have to call something on the AHRS
        // class I believe.
        // TODO AHRS also has getFusedHeading(). This method is probably more accurate,
        // but is designed to just correct the compass, so the value is always 0 to 360
        // unlike getAngle, which is continuous.
        return imu.getAngle();
    }

    public double getPitch() {
        // TODO: Verify rio is mounted properly to return pitch
        return -imu.getPitch();
    }

    public double getRoll() {
        // TODO: Verify rio is mounted properly to return pitch
        return imu.getRoll();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Pitch", this::getPitch, null);
        builder.addDoubleProperty("Heading", this::getHeadingDegrees, null);
        builder.addDoubleProperty("Roll", this::getRoll, null);

        // This syntax might look a little weird. Here's what it means:
        //
        // This part:
        // () -> { return frontLeftEncoderRev.getPosition(); }
        //
        // is called a lambda function. It's away of defining a function where it's
        // being used.
        // In the Pitch line above, we use the this::getPitch argument instead because
        // we have
        // an actual function already defined.

        builder.addDoubleProperty("FL Encoder", () -> {
            return frontLeftEncoderRev.getPosition();
        }, null);
        builder.addDoubleProperty("FR Encoder", () -> {
            return frontRightEncoderRev.getPosition();
        }, null);
        builder.addDoubleProperty("BL Encoder", () -> {
            return backLeftEncoderRev.getPosition();
        }, null);
        builder.addDoubleProperty("BR Encoder", () -> {
            return backRightEncoderRev.getPosition();
        }, null);
    }

}
