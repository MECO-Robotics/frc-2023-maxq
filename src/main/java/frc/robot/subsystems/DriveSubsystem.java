// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.math.util.Units;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import frc.robot.Constants;

/**
 * A tank drive train driven by dual motors for each side.
 */
public class DriveSubsystem extends SubsystemBase {

  private static final int TICKS_PER_REV = 360;
  private static final double WHEEL_DIAMETER_INCHES = 6.0; // inches
  private static final double WHEEL_CIRCUM_METERS = Units.inchesToMeters(WHEEL_DIAMETER_INCHES) * Math.PI;
  public static final double TRACK_WIDTH_METERS = 0.71;       // 28" Wheelbase in meters. Distance from center of left wheels to center of right wheels

  // This object defines the properties of how the robot turns
  public static final DifferentialDriveKinematics DRIVE_KINEMATICS =
      new DifferentialDriveKinematics(TRACK_WIDTH_METERS);

  // Real sensors & actuators
  private MotorControllerGroup leftMotors, rightMotors;
  private Encoder leftEncoder, rightEncoder;
  
  // The Inertial Measurement Unit (IMU) connects to the RoboRio through the MXP port,
  // which is the wide female pin header in the middle of the Rio. Through the MXP, 
  // the Serial Port Interface (SPI) is used.
  private final AHRS imu = new AHRS(SPI.Port.kMXP);

  // Converts tank or arcade drive speed requests to voltage requests to the motor controllers
  private final DifferentialDrive drive;

  // Keeps track of where we are on the field based on gyro and encoder inputs.
  private final DifferentialDriveOdometry odometry;

  // This member will limit acceleration to reduce skid
  SlewRateLimiter speedFilter = new SlewRateLimiter(0.5);

  // Sensor simulations
  private final Field2d field2d = new Field2d();
  private final EncoderSim leftEncoderSim;
  private final EncoderSim rightEncoderSim;
  private final SimDeviceSim imuSim;
  
  // Drivetrain sim using standard kit of parts
  private final DifferentialDrivetrainSim driveSim = DifferentialDrivetrainSim.createKitbotSim(
    KitbotMotor.kDualCIMPerSide, // 2 CIMs per side.
    KitbotGearing.k10p71,        // 10.71:1
    KitbotWheelSize.kSixInch,     // 6" diameter wheels.
    null);                       // No measurement noise.

  // Simple "fake" 3 gear system. For this drive train, it just changes the speed range.
  private int gear = 3;

  /** Creates a new Subsystem. */
  public DriveSubsystem() {

    // Create the WPI_VictorSPX individually and call configOpenLoopRamp on it.
    WPI_VictorSPX left1 = new WPI_VictorSPX(Constants.LEFT_DRIVE_1);
    WPI_VictorSPX left2 = new WPI_VictorSPX(Constants.LEFT_DRIVE_2);
    WPI_VictorSPX right1 = new WPI_VictorSPX(Constants.RIGHT_DRIVE_1);
    WPI_VictorSPX right2 = new WPI_VictorSPX(Constants.RIGHT_DRIVE_2);

    leftMotors = new MotorControllerGroup(left1, left2);
    rightMotors = new MotorControllerGroup(right1, right2);

    // So positive values cause forward movement
    rightMotors.setInverted(true);

    drive = new DifferentialDrive(leftMotors, rightMotors);

    // Left: reverse direction (decreasing values go forward)
    leftEncoder = new Encoder(Constants.LEFT_ENCODER_1, Constants.LEFT_ENCODER_2, true);
    // Right
    rightEncoder = new Encoder(Constants.RIGHT_ENCODER_1, Constants.RIGHT_ENCODER_2, false);
    
    leftEncoder.setDistancePerPulse(WHEEL_CIRCUM_METERS / TICKS_PER_REV);
    rightEncoder.setDistancePerPulse(WHEEL_CIRCUM_METERS / TICKS_PER_REV);

    if(RobotBase.isSimulation()) {
      leftEncoderSim = new EncoderSim(leftEncoder);
      rightEncoderSim = new EncoderSim(rightEncoder);
      imuSim = new SimDeviceSim("navX-Sensor[0]");

    } else {
      leftEncoderSim = null;
      rightEncoderSim = null;
      imuSim = null;
    }

    // Set the initial position (0,0) and heading (whatever it is) of the robot on the field
    odometry = new DifferentialDriveOdometry(imu.getRotation2d());

    addChild("Left Motors", leftMotors);
    addChild("Left Encoder", leftEncoder);
    addChild("Right Encoder", rightEncoder);
    addChild("Right Motors", rightMotors);
    addChild("Diff Drive", drive);
    addChild("IMU", imu);
    addChild("Field", field2d);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(imu.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
    field2d.setRobotPose(odometry.getPoseMeters());
  }

  /**
   * Called with every loop to update the simulation. periodic() is also called
   * as normal.
   */
  @Override
  public void simulationPeriodic() {

    // This method will be called once per scheduler run during simulation
    // Set the inputs to the system. Note that we need to convert
    // the [-1, 1] PWM signal to voltage by multiplying it by the
    // robot controller voltage.
    driveSim.setInputs(
      leftMotors.get() * RobotController.getInputVoltage(),
      rightMotors.get() * RobotController.getInputVoltage());

    // Advance the model by 20 ms. Note that if you are running this
    // subsystem in a separate thread or have changed the nominal timestep
    // of TimedRobot, this value needs to match it.
    driveSim.update(0.02);

    // Update all of our sensors.
    leftEncoderSim.setDistance(driveSim.getLeftPositionMeters());
    leftEncoderSim.setRate(driveSim.getLeftVelocityMetersPerSecond());
    rightEncoderSim.setDistance(driveSim.getRightPositionMeters());
    rightEncoderSim.setRate(driveSim.getRightVelocityMetersPerSecond());
    imuSim.getDouble("Yaw").set(-driveSim.getHeading().getDegrees());
  }

  /**
   * Normally, this should not be called, except form the setPose() method.
   * Using setPose() instead also updates the drive train, odometry, and drive sim.bb
   */
  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  public void tankDrive(double left, double right) {
    double factor = speedFilter.calculate((double)gear / 3.0);
    drive.tankDrive(left * factor, right * factor);
  }

  public void arcadeDrive(double throttle, double turn) {
    // Currently, applying factor to both throttle and turn, but we may want to consider
    // only applying to throttle.
    double factor =  speedFilter.calculate((double)gear / 3.0);
    drive.arcadeDrive(throttle * factor, turn * factor);
  }

  /**
   * Get the distance, in meters the left side has traveled since the last reset.
   */
  public double getLeftDistance() {
    return leftEncoder.getDistance();
  }

  /**
   * Get the distance, in meters the right side has traveled since the last reset.
   * @return
   */
  public double getRightDistance() {
    return rightEncoder.getDistance();
  }

  public Field2d getField2d(){
    return field2d;
  }

  public void setPoseMeters(Pose2d pose) {
    field2d.setRobotPose(pose);
    imu.reset();
    odometry.resetPosition(pose, imu.getRotation2d());
    resetEncoders();
    if(RobotBase.isSimulation()) {
      driveSim.setPose(pose);
    }
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
    builder.addDoubleProperty("Gear", this::getGear, this::setGear);
  }

  public double getGear() {
    return gear;
  }

  public void setGear(double g) {
    gear = (int)g;
  }

  public void shift(boolean up) {
    if(up) {
      gear++;
    } else {
      gear--;
    }
  }
}

