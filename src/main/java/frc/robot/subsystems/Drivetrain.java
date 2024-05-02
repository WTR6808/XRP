// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.xrp.XRPGyro;
import edu.wpi.first.wpilibj.xrp.XRPMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Seconds;


public class Drivetrain extends SubsystemBase {
//  private static final double kGearRatio =
//      (30.0 / 14.0) * (28.0 / 16.0) * (36.0 / 9.0) * (26.0 / 8.0); // 48.75:1
//  private static final double kCountsPerMotorShaftRev = 12.0;
//  private static final double kCountsPerRevolution = kCountsPerMotorShaftRev * kGearRatio; // 585.0
//  private static final double kWheelDiameterInch = 2.3622; // 60 mm

  // The XRP has the left and right motors set to
  // channels 0 and 1 respectively
  private final XRPMotor m_leftMotor = new XRPMotor(Constants.DriveTrainConstants.kLeftMotorPort);
  private final XRPMotor m_rightMotor = new XRPMotor(Constants.DriveTrainConstants.kRightMotorPort);

  // The XRP has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(Constants.DriveTrainConstants.kLeftEncoderA,
                                                    Constants.DriveTrainConstants.kLeftEncoderB);
  private final Encoder m_rightEncoder = new Encoder(Constants.DriveTrainConstants.kRightEncoderA,
                                                     Constants.DriveTrainConstants.kRightEncoderB);

  // Set up the differential drive controller
  private final DifferentialDrive m_diffDrive =
      new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);

  // Set up the XRPGyro
  private final XRPGyro m_gyro = new XRPGyro();

  // Set up the BuiltInAccelerometer
  private final BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer();

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

;  // Create a new SysId routine for characterizing the drive.
  private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(null,Volts.of(6),Seconds.of(20)),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motors.
              (Measure<Voltage> volts) -> {
                m_leftMotor.setVoltage(volts.in(Volts));
                m_rightMotor.setVoltage(volts.in(Volts));
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the left motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive-left")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            m_leftMotor.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(getLeftDistanceMeter(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(getLeftVelocityMeterSec(), MetersPerSecond));
                // Record a frame for the right motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive-right")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            m_rightMotor.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(getRightDistanceMeter(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(getRightVelocityMeterSec(), MetersPerSecond));
                log.motor("drive-average")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            ((-m_rightMotor.get()+m_leftMotor.get())/2.0) * (RobotController.getBatteryVoltage()/2.0), Volts))
                    .linearPosition(m_distance.mut_replace(getAvgDistanceMeter(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(getAvgVelocityMeterSec(), MetersPerSecond));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("drive")
              this));

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    SendableRegistry.addChild(m_diffDrive, m_leftMotor);
    SendableRegistry.addChild(m_diffDrive, m_rightMotor);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.setInverted(true);

    // Use meters as unit for encoder distances for Odometry class
    m_leftEncoder.setDistancePerPulse((Math.PI * Constants.DriveTrainConstants.kWheelDiameterMeter) / Constants.DriveTrainConstants.kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * Constants.DriveTrainConstants.kWheelDiameterMeter) / Constants.DriveTrainConstants.kCountsPerRevolution);
    resetEncoders();

    //Initialize the Gyro
    m_gyro.reset();
  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public int getLeftEncoderCount() {
    return m_leftEncoder.get();
  }

  public int getRightEncoderCount() {
    return m_rightEncoder.get();
  }

  public double getLeftDistanceInch() {
    return Units.metersToInches(m_leftEncoder.getDistance());
  }

  public double getRightDistanceInch() {
    return Units.metersToInches(m_rightEncoder.getDistance());
  }

  public double getAverageDistanceInch() {
    return (getLeftDistanceInch() + getRightDistanceInch()) / 2.0;
  }

  public double getLeftVelocityFeetSec(){
    return Units.metersToFeet(getLeftVelocityMeterSec());
  }

  public double getRightVelocityFeetSec(){
    return Units.metersToFeet(getRightVelocityMeterSec());
  }

  //Methods to get Distances & Velocities in Metric Units
  public double getLeftDistanceMeter() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistanceMeter() {
    return m_rightEncoder.getDistance();
  }

  public double getAvgDistanceMeter() {
    return (getLeftDistanceMeter()+getRightDistanceMeter())/2.0;
  }

  public double getLeftVelocityMeterSec(){
  //  if (getLeftDistanceMeter() < 0.0064){
  //    return 0.0;
  //  } else {
    return m_leftEncoder.getRate();
  //  }
  }

  public double getRightVelocityMeterSec(){
  //  if (getRightDistanceMeter() < 0.0064){
  //    return 0.0;
  //  } else {
    return m_rightEncoder.getRate();
   // }
  }

  public double getAvgVelocityMeterSec(){
    return ((getLeftVelocityMeterSec() + getRightVelocityMeterSec())/2.0);
  }
  /**
   * The acceleration in the X-axis.
   *
   * @return The acceleration of the XRP along the X-axis in Gs
   */
  public double getAccelX() {
    return m_accelerometer.getX();
  }

  /**
   * The acceleration in the Y-axis.
   *
   * @return The acceleration of the XRP along the Y-axis in Gs
   */
  public double getAccelY() {
    return m_accelerometer.getY();
  }

  /**
   * The acceleration in the Z-axis.
   *
   * @return The acceleration of the XRP along the Z-axis in Gs
   */
  public double getAccelZ() {
    return m_accelerometer.getZ();
  }

  /**
   * Current angle of the XRP around the X-axis.
   *
   * @return The current angle of the XRP in degrees
   */
  public double getGyroAngleX() {
    return m_gyro.getAngleX();
  }

  /**
   * Current angle of the XRP around the Y-axis.
   *
   * @return The current angle of the XRP in degrees
   */
  public double getGyroAngleY() {
    return m_gyro.getAngleY();
  }

  /**
   * Current angle of the XRP around the Z-axis.
   *
   * @return The current angle of the XRP in degrees
   */
  public double getGyroAngleZ() {
    return m_gyro.getAngleZ();
  }

  /** Reset the gyro. */
  public void resetGyro() {
    m_gyro.reset();
  }

  //Routines for Characterization
    /**
   * Returns a command that will execute a quasistatic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  /**
   * Returns a command that will execute a dynamic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //System.out.println(m_leftEncoder.getDistance());

  }
}
