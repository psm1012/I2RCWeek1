// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */

  private final WPI_TalonSRX leftDriveTalon;
  private final WPI_TalonSRX rightDriveTalon;

  private final TalonSRXSimCollection leftDriveSim;
  private final TalonSRXSimCollection rightDriveSim;

  private final DifferentialDrive drive;
  private final DifferentialDrivetrainSim driveSim;

  private static AHRS navx = new AHRS(SPI.Port.kMXP);

  private Field2d field;

  // Provides variable to store motor voltage for simulator use
  private double simRightVolts;
  private double simLeftVolts;

  public DriveTrain() {

    leftDriveTalon = new WPI_TalonSRX(Constants.OperatorConstants.leftDriveTalonPort);
    rightDriveTalon = new WPI_TalonSRX(Constants.OperatorConstants.rightDriveTalonPort);

    leftDriveSim = leftDriveTalon.getSimCollection();
    rightDriveSim = rightDriveTalon.getSimCollection();

    // Create the simulation model of our drivetrain.
    driveSim = new DifferentialDrivetrainSim(
      // Create a linear system from our identification gains.
      LinearSystemId.identifyDrivetrainSystem(Constants.SimConstants.kV,
      Constants.SimConstants.kA, Constants.SimConstants.kVangular,
      Constants.SimConstants.kAangular),
      DCMotor.getCIM(1), // 1 CIM motor on each side of the drivetrain.
      10.71, // 10.71:1 gearing reduction.
      Constants.SimConstants.kTrackwidthMeters, // The track width is 0.7112
      Units.inchesToMeters(Constants.OperatorConstants.wheelDiameterInInches/2), // The robot uses 3" radius wheels.
        
      // The standard deviations for measurement noise:
      // x and y: 0.001 m
      // heading: 0.001 rad
      // l and r velocity: 0.1 m/s
      // l and r position: 0.005 m
      VecBuilder.fill(0.0001, 0.0001, 0.0001, 0.01, 0.01, 0.0005, 0.0005)
    );
    driveSim.setPose(new Pose2d(4,4,new Rotation2d(0)));    

    // Motor settings
    leftDriveTalon.setNeutralMode(NeutralMode.Brake);
    rightDriveTalon.setNeutralMode(NeutralMode.Brake);

    leftDriveTalon.setInverted(false);
    rightDriveTalon.setInverted(true);

    leftDriveTalon.setSensorPhase(true);
    rightDriveTalon.setSensorPhase(true);

    leftDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    rightDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

    resetEncoders();
    navx.reset();
    field = new Field2d();
    field.setRobotPose(driveSim.getPose());

    drive = new DifferentialDrive(leftDriveTalon, rightDriveTalon);

    leftDriveTalon.setExpiration(.02);
    rightDriveTalon.setExpiration(.02);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // Set inputs of voltage
    driveSim.setInputs(simLeftVolts, simRightVolts);
    // Update with dt of 0.02
    driveSim.update(0.02);

    // Update Quadrature for Left

    leftDriveSim.setQuadratureRawPosition(
        distanceToNativeUnits(
            -driveSim.getLeftPositionMeters()));
    leftDriveSim.setQuadratureVelocity(
        velocityToNativeUnits(
            -driveSim.getLeftVelocityMetersPerSecond()));

    // Update Quadrature for Right
    // Have to flip, to match phase of real encoder
    // Left wheel goes CCW, Right goes CW for forward by default

    rightDriveSim.setQuadratureRawPosition(
        distanceToNativeUnits(
            driveSim.getRightPositionMeters()));
    rightDriveSim.setQuadratureVelocity(
        velocityToNativeUnits(
            driveSim.getRightVelocityMetersPerSecond()));

    // Update Gyro
    int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    angle.set(driveSim.getHeading().getDegrees());

    field.setRobotPose(driveSim.getPose());
    SmartDashboard.putData("Field", field);

  }

  
  /**
   * Controls the left and right sides of the drive directly with voltages.
   * Voltage is the native unit of Feedforward with WPILib
   *
   * @param left  the commanded left output
   * @param right the commanded right output
   */
  public void tankDrive(double left, double right) {
    simLeftVolts = left*12.0;
    simRightVolts = right*12.0;
    leftDriveTalon.setVoltage(left);
    rightDriveTalon.setVoltage(right);
    // WPILib would spit out "Looptime Overrun!" if this isn't included!
    drive.feed();
  }

  /**
   * Resets the chassis encoders to 0 ticks.
   */
  public void resetEncoders() {
    leftDriveTalon.setSelectedSensorPosition(0);
    rightDriveTalon.setSelectedSensorPosition(0);
  }

  // CTRE SIM methods:

  private int distanceToNativeUnits(double positionMeters) {
    double wheelRotations = positionMeters
        / (Math.PI * Units.inchesToMeters(Constants.OperatorConstants.wheelDiameterInInches));
    double motorRotations = wheelRotations * 1.0;
    int sensorCounts = (int) (motorRotations * 4096.0);
    return sensorCounts;
  }
  
  private int velocityToNativeUnits(double velocityMetersPerSecond) {
    double wheelRotationsPerSecond = velocityMetersPerSecond
        / (Math.PI * Units.inchesToMeters(Constants.OperatorConstants.wheelDiameterInInches));
    double motorRotationsPerSecond = wheelRotationsPerSecond * 1.0;
    double motorRotationsPer100ms = motorRotationsPerSecond / 10.0;
    int sensorCountsPer100ms = (int) (motorRotationsPer100ms * 4096.0);
    return sensorCountsPer100ms;
  }

  public double metersToTicks(double positionMeters){
    return positionMeters / (0.1524 * Math.PI) * 4096.0;
  }

  public double getTicks() {
    return (leftDriveTalon.getSelectedSensorPosition(0) + rightDriveTalon.getSelectedSensorPosition(0)) / 2.0;
  }

  public double ticksToMeters() {
    return (0.1524 * Math.PI / 4096.0) * getTicks();
  }

  public double getAngle() {
    return navx.getAngle();
  }

  public void resetNavx() {
    navx.reset();
  }
}