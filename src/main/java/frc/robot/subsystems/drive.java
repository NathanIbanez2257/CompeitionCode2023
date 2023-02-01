// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.ctre.phoenixpro.configs.CurrentLimitsConfigs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.KineConstants;

// https://software-metadata.revrobotics.com/REVLib-2023.json       REV ROBOTICS JAVA DOWNLOAD ONLINE LIBRARY
// https://maven.photonvision.org/repository/internal/org/photonvision/PhotonLib-json/1.0/PhotonLib-json-1.0.json   PHOTONVISION JAVA DOWNLOAD ONLINE LIBRARY

public class drive extends SubsystemBase {

  WPI_TalonFX leftFront = new WPI_TalonFX(DriveConstants.leftFrontID);
  WPI_TalonFX leftBack = new WPI_TalonFX(DriveConstants.leftBackID);

  WPI_TalonFX rightFront = new WPI_TalonFX(DriveConstants.rightFrontID);
  WPI_TalonFX rightBack = new WPI_TalonFX(DriveConstants.rightBackID);

  MotorControllerGroup leftSide = new MotorControllerGroup(leftBack, leftFront);
  MotorControllerGroup rightSide = new MotorControllerGroup(rightBack, rightFront);

  DifferentialDrive drive = new DifferentialDrive(leftSide, rightSide);

  PhotonCamera camera = new PhotonCamera("limelight");

  public static final Gyro gyro = new WPI_Pigeon2(DriveConstants.gyroID);

  public static final WPI_Pigeon2 pigeon = new WPI_Pigeon2(DriveConstants.gyroID);

  PhotonPipelineResult result = camera.getLatestResult();

  private final DifferentialDriveOdometry m_Odometry;

  double x, y, a, distanceToTarget;

  public drive() {

    // returnResult();
    resetEncoders();

    // PortForwarder.add(5800, "photonvision.local", 5800);
    setBreakMode();
    followSides();
    // gyro.reset();

    gyro.reset();
    gyro.calibrate();

    m_Odometry = new DifferentialDriveOdometry(pigeon.getRotation2d(), leftNativeDistanceInMeters(),
        rightNativeDistanceInMeters());

    m_Odometry.resetPosition(gyro.getRotation2d(), leftNativeDistanceInMeters(), rightNativeDistanceInMeters(),
        new Pose2d());

  }

  public PhotonPipelineResult returnResult() {
    return result;
  }

  public boolean hasTargets() {
    return result.hasTargets();
  }

  public double getLimelightYaw() {
    return result.getBestTarget().getYaw();
  }

  public double getLimelightPitch() {
    return result.getBestTarget().getPitch();
  }

  public void move(double leftSpeed, double rightSpeed) {
    rightSide.setInverted(true);
    drive.tankDrive(leftSpeed, rightSpeed);
  }

  public void arcadeMove(double fowardSpeed, double turnSpeed) {
    leftSide.setInverted(true);
    drive.arcadeDrive(fowardSpeed, turnSpeed);
  }

  private void followSides() {
    leftBack.follow(leftFront);
    rightBack.follow(rightFront);
  }

  public void setBreakMode() {
    leftBack.setNeutralMode(NeutralMode.Brake);
    leftFront.setNeutralMode(NeutralMode.Brake);

    rightBack.setNeutralMode(NeutralMode.Brake);
    rightFront.setNeutralMode(NeutralMode.Brake);
  }

  public void setCoastMode() {
    leftBack.setNeutralMode(NeutralMode.Coast);
    leftFront.setNeutralMode(NeutralMode.Coast);

    rightBack.setNeutralMode(NeutralMode.Coast);
    rightFront.setNeutralMode(NeutralMode.Coast);
  }

  public void resetEncoders() {
    leftBack.setSelectedSensorPosition(0);
    leftFront.setSelectedSensorPosition(0);

    rightBack.setSelectedSensorPosition(0);
    rightFront.setSelectedSensorPosition(0);
  }

  public double getRightSideEncoderPosition() {
    return -rightFront.getSelectedSensorPosition();
  }

  public double getLeftSideEncoderPosition() {
    return -leftFront.getSelectedSensorPosition();
  }

  public double getVertical() {
    return -pigeon.getPitch();
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftFront.setVoltage(leftVolts);
    rightFront.setVoltage(rightVolts);
    drive.feed();
  }

  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
  }

  public double leftNativeDistanceInMeters() {
    double motorRotations = leftFront.getSelectedSensorPosition() / KineConstants.kCountsPerRev;
    double wheelRotations = motorRotations / KineConstants.kGearRatio;

    double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(KineConstants.kWheelRadiusInches));

    return -positionMeters;

  }

  public double rightNativeDistanceInMeters() {
    double motorRotations = rightFront.getSelectedSensorPosition() / KineConstants.kCountsPerRev;
    double wheelRotations = motorRotations / KineConstants.kGearRatio;

    double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(KineConstants.kWheelRadiusInches));

    return -positionMeters;

  }

  public double leftNativeVelocityInMeters() {
    double motorRotations = rightFront.getSelectedSensorPosition() / KineConstants.kCountsPerRev;
    double wheelRotations = motorRotations / KineConstants.kGearRatio;

    double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(KineConstants.kWheelRadiusInches));

    return -positionMeters / 60;

  }

  public double rightNativeVelocityInMeters() {
    double motorRotations = rightFront.getSelectedSensorPosition() / KineConstants.kCountsPerRev;
    double wheelRotations = motorRotations / KineConstants.kGearRatio;

    double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(KineConstants.kWheelRadiusInches));

    return -positionMeters / 60;

  }

  public double getHeading() {
    return gyro.getRotation2d().getDegrees();
  }

  public double getTurnRate() {
    return -gyro.getRate();
  }

  public Pose2d getPose() {
    return m_Odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_Odometry.resetPosition(gyro.getRotation2d(), leftNativeDistanceInMeters(), rightNativeDistanceInMeters(), pose);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftNativeVelocityInMeters(), rightNativeVelocityInMeters());
  }

  public double getAvgPositionDistance() {
    return (leftNativeDistanceInMeters() + rightNativeDistanceInMeters() / 2.0);
  }

  public void zeroHeading() {
    gyro.calibrate();
    gyro.reset();
  }

  public Gyro getGyro() {
    return getGyro();
  }

  public double angleOff() {
    return x;
  }

  public double verticalOffset() {
    return y;
  }

  public double targetArea() {
    return a;
  }

  public double currentDistance() {
    return distanceToTarget;
  }

  @Override
  public void periodic() {

    camera.getLatestResult();
    result.getTargets();
    result.hasTargets();

    /*
     * NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
     * NetworkTableEntry tx = table.getEntry("tx");
     * NetworkTableEntry ty = table.getEntry("ty");
     * NetworkTableEntry ta = table.getEntry("ta");
     * 
     * x = tx.getDouble(0.0);
     * y = ty.getDouble(0.0);
     * a = ta.getDouble(0.0);
     * 
     * SmartDashboard.putNumber("LimelightX", x);
     * SmartDashboard.putNumber("LimelightY", y);
     * SmartDashboard.putNumber("LimelightArea", a);
     * 
     * double limeHeightToTarget = 34 - 29;
     * double theta = Math.toRadians(90 + y);
     * distanceToTarget = limeHeightToTarget / (Math.tan(theta));
     */

    SmartDashboard.putNumber("Pitch", getVertical());
    SmartDashboard.putBoolean("Lime Works", result.hasTargets());

    m_Odometry.update(pigeon.getRotation2d(), leftNativeDistanceInMeters(),
        rightNativeDistanceInMeters());

    SmartDashboard.putNumber("Pose X", m_Odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Pose Y", m_Odometry.getPoseMeters().getY());
    SmartDashboard.putNumber("Pose Gyro", gyro.getAngle());

    SmartDashboard.putNumber("Left Side Encoders Meters", leftNativeDistanceInMeters());
    SmartDashboard.putNumber("Right Side Encoders Meters", rightNativeDistanceInMeters());

  }
}
