// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import edu.wpi.first.wpilibj2.command.Command;
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

  public static final Gyro gyro = new WPI_Pigeon2(DriveConstants.gyroID);

  public static final WPI_Pigeon2 pigeon = new WPI_Pigeon2(DriveConstants.gyroID);

  private final DifferentialDriveOdometry m_Odometry;
  
  CameraServer camera;
  

  double x, y, a, distanceToTarget;

  public drive() {
    camera.startAutomaticCapture();

    
    resetEncoders();
  
    // PortForwarder.add(5800, "photonvision.local", 5800);
    setBreakMode();
    followSides();
    
    rightSide.setInverted(true);

    m_Odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), leftNativeDistanceInMetersOdometry(),
        rightNativeDistanceInMetersOdometry());

    m_Odometry.resetPosition(gyro.getRotation2d(), leftNativeDistanceInMetersOdometry(), rightNativeDistanceInMetersOdometry(),
        new Pose2d());
  }

  public double getAngle() {
    return -gyro.getAngle();
  }

  public void move(double leftSpeed, double rightSpeed) {
    rightSide.setInverted(true);
    drive.tankDrive(leftSpeed, rightSpeed);
  }

  public void arcadeMove(double fowardSpeed, double turnSpeed) {
    rightSide.setInverted(true);
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
    return rightFront.getSelectedSensorPosition();
  }

  public double getLeftSideEncoderPosition() {
    return leftFront.getSelectedSensorPosition();
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

  public void tankDriveVoltsOdometry(double leftVolts, double rightVolts) {
    leftFront.setVoltage(leftVolts);
    rightFront.setVoltage(rightVolts);
    drive.feed();
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

    return positionMeters;
  }

  public double leftNativeDistanceInMetersOdometry() {
    double motorRotations = leftFront.getSelectedSensorPosition() / KineConstants.kCountsPerRev;
    double wheelRotations = motorRotations / KineConstants.kGearRatio;

    double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(KineConstants.kWheelRadiusInches));

    return -positionMeters;
  }

  public double rightNativeDistanceInMetersOdometry() {
    double motorRotations = rightFront.getSelectedSensorPosition() / KineConstants.kCountsPerRev;
    double wheelRotations = motorRotations / KineConstants.kGearRatio;

    double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(KineConstants.kWheelRadiusInches));

    return positionMeters;
  }

  public double leftNativeVelocityInMetersOdometry() {
    double motorRotations = rightFront.getSelectedSensorVelocity() / KineConstants.kCountsPerRev;
    double wheelRotations = motorRotations / KineConstants.kGearRatio;

    double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(KineConstants.kWheelRadiusInches));

    return positionMeters / 60;
  }

  public double rightNativeVelocityInMetersOdometry() {
    double motorRotations = rightFront.getSelectedSensorVelocity() / KineConstants.kCountsPerRev;
    double wheelRotations = motorRotations / KineConstants.kGearRatio;

    double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(KineConstants.kWheelRadiusInches));

    return positionMeters / 60;
  }

  public DifferentialDriveWheelSpeeds getWheelSpeedsOdometry() {
    return new DifferentialDriveWheelSpeeds(leftNativeVelocityInMeters(), rightNativeVelocityInMeters());
  }

  public double leftNativeVelocityInMeters() {
    double motorRotations = rightFront.getSelectedSensorVelocity() / KineConstants.kCountsPerRev;
    double wheelRotations = motorRotations / KineConstants.kGearRatio;

    double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(KineConstants.kWheelRadiusInches));

    return -positionMeters / 60;
  }

  public double rightNativeVelocityInMeters() {
    double motorRotations = rightFront.getSelectedSensorVelocity() / KineConstants.kCountsPerRev;
    double wheelRotations = motorRotations / KineConstants.kGearRatio;

    double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(KineConstants.kWheelRadiusInches));

    return positionMeters / 60;
  }

  public double getHeading() {
    return gyro.getRotation2d().getDegrees();
  }

  public double getYaw()
  {
    return pigeon.getYaw();
  }

  public void resetYaw()
  {
    pigeon.setYaw(0);
  }

  public double getTurnRate() {
    return -gyro.getRate();
  }

  public Pose2d getPose() {
    return m_Odometry.getPoseMeters();// check this
  }
  

  public void resetOdometry(Pose2d pose) {
    
    resetEncoders();
    gyro.reset();//
    m_Odometry.resetPosition(gyro.getRotation2d(), leftNativeDistanceInMetersOdometry(), rightNativeDistanceInMetersOdometry(), pose);
    System.out.println("reset started");
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftNativeVelocityInMeters(), rightNativeVelocityInMeters());
  }

  public double getAvgPositionDistance() {
    return ((leftNativeDistanceInMeters() + rightNativeDistanceInMeters()) / 2.0);
  }

  public void zeroHeading() {
    //gyro.calibrate();
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

  public double getRate() {
    return -gyro.getRate();
  }

  public double currentDistance() {
    return distanceToTarget;
  }

  @Override
  public void periodic() {

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    a = ta.getDouble(0.0);

    SmartDashboard.putNumber("Limelight X", x);
    SmartDashboard.putNumber("Limelight Y", y);
    SmartDashboard.putNumber("Limelight Area", a);

    double limelightMountDegrees = 30,
           limelightHeightInches = 19,
           goalHeightInches = 44;

    double angleToGoal = limelightMountDegrees + y,
           angletoGoalRadians = Units.degreesToRadians(angleToGoal); 
    
    double distanceFromLimelightToGoalInches = (goalHeightInches - limelightHeightInches) / Math.tan(angletoGoalRadians);

    SmartDashboard.putNumber("Limelight Distance From Target", distanceFromLimelightToGoalInches);

    SmartDashboard.putNumber("Pitch", getVertical());

    m_Odometry.update(gyro.getRotation2d(), leftNativeDistanceInMetersOdometry(),
        rightNativeDistanceInMetersOdometry());

    // SmartDashboard.putNumber("Pose X", m_Odometry.getPoseMeters().getX());
    // SmartDashboard.putNumber("Pose Y", m_Odometry.getPoseMeters().getY());
    // SmartDashboard.putNumber("Pose Gyro", pigeon.getRotation2d().getDegrees());
    // SmartDashboard.putNumber(" Gyro pose thingy", gyro.getRotation2d().getDegrees());

    SmartDashboard.putNumber("Gyro Yaw", pigeon.getYaw());


    SmartDashboard.putNumber("Gyro Rate", getTurnRate());

    SmartDashboard.putNumber("Left Side Encoders Meters", leftNativeDistanceInMetersOdometry());
    SmartDashboard.putNumber("Right Side Encoders Meters", rightNativeDistanceInMetersOdometry());

    SmartDashboard.putNumber("Left Side Encoders Velocity", leftNativeVelocityInMetersOdometry());
    SmartDashboard.putNumber("Right Side Encoders Velocity", rightNativeVelocityInMetersOdometry());

  }
}
