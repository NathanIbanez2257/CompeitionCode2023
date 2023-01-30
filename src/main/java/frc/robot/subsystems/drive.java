// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

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

  PhotonCamera camera = new PhotonCamera("photonvision");

  public static final Gyro pigeon2 = new WPI_Pigeon2(DriveConstants.gyroID);

  public static final WPI_Pigeon2 gyro = new WPI_Pigeon2(DriveConstants.gyroID);

  PhotonPipelineResult result = camera.getLatestResult();

  public drive() {

    // PortForwarder.add(5800, "photonvision.local", 5800);
    setBreakMode();
    followSides();
    // gyro.reset();

  }

  public PhotonPipelineResult returnResult() {
    return result;
  }

  public boolean hasTargets()
  {
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

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftFront.setVoltage(-leftVolts);
    rightFront.setVoltage(rightVolts);
    drive.feed();
  }

  public double getVertical() {
    return -gyro.getPitch();
  }

  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Pitch", getVertical());

  }
}
