// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
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

  public static final WPI_Pigeon2 gyro = new WPI_Pigeon2(DriveConstants.gyroID);

  public drive() {
    setBreakMode();
    followSides();
    gyro.reset();
  }

  public void move(double leftSpeed, double rightSpeed) {
    leftSide.setInverted(true);
    drive.tankDrive(leftSpeed, rightSpeed);
  }

  public void arcadeMove(double fowardSpeed, double turnSpeed)  {
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

  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
  }

  

  @Override
  public void periodic() {

  }
}
