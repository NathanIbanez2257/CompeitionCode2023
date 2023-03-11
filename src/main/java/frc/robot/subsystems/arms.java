// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class arms extends SubsystemBase {

  WPI_TalonFX leftArm = new WPI_TalonFX(ArmConstants.leftArmID);
  WPI_TalonFX rightArm = new WPI_TalonFX(ArmConstants.rightArmID);

  public static CameraServer camera;

  /** Creates a new arms. */
  public arms() {

    // leftArm.setSelectedSensorPosition(-41368);
    // rightArm.setSelectedSensorPosition(-41368);

    // resetEncoders();
    //41368
    breakMode();
    leftArm.configForwardSoftLimitEnable(false);

    leftArm.configReverseSoftLimitEnable(false);
    rightArm.configReverseSoftLimitEnable(false);

    rightArm.configForwardSoftLimitEnable(false);
    rightArm.configReverseSoftLimitThreshold(39620);

    leftArm.setInverted(true);
    leftArm.follow(rightArm, FollowerType.PercentOutput);

  }

  public void move(double speed) {
    rightArm.set(speed);
  }

  private void resetEncoders() {
    leftArm.setSelectedSensorPosition(0);
    rightArm.setSelectedSensorPosition(0);
  }

  public void breakMode() {
    leftArm.setNeutralMode(NeutralMode.Brake);
    rightArm.setNeutralMode(NeutralMode.Brake);
  }

  public double armTickToDegrees() {

    double motorRotations = rightArm.getSelectedSensorPosition() / (2048 * 200);
    double cascadeTicksPerDegree = motorRotations * ArmConstants.kArmScaleFactor; // 359.489141

    // double answer = cascadeTicksFor90Degrees;

    return cascadeTicksPerDegree;

    // 137658 36720 103123

    /*
     * double motorRotations = rightArm.getSelectedSensorPosition() /
     * ArmConstants.kCountsPerRev;
     * double cascadeTicksFor90Degrees = motorRotations/
     * (ArmConstants.kArmGearRatio);
     * 
     * double answer = ((90/cascadeTicksFor90Degrees) * cascadeTicksFor90Degrees);
     * 
     * return answer;
     */
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Encoder Position", rightArm.getSelectedSensorPosition());
    SmartDashboard.putNumber("Arm Encoder In Degrees Arm", armTickToDegrees());
  }
}
