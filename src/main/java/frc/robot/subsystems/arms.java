// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class arms extends SubsystemBase {

  WPI_TalonFX leftArm = new WPI_TalonFX(ArmConstants.leftArmID);
  WPI_TalonFX rightArm = new WPI_TalonFX(ArmConstants.rightArmID);


  /** Creates a new arms. */
  public arms() {
    leftArm.configForwardSoftLimitEnable(true);
    leftArm.configForwardSoftLimitThreshold(0);

    rightArm.configReverseSoftLimitEnable(true);
    rightArm.configForwardSoftLimitThreshold(0);

    leftArm.setInverted(true);
    leftArm.follow(rightArm, FollowerType.PercentOutput);
    breakMode();
   
  }

  public void move(double speed) {
    rightArm.set(speed);
 
  }
  
  private void resetEncoders()
  {
    leftArm.setSelectedSensorPosition(0);
    rightArm.setSelectedSensorPosition(0);
  }

  public void breakMode() {
    leftArm.setNeutralMode(NeutralMode.Coast);
    rightArm.setNeutralMode(NeutralMode.Coast);
  }

  public double armTickToDegrees() {

    
    //137658 36720

    double motorRotations = rightArm.getSelectedSensorPosition() / ArmConstants.kCountsPerRev;
    double cascadeTicksFor90Degrees = motorRotations/ (ArmConstants.kArmGearRatio * ArmConstants.kArmScaleFactor);
    double cascadeTickPerDegree = cascadeTicksFor90Degrees * (Math.pow(cascadeTicksFor90Degrees, -1));

    return cascadeTickPerDegree;
  }




  @Override
  public void periodic() {
    SmartDashboard.putNumber("Encoder Position Arms", rightArm.getSelectedSensorPosition());
    SmartDashboard.putNumber("Encoder In Degrees Arm", armTickToDegrees());
  }
}
