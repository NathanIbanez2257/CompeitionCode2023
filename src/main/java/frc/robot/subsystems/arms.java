// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class arms extends SubsystemBase {

  WPI_TalonFX leftArm = new WPI_TalonFX(ArmConstants.leftArmID);
  WPI_TalonFX rightArm = new WPI_TalonFX(ArmConstants.rightArmID);

  /** Creates a new arms. */
  public arms() {
    leftArm.setInverted(true);
    leftArm.follow(rightArm, FollowerType.PercentOutput);
    breakMode();
  }

  public void move(double speed) {
    rightArm.set(speed);
  }

  public void breakMode()
  {
    leftArm.setNeutralMode(NeutralMode.Brake);
    rightArm.setNeutralMode(NeutralMode.Brake);

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
