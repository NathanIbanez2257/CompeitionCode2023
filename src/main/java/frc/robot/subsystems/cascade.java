// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CascadeConstants;

public class cascade extends SubsystemBase {

  WPI_TalonFX cascade = new WPI_TalonFX(CascadeConstants.cascadeID);

  public cascade() {
    breakMode();

    // cascade.setSelectedSensorPosition(-2432);

    cascade.configForwardSoftLimitEnable(true);
    cascade.configReverseSoftLimitEnable(true);

    // 118166
    // cascade.configForwardSoftLimitThreshold(116825);
    // cascade.configForwardSoftLimitThreshold(116000);
    cascade.configForwardSoftLimitThreshold(115900);
    

    // cascade.configReverseSoftLimitThreshold(1100);
    cascade.configReverseSoftLimitThreshold(-700);

  }

  public void setVoltage(double volts) {
    cascade.setVoltage(volts);
  }

  public void move(double speed) {
    cascade.set(speed);
  }

  public void breakMode() {
    cascade.setNeutralMode(NeutralMode.Brake);
  }

  public double cascadeTick2Feet() {
    double motorRotations = cascade.getSelectedSensorPosition() / CascadeConstants.kCountsPerRev;
    double cascadeTicksToFeet = motorRotations
        / (CascadeConstants.kCascadeGearRatio * CascadeConstants.kCascadeScaleFactor);

    return cascadeTicksToFeet;
  }

  public double cascadeVelocityFeetPerSecond() {
    double motorRotations = cascade.getSelectedSensorPosition() / CascadeConstants.kCountsPerRev;
    double cascadeTicksToFeet = motorRotations
        / (CascadeConstants.kCascadeGearRatio * CascadeConstants.kCascadeScaleFactor);

    return cascadeTicksToFeet / 60;
  }

  @Override
  public void periodic() {

    // cascadeBottom();
    // cascadeTop();

    SmartDashboard.putNumber("Cascade Position", cascadeTick2Feet());
    SmartDashboard.putNumber("Cascade Encoder Position", cascade.getSelectedSensorPosition());
  }
}
