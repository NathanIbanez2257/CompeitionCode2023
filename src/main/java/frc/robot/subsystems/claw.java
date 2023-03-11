// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;
import frc.robot.commands.clawPIDCommand;

public class claw extends SubsystemBase {

  WPI_TalonFX claw = new WPI_TalonFX(ClawConstants.clawID);

  public claw() {
    // test claw PIDS tommorow morning

    // claw.setSelectedSensorPosition(0);

    claw.setNeutralMode(NeutralMode.Brake);
  }

  public void move(double speed) {
    claw.set(speed);
  }

  public double clawTickToDegrees() {
    double motorRotations = claw.getSelectedSensorPosition()
        / (ClawConstants.kCountsPerRev * ClawConstants.kClawGearRatio);
    double cascadeTicksToInches = motorRotations * ClawConstants.kClawScaleFactor;

    return cascadeTicksToInches;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Claw Encoder", claw.getSelectedSensorPosition());
    SmartDashboard.putNumber("Claw To Inches", clawTickToDegrees());
  }


}
