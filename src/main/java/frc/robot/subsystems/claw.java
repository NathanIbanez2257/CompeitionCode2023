// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

public class claw extends SubsystemBase {

  WPI_TalonFX claw = new WPI_TalonFX(ClawConstants.clawID);

  /** Creates a new claw. */
  public claw() {

    claw.setNeutralMode(NeutralMode.Brake);
  }

  public void move(double speed) {
    claw.set(speed);
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
