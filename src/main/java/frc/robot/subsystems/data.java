// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class data extends SubsystemBase {

  PowerDistribution PDH = new PowerDistribution(1, ModuleType.kRev);
  /** Creates a new data. */
  public data() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
