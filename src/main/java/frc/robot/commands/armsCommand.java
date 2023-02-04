// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arms;

public class armsCommand extends CommandBase {
  arms armsSub;
  double speed;

  public armsCommand(arms arms, double armSpeed) {
    armsSub = arms;
    speed = armSpeed;
    addRequirements(armsSub);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    armsSub.move(speed);
  }

  @Override
  public void end(boolean interrupted) {
    armsSub.move(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
