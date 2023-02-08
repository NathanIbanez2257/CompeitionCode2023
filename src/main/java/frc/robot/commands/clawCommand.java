// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.claw;

public class clawCommand extends CommandBase {
  claw clawSub;
  double speed;

  public clawCommand(double clawSpeed, claw claw) {
    clawSub = claw;
    speed = clawSpeed;
    addRequirements(clawSub);
  }

  @Override
  public void initialize() {
    clawSub.move(speed);
  }

  @Override
  public void execute() {
    clawSub.move(speed);
  }

  @Override
  public void end(boolean interrupted) {
    clawSub.move(0);

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
