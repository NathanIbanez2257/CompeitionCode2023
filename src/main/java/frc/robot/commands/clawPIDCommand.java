// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.claw;

public class clawPIDCommand extends CommandBase {
  private final claw clawSub;
  private final double closePostions;
  private boolean done;

  private final PIDController clawPID; 

  /** Creates a new clawPIDCommand. */
  public clawPIDCommand(double clawDistance, claw claw) {
    closePostions = clawDistance;
    clawSub = claw;
    clawPID = new PIDController(ClawConstants.clawKP, ClawConstants.clawKI, ClawConstants.clawKD);
    clawPID.setSetpoint(clawDistance);
    addRequirements(clawSub);
  }

  @Override
  public void initialize() {
    clawPID.reset();
    clawPID.setTolerance(0.1);  
  }

  @Override
  public void execute() {
    done = clawPID.atSetpoint();
    double speed = clawPID.calculate(clawSub.clawTickToDegrees());
    clawSub.move(speed);

    SmartDashboard.putBoolean("Claw Tolerance", clawPID.atSetpoint());
    SmartDashboard.putNumber("Claw Error", clawPID.getPositionError());
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
