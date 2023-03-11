// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.CascadeConstants;
import frc.robot.subsystems.cascade;

public class cascadePIDCommand extends CommandBase {

  private final cascade cascadeSub;
  private final PIDController cascadePID;
  private final double goal;
  private boolean done;

  public cascadePIDCommand(double setPoint, cascade cascade) {
    goal = setPoint;
    cascadeSub = cascade;
    cascadePID = new PIDController(CascadeConstants.KP, CascadeConstants.KI, CascadeConstants.KD);


    cascadePID.setSetpoint(setPoint);
    
    addRequirements(cascadeSub);
  }

  @Override
  public void initialize() {
    cascadePID.reset();
    cascadePID.setTolerance(0.005);
    System.out.println("\n\nCascade PID Command Has Started\n\n");

  }

  @Override
  public void execute() {

    done = cascadePID.atSetpoint();
    double speed = cascadePID.calculate(cascadeSub.cascadeTick2Feet(), goal);
    cascadeSub.move(speed);
  
    SmartDashboard.putBoolean("Cascade Check", cascadePID.atSetpoint());
    SmartDashboard.putNumber("Cascade Error", cascadePID.getPositionError());
  }

  @Override
  public void end(boolean interrupted) {
    cascadeSub.move(0);
    System.out.println("\n\nCascade PID Command Has Finished\n\n");

  }

  @Override
  public boolean isFinished() {
    return done;
  }

}
