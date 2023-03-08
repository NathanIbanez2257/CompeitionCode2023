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

  public cascadePIDCommand(double setPoint, cascade cascade) {
    goal = setPoint;
    cascadeSub = cascade;
    cascadePID = new PIDController(CascadeConstants.KP, CascadeConstants.KI, CascadeConstants.KD);

    // feedForward = new ElevatorFeedforward(CascadeConstants.ksVolts,
    // CascadeConstants.kgVolts, CascadeConstants.kvVoltSecondPerMeters);

    cascadePID.setSetpoint(setPoint);
    
    addRequirements(cascadeSub);
  }

  @Override
  public void initialize() {
    cascadePID.reset();
    cascadePID.setTolerance(.1);
  }

  @Override
  public void execute() {

    double speed = cascadePID.calculate(cascadeSub.cascadeTick2Feet());
    cascadeSub.move(speed);
    //cascadePID.calculate(cascadeSub.cascadeTick2Feet(), goal);
     /*
     cascadeSub.setVoltage(cascadePID.calculate(cascadeSub.cascadeTick2Feet(),
     goal) + feedForward.calculate(goal));

     
     */

    SmartDashboard.putBoolean("Tolerance Check", cascadePID.atSetpoint());
  }

  @Override
  public void end(boolean interrupted) {
    cascadeSub.move(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  // private void cascadeWithFeedforwardPID(double cascadeVelocitySetpoint) {
  //   cascadeSub.setVoltage(feedForward.calculate(cascadeVelocitySetpoint)
  //       + cascadePID.calculate(cascadeSub.cascadeVelocityFeetPerSecond(), cascadeVelocitySetpoint));
  // }

}
