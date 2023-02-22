// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GyroConstants;
import frc.robot.subsystems.drive;

public class chargeCommand extends CommandBase {
  drive driveSub;

  double pGyro, iGyro, dGyro,

      errorGyro, oldErrorGyro, speedGyro, pastTime, levelGyro;

  public chargeCommand(drive drive) {

    driveSub = drive;
    addRequirements(driveSub);
  }

  @Override
  public void initialize() {
    oldErrorGyro = 0;
    pastTime = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {
    if (Math.abs(driveSub.getVertical()) > 1) {

      if (Math.abs(driveSub.getVertical()) < 5) {
        errorGyro = GyroConstants.gyroAngle + (driveSub.getVertical());

        pGyro = errorGyro * GyroConstants.shortGyroKP;

        double dt = Timer.getFPGATimestamp() - pastTime;
        pastTime = Timer.getFPGATimestamp();

        iGyro += (errorGyro * dt) * GyroConstants.shortGyroKI;

        oldErrorGyro = errorGyro;

        speedGyro = pGyro + iGyro;

        driveSub.move(speedGyro, speedGyro);
      }

      else if (Math.abs(driveSub.getVertical()) > 5) {
        errorGyro = GyroConstants.gyroAngle + (driveSub.getVertical());

        pGyro = errorGyro * GyroConstants.longGyroKP;

        double dt = Timer.getFPGATimestamp() - pastTime;
        pastTime = Timer.getFPGATimestamp();

        iGyro += (errorGyro * dt) * GyroConstants.longGyroKI;

        double dxDistance = errorGyro - oldErrorGyro;

        dGyro = (dxDistance / dt) * GyroConstants.longGyroKD;

        speedGyro = pGyro + iGyro - dGyro;

        oldErrorGyro = errorGyro;
        // speedDistance = proportionDistance + integralDistance + derivativeDistance;

        driveSub.move(speedGyro, speedGyro); // + speedDistance, -1* speedAim + speedDistance
      }

      else 
      {
        driveSub.move(0, 0);
      }
    
  }

    SmartDashboard.putNumber("Aim Error", errorGyro);

  }

  @Override
  public void end(boolean interrupted) {
    driveSub.move(0, 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
