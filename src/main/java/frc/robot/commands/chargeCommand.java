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

  
  /** Creates a new chargeCommand. */
  public chargeCommand(drive drive, double levelAngle) {

    driveSub = drive;
    levelAngle = levelGyro;
    addRequirements(driveSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    oldErrorGyro = 0;
    pastTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(driveSub.getVertical() == 0)
    {
      errorGyro = GyroConstants.gyroAngle + (-1 * driveSub.getVertical());
      
      pGyro = errorGyro * GyroConstants.gyroKP;

      double dt = Timer.getFPGATimestamp() - pastTime;
      pastTime = Timer.getFPGATimestamp();

      iGyro += (errorGyro *dt) * GyroConstants.gyroKI;


      oldErrorGyro = errorGyro;
      
      speedGyro = pGyro + iGyro;

      driveSub.move(speedGyro, speedGyro);

    }

    else
    {
      driveSub.move(0, 0);
    }

    SmartDashboard.putNumber("Aim Error", errorGyro);
    SmartDashboard.putNumber("Aim Speed", errorGyro);

  }

  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSub.move(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
