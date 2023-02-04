// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive;

public class limelightTrackingCommand extends CommandBase {

  drive driveSub;
  PIDController forwardPID, turnPID;
  double fowardSpeed, rotationSpeed;

  public limelightTrackingCommand(drive drive) {
    driveSub = drive;

    turnPID = new PIDController(DriveConstants.KP, DriveConstants.KI, DriveConstants.KD);
    turnPID.setSetpoint(fowardSpeed);
    ;
    addRequirements(driveSub);
  }

  @Override
  public void initialize() {

    System.out.println("Command is being run");
    turnPID.reset();

    // turnPID.setTolerance(0);
  }

  @Override
  public void execute() {

    driveSub.returnResult();

    if (driveSub.hasTargets()) {
      rotationSpeed = -turnPID.calculate(driveSub.getLimelightYaw(), 0);
    }

    else {
      rotationSpeed = 0;
    }

    SmartDashboard.putBoolean("Has Targets Command", driveSub.hasTargets());
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
