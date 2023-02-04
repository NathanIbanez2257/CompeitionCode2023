// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.drive;

public class limelightDistanceCommand extends CommandBase {

  drive driveSub;
  PIDController forwardPID, turnPID;
  double fowardSpeed, rotationSpeed;

  public limelightDistanceCommand(double setPoint, drive drive) {
    driveSub = drive;

    forwardPID = new PIDController(DriveConstants.KP, DriveConstants.KI, DriveConstants.KD);
    forwardPID.setSetpoint(setPoint);
    addRequirements(driveSub);
  }

  @Override
  public void initialize() {
    forwardPID.reset();
    forwardPID.setTolerance(.5);

    driveSub.returnResult();

  }

  @Override
  public void execute() {
    if (driveSub.hasTargets()) {
      double range = PhotonUtils.calculateDistanceToTargetMeters(LimelightConstants.cameraHeightMeters,
          LimelightConstants.targetHeightMeters,
          LimelightConstants.cameraPitchRadians, Units.degreesToRadians(driveSub.getLimelightPitch()));

      forwardPID.calculate(range, LimelightConstants.targetRangeMeters);
    }

    else {
      fowardSpeed = 0;
    }
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