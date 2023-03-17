// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GyroConstants;
import frc.robot.subsystems.drive;

public class chargePIDCommand extends CommandBase {

  private final drive driveSub;
  private final PIDController gyroLong, gyroShort;
  private final double goal;
  private boolean done, done2;

  public chargePIDCommand(double setPoint, drive drive) {
    goal = setPoint;
    driveSub = drive;
    gyroLong = new PIDController(GyroConstants.longGyroKP, GyroConstants.longGyroKI, GyroConstants.longGyroKD);
    gyroShort = new PIDController(GyroConstants.shortGyroKP, GyroConstants.shortGyroKI, GyroConstants.shortGyroKD);

    gyroLong.setSetpoint(setPoint);
    gyroShort.setSetpoint(setPoint);

    addRequirements(driveSub);
  }

  @Override
  public void initialize() {
    gyroLong.reset();
    gyroShort.reset();

    // drivePID.setTolerance(.021);
    System.out.println("Charge PID Command Has Started");

  }

  @Override
  public void execute() {

    if(Math.abs(driveSub.getVertical()) > 4)
    {
      done = gyroLong.atSetpoint();
      double speedLong = gyroLong.calculate(driveSub.getVertical(), goal);

      driveSub.move(-speedLong, -speedLong);
    }


    SmartDashboard.putNumber("GyroLong Error", gyroLong.getPositionError());
    SmartDashboard.putBoolean("GyroLong Check", gyroLong.atSetpoint());
  }

  @Override
  public void end(boolean interrupted) {

    System.out.println("\n \n Charge Command Has Ended");

  }

  @Override
  public boolean isFinished() {

    // return done || done2;
    return done;

  }

}
