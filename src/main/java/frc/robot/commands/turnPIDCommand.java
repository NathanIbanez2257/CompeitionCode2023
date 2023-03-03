// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutonConstatns;

import frc.robot.subsystems.drive;

public class turnPIDCommand extends CommandBase {

  private final drive driveSub;
  private final PIDController drivePID;
  private final double goal;
  private boolean done;

  public turnPIDCommand(double setPoint, drive drive) {
    goal = setPoint;
    driveSub = drive;
    drivePID = new PIDController(AutonConstatns.turnKP, AutonConstatns.turnKI, AutonConstatns.turnKD);

    drivePID.setSetpoint(setPoint);

    addRequirements(driveSub);
  }

  @Override
  public void initialize() {
    drivePID.reset();
    //drivePID.setTolerance(.021);
    driveSub.resetYaw();

    System.out.println("Turn PID Command Has Started");

    // cascadePID.calculate(cascadeSub.cascadeTick2Feet(), goal);
    /*
     * cascadeSub.setVoltage(cascadePID.calculate(cascadeSub.cascadeTick2Feet(),
     * goal) + feedForward.calculate(goal));
     * 
     * 
     */

  }

  @Override
  public void execute() {

    done = drivePID.atSetpoint();

    double speed = drivePID.calculate(driveSub.getYaw(), goal);

    driveSub.move(speed, -speed);

    SmartDashboard.putBoolean("Tolerance Check Turn", drivePID.atSetpoint());
    SmartDashboard.putNumber("Drive Error Turn", drivePID.getPositionError());

  }

  @Override
  public void end(boolean interrupted) {

    System.out.println("\n \n Turn Command Has Ended");

  }

  @Override
  public boolean isFinished() {
    return done;
  }

}
