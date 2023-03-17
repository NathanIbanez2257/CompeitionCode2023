// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.arms;

public class armsPIDCommand extends CommandBase {

  private final arms armSub;
  private final PIDController armPID;
  private final double goal;
  private boolean done;  

  public armsPIDCommand(double setPoint, arms arms) {
    goal = setPoint;
    armSub = arms;
    armPID = new PIDController(ArmConstants.KP, ArmConstants.KI, ArmConstants.KD);
    armPID.setSetpoint(setPoint);

    addRequirements(armSub);
  }

  @Override
  public void initialize() {
    armPID.reset();
    System.out.println("\n\nArm PID Command Has Started\n\n");
    armPID.setTolerance(.25);
  }

  @Override
  public void execute() {

    done = armPID.atSetpoint();
    
    double speed = armPID.calculate(armSub.armTickToDegrees(), goal);
    armSub.move(speed);

    SmartDashboard.putBoolean("Arm Tolerance Check", armPID.atSetpoint());
    SmartDashboard.putNumber("Arm Tolerance", armPID.getPositionError());

  }

  @Override
  public void end(boolean interrupted) {

    System.out.println("\n\nArm PID Command Has Finished\n\n");
    armSub.move(0);
  }

  @Override
  public boolean isFinished() {
    return done;
  }

}
