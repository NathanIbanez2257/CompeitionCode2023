// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.GioControllerConstants;
import frc.robot.Constants.GyroConstants;
import frc.robot.Constants.NathanControllerConstants;
import frc.robot.Constants.SpeedConstants;
import frc.robot.commands.armsCommand;
import frc.robot.commands.cascadeCommand;
import frc.robot.commands.cascadePIDCommand;
import frc.robot.commands.chargeCommand;
import frc.robot.commands.clawCommand;
import frc.robot.subsystems.arms;
import frc.robot.subsystems.cascade;
import frc.robot.subsystems.claw;
import frc.robot.subsystems.drive;

public class RobotContainer {

  private static final drive driveSub = new drive();
  private static final claw clawSub = new claw();
  private static final cascade cascadeSub = new cascade();
  private static final arms armsSub = new arms();

  private static final armsCommand armUpCommand = new armsCommand(armsSub, SpeedConstants.armSpeed);
  private static final armsCommand armDownCommand = new armsCommand(armsSub, -SpeedConstants.armSpeed);

  private static final cascadeCommand cascadeUpCommand = new cascadeCommand(cascadeSub, SpeedConstants.cascadeSpeed);
  private static final cascadeCommand cascadeDownCommand = new cascadeCommand(cascadeSub, -SpeedConstants.cascadeSpeed);

  private static final cascadePIDCommand cascadePIDMid = new cascadePIDCommand(1.4166, cascadeSub); 
  private static final cascadePIDCommand cascadePIDBase = new cascadePIDCommand(0, cascadeSub); 


  private static final clawCommand clawOpenCommand = new clawCommand(clawSub, SpeedConstants.clawSpeed);
  private static final clawCommand clawCloseCommand = new clawCommand(clawSub, -SpeedConstants.clawSpeed);

  private static final chargeCommand chargeBalanceCommand = new chargeCommand(driveSub, GyroConstants.gyroAngle);

  private static final Joystick nathan = new Joystick(NathanControllerConstants.nathan);
  private static final Joystick gio = new Joystick(GioControllerConstants.gio);


  RunCommand nathanMove = new RunCommand(
      () -> driveSub.move(SpeedConstants.driveSpeed * nathan.getRawAxis(NathanControllerConstants.leftDriveAxis),
      SpeedConstants.driveSpeed * nathan.getRawAxis(NathanControllerConstants.rightDriveAxis)),
      driveSub);


  
  public RobotContainer() {

    driveSub.setDefaultCommand(nathanMove);
 
    JoystickButton armUp = new JoystickButton(gio, GioControllerConstants.armUpButton);
    armUp.whileTrue(armUpCommand);

    JoystickButton armDown = new JoystickButton(gio, GioControllerConstants.armDownButton);
    armDown.whileTrue(armDownCommand);


    JoystickButton clawOpen = new JoystickButton(gio, GioControllerConstants.clawOpenButton);
    clawOpen.whileTrue(clawOpenCommand);

    JoystickButton clawClose = new JoystickButton(gio, GioControllerConstants.clawCloseButton);
    clawClose.whileTrue(clawCloseCommand);

    JoystickButton cascadeUp = new JoystickButton(nathan, NathanControllerConstants.cascadeUpButton);
    cascadeUp.whileTrue(cascadeUpCommand);

    JoystickButton cascadeDown = new JoystickButton(nathan, NathanControllerConstants.cascadeDownButton);
    cascadeDown.whileTrue(cascadeDownCommand);

    /*JoystickButton chargeBalance = new JoystickButton(nathan, NathanControllerConstants.gyroBalanceButton);
    chargeBalance.whileTrue(chargeBalanceCommand);*/

    JoystickButton cascadePIDMid = new JoystickButton(nathan, NathanControllerConstants.gyroBalanceButton);
    cascadePIDMid.onTrue(RobotContainer.cascadePIDMid);

    JoystickButton cascadePIDBase = new JoystickButton(nathan, NathanControllerConstants.cascadeZeroButton);
    cascadePIDBase.onTrue(RobotContainer.cascadePIDBase);
    
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    
  }



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
