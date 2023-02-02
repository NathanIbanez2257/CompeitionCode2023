// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.GioControllerConstants;
import frc.robot.Constants.KineConstants;
import frc.robot.Constants.NathanControllerConstants;
import frc.robot.Constants.SpeedConstants;
import frc.robot.commands.aimRobotCommand;
import frc.robot.commands.armsCommand;
import frc.robot.commands.cascadeCommand;
import frc.robot.commands.cascadePIDCommand;
import frc.robot.commands.clawCommand;
import frc.robot.commands.limelightTrackingCommand;
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

  // private static final limelightTrackingCommand limeTrackCommand = new
  // limelightTrackingCommand(driveSub);

  private static final cascadeCommand cascadeUpCommand = new cascadeCommand(cascadeSub, SpeedConstants.cascadeSpeed);
  private static final cascadeCommand cascadeDownCommand = new cascadeCommand(cascadeSub, -SpeedConstants.cascadeSpeed);

  private static final cascadePIDCommand cascadePIDMid = new cascadePIDCommand(1.3, cascadeSub);
  private static final cascadePIDCommand cascadePIDBase = new cascadePIDCommand(0, cascadeSub);

  private static final aimRobotCommand limeTestCommand = new aimRobotCommand(driveSub, 15);

  private static final clawCommand clawOpenCommand = new clawCommand(clawSub, SpeedConstants.clawSpeed);
  private static final clawCommand clawCloseCommand = new clawCommand(clawSub, -SpeedConstants.clawSpeed);

  // private static final chargeCommand chargeBalanceCommand = new
  // chargeCommand(driveSub, GyroConstants.gyroAngle);

  private static final Joystick nathan = new Joystick(NathanControllerConstants.nathan);
  private static final Joystick gio = new Joystick(GioControllerConstants.gio);

  String straightPath = "straight.wpilib.json";
  String path1 = "NEWPATH.wpilib.json";
  String AutonPath1 = "AutonPath1.wpilib.json";
  String AutonPath2 = "AutonPath2.wpilib.json";
  String OfficialTestingPath = "OfficialTestingAuton.wpilib.json";

  RunCommand nathanMove = new RunCommand(
      () -> driveSub.move(SpeedConstants.driveSpeed * nathan.getRawAxis(NathanControllerConstants.leftDriveAxis),
          SpeedConstants.driveSpeed * nathan.getRawAxis(NathanControllerConstants.rightDriveAxis)),
      driveSub);

  SendableChooser<Command> chooser = new SendableChooser<>();

  public RobotContainer() {

    // chooser.addOption("Straight Path", null);
    // chooser.addOption("Straight Path", null);

    // Shuffleboard.getTab("Autonomous").add(chooser);

    driveSub.setDefaultCommand(nathanMove);

    chooser.addOption("Official Testing",
        loadPathPlannerTrajectoryToRamseteCommand(
            OfficialTestingPath, true));

    chooser.addOption("straightPath",
        loadPathPlannerTrajectoryToRamseteCommand(
            straightPath, true));

    chooser.addOption("AutonPath1",
        loadPathPlannerTrajectoryToRamseteCommand(
            AutonPath1, true));

    chooser.addOption("AutonPath2",
        loadPathPlannerTrajectoryToRamseteCommand(
            AutonPath2, true));

    chooser.addOption("Path1",
        loadPathPlannerTrajectoryToRamseteCommand(
            path1, true));

    Shuffleboard.getTab("Autonomous").add(chooser);

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

    /*
     * JoystickButton chargeBalance = new JoystickButton(nathan,
     * NathanControllerConstants.gyroBalanceButton);
     * chargeBalance.whileTrue(chargeBalanceCommand);
     */

    JoystickButton cascadePIDMid = new JoystickButton(nathan, NathanControllerConstants.gyroBalanceButton);
    cascadePIDMid.onTrue(RobotContainer.cascadePIDMid);

    JoystickButton cascadePIDBase = new JoystickButton(nathan, NathanControllerConstants.cascadeZeroButton);
    cascadePIDBase.onTrue(RobotContainer.cascadePIDBase);

    JoystickButton limeTrack = new JoystickButton(gio, 14);
    limeTrack.whileTrue(limeTestCommand);

    configureBindings();

  }

  public Command loadPathPlannerTrajectoryToRamseteCommand(String filename, boolean resetOdometry) {
    Trajectory trajectory;

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filename);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      System.out.println("winning");
    }

    catch (Exception exception) {
      DriverStation.reportError("Unable To Open Trajectory " + filename, exception.getStackTrace());
      System.out.println("Unable to read from file" + filename);
      return new InstantCommand();
    }

    RamseteCommand ramseteCommand = new RamseteCommand(trajectory, driveSub::getPose,
        new RamseteController(KineConstants.kRamseteB, KineConstants.kRamseteZeta),
        new SimpleMotorFeedforward(KineConstants.ksVolts, KineConstants.kvVoltSecondsPerMeter,
            KineConstants.kaVoltSecondSquaredPerMeter),
        KineConstants.kDrive,
        driveSub::getWheelSpeeds,
        new PIDController(KineConstants.kpDriveVelocity, 0, 0),
        new PIDController(KineConstants.kpDriveVelocity, 0, 0),
        driveSub::tankDriveVolts,
        driveSub);

    if (resetOdometry) {
      return new SequentialCommandGroup(
          new InstantCommand(() -> driveSub.resetOdometry(trajectory.getInitialPose())), ramseteCommand);
    }

    else {
      return ramseteCommand;
    }

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
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
    return chooser.getSelected();
  }
}
