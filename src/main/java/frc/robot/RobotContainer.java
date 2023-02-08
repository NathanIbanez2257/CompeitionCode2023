// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.nio.file.Path;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GioControllerConstants;
import frc.robot.Constants.GyroConstants;
import frc.robot.Constants.KineConstants;
import frc.robot.Constants.NathanControllerConstants;
import frc.robot.Constants.SpeedConstants;
import frc.robot.commands.aimRobotCommand;
import frc.robot.commands.armsCommand;
import frc.robot.commands.armsPIDCommand;
import frc.robot.commands.cascadeCommand;
import frc.robot.commands.cascadePIDCommand;
import frc.robot.commands.chargeCommand;
import frc.robot.commands.clawCommand;
import frc.robot.commands.clawPIDCommand;
import frc.robot.commands.driveAutonPIDCommand;
import frc.robot.commands.limelightTrackingCommand;
import frc.robot.commands.meterDriveCommand;
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

        private static final armsPIDCommand armsZeroCommand = new armsPIDCommand(0, armsSub);
        private static final armsPIDCommand armsHighCommand = new armsPIDCommand(143, armsSub);

        // 143

        // private static final limelightTrackingCommand limeTrackCommand = new
        // limelightTrackingCommand(driveSub);

        private static final cascadeCommand cascadeUpCommand = new cascadeCommand(cascadeSub,
                        SpeedConstants.cascadeSpeed);
        private static final cascadeCommand cascadeDownCommand = new cascadeCommand(cascadeSub,
                        -SpeedConstants.cascadeSpeed);

        private static final cascadePIDCommand cascadePIDMid = new cascadePIDCommand(1.3, cascadeSub);
        private static final cascadePIDCommand cascadePIDZero = new cascadePIDCommand(0, cascadeSub);

        private static final aimRobotCommand limeTestCommand = new aimRobotCommand(driveSub, 15);

        private static final clawCommand clawOpenCommand = new clawCommand(SpeedConstants.clawSpeed, clawSub);
        private static final clawCommand clawCloseCommand = new clawCommand(-SpeedConstants.clawSpeed, clawSub);

        private static final clawPIDCommand clawOpenPIDCommand = new clawPIDCommand(80, clawSub);

        private static final chargeCommand chargeBalanceCommand = new chargeCommand(driveSub, GyroConstants.gyroAngle);

        private static final Joystick nathan = new Joystick(NathanControllerConstants.nathan);
        private static final Joystick gio = new Joystick(GioControllerConstants.gio);

        String straightPath = "straight.wpilib.json";
        String path1 = "NEWPATH.wpilib.json";
        String AutonPath1 = "AutonPath1.wpilib.json";
        String AutonPath2 = "AutonPath2.wpilib.json";
        String OfficialTestingPath = "OfficialTestingAuton.wpilib.json";
        String andresPath = "testwitandres.wpilib.json";

        RunCommand nathanMove = new RunCommand(
                        () -> driveSub.arcadeMove(
                                        SpeedConstants.driveSpeed
                                                        * nathan.getRawAxis(NathanControllerConstants.forwardAxis),
                                        SpeedConstants.driveSpeed
                                                        * nathan.getRawAxis(NathanControllerConstants.turnAxis)),
                        driveSub);

        SendableChooser<Command> chooser = new SendableChooser<>();

        public RobotContainer() {

                // chooser.addOption("Straight Path", null);
                // chooser.addOption("Straight Path", null);

                // Shuffleboard.getTab("Autonomous").add(chooser);

                driveSub.setDefaultCommand(nathanMove);

                chooser.addOption("Andres Testing",
                                loadPathPlannerTrajectoryToRamseteCommand(
                                                andresPath, true));

                chooser.addOption("Official Testing",
                                loadPathPlannerTrajectoryToRamseteCommand(
                                                OfficialTestingPath, false));

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

                ///////////////// Gio Controls ///////////////////////

                JoystickButton armUp = new JoystickButton(gio, GioControllerConstants.armUpButton);
                armUp.whileTrue(armUpCommand);

                JoystickButton armDown = new JoystickButton(gio, GioControllerConstants.armDownButton);
                armDown.whileTrue(armDownCommand);

                JoystickButton clawOpen = new JoystickButton(gio, GioControllerConstants.clawOpenButton);
                clawOpen.whileTrue(clawOpenCommand);

                JoystickButton clawClose = new JoystickButton(gio, GioControllerConstants.clawCloseButton);
                clawClose.whileTrue(clawCloseCommand);

                JoystickButton armsHigh = new JoystickButton(gio, 4);
                armsHigh.onTrue(armsHighCommand);

                JoystickButton armsZero = new JoystickButton(gio, 2);
                armsZero.onTrue(armsZeroCommand);

                JoystickButton clawOpenPID = new JoystickButton(gio, 1);
                clawOpenPID.onTrue(clawOpenPIDCommand);

                ///////////////// Nathan Controls ///////////////////////

                JoystickButton cascadeUp = new JoystickButton(nathan, NathanControllerConstants.cascadeUpButton);
                cascadeUp.whileTrue(cascadeUpCommand);

                JoystickButton cascadeDown = new JoystickButton(nathan, NathanControllerConstants.cascadeDownButton);
                cascadeDown.whileTrue(cascadeDownCommand);

                JoystickButton chargeBalance = new JoystickButton(nathan,
                                NathanControllerConstants.gyroBalanceButton);
                chargeBalance.whileTrue(chargeBalanceCommand);

                ///////////////// Nathan Controls ///////////////////////

                JoystickButton cascadePIDMid = new JoystickButton(nathan,
                                NathanControllerConstants.cascadeConeMidButton);
                cascadePIDMid.onTrue(RobotContainer.cascadePIDMid);

                JoystickButton cascadePIDZero = new JoystickButton(nathan, NathanControllerConstants.cascadeZeroButton);
                cascadePIDZero.onTrue(RobotContainer.cascadePIDZero);

                JoystickButton limeTrack = new JoystickButton(nathan, NathanControllerConstants.limeTrackButton);
                limeTrack.whileTrue(limeTestCommand);

                configureBindings();

        }

        ParallelCommandGroup AutonomousPractice = new ParallelCommandGroup(
                        loadPathPlannerTrajectoryToRamseteCommand(OfficialTestingPath, true));

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

                RamseteCommand ramseteCommand = new RamseteCommand(

                                trajectory, driveSub::getPose,
                                new RamseteController(KineConstants.kRamseteB, KineConstants.kRamseteZeta),
                                new SimpleMotorFeedforward(KineConstants.ksVolts, KineConstants.kvVoltSecondsPerMeter,
                                                KineConstants.kaVoltSecondSquaredPerMeter),
                                KineConstants.kDrive,
                                driveSub::getWheelSpeedsOdometry,
                                new PIDController(KineConstants.kpDriveVelocity, 0, 0),
                                new PIDController(KineConstants.kpDriveVelocity, 0, 0),
                                driveSub::tankDriveVoltsOdometry,
                                driveSub);

                if (resetOdometry) {
                        return new SequentialCommandGroup(
                                        new InstantCommand(() -> driveSub.resetOdometry(trajectory.getInitialPose())

                                        ),

                                        new InstantCommand(() -> driveSub.resetOdometry(trajectory.getInitialPose())

                                        ), new InstantCommand(() -> driveSub.resetOdometry(trajectory.getInitialPose())

                                        ).handleInterrupt(() -> System.out.println("\ngot interrupted\n")),

                                        ramseteCommand.andThen(() -> driveSub.tankDriveVolts(0, 0)));
                }
                // added new wait commmand 1 second
                // needs to be tested

                else {
                        return ramseteCommand;
                }

                // driveSub.resetOdometry(trajectory.getInitialPose());
                // return ramseteCommand.andThen(()-> driveSub.tankDriveVolts(0, 0));

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
         * // *
         * // * @return the command to run in autonomous
         * //
         */
        public Command getAutonomousCommand() {

                // // Create a voltage constraint to ensure we don't accelerate too fast
                // DifferentialDriveVoltageConstraint autoVoltageConstraint = new
                // DifferentialDriveVoltageConstraint(
                // new SimpleMotorFeedforward(
                // KineConstants.ksVolts,
                // KineConstants.kvVoltSecondsPerMeter,
                // KineConstants.kaVoltSecondSquaredPerMeter),
                // KineConstants.kDrive,
                // 10);

                // TrajectoryConfig config = new TrajectoryConfig(
                // KineConstants.kMaxSpeedMetersPerSecond,
                // KineConstants.kMaxAccelerationMetersPerSecSquared)
                // // Add kinematics to ensure max speed is actually obeyed
                // .setKinematics(KineConstants.kDrive)
                // // Apply the voltage constraint
                // .addConstraint(autoVoltageConstraint);

                // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                // // Start at the origin facing the +X direction
                // new Pose2d(0, 0, new Rotation2d(0)),
                // // Pass through these two interior waypoints, making an 's' curve path
                // List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                // // End 3 meters straight ahead of where we started, facing forward
                // new Pose2d(3, 0, new Rotation2d(0)),
                // // Pass config
                // config);

                // RamseteCommand ramseteCommand = new RamseteCommand(exampleTrajectory,
                // driveSub::getPose,
                // new RamseteController(KineConstants.kRamseteB, KineConstants.kRamseteZeta),
                // new SimpleMotorFeedforward(KineConstants.ksVolts,
                // KineConstants.kvVoltSecondsPerMeter,
                // KineConstants.kaVoltSecondSquaredPerMeter),
                // KineConstants.kDrive,
                // driveSub::getWheelSpeeds,
                // new PIDController(KineConstants.kpDriveVelocity, 0, 0),
                // new PIDController(KineConstants.kpDriveVelocity, 0, 0),
                // driveSub::tankDriveVolts,
                // driveSub);

                // driveSub.resetOdometry(exampleTrajectory.getInitialPose());

                // return ramseteCommand.andThen(() -> driveSub.tankDriveVolts(0, 0));

                // return loadPathPlannerTrajectoryToRamseteCommand(andresPath, true);
                // return loadPathPlannerTrajectoryToRamseteCommand(OfficialTestingPath, true);

                ParallelCommandGroup FirstStageAuton = new ParallelCommandGroup(
                                new driveAutonPIDCommand(3, driveSub).withTimeout(4.4),
                                new armsPIDCommand(120, armsSub).withTimeout(2.5).beforeStarting(new WaitCommand(.2))
                                                .andThen(new clawCommand(.4, clawSub).withTimeout(.3)));

                SequentialCommandGroup auton1 = new SequentialCommandGroup(FirstStageAuton,
                                new driveAutonPIDCommand(-2, driveSub).withTimeout(3));

                SequentialCommandGroup driveInitial = new SequentialCommandGroup(
                                new driveAutonPIDCommand(.85, driveSub).withTimeout(4));

                SequentialCommandGroup armRaise = new SequentialCommandGroup(
                                new armsPIDCommand(150, armsSub).withTimeout(2));

                SequentialCommandGroup cascadeRaise = new SequentialCommandGroup(
                                new cascadePIDCommand(1.15, cascadeSub).withTimeout(1.75));

                SequentialCommandGroup clawOpen = new SequentialCommandGroup(
                                new clawCommand(.3, clawSub).withTimeout(.4));

                SequentialCommandGroup driveBackSlow = new SequentialCommandGroup(
                                new driveAutonPIDCommand(-.7, driveSub));

                SequentialCommandGroup driveBack = new SequentialCommandGroup(
                                new driveAutonPIDCommand(-.6, driveSub));

                SequentialCommandGroup armLower = new SequentialCommandGroup(
                                new armsPIDCommand(120, armsSub));

                SequentialCommandGroup cascadeLower = new SequentialCommandGroup(
                                new cascadePIDCommand(0, cascadeSub));

                ParallelCommandGroup davinciTest = new ParallelCommandGroup(
                                driveInitial.beforeStarting(new WaitCommand(2.5)),
                                armRaise,
                                cascadeRaise.beforeStarting(new WaitCommand(2.75)),
                                clawOpen.beforeStarting(new WaitCommand(3.5)));

                SequentialCommandGroup firstMovementAuton = new SequentialCommandGroup(
                                davinciTest,
                                driveBackSlow.withTimeout(2.25),
                                cascadeLower.withTimeout(2),
                                armLower.withTimeout(2.5),
                                driveBack);

                SequentialCommandGroup auto = new SequentialCommandGroup(

                                // new armsPIDCommand(150, armsSub).withTimeout(3),

                                new driveAutonPIDCommand(.85, driveSub).withTimeout(10),

                                new cascadePIDCommand(1.25, cascadeSub).withTimeout(1),

                                new WaitCommand(2),

                                new clawCommand(.3, clawSub).withTimeout(.4));

                new driveAutonPIDCommand(-1, driveSub).beforeStarting(new WaitCommand(2));

                return firstMovementAuton;

        }
}