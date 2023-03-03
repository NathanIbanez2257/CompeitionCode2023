// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import java.nio.file.Path;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.RamseteController;
// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.TrajectoryGenerator;
// import edu.wpi.first.math.trajectory.TrajectoryUtil;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.Filesystem;
// import edu.wpi.first.wpilibj2.command.RamseteCommand;
// import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.SebasControllerConstants;
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
import frc.robot.commands.drivePIDHardCommand;
import frc.robot.commands.turnPIDCommand;
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

        private static final cascadeCommand cascadeUpCommand = new cascadeCommand(cascadeSub,
                        SpeedConstants.cascadeSpeed);
        private static final cascadeCommand cascadeDownCommand = new cascadeCommand(cascadeSub,
                        -SpeedConstants.cascadeSpeed);

        private static final cascadePIDCommand cascadePIDMid = new cascadePIDCommand(1.32, cascadeSub);
        private static final cascadePIDCommand cascadePIDZero = new cascadePIDCommand(0, cascadeSub);

        private static final aimRobotCommand limelightAimCommand = new aimRobotCommand(driveSub, 15);

        private static final clawCommand clawOpenCommand = new clawCommand(SpeedConstants.clawSpeed, clawSub);
        private static final clawCommand clawCloseCommand = new clawCommand(-SpeedConstants.clawSpeed, clawSub);

        private static final clawPIDCommand clawOpenPIDCommand = new clawPIDCommand(80, clawSub);

        private static final turnPIDCommand RedTurnCommand = new turnPIDCommand(-90, driveSub);

        private static final chargeCommand chargeBalanceCommand = new chargeCommand(driveSub);

        private static final Joystick nathan = new Joystick(NathanControllerConstants.nathan);
        private static final Joystick sebas = new Joystick(SebasControllerConstants.sebas);

        /*
         * String straightPath = "straight.wpilib.json";
         * String path1 = "NEWPATH.wpilib.json";
         * String AutonPath1 = "AutonPath1.wpilib.json";
         * String AutonPath2 = "AutonPath2.wpilib.json";
         * String OfficialTestingPath = "OfficialTestingAuton.wpilib.json";
         * String andresPath = "testwitandres.wpilib.json";
         */

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

                chooser.addOption("Middle Auto", MiddleAuto);
                chooser.addOption("Red Side Auto", RedSideAuto);
                chooser.addOption("No Auton", null);

                SmartDashboard.putData(chooser);

                /*
                 * chooser.addOption("Andres Testing",
                 * loadPathPlannerTrajectoryToRamseteCommand(
                 * andresPath, true));
                 * 
                 * chooser.addOption("Official Testing",
                 * loadPathPlannerTrajectoryToRamseteCommand(
                 * OfficialTestingPath, false));
                 * 
                 * chooser.addOption("straightPath",
                 * loadPathPlannerTrajectoryToRamseteCommand(
                 * straightPath, true));
                 * 
                 * chooser.addOption("AutonPath1",
                 * loadPathPlannerTrajectoryToRamseteCommand(
                 * AutonPath1, true));
                 * 
                 * chooser.addOption("AutonPath2",
                 * loadPathPlannerTrajectoryToRamseteCommand(
                 * AutonPath2, true));
                 * 
                 * chooser.addOption("Path1",
                 * loadPathPlannerTrajectoryToRamseteCommand(
                 * path1, true));
                 */

                ///////////////// Gio Controls ///////////////////////

                JoystickButton armUp = new JoystickButton(sebas, SebasControllerConstants.armUpButton);
                armUp.whileTrue(armUpCommand);

                JoystickButton armDown = new JoystickButton(sebas, SebasControllerConstants.armDownButton);
                armDown.whileTrue(armDownCommand);

                JoystickButton clawOpen = new JoystickButton(sebas, SebasControllerConstants.clawOpenButton);
                clawOpen.whileTrue(clawOpenCommand);

                JoystickButton clawClose = new JoystickButton(sebas, SebasControllerConstants.clawCloseButton);
                clawClose.whileTrue(clawCloseCommand);

                JoystickButton armsHigh = new JoystickButton(sebas, 4);
                armsHigh.onTrue(armsHighCommand);

                JoystickButton armsZero = new JoystickButton(sebas, 2);
                armsZero.onTrue(armsZeroCommand);

                JoystickButton clawOpenPID = new JoystickButton(sebas, 1);
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
                limeTrack.whileTrue(limelightAimCommand);

                // turning pid testing 
                
                // JoystickButton RedTurnPID = new JoystickButton(nathan, 15);
                // RedTurnPID.onTrue(RedTurnCommand.until(cascadePIDZero)); 

                configureBindings();

        }

        //////////////// Autonomous Commands ////////////

        //

        ///////////////////////////// Middle Autonomous /////////////////////////////

        SequentialCommandGroup driveInitial = new SequentialCommandGroup(
                        new driveAutonPIDCommand(.85, driveSub).withTimeout(2.25));

        SequentialCommandGroup armRaise = new SequentialCommandGroup(
                        new armsPIDCommand(125, armsSub).withTimeout(1.75));

        SequentialCommandGroup cascadeRaise = new SequentialCommandGroup(
                        new cascadePIDCommand(1.15, cascadeSub).withTimeout(1.75));

        SequentialCommandGroup clawOpenAuto = new SequentialCommandGroup(
                        new clawCommand(.3, clawSub).withTimeout(.4));

        SequentialCommandGroup clawCloseAuto = new SequentialCommandGroup(
                        new clawCommand(-.3, clawSub).withTimeout(.4));

        ParallelCommandGroup driveBackSlow = new ParallelCommandGroup(
                        new driveAutonPIDCommand(-.8, driveSub).withTimeout(1.25), 
                        new armsPIDCommand(130, armsSub).withTimeout(.75));

        SequentialCommandGroup driveBack = new SequentialCommandGroup(
                        new driveAutonPIDCommand(-.76, driveSub).withTimeout(1.5));

        ParallelCommandGroup driveBack2ndStage = new ParallelCommandGroup(
                        new drivePIDHardCommand(-.9, driveSub).withTimeout(2));
        // new armsPIDCommand(66, armsSub).withTimeout(.75)

        ParallelCommandGroup driveBack3ndStage = new ParallelCommandGroup(
                        new driveAutonPIDCommand(-.5, driveSub).withTimeout(1.5));

        SequentialCommandGroup armPostRaise = new SequentialCommandGroup(
                        new armsPIDCommand(140, armsSub).withTimeout(1.25));

        SequentialCommandGroup armPostLower = new SequentialCommandGroup(
                        new armsPIDCommand(-25, armsSub).withTimeout(1.25));

        SequentialCommandGroup cascadeLower = new SequentialCommandGroup(
                        new cascadePIDCommand(0, cascadeSub).withTimeout(1.25));

        ParallelCommandGroup ArmCascadeClawDown = new ParallelCommandGroup(
                        cascadeLower,
                        armPostLower.beforeStarting(new WaitCommand(.25)),
                        clawCloseAuto);

        // 66

        ParallelCommandGroup CubeScore = new ParallelCommandGroup(
                        driveInitial.beforeStarting(new WaitCommand(2.5)),
                        armRaise,
                        cascadeRaise.beforeStarting(new WaitCommand(2.25)),
                        clawOpenAuto.beforeStarting(new WaitCommand(3.5)));

        SequentialCommandGroup MiddleAuto = new SequentialCommandGroup(
                        CubeScore.andThen(() -> System.out.println("Initial Auton Done")),
                        driveBackSlow,
                        ArmCascadeClawDown,
                        driveBack,
                        driveBack2ndStage,
                        driveBack3ndStage,

                        chargeBalanceCommand.withTimeout(2)
                                        .andThen(() -> System.out.println("Charge Command Has Finished")));

        //

        ///////////////////////////// Red Side Autonomous /////////////////////////////

        SequentialCommandGroup RedDriveInitial = new SequentialCommandGroup(
                        new driveAutonPIDCommand(.85, driveSub).withTimeout(2.25));

        SequentialCommandGroup RedArmRaise = new SequentialCommandGroup(
                        new armsPIDCommand(125, armsSub).withTimeout(1.75));

        SequentialCommandGroup RedCascadeRaise = new SequentialCommandGroup(
                        new cascadePIDCommand(1.15, cascadeSub).withTimeout(1.75));

        SequentialCommandGroup RedClawOpenAuto = new SequentialCommandGroup(
                        new clawCommand(.3, clawSub).withTimeout(.4));

        ParallelCommandGroup RedCubeScore = new ParallelCommandGroup(
                        RedDriveInitial.beforeStarting(new WaitCommand(2.5)),
                        RedArmRaise,
                        RedCascadeRaise.beforeStarting(new WaitCommand(2.25)),
                        RedClawOpenAuto.beforeStarting(new WaitCommand(3.5)));

        //

        SequentialCommandGroup RedClawCloseAuto = new SequentialCommandGroup(
                        new clawCommand(-.3, clawSub).withTimeout(.4));

        SequentialCommandGroup RedDriveBackSlow = new SequentialCommandGroup(
                        new driveAutonPIDCommand(-.8, driveSub).withTimeout(1.25));

        SequentialCommandGroup RedDriveBack = new SequentialCommandGroup(
                        new driveAutonPIDCommand(-.7, driveSub).withTimeout(1.5));

        ParallelCommandGroup RedDriveBack2ndStage = new ParallelCommandGroup(
                        new driveAutonPIDCommand(-.5, driveSub).withTimeout(2));
        // new armsPIDCommand(66, armsSub).withTimeout(.75)

        ParallelCommandGroup RedDriveBack3ndStage = new ParallelCommandGroup(
                        new driveAutonPIDCommand(-1.2, driveSub).withTimeout(1.5));

        SequentialCommandGroup RedArmPostRaise = new SequentialCommandGroup(
                        new armsPIDCommand(140, armsSub).withTimeout(1.25));

        SequentialCommandGroup RedArmPostLower = new SequentialCommandGroup(
                        new armsPIDCommand(-25, armsSub).withTimeout(1.25));

        SequentialCommandGroup RedCascadeLower = new SequentialCommandGroup(
                        new cascadePIDCommand(0, cascadeSub).withTimeout(1.25));

        ParallelCommandGroup RedArmCascadeClawDown = new ParallelCommandGroup(
                        RedCascadeLower,
                        RedArmPostLower.beforeStarting(new WaitCommand(.25)),
                        RedClawCloseAuto);

        //
        SequentialCommandGroup RedSideAuto = new SequentialCommandGroup(
                        RedCubeScore.andThen(() -> System.out.println("Charge Command Has Finished")),
                        RedDriveBackSlow,
                        RedArmCascadeClawDown,
                        RedDriveBack,
                        new WaitCommand(.5),

                        // have to adjust these values at competiton 
                        RedDriveBack2ndStage,
                        RedDriveBack3ndStage);

        /*
         * ParallelCommandGroup AutonomousPractice = new ParallelCommandGroup(
         * loadPathPlannerTrajectoryToRamseteCommand(OfficialTestingPath, true));
         * 
         * public Command loadPathPlannerTrajectoryToRamseteCommand(String filename,
         * boolean resetOdometry) {
         * Trajectory trajectory;
         * 
         * try {
         * Path trajectoryPath =
         * Filesystem.getDeployDirectory().toPath().resolve(filename);
         * trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
         * System.out.println("winning");
         * 
         * }
         * 
         * catch (Exception exception) {
         * DriverStation.reportError("Unable To Open Trajectory " + filename,
         * exception.getStackTrace());
         * System.out.println("Unable to read from file" + filename);
         * return new InstantCommand();
         * }
         * 
         * RamseteCommand ramseteCommand = new RamseteCommand(
         * 
         * trajectory, driveSub::getPose,
         * new RamseteController(KineConstants.kRamseteB, KineConstants.kRamseteZeta),
         * new SimpleMotorFeedforward(KineConstants.ksVolts,
         * KineConstants.kvVoltSecondsPerMeter,
         * KineConstants.kaVoltSecondSquaredPerMeter),
         * KineConstants.kDrive,
         * driveSub::getWheelSpeedsOdometry,
         * new PIDController(KineConstants.kpDriveVelocity, 0, 0),
         * new PIDController(KineConstants.kpDriveVelocity, 0, 0),
         * driveSub::tankDriveVoltsOdometry,
         * driveSub);
         * 
         * if (resetOdometry) {
         * return new SequentialCommandGroup(
         * new InstantCommand(() -> driveSub.resetOdometry(trajectory.getInitialPose())
         * 
         * ),
         * 
         * new InstantCommand(() -> driveSub.resetOdometry(trajectory.getInitialPose())
         * 
         * ), new InstantCommand(() ->
         * driveSub.resetOdometry(trajectory.getInitialPose())
         * 
         * ).handleInterrupt(() -> System.out.println("\ngot interrupted\n")),
         * 
         * ramseteCommand.andThen(() -> driveSub.tankDriveVolts(0, 0)));
         * }
         * // added new wait commmand 1 second
         * // needs to be tested
         * 
         * else {
         * return ramseteCommand;
         * }
         * 
         * 
         * // driveSub.resetOdometry(trajectory.getInitialPose());
         * // return ramseteCommand.andThen(()-> driveSub.tankDriveVolts(0, 0));
         * 
         * }
         */

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

                /*
                 * // Create a voltage constraint to ensure we don't accelerate too fast
                 * DifferentialDriveVoltageConstraint autoVoltageConstraint = new
                 * DifferentialDriveVoltageConstraint(
                 * new SimpleMotorFeedforward(
                 * KineConstants.ksVolts,
                 * KineConstants.kvVoltSecondsPerMeter,
                 * KineConstants.kaVoltSecondSquaredPerMeter),
                 * KineConstants.kDrive,
                 * 10);
                 * 
                 * TrajectoryConfig config = new TrajectoryConfig(
                 * KineConstants.kMaxSpeedMetersPerSecond,
                 * KineConstants.kMaxAccelerationMetersPerSecSquared)
                 * // Add kinematics to ensure max speed is actually obeyed
                 * .setKinematics(KineConstants.kDrive)
                 * // Apply the voltage constraint
                 * .addConstraint(autoVoltageConstraint);
                 * 
                 * Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                 * // Start at the origin facing the +X direction
                 * new Pose2d(0, 0, new Rotation2d(0)),
                 * // Pass through these two interior waypoints, making an 's' curve path
                 * List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                 * // End 3 meters straight ahead of where we started, facing forward
                 * new Pose2d(3, 0, new Rotation2d(0)),
                 * // Pass config
                 * config);
                 * 
                 * RamseteCommand ramseteCommand = new RamseteCommand(exampleTrajectory,
                 * driveSub::getPose,
                 * new RamseteController(KineConstants.kRamseteB, KineConstants.kRamseteZeta),
                 * new SimpleMotorFeedforward(KineConstants.ksVolts,
                 * KineConstants.kvVoltSecondsPerMeter,
                 * KineConstants.kaVoltSecondSquaredPerMeter),
                 * KineConstants.kDrive,
                 * driveSub::getWheelSpeeds,
                 * new PIDController(KineConstants.kpDriveVelocity, 0, 0),
                 * new PIDController(KineConstants.kpDriveVelocity, 0, 0),
                 * driveSub::tankDriveVolts,
                 * driveSub);
                 * 
                 * driveSub.resetOdometry(exampleTrajectory.getInitialPose());
                 * 
                 * return ramseteCommand.andThen(() -> driveSub.tankDriveVolts(0, 0));
                 * 
                 * return loadPathPlannerTrajectoryToRamseteCommand(andresPath, true);
                 * return loadPathPlannerTrajectoryToRamseteCommand(OfficialTestingPath, true);
                 */

                // return firstMovementAuton;
               return chooser.getSelected();
        //        return RedTurnCommand;

        }
}