package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {

  public static final class LimelightOriginalConstants {

    public static final double shortAimKP = 0.01, shortAimKI = .025,

        longAimKP = .007, longAimKI = 0.01,

        aimKD = 0.6, targetAngle = 0.00,

        //////////////////////// PIDS for drive ////////////////////////////////

        // aimKP = 0.05, aimKI = 0, aimKD1 = 0,

        distanceKP = 0.38, distanceKI = 0.09, distanceKD = 0.07;
  }

  public final class NathanControllerConstants {

    public static final int

    nathan = 0, // joystick port

        leftDriveAxis = 1, rightDriveAxis = 5, // drive axis for tank drive

        forwardAxis = 1, turnAxis = 2, // drive Axis for arcade drive

        ///////////// Cascade Buttons /////////////

        cascadeUpButton = 7, cascadeDownButton = 8, // manual cascade buttons

        cascadeZeroButton = 2, // cascade zero button

        cascadeConeTopButton = 0, cascadeConeMidButton = 1, // cascade buttons for cones

        cascadeCubeTopButton = 0, cascadeCubeMidButton = 0, // cascade buttons for cubes

        ///////////// Arm Buttons /////////////

        armUpButton = 8, armDownButton = 6, // manual arm buttons

        ///////////// Claw Buttons /////////////

        clawOpenButton = 8, clawCloseButton = 7,

        gyroBalanceButton = 9, limeTrackButton = 14, limeDistanceButton = 13; // charge station balance button

  }

  public final class GioControllerConstants {

    public static final int

    gio = 1, // joystick port

        leftDriveAxis = 1, rightDriveAxis = 5, // drive axis for tank drive

        forwardAxis = 1, turnAxis = 0, // drive Axis for arcade drive

        ///////////// Cascade Buttons /////////////

        cascadeUpButton = 7, cascadeDownButton = 5,

        ///////////// Arm Buttons /////////////

        armUpButton = 6, armDownButton = 5,

        clawOpenButton = 7, clawCloseButton = 8;

  }

  public static final class DriveConstants {

    public static final int

    leftFrontID = 1, leftBackID = 2,

        rightFrontID = 3, rightBackID = 4,

        gyroID = 9;

    public static final double

    KP = .05, KI = 0, KD = 0;

  }

  public static final class AutonConstatns {
    public static final double

    KP = .72, KI = 0.5, KD = 0.002;

    // kp .37 kd .17

    // KP = .97, KI = 0, KD = .2;//KD = .035;
  }

  public static final class KineConstants {
    public static final double kCountsPerRev = 2048,
        kGearRatio = 6.9048,
        kWheelRadiusInches = 3,

        ksVolts = 0.21274,
        kvVoltSecondsPerMeter = 1.5901,
        kaVoltSecondSquaredPerMeter = 0.2235,
        kpDriveVelocity = 0.43958,

        kTrackWidthMeters = Units.inchesToMeters(25),

        kMaxSpeedMetersPerSecond = 2,
        kMaxAccelerationMetersPerSecSquared = 2,

        kRamseteB = 2,
        kRamseteZeta = .7;

    public static final DifferentialDriveKinematics kDrive = new DifferentialDriveKinematics(kTrackWidthMeters);

  }

  public static final class CascadeConstants {

    public static final int

    cascadeID = 5;

    public static final double

    KP = 4.8, KI = .27, KD = .11,

        ksVolts = 0.15285,

        kgVolts = 0.2239,

        kvVoltSecondPerMeters = 15.334,

        kaVoltSecondSquaredPerRad = 0.37419,

        kMaxVelocityRadPerSecond = 3,

        kMaxAccelerationRadPerSecSquared = 10,

        kCountsPerRev = 2048, kCascadeGearRatio = 16, kCascadeScaleFactor = (12 / (1.432 * Math.PI));

  }

  public static final class ArmConstants {

    public static final int

    leftArmID = 6, rightArmID = 7;

    public static final double

    kArmGearRatio = 200, kCountsPerRev = 2048, kArmScaleFactor = (360 / (410068 / (2048 * 200))),
        KP = .02, KI = .0015, KD = .001;
  }

  public static final class ClawConstants {

    public static final int

    clawID = 8;

    public static final double

    clawKP = .04, clawKI = .0007, clawKD = 0,
        kClawGearRatio = 100, kCountsPerRev = 2048, kClawScaleFactor = 600;

  }

  public static final class GyroConstants {

    public static final double

    gyroAngle = 0,

        shortGyroKP = .015, shortGyroKI = .005, shortGyroKD = .00,

        longGyroKP = .005, longGyroKI = .005, longGyroKD = .00;
  }

  public static final class SpeedConstants {

    public static final double

    driveSpeed = .85, cascadeSpeed = .55, armSpeed = .45, clawSpeed = .35;

  }

  public static final class LimelightConstants {

    public static final double

    // Constants such as camera and target height stored. Change per robot and goal!
    cameraHeightMeters = Units.inchesToMeters(0),

        targetHeightMeters = Units.inchesToMeters(0),

        // Angle between horizontal and the camera.
        cameraPitchRadians = Units.degreesToRadians(90),

        // How far from the target we want to be
        targetRangeMeters = Units.feetToMeters(2),

        KP = 0, KI = 0, KD = 0

    ;

  }

}
