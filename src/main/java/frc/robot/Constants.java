package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {

  public final class NathanControllerConstants {

    public static final int

    nathan = 0, // joystick port

        leftDriveAxis = 1, rightDriveAxis = 5, // drive axis for tank drive

        forwardAxis = 1, turnAxis = 0, // drive Axis for arcade drive

        ///////////// Cascade Buttons /////////////

        cascadeUpButton = 7, cascadeDownButton = 8, // manual cascade buttons

        cascadeZeroButton = 13, // cascade zero button

        cascadeConeTopButton = 0, cascadeConeMidButton = 0, // cascade buttons for cones

        cascadeCubeTopButton = 0, cascadeCubeMidButton = 0, // cascade buttons for cubes

        ///////////// Arm Buttons /////////////

        armUpButton = 8, armDownButton = 6, // manual arm buttons

        ///////////// Claw Buttons /////////////

        clawOpenButton = 8, clawCloseButton = 7,

        gyroBalanceButton = 14; // charge station balance button

  }

  public final class GioControllerConstants {

    public static final int

    gio = 1, // joystick port

        leftDriveAxis = 1, rightDriveAxis = 5, // drive axis for tank drive

        forwardAxis = 1, turnAxis = 0, // drive Axis for arcade drive

        ///////////// Cascade Buttons /////////////

        cascadeUpButton = 7, cascadeDownButton = 5,

        ///////////// Arm Buttons /////////////

        armUpButton = 8, armDownButton = 6,

        clawOpenButton = 7, clawCloseButton = 5;

  }

  public static final class DriveConstants {

    public static final int

    leftFrontID = 1, leftBackID = 2,

        rightFrontID = 3, rightBackID = 4,

        gyroID = 9;

    public static final double

    KP = .05, KI = 0, KD = 0;

  }

  public static final class CascadeConstants {

    public static final int

    cascadeID = 5,

        bumpSwitch1 = 0, bumpSwitch2 = 1;

    public static final double

    KP = 3.3, KI = .25, KD = .23,

        kSVolts = 1,

        kGVolts = 1,

        kVVoltSecondPerRad = 0.5,

        kAVoltSecondSquaredPerRad = 0.1,

        kMaxVelocityRadPerSecond = 3,

        kMaxAccelerationRadPerSecSquared = 10,

        kCountsPerRev = 2048, kCascadeGearRatio = 16, kCascadeScaleFactor = (12 / (1.432 * Math.PI));

  }

  public static final class ArmConstants {

    public static final int

    leftArmID = 6, rightArmID = 7;

  }

  public static final class ClawConstants {

    public static final int

    clawID = 8;

  }

  public static final class GyroConstants {

    public static final double

    gyroAngle = 0,

        gyroKP = .1, gyroKI = .04;
  }

  public static final class SpeedConstants {

    public static final double

    driveSpeed = .7, cascadeSpeed = .45, armSpeed = .45, clawSpeed = .35;

  }

  public static final class LimelightConstants {

    public static final double

    // Constants such as camera and target height stored. Change per robot and goal!
    cameraHeightMeters = Units.inchesToMeters(0),

        targetHeightMeters = Units.inchesToMeters(0),

        // Angle between horizontal and the camera.
        cameraPitchRadians = Units.degreesToRadians(0),

        // How far from the target we want to be
        targetRangeMeters = Units.feetToMeters(2),

        KP = 0, KI = 0, KD = 0

    ;

  }

}
