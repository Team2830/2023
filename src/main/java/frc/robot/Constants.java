package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.commands.DriveArmToPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public final class Constants {
        public static final double stickDeadband = 0.1;

        public static final class ArmConstants {
                public static final double encoderZeroValue = 59.5;
                public static final int climberRMotorID = 14;
                public static final int climberLMotorID = 13;
                public static final int armForwardID = 7;
                public static final int armReverseID = 6;
                public static final double startAngle = -58.76;

                public static final double angleError = 1;

                public static final double substationCubeAngle = 10.5;
                public static final double substationConeAngle = 14.5; // 3:12pm 9.0
                public static final double highAngle = 153.5; // 3:12pm 148
                public static final double cubeHigh = 165.0;

                public static final double midAngle = 165;
                public static final double homeAngle = -47;
        }

        public static final class ArmMotorPID {
                public static final int kSlotIdx = 0;
                public static final int kPIDLoopIdx = 0;
                public static final int kTimeoutMs = 30;
                public static boolean kSensorPhase = true;

                // 100283.7 encoder per 58.76 degrees
                public static final double kP = .05; //.04
                public static final double kI = 0;
                public static final double kD = 0;
                public static final double kG = (.8 / 12.0);
                public static double allowableError = 10;

                public static final int kIzone = 0;
                public final double kPeakOutput = 1.0;

        }

        public static final class WristConstants {
                public static final int wristID = 15;

                public static final double startAngle = 0;

                public static final double angleError = 1;

                public static final double substationCubeAngle = -17.0;
                public static final double substationAngle = -14.0; // 3:12pm 9.0
                public static final double highAngle = 75.0; // 3:12pm 148
                public static final double cubeHigh = 80.0;
                public static final double groundAngle = 33.0;
                public static final double shootAngle = 100.0;

                public static final double midAngle = 89.0;

                public static final double homeAngle = 47;
                public static final double coneAngle = 39;

                public static final double flipAngle = 70.0;
        }

        public static final class WristMotorPID {
                public static final double kP = .035;
                public static final double kI = 0;
                public static final double kD = 0;
                public static final double kG = (0 / 12);// (.15 / 12);
                public static double allowableError = 10;
                public final double kPeakOutput = 1.0;

        }

        public static final class IntakeConstants {
                public static final int intakeID = 16;

                public static final double defaultSpeed = 0;
                public static final double intakeSpeed = 1;
                public static final double holdSpeed = .1;
                public static final double vomitSpeed = -.65;
                public static final double vomitSeconds = .5;

                public static final double intakeCurrent = 220;
        }

        public static enum RobotStates {
                IDLE,
                INTAKE,
                HOLDING
        }

        public static enum ArmStates {
                DEFAULT,
                HOME,
                CUBE,
                CONE,
                MID,
                HIGH,
                CHECK
        }

        public static final class AutoConstants { // TODO: The below constants are used in the example auto, and must be
                // tuned to specific robot
                public static final double kMaxSpeedMetersPerSecond = 3;
                public static final double kMaxAccelerationMetersPerSecondSquared = 3;
                public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
                public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

                public static final double kPXController = 1;
                public static final double kPYController = 1;
                public static final double kPThetaController = -.6;

                /* Constraint for the motion profilied robot angle controller */
                public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
        }

        /*
         * TRANSLATION VALUES FOR THE RED SIDE OF THE FIELD
         * +Y IS RIGHT, -X IS FORWARD
         * ORIGIN IS AT OUTSIDE_CONE1 - The furthest left cone node
         */

        public static final class Translations {

                /*
                 * MISC
                 */

                public static final Translation2d INSIDE_COMMUNITY_EXIT = new Translation2d(-Units.inchesToMeters(41),
                                Units.inchesToMeters(163));
                public static final Translation2d INSIDE_CHARGE_CORNER = new Translation2d(-Units.inchesToMeters(136),
                                Units.inchesToMeters(163));

                public static final Translation2d CUBE_MOBILITY = new Translation2d(-Units.inchesToMeters(170),
                                Units.inchesToMeters(170));

                public static final Translation2d MOBILITY = new Translation2d(-Units.inchesToMeters(180),
                                Units.inchesToMeters(0));

                /*
                 * CHARGE STATION
                 */
                public static final Translation2d CHARGE_PAST_LEFT = new Translation2d(-Units.inchesToMeters(180),
                                Units.inchesToMeters(66));
                public static final Translation2d CHARGE_PAST_CENTER = new Translation2d(-Units.inchesToMeters(195),
                                Units.inchesToMeters(88));

                public static final Translation2d CHARGE_CENTER = new Translation2d(-Units.inchesToMeters(90),
                                Units.inchesToMeters(88));
                
                public static final Translation2d CHARGE_NEAR_CENTER = new Translation2d(-Units.inchesToMeters(120),
                                Units.inchesToMeters(88));

                public static final Translation2d CHARGE_LEFT = new Translation2d(-Units.inchesToMeters(155),
                                Units.inchesToMeters(66));

                /*
                 * SCORING NODES
                 * Grids are LEFT, CENTER, RIGHT, from driverstations facing the field
                 * Grids are CONE1, CUBE, CONE2 from left to right
                 */

                // FURTHEST LEFT CONE NODE.
                public static final Translation2d OUTSIDE_CONE1 = new Translation2d(0, Units.inchesToMeters(0));

                public static final Translation2d OUTSIDE_CUBE = new Translation2d(0, Units.inchesToMeters(22));

                public static final Translation2d OUTSIDE_CONE2 = new Translation2d(0, Units.inchesToMeters(44));

                public static final Translation2d CENTER_CONE1 = new Translation2d(0, Units.inchesToMeters(66));

                public static final Translation2d CENTER_CUBE = new Translation2d(0, Units.inchesToMeters(88));

                public static final Translation2d CENTER_CONE2 = new Translation2d(0, Units.inchesToMeters(110));

                public static final Translation2d INSIDE_CONE1 = new Translation2d(0, Units.inchesToMeters(132));

                public static final Translation2d INSIDE_CUBE = new Translation2d(0, Units.inchesToMeters(154));

                public static final Translation2d INSIDE_CONE2 = new Translation2d(0, Units.inchesToMeters(176));

                /*
                 * FLOOR PRESETS
                 * REFLECTS ROBOT POSITION, NOT ACTUAL GAMEPIECE POSITION
                 */

                public static final Translation2d FLOOR1 = new Translation2d(-Units.inchesToMeters(182),
                                Units.inchesToMeters(16.25));
                public static final Translation2d FLOOR2 = new Translation2d(-Units.inchesToMeters(182),
                                Units.inchesToMeters(64.25));
                public static final Translation2d FLOOR3 = new Translation2d(-Units.inchesToMeters(180),
                                Units.inchesToMeters(126));
                public static final double FLOOR3_ANGLE = 35;
                public static final Translation2d FLOOR4 = new Translation2d(-Units.inchesToMeters(182),
                                Units.inchesToMeters(160)); // 160.25

                /**
                 * OLD SUCCESSFUL 2 PIECE VALUES
                 */
                public static final Translation2d OLD_INSIDE_CUBE = new Translation2d(0, Units.inchesToMeters(164));

                public static final Translation2d OLD_COMMUNITY_EXIT = new Translation2d(-Units.inchesToMeters(65),
                                Units.inchesToMeters(170));

                public static final Translation2d OLD_FLOOR4 = new Translation2d(-Units.inchesToMeters(193),
                                Units.inchesToMeters(169));
        }

        /*
         * TRANSLATION VALUES FOR THE BLUE SIDE OF THE FIELD
         */

        public static final class FlippedTranslations {

                /*
                 * MISC
                 */
                public static final Translation2d INSIDE_COMMUNITY_EXIT = new Translation2d(-Units.inchesToMeters(41),
                                -Units.inchesToMeters(163));

                public static final Translation2d INSIDE_CHARGE_CORNER = new Translation2d(-Units.inchesToMeters(136),
                                -Units.inchesToMeters(163));

                public static final Translation2d CUBE_MOBILITY = new Translation2d(-Units.inchesToMeters(170),
                                -Units.inchesToMeters(170));

                public static final Translation2d COMMUNITY_EXIT = new Translation2d(-Units.inchesToMeters(65),
                                -Units.inchesToMeters(170));

                /*
                 * CHARGE STATION
                 */
                public static final Translation2d CHARGE_PAST_LEFT = new Translation2d(-Units.inchesToMeters(180),
                                -Units.inchesToMeters(66));
                public static final Translation2d CHARGE_PAST_CENTER = new Translation2d(-Units.inchesToMeters(180),
                                -Units.inchesToMeters(89));

                public static final Translation2d CHARGE_CENTER = new Translation2d(-Units.inchesToMeters(100),
                                -Units.inchesToMeters(89));
                public static final Translation2d CHARGE_DOCK = new Translation2d(-Units.inchesToMeters(120),
                                -Units.inchesToMeters(89));

                public static final Translation2d CHARGE_LEFT = new Translation2d(-Units.inchesToMeters(140),
                                -Units.inchesToMeters(66));

                /*
                 * SCORING NODES
                 * Grids are LEFT, CENTER, RIGHT, from driverstations facing the field
                 * Grids are CONE1, CUBE, CONE2 from left to right
                 */

                // FURTHEST LEFT CONE NODE.
                public static final Translation2d OUTSIDE_CONE2 = new Translation2d(0, -Units.inchesToMeters(0));

                public static final Translation2d OUTSIDE_CUBE = new Translation2d(0, -Units.inchesToMeters(22));

                public static final Translation2d OUTSIDE_CONE1 = new Translation2d(0, -Units.inchesToMeters(44));

                public static final Translation2d CENTER_CONE2 = new Translation2d(0, -Units.inchesToMeters(66));

                public static final Translation2d CENTER_CUBE = new Translation2d(0, -Units.inchesToMeters(88));

                public static final Translation2d CENTER_CONE1 = new Translation2d(0, -Units.inchesToMeters(110));

                public static final Translation2d INSIDE_CONE2 = new Translation2d(0, -Units.inchesToMeters(132));

                public static final Translation2d INSIDE_CUBE = new Translation2d(0, -Units.inchesToMeters(154));

                public static final Translation2d INSIDE_CONE1 = new Translation2d(0, -Units.inchesToMeters(176));

                /*
                 * FLOOR PRESETS
                 */

                public static final Translation2d FLOOR1 = new Translation2d(-Units.inchesToMeters(193), //204 inches away
                                -Units.inchesToMeters(16));
                public static final Translation2d FLOOR2 = new Translation2d(-Units.inchesToMeters(193),
                                -Units.inchesToMeters(64));
                public static final Translation2d FLOOR3 = new Translation2d(-Units.inchesToMeters(193),
                                -Units.inchesToMeters(112));
                public static final Translation2d FLOOR4 = new Translation2d(-Units.inchesToMeters(193),
                                -Units.inchesToMeters(160)); // 160.25
                public static final double FLOOR3_ANGLE = -35;

                /**
                 * OLD SUCCESSFUL 2 PIECE VALUES
                 */
                public static final Translation2d OLD_INSIDE_CUBE = new Translation2d(0, -Units.inchesToMeters(164));

                public static final Translation2d OLD_COMMUNITY_EXIT = new Translation2d(-Units.inchesToMeters(65),
                                -Units.inchesToMeters(170));

                public static final Translation2d OLD_FLOOR4 = new Translation2d(-Units.inchesToMeters(193),
                                -Units.inchesToMeters(169));
        }

        public static final class Swerve {

                /* Turn to Angle Constants */
                public static final double rotationMultiplier = .016;

                public static final double maxRotationalSpeed = .4;

                // Vision Constants
                public static final double visionAlignConstant = -0.0075;

                // ----

                public static final double slewRate = 1.7; // 2.0

                public static final double slowRate = .9;
                public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-

                public static final COTSFalconSwerveConstants chosenModule = // TODO: This must be tuned to specific
                                                                             // robot
                                COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L3);

                /* Drivetrain Constants */
                public static final double trackWidth = Units.inchesToMeters(20.75); // TODO: This must be tuned to
                                                                                     // specific
                                                                                     // robot
                public static final double wheelBase = Units.inchesToMeters(24.75); // TODO: This must be tuned to
                                                                                    // specific
                                                                                    // robot
                public static final double wheelCircumference = chosenModule.wheelCircumference;

                /*
                 * Swerve Kinematics
                 * No need to ever change this unless you are not doing a traditional
                 * rectangular/square 4 module swerve
                 */
                public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

                /* Module Gear Ratios */
                public static final double driveGearRatio = chosenModule.driveGearRatio;
                public static final double angleGearRatio = chosenModule.angleGearRatio;

                /* Motor Inverts */
                public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
                public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

                /* Angle Encoder Invert */
                public static final boolean canCoderInvert = chosenModule.canCoderInvert;

                /* Swerve Current Limiting */
                public static final int angleContinuousCurrentLimit = 25;
                public static final int anglePeakCurrentLimit = 40;
                public static final double anglePeakCurrentDuration = 0.1;
                public static final boolean angleEnableCurrentLimit = true;

                public static final int driveContinuousCurrentLimit = 35;
                public static final int drivePeakCurrentLimit = 60;
                public static final double drivePeakCurrentDuration = 0.1;
                public static final boolean driveEnableCurrentLimit = true;

                /*
                 * These values are used by the drive falcon to ramp in open loop and closed
                 * loop driving.
                 * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
                 */
                public static final double openLoopRamp = 0.25;
                public static final double closedLoopRamp = 0.0;

                /* Angle Motor PID Values */
                public static final double angleKP = chosenModule.angleKP;
                public static final double angleKI = chosenModule.angleKI;
                public static final double angleKD = chosenModule.angleKD;
                public static final double angleKF = chosenModule.angleKF;

                /* Drive Motor PID Values */
                public static final double driveKP = 0.00055097;// 0.00066572; // TODO: This must be tuned to specific
                                                                // robot
                public static final double driveKI = 0.0;
                public static final double driveKD = 0.0;
                public static final double driveKF = 0.0;

                /*
                 * Drive Motor Characterization Values
                 * Divide SYSID values by 12 to convert from volts to percent output for CTRE
                 */
                // FIXME: Tune for new drivetrain

                public static final double driveKS = (0.21441 / 12); // TODO: This must be tuned to specific robot
                public static final double driveKV = (1.9742 / 12);
                public static final double driveKA = (0.68428 / 12);

                /* Swerve Profiling Values */
                /** Meters per Second */
                public static final double maxSpeed = Units.feetToMeters(18); // FIXME: This must be tuned to specific
                                                                              // robot
                /** Radians per Second */
                public static final double maxAngularVelocity = 11.5; // FIXME: This must be tuned to specific robot

                /* Neutral Modes */
                public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
                public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

                /* Module Specific Constants */
                /* Front Left Module - Module 0 */
                public static final class Mod0 { // TODO: This must be tuned to specific robot
                        public static final int driveMotorID = 1;
                        public static final int angleMotorID = 2;
                        public static final int canCoderID = 3;
                        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(314.296875);// Rotation2d.fromDegrees(77.34+180);
                        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                                        angleMotorID,
                                        canCoderID, angleOffset);
                }

                /* Front Right Module - Module 1 */
                public static final class Mod1 { // TODO: This must be tuned to specific robot
                        public static final int driveMotorID = 4;
                        public static final int angleMotorID = 5;
                        public static final int canCoderID = 6;
                        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(157.15);// Rotation2d.fromDegrees(75.41+180);
                        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                                        angleMotorID,
                                        canCoderID, angleOffset);
                }

                /* Back Left Module - Module 2 */
                public static final class Mod2 { // TODO: This must be tuned to specific robot
                        public static final int driveMotorID = 7;
                        public static final int angleMotorID = 8;
                        public static final int canCoderID = 9;
                        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(283.79);// Rotation2d.fromDegrees(276.50-180);
                        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                                        angleMotorID,
                                        canCoderID, angleOffset);
                }

                /* Back Right Module - Module 3 */
                public static final class Mod3 { // TODO: This must be tuned to specific robot
                        public static final int driveMotorID = 10;
                        public static final int angleMotorID = 11;
                        public static final int canCoderID = 12;
                        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(161.98);// Rotation2d.fromDegrees(103.79+180);
                        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                                        angleMotorID,
                                        canCoderID, angleOffset);
                }
        }
}
