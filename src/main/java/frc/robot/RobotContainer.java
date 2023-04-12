package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmStates;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.RobotStates;
import frc.robot.Constants.WristConstants;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    private SendableChooser<Command> autoChooser;

    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton turnToForward = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton turnToBackward = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton alignToTarget = new JoystickButton(driver, XboxController.Button.kX.value);
  
    private final Trigger slowMode = new Trigger(
        () -> (Math.abs(driver.getRawAxis(XboxController.Axis.kLeftTrigger.value)) > 0.2));

    private final JoystickButton intakeToggle = new JoystickButton(driver, XboxController.Button.kRightBumper.value);

    private final JoystickButton intakeVomit = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    /* Operator Buttons */
    private final JoystickButton toggleArmPiston = new JoystickButton(operator,
            XboxController.Button.kLeftBumper.value);

    private final JoystickButton armToHigh = new JoystickButton(operator, XboxController.Button.kY.value);

    private final JoystickButton armToMid = new JoystickButton(operator, XboxController.Button.kA.value);

    private final JoystickButton armToCube = new JoystickButton(operator, XboxController.Button.kX.value);

    private final JoystickButton armToCone = new JoystickButton(operator, XboxController.Button.kB.value);

    private final Trigger armHome = new Trigger(() -> (operator.getPOV() > 90 && operator.getPOV() < 270));

    // left trigger
    private final Trigger armManual = new Trigger(
            () -> (Math.abs(operator.getRawAxis(XboxController.Axis.kLeftTrigger.value)) > 0.2));
    // right trigger
    private final Trigger wristManual = new Trigger(
            () -> (Math.abs(operator.getRawAxis(XboxController.Axis.kRightTrigger.value)) > 0.2));

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();

    private final Arm arm = new Arm();

    private final Wrist wrist = new Wrist();

    private final Intake intake = new Intake();

    private final Vision vision = new Vision();

    public static RobotStates RobotState = RobotStates.IDLE;
    public static ArmStates ArmState = ArmStates.DEFAULT;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        // TODO: Remove no longer relevant autonomous commands
        SmartDashboard.putNumber("Cone adjust", 0);
        autoChooser = new SendableChooser<>();
        autoChooser.setDefaultOption("Red C_C1 Charge", new ChargeAndMobility(s_Swerve, arm, intake));
        autoChooser.addOption("Blue C_C1 Charge", new FlippedChargeAndMobility(s_Swerve, arm, intake));
        //autoChooser.addOption("Red I_C1 Two Piece", new TwoPieceAuto(s_Swerve, arm, intake));
        //autoChooser.addOption("Blue I_C2 Two Piece", new FlippedTwoPieceAuto(s_Swerve, arm, intake));
        //autoChooser.addOption("Mobility", new Mobility(s_Swerve, arm, intake));
        //autoChooser.addOption("L Cube Mobility", new LeftCubeMobility(s_Swerve, arm, intake));
        //autoChooser.addOption("R Cube Mobility", new RightCubeMobility(s_Swerve, arm, intake));
        autoChooser.addOption("Cube Charge", new CubeCharge(s_Swerve, arm, intake));
        autoChooser.addOption("Red I_CU Three Piece", new ThreePieceAuto(s_Swerve, arm, intake, wrist));
        //autoChooser.addOption("Blue I_CU Two Piece", new FlippedTwoPieceAuto(s_Swerve, arm, intake));
        //autoChooser.addOption("Auto Test", new DriveAUto(s_Swerve));

        SmartDashboard.putData("autoChooser", autoChooser);
        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> driver.getRawAxis(translationAxis),
                        () -> driver.getRawAxis(strafeAxis),
                        () -> driver.getRawAxis(rotationAxis),
                        () -> turnToForward.getAsBoolean(),
                        () -> turnToBackward.getAsBoolean(),
                        () -> slowMode.getAsBoolean()));

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        alignToTarget.whileTrue(new ScoreAlignSwerve(s_Swerve, vision));

        intakeVomit.onTrue(new TimedVomit(intake).andThen(new InstantCommand(() -> arm.retractPiston(), arm)));
        intakeToggle.onFalse(new IntakeToggle(intake, intakeToggle));
        // aimToTarget.whileTrue(new AimToTarget(s_Swerve, vision));

        /* Operator Buttons */

        toggleArmPiston.onTrue(new InstantCommand(() -> arm.togglePiston(), arm))
        .onTrue(new SetArmState(arm, wrist, ArmStates.CHECK));

        armToHigh.onTrue(new SetArmState(arm, wrist, ArmStates.HIGH));

        armToMid.onTrue(new SetArmState(arm, wrist, ArmStates.MID));
                
        armToCube.onTrue(new SetArmState( arm, wrist, ArmStates.CUBE))
        .onTrue(new InstantCommand(() -> RobotState = RobotStates.INTAKE))
        .onTrue(new InstantCommand(() -> intake.setMotorSpeed(IntakeConstants.intakeSpeed)));

        armToCone.onTrue(new SetArmState(arm, wrist, ArmStates.CONE))
        .onTrue(new InstantCommand(() -> RobotState = RobotStates.INTAKE))
        .onTrue(new InstantCommand(() -> intake.setMotorSpeed(IntakeConstants.intakeSpeed)));

        armHome.onTrue(new InstantCommand(() -> arm.retractPiston()))
                .onTrue(new SetArmState(arm, wrist, ArmStates.HOME));

        armManual.onTrue(new InstantCommand(() -> ArmState = ArmStates.DEFAULT))
                .onTrue(new ArmManual(arm, () -> (operator.getRawAxis(XboxController.Axis.kLeftY.value))));

        wristManual.whileTrue(new WristManual(wrist, () -> (operator.getRawAxis(XboxController.Axis.kRightY.value))));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        // return new exampleAuto(s_Swerve);
        // return new AutoBalance(s_Swerve);
        // return new TwoPieceAuto(s_Swerve, arm, intake);
        // return new ChargeAndMobility(s_Swerve,arm,intake);
        return autoChooser.getSelected();
    }
}
