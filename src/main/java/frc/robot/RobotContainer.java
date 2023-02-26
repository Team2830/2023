package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton lockStrafe = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton lockTransation = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton turnToAngle = new JoystickButton(driver, XboxController.Button.kA.value);
    /* Operator Controls */
    private final int armRotation = XboxController.Axis.kLeftY.value;

    /* Operator Buttons */
    private final JoystickButton toggleArmPiston = new JoystickButton(operator,
            XboxController.Button.kLeftBumper.value);

    private final JoystickButton toggleIntakePiston = new JoystickButton(operator,
            XboxController.Button.kRightBumper.value);

    private final JoystickButton armToHigh = new JoystickButton(operator, XboxController.Button.kY.value);

    private final JoystickButton resetArmEncoder = new JoystickButton(operator, XboxController.Button.kStart.value);

    private final JoystickButton armToMid = new JoystickButton(operator, XboxController.Button.kA.value);
   
    private final JoystickButton armToCube = new JoystickButton(operator, XboxController.Button.kX.value);

    private final JoystickButton armToCone = new JoystickButton(operator, XboxController.Button.kB.value);

    private final Trigger armHome = new Trigger(() -> (operator.getPOV() >90 && operator.getPOV()<270));

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();

    private final Arm arm = new Arm();

    private final Intake intake = new Intake();

    private final Vision vision = new Vision();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> -driver.getRawAxis(translationAxis) * 0.75,
                        () -> -driver.getRawAxis(strafeAxis) * 0.75,
                        () -> driver.getRawAxis(rotationAxis) * 0.5,
                        () -> false,
                        () -> turnToAngle.getAsBoolean(),
                        () -> lockTransation.getAsBoolean(),
                        () -> lockStrafe.getAsBoolean()));
        arm.setDefaultCommand(new InstantCommand(() -> arm.setMotorSpeed(-operator.getRawAxis(armRotation) * .5), arm));

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
    //    aimToTarget.whileTrue(new AimToTarget(s_Swerve, vision));
        

        /* Operator Buttons */

        toggleArmPiston.onTrue(new InstantCommand(() -> arm.togglePiston(), arm));
        toggleIntakePiston.onTrue(new InstantCommand(() -> intake.togglePiston(), intake));

        resetArmEncoder.onTrue(new InstantCommand(() -> arm.resetArm(), arm));

        armToHigh.whileTrue(new DriveArmToPosition(arm, Constants.Arm.highAngle));

        armToMid.whileTrue(new DriveArmToPosition(arm, Constants.Arm.midAngle));

        armToCube.whileTrue(new DriveArmToPosition(arm, Constants.Arm.substationCubeAngle));

        armToCone.whileTrue(new DriveArmToPosition(arm, Constants.Arm.substationConeAngle));

        armHome.onTrue(new InstantCommand(() -> arm.retractPiston(),arm));

        armHome.whileTrue(new DriveArmToPosition(arm, Constants.Arm.homeAngle));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        // return new exampleAuto(s_Swerve);
        return new AutoBalance(s_Swerve);
    }
}
