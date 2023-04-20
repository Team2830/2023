package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Vision;
import frc.robot.autos.DriveAUto;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ScoreAlignSwerve extends CommandBase {
    private Swerve s_Swerve;
    private Vision s_Vision;
    private double goalDistance = 1;
    private boolean drive;
    private double targetSide;
    private double initialY = 0;
    private boolean inRange = false;

    public ScoreAlignSwerve(Swerve s_Swerve, Vision s_Vision, double targetSide) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
        this.s_Vision = s_Vision;
        this.targetSide = targetSide;
        drive = false;
    }

    @Override
    public void initialize() {
        inRange = false;
        drive = false;
        s_Vision.processVision();
        if (s_Vision.isTargetPresent()) {
            this.initialY = s_Vision.getTargetY();
        }
    }

    @Override
    public void execute() {
        double rotationVal;

        rotationVal = s_Swerve.calculateSwerveRotation(0);
        rotationVal = MathUtil.clamp(rotationVal, -Constants.Swerve.maxRotationalSpeed,
                Constants.Swerve.maxRotationalSpeed);

        double translationVal = 0;
        double strafeVal = 0;

        s_Vision.processVision();

        double goalY;
        double error;

        if (s_Vision.isTargetPresent() && !inRange) {
            SmartDashboard.putNumber("current Y", s_Vision.getTargetY());
            if (targetSide == 1) {
                goalY = 0.51; // RIGHT SIDE (DRIVER)
                error = s_Vision.getTargetY() - goalY;
                inRange = Math.abs(error) < .02;

                strafeVal = s_Vision.calculateStrafe(error, drive);
            } else if (targetSide == -1) {
                goalY = -0.57; // LEFT SIDE (DRIVER)
                error = s_Vision.getTargetY() - goalY;
                inRange = Math.abs(error) < .02;

                strafeVal = s_Vision.calculateStrafe(error, drive);
            } else {
                strafeVal = 0;
            }
        }
        if (inRange) {
            translationVal = .23;
            strafeVal = 0;
        }

        /* Drive */
        s_Swerve.drive(
                new Translation2d(translationVal, strafeVal)
                        .times(Constants.Swerve.maxSpeed),
                rotationVal * Constants.Swerve.maxAngularVelocity,
                true,
                true);
    }
}