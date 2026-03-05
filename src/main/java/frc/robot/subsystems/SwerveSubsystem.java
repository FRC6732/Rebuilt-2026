package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class SwerveSubsystem extends SubsystemBase {
    public SwerveDrive swerveDrive;

    public SwerveSubsystem() {
        double maximumSpeed = Units.feetToMeters(3);
        File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
        try {
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Command to drive the robot using field relative positions
     *
     * @param translationX     Translation (movement) in the X direction.
     * @param translationY     Translation (movement) in the Y direction.
     * @param angularRotationX Rotation of the robot to set
     * @return Drive command.
     */
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
            DoubleSupplier angularRotationX) {
        return driveCommand(translationX, translationY, angularRotationX, false);
    }

    /**
    * Command to drive the robot using field relative positions
    *
    * @param translationX     Translation (movement) in the X direction.
    * @param translationY     Translation (movement) in the Y direction.
    * @param angularRotationX Rotation of the robot to set
    * @param fieldRelative    Should the robot drive and rotate relative to the field?
    * @return Drive command.
    */
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
            DoubleSupplier angularRotationX, boolean fieldRelative) {        
        return run(() -> {
            // Make the robot move
            swerveDrive.drive(new Translation2d(translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                    translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()),
                    angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
                    fieldRelative,
                    false);
        });
    }

    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }

    public void driveFieldOriented(ChassisSpeeds velocity) {
        swerveDrive.driveFieldOriented(velocity);
    }

    public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
        return run(() -> {
            swerveDrive.driveFieldOriented(velocity.get());
        });
    }
}
