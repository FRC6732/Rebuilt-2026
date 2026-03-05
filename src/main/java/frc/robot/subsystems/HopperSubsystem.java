package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperSubsystem extends SubsystemBase {
    private final SparkMax m_hopperMotor = new SparkMax(16, MotorType.kBrushed);

    public Command Hopper() {
        return runEnd(
                () -> {
                    m_hopperMotor.set(.5);
                },
                () -> {
                    m_hopperMotor.set(0);
                });
    }
}
