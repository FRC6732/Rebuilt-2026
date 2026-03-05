package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FireSubsystem extends SubsystemBase {
    private final SparkMax m_fireMotorA = new SparkMax(17, MotorType.kBrushed) ;
    private final SparkMax m_fireMotorB = new SparkMax(18, MotorType.kBrushed);

    public Command Fire() {
        return runEnd(
                () -> {
                    m_fireMotorA.set(1);
                    m_fireMotorB.set(-1);
                },
                () -> {
                    m_fireMotorA.set(0);
                    m_fireMotorB.set(0);
                });
    }
}
