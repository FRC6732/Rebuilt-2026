package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private final SparkMax m_intakeMotorA = new SparkMax(13, MotorType.kBrushed);
  private final SparkMax m_intakeMotorB = new SparkMax(15, MotorType.kBrushed);

  public Command Intake() {
    return runEnd(
        () -> {
          m_intakeMotorA.set(1);
          m_intakeMotorB.set(0.5);
        },
        () -> {
          m_intakeMotorA.set(0);
          m_intakeMotorB.set(0);
        });
  }
}
