package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PlowSubsystem extends SubsystemBase {
    private final SparkMax m_plowMotor = new SparkMax(14, MotorType.kBrushed);

    public Command PlowUp() {
        return runEnd(
                () -> {
                    m_plowMotor.set(-0.4);
                },
                () -> {
                    m_plowMotor.set(0);
                });
    }
    public Command PlowDown() {
        return runEnd(
            () -> {
                m_plowMotor.set(0.25);
            },
            () -> {
                m_plowMotor.set(0);
            }
        );
    }
    @Override
    public void periodic(){
        m_plowMotor.set(-.15);
    }
}
