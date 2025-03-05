// Runs CAN ID 15 at a set speed while a button is held on the operator stick until a current limit
// is reached, then will stop.
package frc.robot.subsystems.Coral;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Coral {
  private TalonFX motor = new TalonFX(15);

  private final double normVolts = 5; // Normal Voltage - 5 Volts
  private final double curLimit = 10; // Max current draw - 10 Amps

  private final VoltageOut outputVolts = new VoltageOut(0);

  public Coral() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = 0.1; // P
    config.Slot0.kI = 0.0; // I
    config.Slot0.kD = 0.0; // D
    motor.getConfigurator().apply(config);
  }

  public void stopMotor() { // Outputs 0 Volts on Motor
    outputVolts.Output = 0;
    motor.setControl(outputVolts);
  }

  public void intakeCoral() { // Runs motor at 5 Volts until current limit is reached.
    double current = motor.getSupplyCurrent().getValueAsDouble();
    if (current < curLimit) {
      outputVolts.Output = normVolts;
      motor.setControl(outputVolts);
    } else {
      stopMotor();
    }
  }

  public void outtakeCoral() { // Runs motor at -5 volts
    outputVolts.Output = -normVolts;
    motor.setControl(outputVolts);
  }

  public Command intakeCoralCommand() {
    return Commands.run(() -> intakeCoral());
  }

  public Command outtakeCoralCommand() {
    return Commands.run(() -> outtakeCoral());
  }

  public Command stopCoralCommand() {
    return Commands.run(() -> stopMotor());
  }
}
