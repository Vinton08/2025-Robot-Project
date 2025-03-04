// Runs CAN IDs 14 and 13 until a current limit is reached, then runs both slowly
// CAN ID 14 is the "Leader" 13 is the follower and runs REVERSE of the leader
package frc.robot.subsystems.Algae;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.*;

public class Algae {
  private final TalonFX leader = new TalonFX(14); // Leader “Left Algae”
  private final TalonFX follower = new TalonFX(13); // Follower “Right Algae”

  private final double normVolts = 7; // Normal VoltageOut - 7 Volts
  private final double maxVolts = 12; // Max VoltageOut - 12 Volts (For scoring in net)
  private final double minVolts = 0.5; // Slow Speed - 0.5 Volts
  private final double curLimit = 10; // Max Current - 10 Amps

  private final VoltageOut outputVolts = new VoltageOut(0);

  public Algae() {
    follower.setControl(
        new Follower(14, true)); // Sets ID 13 as the follower and reverses the direction

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = 0.1;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;
    leader.getConfigurator().apply(config);
  }

  public void intakeAlgae() {
    double current = leader.getSupplyCurrent().getValueAsDouble();
    double targetVoltage = (current < curLimit) ? normVolts : minVolts;

    outputVolts.Output = targetVoltage;
    leader.setControl(outputVolts);
  }

  public void outtakeAlgae() {
    double targetVoltage = -normVolts;

    outputVolts.Output = targetVoltage;
    leader.setControl(outputVolts);
  }

  public void shootAlgae() {
    double targetVoltage = -maxVolts;

    outputVolts.Output = targetVoltage;
    leader.setControl(outputVolts);
  }

  public void stopMotors() {
    outputVolts.Output = 0;
    leader.setControl(outputVolts);
  }
}
