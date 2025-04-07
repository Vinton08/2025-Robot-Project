package frc.robot.subsystems.Wrist;

import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class BetterWrist {

  private final TalonFX wristMotor = new TalonFX(15);

  private final StaticBrake staticBrake = new StaticBrake();

  private static final double voltage = 3;
  private static final double curLimit = 20;
  private static final double holdVolt = -0.15;

  private final VoltageOut outputVolts = new VoltageOut(0);

  public BetterWrist() {
    // TalonFXConfiguration config = new TalonFXConfiguration();
    // config.Slot0.kP = 0.1;
    // config.Slot0.kI = 0;
    // config.Slot0.kD = 0;
    // wristMotor.getConfigurator().apply(config);
  }

  public void wristUp() {
    double current = wristMotor.getSupplyCurrent().getValueAsDouble();
    if (current < curLimit) {
      outputVolts.Output = -voltage;
      wristMotor.setControl(outputVolts);
    } else {
      stopWrist();
    }
  }

  public void wristDown() {
    double current = wristMotor.getSupplyCurrent().getValueAsDouble();
    if (current < curLimit) {
      outputVolts.Output = voltage;
      wristMotor.setControl(outputVolts);
    } else {
      stopWrist();
    }
  }

  public void holdWrist() {
    double current = wristMotor.getSupplyCurrent().getValueAsDouble();
    if (current < curLimit) {
      outputVolts.Output = holdVolt;
      wristMotor.setControl(outputVolts);
    } else {
      stopWrist();
    }
  }

  public void stopWrist() {
    wristMotor.setNeutralMode(NeutralModeValue.Brake);

    outputVolts.Output = 0;
    wristMotor.setControl(staticBrake);
  }

  public Command newWristUpCommand() {
    return Commands.run(() -> wristUp());
  }

  public Command newWristDownCommand() {
    return Commands.run(() -> wristDown());
  }

  public Command holdNewWristCommand() {
    return Commands.run(() -> holdWrist());
  }

  public Command stopNewWristCommand() {
    return Commands.run(() -> stopWrist());
  }
}
