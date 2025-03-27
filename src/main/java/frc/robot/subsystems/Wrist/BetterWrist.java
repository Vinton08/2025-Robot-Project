package frc.robot.subsystems.Wrist;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class BetterWrist {
    
    private final TalonFX wristMotor = new TalonFX(15);

    private static final double voltage = 1;
    private static final double curLimit = 10;

    private final VoltageOut outputVolts = new VoltageOut(0);

    public BetterWrist() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 0.1;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;
        wristMotor.getConfigurator().apply(config);
    }

    public void wristUp(){
    double current = wristMotor.getSupplyCurrent().getValueAsDouble();
    if (current < curLimit) {
      outputVolts.Output = voltage;
      wristMotor.setControl(outputVolts);
    } else {
      stopWrist();
    }}

    public void wristDown(){
        double current = wristMotor.getSupplyCurrent().getValueAsDouble();
        if (current < curLimit) {
          outputVolts.Output = -voltage;
          wristMotor.setControl(outputVolts);
        } else {
          stopWrist();
    }}

    public void stopWrist(){
        outputVolts.Output = 0;
        wristMotor.setControl(outputVolts);
    }

}
