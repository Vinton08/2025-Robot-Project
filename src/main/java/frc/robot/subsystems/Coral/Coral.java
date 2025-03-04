// Runs CAN ID 15 at a set speed while a button is held on the operator stick until a current limit
// is reached, then will stop.
package frc.robot.subsystems.Coral;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.*;

public class Coral {
    private TalonFX motor = new TalonFX(15);

    private final double normVolts = 5;
    private final double curLimit = 10;

    private final VoltageOut outputVolts = new VoltageOut(0);

    public Coral(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 0.1;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        motor.getConfigurator().apply(config);
    }
    public void stopMotor(){
        outputVolts.Output = 0;
        motor.setControl(outputVolts);
    }
    public void intakeCoral(){
        double current = motor.getSupplyCurrent().getValueAsDouble();
        if (current < curLimit) {
            outputVolts.Output = normVolts;
            motor.setControl(outputVolts);
        } else {
            stopMotor();
        }
    }
    public void outtakeCoral(){
        outputVolts.Output = -normVolts;
        motor.setControl(outputVolts);
    }
}
