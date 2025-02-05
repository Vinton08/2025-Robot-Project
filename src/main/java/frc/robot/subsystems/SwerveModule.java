package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.Encoder;

public class SwerveModule {
    private final Talon driveMotor;
    private final Talon steerMotor;
    private final Encoder encoder;
    
    private static final double MAX_DRIVE_SPEED = 4.5;  // Max speed in meters per second

    /**
     * Constructor for a Swerve Module.
     *
     * @param driveMotorChannel PWM channel for the drive motor.
     * @param steerMotorChannel PWM channel for the steer motor.
     * @param encoderChannelA   First channel for the encoder.
     * @param encoderChannelB   Second channel for the encoder.
     */
    public SwerveModule(int driveMotorChannel, int steerMotorChannel, int encoderChannelA, int encoderChannelB) {
        this.driveMotor = new Talon(driveMotorChannel);
        this.steerMotor = new Talon(steerMotorChannel);
        this.encoder = new Encoder(encoderChannelA, encoderChannelB);
    }

    /**
     * Sets the desired state of the swerve module.
     *
     * @param state The desired state containing speed and angle.
     */
    public void setDesiredState(SwerveModuleState state) {
        double speed = state.speedMetersPerSecond / MAX_DRIVE_SPEED;
        double angle = state.angle.getRadians();

        // Set drive motor speed
        driveMotor.set(speed);

        // Set steer motor angle (basic implementation)
        double currentAngle = encoder.getDistance();
        double angleError = angle - currentAngle;
        steerMotor.set(angleError * 0.1);  // Proportional control for steering
    }

    /**
     * Gets the current module state.
     *
     * @return The current speed and angle of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(driveMotor.get(), new Rotation2d(encoder.getDistance()));
    }
}