package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    private SparkMax shooterWheel1, shooterWheel2;
    private SparkClosedLoopController m_pidController;

    public enum SHOOTER_SIDE {
        RIGHT, LEFT
    };

    protected SHOOTER_SIDE m_side;

    private double shooterspeed = 60.0; // Default to 60 RPS

    public Shooter(int shooterID1, int shooterID2, SHOOTER_SIDE side) {
        m_side = side;
        shooterWheel1 = new SparkMax(shooterID1, MotorType.kBrushless);
        shooterWheel2 = new SparkMax(shooterID2, MotorType.kBrushless);

        SparkMaxConfig config1 = new SparkMaxConfig();
        EncoderConfig encoderConfig = new EncoderConfig();

        encoderConfig.velocityConversionFactor(1); // Native RPM
        config1.encoder.apply(encoderConfig);

        config1.closedLoop.pid(ShooterConstants.kP, 0, 0);
        config1.closedLoop.velocityFF(ShooterConstants.kV);
        config1.smartCurrentLimit((int) ShooterConstants.CurrentLimit);

        SparkMaxConfig config2 = new SparkMaxConfig();
        config2.apply(config1);

        if (m_side == SHOOTER_SIDE.RIGHT) {
            config2.follow(shooterWheel1, true); // Invert follower for right side
        } else {
            config2.follow(shooterWheel1, false); // Keep same direction for left side
        }

        shooterWheel1.configure(config1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shooterWheel2.configure(config2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_pidController = shooterWheel1.getClosedLoopController();
    }

    public void setSpeed(double rps) {
        shooterspeed = rps;
        double rpm = rps * 60.0;
        m_pidController.setSetpoint(rpm, ControlType.kVelocity);
    }

    public void Spin(double speedMeasurement) {
        // SparkMax expects RPM, so if RPS is given, convert to RPM
        shooterspeed = speedMeasurement;
        double rpm = speedMeasurement * 60.0;
        m_pidController.setSetpoint(rpm, ControlType.kVelocity);
    }

    public void Spin() {
        Spin(shooterspeed);
    }

    public void Stop() {
        Spin(ShooterConstants.IdleSpeed); // Keep it spinning at idle instead of full stop
    }

    public Command runShooterCommand() {
        return run(this::Spin).finallyDo(interrupted -> Stop());
    }

    public void RightSpin(double voltage) {
        shooterWheel1.setVoltage(voltage);
        shooterWheel2.setVoltage(-voltage);
    }

    public void LeftSpin(double voltage) {
        shooterWheel1.setVoltage(voltage);
        shooterWheel2.setVoltage(voltage);
    }

    public boolean isAtSpeed() {
        // SparkMax returns velocity in RPM
        double currentSpeedRpm = shooterWheel1.getEncoder().getVelocity();
        double currentSpeedRps = currentSpeedRpm / 60.0;
        return Math.abs(currentSpeedRps - shooterspeed) <= (Math.abs(shooterspeed) * 0.05);
    }

    @Override
    public void periodic() {
        super.periodic();

        SmartDashboard.putNumber("Shooter " + m_side.name() + "/Wheel1 Speed",
                shooterWheel1.getEncoder().getVelocity());
        SmartDashboard.putNumber("Shooter " + m_side.name() + "/Wheel2 Speed",
                shooterWheel2.getEncoder().getVelocity());
    }
}
