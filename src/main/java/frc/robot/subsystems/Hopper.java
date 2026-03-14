package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;

public class Hopper extends SubsystemBase {
    // Motors
    private final TalonFX flopperMotor;
    private final TalonFX towerMotor;

    // Sensors
    private final CANrange canRange1;
    private final CANrange canRange2;

    // References
    // private final Intake m_intake;

    public Hopper() {
        flopperMotor = new TalonFX(HopperConstants.FlopperCanID);
        towerMotor = new TalonFX(HopperConstants.TowerCanID);

        TalonFXConfiguration floppeConfiguration = new TalonFXConfiguration();
        floppeConfiguration.CurrentLimits.SupplyCurrentLimit = 70;
        floppeConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;

        flopperMotor.getConfigurator().apply(floppeConfiguration);

        TalonFXConfiguration toweConfiguration = new TalonFXConfiguration();
        toweConfiguration.CurrentLimits.SupplyCurrentLimit = 70;
        toweConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;

        towerMotor.getConfigurator().apply(toweConfiguration);

        canRange1 = new CANrange(HopperConstants.CanRangeID1);
        canRange2 = new CANrange(HopperConstants.CanRangeID2);

        // Safety: Current Limits
        // SparkFlexConfig flexConfig = new SparkFlexConfig();
        // flexConfig.smartCurrentLimit(HopperConstants.HopperCurrentLimit);

        // flopperMotor.configure(flexConfig, ResetMode.kResetSafeParameters,
        // PersistMode.kPersistParameters);
        // towerMotor.configure(flexConfig, ResetMode.kResetSafeParameters,
        // PersistMode.kPersistParameters);

        // Optionally, apply invert if needed based on testing
        // flexConfig.inverted(true);
    }

    @Override
    public void periodic() {
        super.periodic();

        SmartDashboard.putNumber("Hopper/Flopper Current", flopperMotor.getTorqueCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Hopper/Tower Current", towerMotor.getTorqueCurrent().getValueAsDouble());
    }

    /** Sets both hopper motors to the same speed. */
    public void setSpeed(double speed) {
        flopperMotor.set(-speed);
        towerMotor.set(-speed);
    }

    public void stop() {
        setSpeed(0.0);
    }

    public void feed(double speed) {
        setSpeed(Math.abs(speed));
    }

    public void reverse(double speed) {
        setSpeed(-Math.abs(speed));
    }

    public Command runHopper(double speed) {
        return run(() -> setSpeed(speed));
    }

    /** Runs both hoppers at default feed speed; stops when a ball is present. */
    public Command runHopperCommand() {
        return run(() -> {
            if (hasBall()) {
                stop();
            } else {
                feed(HopperConstants.HopperFeedSpeed);
            }
        }).finallyDo(interrupted -> stop());
    }

    public Command feedCommand(double speed) {
        return run(() -> feed(speed));
    }

    public Command stopCommand() {
        return runOnce(this::stop);
    }

    /**
     * Checks if a ball is detected by the CANrange sensors.
     * 
     * @return true if ball is present.
     */
    public boolean hasBall() {
        // LaserMinDistance is 200.0 (mm), CANrange gets distance in meters
        boolean range1HasBall = canRange1.getDistance()
                .getValueAsDouble() < (HopperConstants.LaserMinDistance / 1000.0);
        boolean range2HasBall = canRange2.getDistance()
                .getValueAsDouble() < (HopperConstants.LaserMinDistance / 1000.0);
        return range1HasBall || range2HasBall;
    }

    /**
     * Runs Hopper to index balls. Stops if a ball is at the top (hasBall() is
     * true).
     */
    public Command runIndexCommand() {
        return run(() -> {
            if (hasBall()) {
                stop();
            } else {
                feed(HopperConstants.HopperFeedSpeed);
            }
        }).finallyDo(interrupted -> stop());
    }

    public Command runShootCommand() {
        return run(() -> {
            feed(HopperConstants.HopperFeedSpeed);
        }).finallyDo(interrupted -> stop());
    }

    /**
     * Runs Hopper to feed the shooter.
     * Ends when the hopper has been empty (no ball detected) for a set duration.
     */
    public Command runShootFeedCommand() {
        // Debounce the "empty" signal for 1 second to ensure we don't stop prematurely
        // between balls.
        edu.wpi.first.math.filter.Debouncer debouncer = new edu.wpi.first.math.filter.Debouncer(1.0,
                edu.wpi.first.math.filter.Debouncer.DebounceType.kBoth);

        return run(() -> feed(HopperConstants.HopperFeedSpeed))
                .until(() -> debouncer.calculate(!hasBall()))
                .finallyDo(interrupted -> stop());
    }
}
