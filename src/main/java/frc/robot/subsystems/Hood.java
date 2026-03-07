package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HoodConstants;

public class Hood extends SubsystemBase {
    private TalonFX hoodMain;
    private SparkClosedLoopController hoodController;

    private double hoodangle = 0;

    public Hood(int hoodCanId) {
        hoodMain = new TalonFX(hoodCanId);
        //hoodFollower = new SparkFlex(hoodCanId2, MotorType.kBrushless);

        // Configure PID and Safety
        /*com.revrobotics.spark.config.SparkFlexConfig configMain = new com.revrobotics.spark.config.SparkFlexConfig();
        configMain.closedLoop.pid(Constants.HoodConstants.kP, Constants.HoodConstants.kI, Constants.HoodConstants.kD,
                ClosedLoopSlot.kSlot0);
        configMain.closedLoop.outputRange(-1, 1);

        com.revrobotics.spark.config.SparkFlexConfig configFollower = new com.revrobotics.spark.config.SparkFlexConfig();
        configFollower.follow(hoodMain);

        // Apply configuration
        hoodMain.configure(configMain, com.revrobotics.ResetMode.kResetSafeParameters,
                com.revrobotics.PersistMode.kPersistParameters);
        hoodFollower.configure(configFollower, com.revrobotics.ResetMode.kResetSafeParameters,
                com.revrobotics.PersistMode.kPersistParameters);*/

        SmartDashboard.putNumber("Hood/Hood Angle", 0);
    }

    public Command ManualHoodUp() {
        return Commands.runOnce(() -> hoodMain.set(0.1));
    }

    public Command ManualHoodDown() {
        return Commands.runOnce(() -> hoodMain.set(-0.1));
    }

    public Command ManualHoodStop() {
        return Commands.runOnce(() -> hoodMain.set(0));
    }

    @Override
    public void periodic() {
        hoodangle = SmartDashboard.getNumber("Hood Angle", 0);
    }

    public void GoToAngle() {
        double ticks = ConvertDegreesToTicks(hoodangle);
        SmartDashboard.putNumber("Requested Ticks", ticks);
        hoodController.setSetpoint(ticks, ControlType.kPosition);
    }

    private double ConvertTicksToAngle(double ticks) {
        return ticks * HoodConstants.DegreesPerRotation;
    }

    private double ConvertDegreesToTicks(double angle) {
        return angle / HoodConstants.DegreesPerRotation;
    }
}
