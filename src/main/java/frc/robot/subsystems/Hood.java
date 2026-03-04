package frc.robot.subsystems;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HoodConstants;

public class Hood extends SubsystemBase {
    private SparkFlex hoodMain;
    private SparkFlex hoodFollower;
    private SparkClosedLoopController hoodController;
    
    private double hoodangle = 0;

    public Hood(int hoodCanId1, int hoodCanId2) {
        hoodMain = new SparkFlex(hoodCanId1, MotorType.kBrushless);
        hoodFollower = new SparkFlex(hoodCanId2, MotorType.kBrushless);
        hoodController = hoodMain.getClosedLoopController();
        
        // Configure PID and Safety
        com.revrobotics.spark.config.SparkFlexConfig configMain = new com.revrobotics.spark.config.SparkFlexConfig();
        configMain.closedLoop.pid(Constants.HoodConstants.kP, Constants.HoodConstants.kI, Constants.HoodConstants.kD, ClosedLoopSlot.kSlot0);
        configMain.closedLoop.outputRange(-1, 1);
        
        com.revrobotics.spark.config.SparkFlexConfig configFollower = new com.revrobotics.spark.config.SparkFlexConfig();
        configFollower.follow(hoodMain);

        // Apply configuration
        hoodMain.configure(configMain, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters);
        hoodFollower.configure(configFollower, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters);

        hoodMain.getEncoder().setPosition(0);
        
        SmartDashboard.putNumber("Hood Angle", 0);
    }

    @Override
    public void periodic() {
        hoodangle = SmartDashboard.getNumber("Hood Angle", 0);
        SmartDashboard.putNumber("Actual Hood Angle", hoodMain.getEncoder().getPosition() * HoodConstants.DegreesPerRotation);
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
