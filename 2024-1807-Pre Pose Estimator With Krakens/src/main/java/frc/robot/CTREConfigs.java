package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.ModuleConstants;

public final class CTREConfigs {
    public static  TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();

    public CTREConfigs(){
        /** Swerve Drive Motor Configuration */
        /* Motor Inverts and Neutral Mode */
        swerveDriveFXConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        swerveDriveFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        /* Gear Ratio Config */
        swerveDriveFXConfig.Feedback.SensorToMechanismRatio = ModuleConstants.DRIVE_MOTOR_REDUCTION;

        /* Current Limiting */
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = ModuleConstants.DRIVE_MOTOR_CURRENT_LIMIT;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentThreshold = 60;
        swerveDriveFXConfig.CurrentLimits.SupplyTimeThreshold = .1;

        /* PID Config */
        swerveDriveFXConfig.Slot0.kP = 0.5;
        swerveDriveFXConfig.Slot0.kI = 0.0;
        swerveDriveFXConfig.Slot0.kD = 0.1;

        /* Open and Closed Loop Ramping */
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = ModuleConstants.OPEN_LOOP_RAMP_RATE;

        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = ModuleConstants.CLOSED_LOOP_RAMP_RATE;
    }
}