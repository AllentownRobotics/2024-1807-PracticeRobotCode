// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ModuleConstants;


public class KrakenSubsystem extends SubsystemBase {
  TalonFX frontLeftKraken;
  TalonFX frontRightKraken;
  TalonFX backRightKraken;
  TalonFX backLeftKraken;

  CANSparkMax frontLeftTurn;
  CANSparkMax frontRightTurn;
  CANSparkMax backRightTurn;
  CANSparkMax backLeftTurn;

  AbsoluteEncoder frontLeftEncoder;
  AbsoluteEncoder frontRightEncoder;
  AbsoluteEncoder backLeftEncoder;
  AbsoluteEncoder backRightEncoder;

  SparkPIDController frontLeftPIDController;
  SparkPIDController frontRightPIDController;
  SparkPIDController backLeftPIDController;
  SparkPIDController backRightPIDController;
  
  double constantVoltage = 0.0;

  SimpleMotorFeedforward feedforward;

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

  private Measure<Velocity<Voltage>> rampRate = Volts.of(.5).per(Second);
  private Measure<Voltage> stepVoltage = Volts.of(4);

  private final SysIdRoutine m_sysIdRoutine;
  
  /** Creates a new KrakenSubsystem. */
  public KrakenSubsystem() {
    frontLeftKraken = new TalonFX(1);
    frontRightKraken = new TalonFX(3);
    backRightKraken = new TalonFX(5);
    backLeftKraken = new TalonFX(7);

    frontLeftTurn = new CANSparkMax(2, MotorType.kBrushless);
    frontRightTurn = new CANSparkMax(4, MotorType.kBrushless);
    backRightTurn = new CANSparkMax(6, MotorType.kBrushless);
    backLeftTurn = new CANSparkMax(8, MotorType.kBrushless);
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Feedback.SensorToMechanismRatio = 5.08/(0.076*Math.PI);
    config.Slot0.kP = 0.05;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    backLeftKraken.getConfigurator().apply(config);
    frontLeftKraken.getConfigurator().apply(config);
    frontRightKraken.getConfigurator().apply(config);
    backRightKraken.getConfigurator().apply(config);

  
  frontLeftTurn.restoreFactoryDefaults();
  frontRightTurn.restoreFactoryDefaults();
  backLeftTurn.restoreFactoryDefaults();
  backRightTurn.restoreFactoryDefaults(); 

  frontLeftEncoder = frontLeftTurn.getAbsoluteEncoder(Type.kDutyCycle);
  frontRightEncoder = frontRightTurn.getAbsoluteEncoder(Type.kDutyCycle);
  backLeftEncoder = backRightTurn.getAbsoluteEncoder(Type.kDutyCycle);
  backRightEncoder = backLeftTurn.getAbsoluteEncoder(Type.kDutyCycle);


  frontLeftPIDController = frontLeftTurn.getPIDController();
  frontRightPIDController = frontRightTurn.getPIDController();
  backLeftPIDController = backRightTurn.getPIDController();
  backRightPIDController = backLeftTurn.getPIDController();
  
  frontLeftPIDController.setFeedbackDevice(frontLeftEncoder);
  frontRightPIDController.setFeedbackDevice(frontRightEncoder);
  backLeftPIDController.setFeedbackDevice(backLeftEncoder);
  backRightPIDController.setFeedbackDevice(backRightEncoder);

  frontLeftEncoder.setPositionConversionFactor(ModuleConstants.TURN_ENCODER_POS_FACTOR);
  frontRightEncoder.setPositionConversionFactor(ModuleConstants.TURN_ENCODER_POS_FACTOR);
  backLeftEncoder.setPositionConversionFactor(ModuleConstants.TURN_ENCODER_POS_FACTOR);
  backRightEncoder.setPositionConversionFactor(ModuleConstants.TURN_ENCODER_POS_FACTOR);
  frontLeftEncoder.setVelocityConversionFactor(ModuleConstants.TURN_ENCODER_VELOCITY_FACTOR);
  frontRightEncoder.setVelocityConversionFactor(ModuleConstants.TURN_ENCODER_VELOCITY_FACTOR);
  backLeftEncoder.setVelocityConversionFactor(ModuleConstants.TURN_ENCODER_VELOCITY_FACTOR);
  backRightEncoder.setVelocityConversionFactor(ModuleConstants.TURN_ENCODER_VELOCITY_FACTOR);

  frontLeftEncoder.setInverted(ModuleConstants.invertTurnEncoder);
  frontRightEncoder.setInverted(ModuleConstants.invertTurnEncoder);
  backLeftEncoder.setInverted(ModuleConstants.invertTurnEncoder);
  backRightEncoder.setInverted(ModuleConstants.invertTurnEncoder);
  
  frontLeftPIDController.setPositionPIDWrappingEnabled(true);
  frontRightPIDController.setPositionPIDWrappingEnabled(true);
  backLeftPIDController.setPositionPIDWrappingEnabled(true);
  backRightPIDController.setPositionPIDWrappingEnabled(true);
  frontLeftPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.TURN_ENCODER_POS_MAX_INPUT);
  frontRightPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.TURN_ENCODER_POS_MAX_INPUT);
  backLeftPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.TURN_ENCODER_POS_MAX_INPUT);
  backRightPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.TURN_ENCODER_POS_MAX_INPUT);
  frontLeftPIDController.setPositionPIDWrappingMinInput(ModuleConstants.TURN_ENCODER_POS_MIN_INPUT);
  frontRightPIDController.setPositionPIDWrappingMinInput(ModuleConstants.TURN_ENCODER_POS_MIN_INPUT);
  backLeftPIDController.setPositionPIDWrappingMinInput(ModuleConstants.TURN_ENCODER_POS_MIN_INPUT);
  backRightPIDController.setPositionPIDWrappingMinInput(ModuleConstants.TURN_ENCODER_POS_MIN_INPUT);
  
  //set drive motor PID values
  

  //Set turn motor PID values 
  frontLeftPIDController.setP(ModuleConstants.TURN_P);
  frontRightPIDController.setP(ModuleConstants.TURN_P);
  backLeftPIDController.setP(ModuleConstants.TURN_P);
  backRightPIDController.setP(ModuleConstants.TURN_P);
  frontLeftPIDController.setI(ModuleConstants.TURN_I);
  frontRightPIDController.setI(ModuleConstants.TURN_I);
  backLeftPIDController.setI(ModuleConstants.TURN_I);
  backRightPIDController.setI(ModuleConstants.TURN_I);
  frontLeftPIDController.setD(ModuleConstants.TURN_D);
  frontRightPIDController.setD(ModuleConstants.TURN_D);
  backLeftPIDController.setD(ModuleConstants.TURN_D);
  backRightPIDController.setD(ModuleConstants.TURN_D);
  frontLeftPIDController.setFF(ModuleConstants.TURN_FF);
  frontRightPIDController.setFF(ModuleConstants.TURN_FF);
  backLeftPIDController.setFF(ModuleConstants.TURN_FF);
  backRightPIDController.setFF(ModuleConstants.TURN_FF);
  frontLeftPIDController.setOutputRange(ModuleConstants.TURN_MIN_OUTPUT, ModuleConstants.TURN_MAX_OUTPUT);
  frontRightPIDController.setOutputRange(ModuleConstants.TURN_MIN_OUTPUT, ModuleConstants.TURN_MAX_OUTPUT);
  backLeftPIDController.setOutputRange(ModuleConstants.TURN_MIN_OUTPUT, ModuleConstants.TURN_MAX_OUTPUT);
  backRightPIDController.setOutputRange(ModuleConstants.TURN_MIN_OUTPUT, ModuleConstants.TURN_MAX_OUTPUT);
  
  frontLeftTurn.setIdleMode(ModuleConstants.TURN_MOTOR_IDLE_MODE); 
  frontRightTurn.setIdleMode(ModuleConstants.TURN_MOTOR_IDLE_MODE); 
  backRightTurn.setIdleMode(ModuleConstants.TURN_MOTOR_IDLE_MODE); 
  backLeftTurn.setIdleMode(ModuleConstants.TURN_MOTOR_IDLE_MODE); 
  //driveSparkFlex.setSmartCurrentLimit(ModuleConstants.DRIVE_MOTOR_CURRENT_LIMIT);
  frontLeftTurn.setSmartCurrentLimit(ModuleConstants.TURN_MOTOR_CURRENT_LIMIT);
  frontRightTurn.setSmartCurrentLimit(ModuleConstants.TURN_MOTOR_CURRENT_LIMIT);
  backRightTurn.setSmartCurrentLimit(ModuleConstants.TURN_MOTOR_CURRENT_LIMIT);
  backLeftTurn.setSmartCurrentLimit(ModuleConstants.TURN_MOTOR_CURRENT_LIMIT);

  //saves configs of modules in case of brownout
  //driveSparkFlex.burnFlash();
  frontLeftTurn.burnFlash();
  frontRightTurn.burnFlash();
  backRightTurn.burnFlash();
  backLeftTurn.burnFlash();

    //kS = 0.13
    //kV = 0.583 // poss 1.71
    //feedforward = new SimpleMotorFeedforward(0.13, 1.71);

    //feedforward = new SimpleMotorFeedforward(0.010987, 2.4607, 0.051565);
    //feedforward = new SimpleMotorFeedforward(0.090426, 9.9887, 0.051565);
    //feedforward = new SimpleMotorFeedforward(0.055178, 10.144, 0.25012);
    //feedforward = new SimpleMotorFeedforward(0.26531, 9.9458, 0.33184);
    feedforward = new SimpleMotorFeedforward(0.10606, 2.3452, 0.05037);

    m_sysIdRoutine = new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(/*rampRate,stepVoltage, Second.of(10)*/),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motors.
              (Measure<Voltage> volts) -> {
                driveVoltage(volts.in(Volts));
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the left motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("frontleft")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            frontLeftKraken.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(frontLeftKraken.getPosition().getValue(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(frontLeftKraken.getVelocity().getValue(), MetersPerSecond));
                // Record a frame for the right motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("frontright")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            frontRightKraken.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(frontRightKraken.getPosition().getValue(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(frontRightKraken.getVelocity().getValue(), MetersPerSecond));
                
                log.motor("backleft")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            backLeftKraken.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(backLeftKraken.getPosition().getValue(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(backLeftKraken.getVelocity().getValue(), MetersPerSecond));

                log.motor("backright")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            backRightKraken.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(backRightKraken.getPosition().getValue(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(backRightKraken.getVelocity().getValue(), MetersPerSecond));

              /*log.motor("frontleft")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            frontLeftKraken.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(frontLeftKraken.getPosition().getValue()*0.076*Math.PI, Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(frontLeftKraken.getVelocity().getValue()*0.076*Math.PI, MetersPerSecond)); */
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("drive")
              this));

              /*frontLeftTurn.restoreFactoryDefaults(); 

              //driveEncoder = driveSparkFlex.getEncoder();
              AbsoluteEncoder turningEncoder = frontLeftTurn.getAbsoluteEncoder(Type.kDutyCycle);
            
            
              SparkPIDController turningPIDController = frontLeftTurn.getPIDController();
            
              turningPIDController.setFeedbackDevice(turningEncoder);
            
              //driveEncoder.setPositionConversionFactor(ModuleConstants.DRIVE_ENCODER_POS_FACTOR);
              //driveEncoder.setVelocityConversionFactor(ModuleConstants.DRIVE_ENCODER_VELOCITY_FACTOR); 
              turningEncoder.setPositionConversionFactor(ModuleConstants.TURN_ENCODER_POS_FACTOR);
              turningEncoder.setVelocityConversionFactor(ModuleConstants.TURN_ENCODER_VELOCITY_FACTOR);
            
              turningEncoder.setInverted(ModuleConstants.invertTurnEncoder);
            
              turningPIDController.setPositionPIDWrappingEnabled(true);
              turningPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.TURN_ENCODER_POS_MAX_INPUT);
              turningPIDController.setPositionPIDWrappingMinInput(ModuleConstants.TURN_ENCODER_POS_MIN_INPUT);
              
              //set drive motor PID values
              
            
              //Set turn motor PID values 
              turningPIDController.setP(ModuleConstants.TURN_P);
              turningPIDController.setI(ModuleConstants.TURN_I);
              turningPIDController.setD(ModuleConstants.TURN_D);
              turningPIDController.setFF(ModuleConstants.TURN_FF);
              turningPIDController.setOutputRange(ModuleConstants.TURN_MIN_OUTPUT, ModuleConstants.TURN_MAX_OUTPUT);
              
              //limits and brakes
              //driveSparkFlex.setIdleMode(ModuleConstants.DRIVE_MOTOR_IDLE_MODE);
              driveKraken.setNeutralMode(NeutralModeValue.Brake);
              frontLeftTurn.setIdleMode(ModuleConstants.TURN_MOTOR_IDLE_MODE); 
              //driveSparkFlex.setSmartCurrentLimit(ModuleConstants.DRIVE_MOTOR_CURRENT_LIMIT);
              frontLeftTurn.setSmartCurrentLimit(35);
            
              //saves configs of modules in case of brownout
              //driveSparkFlex.burnFlash();
              frontLeftTurn.burnFlash();*/
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Wheel velocty", backLeftKraken.getVelocity().getValue());
    SmartDashboard.putNumber("Motor Velocity", backLeftKraken.getVelocity().getValue());
    SmartDashboard.putNumber("distance", backLeftKraken.getPosition().getValue());
  }

  public void setKrakens(double speed)
  {
    frontLeftKraken.set(speed);
    frontRightKraken.set(speed);
    backRightKraken.set(speed);
    backLeftKraken.set(speed);
  }

  public void driveVoltage(double voltage)
  {
    frontLeftKraken.setVoltage(voltage);
    frontRightKraken.setVoltage(voltage);
    backRightKraken.setVoltage(voltage);
    backLeftKraken.setVoltage(voltage);

    frontLeftPIDController.setReference(-Math.PI/2, ControlType.kPosition);
    frontRightPIDController.setReference(0, ControlType.kPosition);
    backLeftPIDController.setReference(Math.PI/2, ControlType.kPosition);
    backRightPIDController.setReference(Math.PI, ControlType.kPosition);

  }

  public void setSpeed(double speed)
  {
    backLeftKraken.setControl(new VelocityVoltage(speed).withFeedForward(feedforward.calculate(speed)));
  }

  public void setSpeedOpenLoop(double speed)
  {
    double percent = speed/5*0.2373;
    backLeftKraken.set(percent);
  }


  public void setVoltage(double voltage)
  {
    backLeftKraken.setVoltage(voltage);
  }

  public double getConstantVoltage()
  {
    return constantVoltage;
  }

  public void incrementConstantVoltage(double increment)
  {
    constantVoltage += increment;
  }

  /**
   * Returns a command that will execute a quasistatic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  /**
   * Returns a command that will execute a dynamic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
}