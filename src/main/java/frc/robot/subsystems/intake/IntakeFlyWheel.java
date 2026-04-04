// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import frc.robot.constants.Constants.MechanismConstants;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.SmartMechanism;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;
import yams.motorcontrollers.remote.TalonFXWrapper;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeFlyWheel extends SubsystemBase {
private SparkMax sparkPivot1 = new SparkMax(20, MotorType.kBrushless);
private SparkMax sparkPivot2 = new SparkMax(19, MotorType.kBrushless);


  private SmartMotorControllerConfig IntakeConfig = new SmartMotorControllerConfig(this)
  .withFollowers(Pair.of(sparkPivot2, false))
  .withControlMode(ControlMode.CLOSED_LOOP)
  .withClosedLoopController(10, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
  .withSimClosedLoopController(10, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
  .withFeedforward(new ArmFeedforward(0, 0, 0))
  .withSimFeedforward(new ArmFeedforward(0, 0, 0))
  .withTelemetry("Intake1Motor", TelemetryVerbosity.HIGH)
  // Gearing from the motor rotor to final shaft.
  // In this example GearBox.fromReductionStages(3,4) is the same as GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to your motor.
  .withGearing(new MechanismGearing(GearBox.fromReductionStages(36, 25)))
  // Motor properties to prevent over currenting.
  .withMotorInverted(true)
  .withIdleMode(MotorMode.COAST)    
  .withStatorCurrentLimit(Amps.of(90))
  .withClosedLoopRampRate(Seconds.of(0.25))
  .withOpenLoopRampRate(Seconds.of(0.25));


  private SmartMotorController intakeflycontroller = new SparkWrapper(sparkPivot1, DCMotor.getNEO(1), IntakeConfig);


 private final FlyWheelConfig IntakeFlywheelConfig = new FlyWheelConfig(intakeflycontroller)
  // Diameter of the flywheel.
  .withDiameter(Inches.of(4))
  // Mass of the flywheel.
  .withMass(Pounds.of(1.54))
  // Maximum speed of the shooter.
  .withUpperSoftLimit(RPM.of(4900))
  // Telemetry name and verbosity for the arm.
  .withTelemetry("IntakeFlywheelMech", TelemetryVerbosity.HIGH);
  
   // Shooter Mechanism
  private FlyWheel intakefly = new FlyWheel(IntakeFlywheelConfig);

  /**
   * Gets the current velocity of the shooter.
   *
   * @return Shooter velocity.
   */
  public AngularVelocity getVelocity() {return intakefly.getSpeed();}

  /**
   * Set the shooter velocity.
   *
   * @param speed Speed to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setVelocity(AngularVelocity speed) {return intakefly.run(speed);}
  
  /**
   * Set the shooter velocity setpoint.
   *
   * @param speed Speed to set
   */
  public void setVelocitySetpoint(AngularVelocity speed) {intakefly.setMechanismVelocitySetpoint(speed);}
  
  /**
   * Set the dutycycle of the shooter.
   *
   * @param dutyCycle DutyCycle to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command set(double dutyCycle) {return intakefly.set(dutyCycle);}

  /** Creates a new ExampleSubsystem. */
  public IntakeFlyWheel() {}

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    intakefly.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    intakefly.simIterate();
  }
}
