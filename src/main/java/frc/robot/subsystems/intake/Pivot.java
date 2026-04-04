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

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.MechanismConstants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.SmartMechanism;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;
import yams.motorcontrollers.remote.TalonFXWrapper;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Pivot extends SubsystemBase {

  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
  .withControlMode(ControlMode.CLOSED_LOOP)
  .withClosedLoopController(50, 0, 0)
  .withFeedforward(new ArmFeedforward(0.2, 2, 0))
  .withTelemetry("IntakeMotor", TelemetryVerbosity.HIGH)
   .withGearing(new MechanismGearing(new GearBox(MechanismConstants.intake_pivot_gearbox)))
//   .withGearing(new MechanismGearing(GearBox.fromStages("23:12","42:16","30:15","36:12")))
  .withMotorInverted(false)
  .withIdleMode(MotorMode.BRAKE)
  .withClosedLoopRampRate(Seconds.of(0.25))
  .withOpenLoopRampRate(Seconds.of(0.25));

  // Vendor motor controller object
    TalonFX pivotFx = new TalonFX(15, new CANBus("canivore"));


  // Create our SmartMotorController from our Spark and config with the NEO.
  SmartMotorController pivotController = new TalonFXWrapper(pivotFx, DCMotor.getKrakenX60(1), smcConfig);


  private ArmConfig armCfg = new ArmConfig(pivotController)
  // Soft limit is applied to the SmartMotorControllers PID
  .withSoftLimits(Degrees.of(-15), Degrees.of(100))
  // Hard limit is applied to the simulation.
  .withHardLimit(Degrees.of(-15), Degrees.of(100))
  // Starting position is where your arm starts
  .withStartingPosition(Degrees.of(90))
  // Length and mass of your arm for sim.
  .withLength(Feet.of(3))
  .withMass(Pounds.of(1))
  // Telemetry name and verbosity for the arm.
  .withTelemetry("Arm", TelemetryVerbosity.HIGH);

  // Arm Mechanism
  private Arm pivot = new Arm(armCfg);

  /**
   * Set the angle of the arm, does not stop when the arm reaches the setpoint.
   * @param angle Angle to go to.
   * @return A command.
   */
  public Command setAngle(Angle angle) { return pivot.run(angle);}
  
  /**
   * Set the angle of the arm, ends the command but does not stop the arm when the arm reaches the setpoint.
   * @param angle Angle to go to.
   * @param tolerance Angle tolerance for completion.
   * @return A Command
   */
  public Command setAngleAndStop(Angle angle, Angle tolerance) { return pivot.runTo(angle, tolerance);}
  
  /**
   * Set arm closed loop controller to go to the specified mechanism position.
   * @param angle Angle to go to.
   */
  public void setAngleSetpoint(Angle angle) { pivot.setMechanismPositionSetpoint(angle); }

  /**
   * Move the arm up and down.
   * @param cycle [-1, 1] speed to set the arm too.
   */
  public Command set(double cycle) { return pivot.set(cycle);}

  public Angle getAngle() { return pivot.getAngle(); }

  /**
   * Run sysId on the {@link Arm}
   */
  public Command sysId() { return pivot.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));}

  /** Creates a new ExampleSubsystem. */
  public Pivot() {}

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
    pivot.updateTelemetry();
     SmartDashboard.putNumber("Pivot Encoder Ticks", this.pivot.getAngle().magnitude());


  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    pivot.simIterate();
  }
}
