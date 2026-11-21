// Created by (m*(Square root of c^2))/m

package frc.robot.Subsystmes;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSys extends SubsystemBase {

    // Sim Visuals
    private final Mechanism2d mech2d = new Mechanism2d(1.0, 2.0);
    private final MechanismRoot2d mechRoot = mech2d.getRoot("ElevatorRoot", 0.5, 0.1);
    private final MechanismLigament2d elevatorLigament =
        mechRoot.append(new MechanismLigament2d("Elevator", 0.05, 90, 6, new Color8Bit(Color.kOrange)));

    // Hardware Objects
    DCMotor gearbox = DCMotor.getNEO(1);
    SparkMax motor = new SparkMax(CANDevices.elevatorMotorId, MotorType.kBrushless);
    RelativeEncoder encoder = motor.getEncoder();

    // Simulation Objects
    SparkMaxSim motorSim = new SparkMaxSim(motor, gearbox);

    // Motor plant instantiation
    private final LinearSystem<N2, N1, N2> elevatorPlant =
        LinearSystemId.createElevatorSystem(
        gearbox,
        ElevatorConstants.kCarriageMassKg,
        ElevatorConstants.kElevatorDrumRadius,
        ElevatorConstants.kElevatorGearing);

    // Elevator simulation instantiation
    private final ElevatorSim m_elevatorSim =
        new ElevatorSim(elevatorPlant,
        gearbox,
        ElevatorConstants.minHeightMeters,
        ElevatorConstants.maxHeightMeters,
        true,
        ElevatorConstants.minHeightMeters,
        0.01, 0.01);
    
    // Control Objects
    private final PIDController pid = new PIDController(12.0, 0.0, 0.5);
    private final ElevatorFeedforward feedforward =
        new ElevatorFeedforward(0.1, 0.4, 1.5);
    public double targetHeightMeters = 0.0;

    public ElevatorSys() {
        // Spark Max Config
        SparkMaxConfig config = new SparkMaxConfig();
        config.encoder
            .positionConversionFactor(ElevatorConstants.kPositionConversionFactor)
            .velocityConversionFactor(ElevatorConstants.kVelocityConversionFactor);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Visuals Networktable
        SmartDashboard.putData("Elevator Sim", mech2d);
    }

    /**
     * Method to set the desired height of the elevator in meters
     * @param heightMeters The desired height of the elevator in meters
     */
    public void setTargetHeight(double heightMeters) {
        targetHeightMeters = heightMeters;
    }

    @Override
    public void periodic() {
        // PID Voltage calculation
        double pidVolts = pid.calculate(encoder.getPosition(), targetHeightMeters);
        // FF Voltage calculation
        double ffVolts = feedforward.calculate(0.0);
        // Set the motor Voltage
        motor.setVoltage(pidVolts + ffVolts);

        // Update visual segment with current position of the Elevator
        elevatorLigament.setLength(0.05 + encoder.getPosition());

        // Networktables for Graphing
        SmartDashboard.putNumber("Elevator/Height (m)", encoder.getPosition());
        SmartDashboard.putNumber("Elevator/Target (m)", targetHeightMeters);
        SmartDashboard.putNumber("Elevator/Motor Volts", motor.getAppliedOutput() * motor.getBusVoltage());
    }

    @Override
    public void simulationPeriodic() {
        // Simulation periodic updates
        double busVoltage = RoboRioSim.getVInVoltage();
        m_elevatorSim.setInput(motorSim.getAppliedOutput() * busVoltage);

        m_elevatorSim.update(0.020);

        motorSim.iterate(
            m_elevatorSim.getVelocityMetersPerSecond(),
            busVoltage,
            0.020);
    }
}
