//Package
package frc.robot.subsystems;
//Imports
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//Subsystem Class
public class ElevatorSubsystem extends SubsystemBase {
    private final TalonFX elevatorMotor;
    private final DigitalInput magSwitch;
    private static ElevatorConstants.ElevatorPositions elevatorPosition = ElevatorConstants.ElevatorPositions.DOWN;

    public class TalonFXConstants{
        public static final boolean TALON_FUTURE_PROOF = true;
        public static final String CANIVORE_NAME = "canivoreBus";
    }
//Constants Class
    public static class ElevatorConstants {
        public static final int STAGES = 3;
        public static final double ELEVATOR_DISTANCE_PER_PULSE = 1; // Placeholder
        public static final int ELEVATOR_ID = 10;

        // PID and Motion Constants
        public static final double ELEVATOR_P = 45;
        public static final double ELEVATOR_I = 0;
        public static final double ELEVATOR_D = 0;
        public static final double ELEVATOR_G = 0.12;
        public static final double ELEVATOR_A = 0.02;
        public static final double ELEVATOR_V = 6.22;
        public static final double PROFILE_V = 0.2;
        public static final double PROFILE_A = 0.5;

        // Configurations
        public static final Slot0Configs SLOT_0_CONFIGS = new Slot0Configs()
                .withKP(ELEVATOR_P).withKI(ELEVATOR_I).withKD(ELEVATOR_D)
                .withKA(ELEVATOR_A).withKV(ELEVATOR_V).withKG(ELEVATOR_G)
                .withGravityType(GravityTypeValue.Elevator_Static);

        public static final double POSITION_UPDATE_RATE = 0.05;
        public static final double VELOCITY_UPDATE_RATE = 0.01;

        public static final CurrentLimitsConfigs CURRENT_LIMITS = new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true).withSupplyCurrentThreshold(65).withSupplyCurrentLimit(75).withSupplyTimeThreshold(1)
                .withStatorCurrentLimitEnable(true).withStatorCurrentLimit(75);

        public static final SoftwareLimitSwitchConfigs SOFT_LIMITS = new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(false).withReverseSoftLimitEnable(false);

        public enum ElevatorPositions {
            DOWN(0),
            AMP(18);

            public final double distanceEl;
            public final double sensorUnitsEl;

            ElevatorPositions(double distance) {
                this.distanceEl = distance;
                this.sensorUnitsEl = (distance / STAGES) / ELEVATOR_DISTANCE_PER_PULSE;
            }
        }
    }

    public ElevatorSubsystem() {
        elevatorMotor = new TalonFX(ElevatorConstants.ELEVATOR_ID, Constants.TalonFXConstants.CANIVORE_NAME);
        magSwitch = new DigitalInput(9);

        // Configure motor
        var talonFXConfig = new TalonFXConfiguration();
        talonFXConfig.CurrentLimits = ElevatorConstants.CURRENT_LIMITS;
        talonFXConfig.SoftwareLimitSwitch = ElevatorConstants.SOFT_LIMITS;
        talonFXConfig.Slot0 = ElevatorConstants.SLOT_0_CONFIGS;
        talonFXConfig.MotionMagic.MotionMagicExpo_kV = ElevatorConstants.PROFILE_V;
        talonFXConfig.MotionMagic.MotionMagicExpo_kA = ElevatorConstants.PROFILE_A;

        // Apply configuration
        elevatorMotor.getConfigurator().apply(talonFXConfig);

        // Set update rates and neutral mode
        elevatorMotor.getRotorVelocity().waitForUpdate(ElevatorConstants.VELOCITY_UPDATE_RATE);
        elevatorMotor.getRotorPosition().waitForUpdate(ElevatorConstants.POSITION_UPDATE_RATE);
        elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    // Elevator control methods
    public void goDown(double speed) {
        elevatorMotor.set(-Math.abs(speed));
    }
//Stops
    public void stop() {
        elevatorMotor.stopMotor();
    }

    public boolean isAtBottom() {
        return magSwitch.get();
    }

    private void moveToPosition(ElevatorConstants.ElevatorPositions position) {
        elevatorPosition = position;
        MotionMagicExpoVoltage control = new MotionMagicExpoVoltage(
                position.distanceEl, true, 0.0, 0, true, false, false);
        if (position == ElevatorConstants.ElevatorPositions.AMP) {
            control.withFeedForward(10);
        }
        elevatorMotor.setControl(control);
    }
//Elevator Moves to Position
    public Command moveToPositionCommand(ElevatorConstants.ElevatorPositions position) {
        return runOnce(() -> moveToPosition(position));
    }
//Elevator Goes Down
    public Command goDownCommand(double speed) {
        return startEnd(() -> goDown(speed), this::stop);
    }

    @Override
    public void periodic() {
        // Reset encoder if elevator is at the bottom
        if (isAtBottom() && elevatorPosition == ElevatorConstants.ElevatorPositions.DOWN) {
            elevatorMotor.setPosition(0.0);
        }
    }
}
