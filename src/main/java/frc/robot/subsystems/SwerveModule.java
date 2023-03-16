package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Encoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

    //private final CANSparkMax driveMotor;
    //private final CANSparkMax turningMotor;
    private final TalonFX driveMotor;
    private final TalonFX turningMotor;

    private final PIDController turningPidController;
    private final Encoder driveEncoder;
    private final Encoder turningEncoder;

    private final AnalogInput absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed, int driveEncoderID, int driveEncoderID2, int turningEncoderID,
            int turningEncoderID2) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new AnalogInput(absoluteEncoderId);

        driveMotor = new TalonFX(driveMotorId); // creates a new TalonFX with ID 0
        turningMotor = new TalonFX(turningMotorId)

        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.supplyCurrLimit.enable = true;
        driveConfig.supplyCurrLimit.triggerThresholdCurrent = 40; // the peak supply current, in amps
        driveConfig.supplyCurrLimit.triggerThresholdTime = 1.5; // the time at the peak supply current before the limit triggers, in sec
        driveConfig.supplyCurrLimit.currentLimit = 30; // the current to maintain if the peak supply limit is triggered
        driveMotor.configAllSettings(driveConfig); // apply the config settings; this selects the quadrature encoder

        TalonFXConfiguration turnConfig = new TalonFXConfiguration();
        turnConfig.supplyCurrLimit.enable = true;
        turnConfig.supplyCurrLimit.triggerThresholdCurrent = 40; // the peak supply current, in amps
        turnConfig.supplyCurrLimit.triggerThresholdTime = 1.5; // the time at the peak supply current before the limit triggers, in sec
        turnConfig.supplyCurrLimit.currentLimit = 30; // the current to maintain if the peak supply limit is triggered
        turningMotor.configAllSettings(turnConfig); // apply the config settings; this selects the quadrature encoder


        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        driveEncoder = new AbsoluteEncoder() {
          
        };
        turningEncoder = new Encoder(turningEncoderID, turningEncoderID2);

        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public double getDrivePosition() {
        return driveEncoder.getRaw();
    }

    public double getTurningPosition() {
        return turningEncoder.getRaw();
    }

    public double getDriveVelocity() {
        return driveEncoder.getRate();
    }

    public double getTurningVelocity() {
        return turningEncoder.getRate();
    }

    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        driveEncoder.reset();
        turningEncoder.set(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(TalonFXControlMode.PercentOutput, state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(TalonFXControlMode.PercentOutput, turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
    }

    public void stop() {
        driveMotor.set(TalonFXControlMode.PercentOutput, 0);;
        turningMotor.set(TalonFXControlMode.PercentOutput, 0);
    }
}