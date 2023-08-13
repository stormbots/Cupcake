package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeandWristConstants;


public class Intake {
    
    public static CANSparkMax intakeMotor = new CANSparkMax(Constants.IntakeandWristConstants.kIntakeMotorCanId, MotorType.kBrushless);
    public static CANSparkMax wristMotor = new CANSparkMax(Constants.IntakeandWristConstants.kWristMotorCanId, MotorType.kBrushless);

    public final AbsoluteEncoder m_wristAbsoluteEncoder;
    public final SparkMaxPIDController m_wristPIDController;

    public Intake() {
        intakeMotor.restoreFactoryDefaults();
        wristMotor.restoreFactoryDefaults();

        intakeMotor.setInverted(IntakeandWristConstants.kIntakeMotorInverted);
        intakeMotor.setIdleMode(IdleMode.kBrake);
        intakeMotor.setSmartCurrentLimit(IntakeandWristConstants.kCurrentLimitStall, IntakeandWristConstants.kCurrentLimitFree);
        intakeMotor.setOpenLoopRampRate(0.08);

        m_wristAbsoluteEncoder = wristMotor.getAbsoluteEncoder(Type.kDutyCycle);
        m_wristPIDController = wristMotor.getPIDController();
        m_wristPIDController.setFeedbackDevice(m_wristAbsoluteEncoder);
        wristMotor.setIdleMode(IdleMode.kBrake);
        wristMotor.setSmartCurrentLimit(12,15);
        wristMotor.setInverted(IntakeandWristConstants.kReverseMotor);
        wristMotor.getAbsoluteEncoder(Type.kDutyCycle);
        wristMotor.setSoftLimit(SoftLimitDirection.kForward, IntakeandWristConstants.kMaxAngle); 
        wristMotor.setSoftLimit(SoftLimitDirection.kReverse, IntakeandWristConstants.kMinAngle); 
        wristMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        wristMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        wristMotor.setClosedLoopRampRate(0.05);
        wristMotor.setOpenLoopRampRate(0.05);
        m_wristAbsoluteEncoder.setZeroOffset(IntakeandWristConstants.kAbsoluteAngleOffset);

        wristMotor.getPIDController().setP(IntakeandWristConstants.kP);
        wristMotor.getPIDController().setI(IntakeandWristConstants.kI);
        wristMotor.getPIDController().setD(IntakeandWristConstants.kD);
        wristMotor.getPIDController().setFF(0);


        intakeMotor.burnFlash();
        wristMotor.burnFlash();


    }
}
