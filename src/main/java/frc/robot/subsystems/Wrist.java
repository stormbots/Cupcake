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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeandWristConstants;

public class Wrist extends SubsystemBase{

    public static CANSparkMax wristMotor = new CANSparkMax(Constants.IntakeandWristConstants.kWristMotorCanId, MotorType.kBrushless);

    public final AbsoluteEncoder m_wristAbsoluteEncoder;
    public static SparkMaxPIDController m_wristPIDController = wristMotor.getPIDController();;
    
    public Wrist() {
        wristMotor.restoreFactoryDefaults();

        m_wristAbsoluteEncoder = wristMotor.getAbsoluteEncoder(Type.kDutyCycle);
        m_wristAbsoluteEncoder.setZeroOffset(IntakeandWristConstants.kAbsoluteAngleOffset);
        m_wristPIDController.setFeedbackDevice(m_wristAbsoluteEncoder);
        wristMotor.setIdleMode(IdleMode.kBrake);
        wristMotor.setSmartCurrentLimit(12,15);
        wristMotor.setInverted(IntakeandWristConstants.kReverseMotor);
        wristMotor.setSoftLimit(SoftLimitDirection.kForward, IntakeandWristConstants.kStowAngle); 
        wristMotor.setSoftLimit(SoftLimitDirection.kReverse, IntakeandWristConstants.kDeployAngle); 
        wristMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        wristMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        wristMotor.setClosedLoopRampRate(0.05);
        wristMotor.setOpenLoopRampRate(0.05);

        wristMotor.getPIDController().setP(IntakeandWristConstants.kP);
        wristMotor.getPIDController().setI(IntakeandWristConstants.kI);
        wristMotor.getPIDController().setD(IntakeandWristConstants.kD);
        wristMotor.getPIDController().setFF(0);

        wristMotor.burnFlash();


    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Wrist Angle", m_wristAbsoluteEncoder.getPosition());
    }

    public CommandBase setWristTarget(double target) {
        return this.runOnce(() -> {
            m_wristPIDController.setReference((target/360), CANSparkMax.ControlType.kPosition);
        });
    }

}
