package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
//import frc.robot.Constants;
import frc.robot.Constants.IntakeandWristConstants;

public class Wrist extends SubsystemBase{

    public CANSparkMax wristMotor = new CANSparkMax(IntakeandWristConstants.kWristMotorCanId, MotorType.kBrushless);

    public final AbsoluteEncoder m_wristAbsoluteEncoder;
    public SparkMaxPIDController m_wristPIDController = wristMotor.getPIDController();;
    
    public Wrist() {
        wristMotor.restoreFactoryDefaults();

        m_wristAbsoluteEncoder = wristMotor.getAbsoluteEncoder(Type.kDutyCycle);
        m_wristAbsoluteEncoder.setZeroOffset(IntakeandWristConstants.kAbsoluteAngleOffset);
        m_wristPIDController.setFeedbackDevice(m_wristAbsoluteEncoder);

        //might not need this just here for easy troubleshooting
       // m_wristAbsoluteEncoder.setPositionConversionFactor(IntakeandWristConstants.kConversionFactor);
       // m_wristAbsoluteEncoder.setVelocityConversionFactor(IntakeandWristConstants.kConversionFactor);

        wristMotor.setIdleMode(IdleMode.kBrake);
        wristMotor.setSmartCurrentLimit(IntakeandWristConstants.kWristCurrentLimitStall,IntakeandWristConstants.kWristCurrentLimitFree);
        wristMotor.setInverted(IntakeandWristConstants.kWristMotorInverted);
        wristMotor.setSoftLimit(SoftLimitDirection.kForward, IntakeandWristConstants.kStowAngle); 
        wristMotor.setSoftLimit(SoftLimitDirection.kReverse, IntakeandWristConstants.kDeployAngle); 
        wristMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        wristMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        wristMotor.setClosedLoopRampRate(0.1);
        wristMotor.setOpenLoopRampRate(0.15);

        m_wristPIDController.setP(IntakeandWristConstants.kP);
        m_wristPIDController.setI(IntakeandWristConstants.kI);
        m_wristPIDController.setD(IntakeandWristConstants.kD);
        m_wristPIDController.setFF(0);
        m_wristPIDController.setOutputRange(IntakeandWristConstants.kWristMinOutput, IntakeandWristConstants.kWristMaxOutput);

        wristMotor.burnFlash();
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Wrist Angle", m_wristAbsoluteEncoder.getPosition()); //might need to multply by 360 to make it degrees
        SmartDashboard.putNumber("Wrist Output", wristMotor.getAppliedOutput());
        SmartDashboard.putNumber("Wrist Velocity", m_wristAbsoluteEncoder.getVelocity());
    }

    public CommandBase setWristTarget(double target) {
        return this.runOnce(() -> {
            m_wristPIDController.setReference((target/360), CANSparkMax.ControlType.kSmartMotion);
        });
    }

    public Command getSetWristTargetCommand(double target) {
        return new RunCommand(() ->
                    this.setWristTarget(target), 
                    this);
    }

}
