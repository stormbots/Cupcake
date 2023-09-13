package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.Constants.IntakeandWristConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Intake extends SubsystemBase {
    
    public static CANSparkMax intakeMotor = new CANSparkMax(Constants.IntakeandWristConstants.kIntakeMotorCanId, MotorType.kBrushless);

    public Intake() {
        intakeMotor.restoreFactoryDefaults();

        intakeMotor.setInverted(IntakeandWristConstants.kIntakeMotorInverted);
        intakeMotor.setIdleMode(IdleMode.kBrake);
        intakeMotor.setSmartCurrentLimit(IntakeandWristConstants.kIntakeCurrentLimitStall, IntakeandWristConstants.kIntakeCurrentLimitFree);
        intakeMotor.setOpenLoopRampRate(IntakeandWristConstants.kIntakeRampRate);

        intakeMotor.burnFlash();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Current", intakeMotor.getOutputCurrent());
    }

    public CommandBase IntakeIn() {
        return this.run(() -> {
            intakeMotor.set(IntakeandWristConstants.kIntakeInSpeed);
        });
    }

    public CommandBase IntakeOut() {
        return this.run(() -> {
            intakeMotor.set(IntakeandWristConstants.kIntakeOutSpeed);
        });
    }

    public CommandBase IntakeIdle() {
        return this.run(() -> {
            intakeMotor.set(IntakeandWristConstants.kIntakeIdleSpeed);
        });
    }

    public Command getShootCubeCommand(Wrist wrist) {
       return new RunCommand(() ->
                        { 
                                        //making sure the wrist is at the right angle before shooting
                            if ((IntakeandWristConstants.kShootAngle - 10)/360 < wrist.m_wristAbsoluteEncoder.getPosition() && wrist.m_wristAbsoluteEncoder.getPosition() < (IntakeandWristConstants.kShootAngle + 10)/360) {
                                intakeMotor.set(IntakeandWristConstants.kIntakeOutSpeed);
                            }
                            else {
                                intakeMotor.set(IntakeandWristConstants.kIntakeIdleSpeed);
                            }
                        },
                        this);
    }
}
