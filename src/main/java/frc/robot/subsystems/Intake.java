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


public class Intake extends SubsystemBase {
    
    public static CANSparkMax intakeMotor = new CANSparkMax(Constants.IntakeandWristConstants.kIntakeMotorCanId, MotorType.kBrushless);

    public Intake() {
        intakeMotor.restoreFactoryDefaults();

        intakeMotor.setInverted(IntakeandWristConstants.kIntakeMotorInverted);
        intakeMotor.setIdleMode(IdleMode.kBrake);
        intakeMotor.setSmartCurrentLimit(IntakeandWristConstants.kIntakeCurrentLimitStall, IntakeandWristConstants.kIntakeCurrentLimitFree);
        intakeMotor.setOpenLoopRampRate(0.01);

        intakeMotor.burnFlash();
    }

    public CommandBase IntakeIn() {
        return this.run(() -> {
            intakeMotor.set(1);
        });
    }

    public CommandBase IntakeOut() {
        return this.run(() -> {
            intakeMotor.set(-1);
        });
    }

    public CommandBase IntakeIdle() {
        return this.run(() -> {
            intakeMotor.set(0.2);
        });
    }

    public Command getShootCubeCommand(Wrist wrist) {
       return new RunCommand(() ->
                        { 
                                        //making sure the wrist is at the right angle before shooting
                            if ((IntakeandWristConstants.kShootAngle - 5)/360 < wrist.m_wristAbsoluteEncoder.getPosition() && wrist.m_wristAbsoluteEncoder.getPosition() < (IntakeandWristConstants.kShootAngle + 5)/360) {
                                intakeMotor.set(-1);
                            }
                            else {
                                intakeMotor.set(0.2);
                            }
                        },
                        this);
    }
}
