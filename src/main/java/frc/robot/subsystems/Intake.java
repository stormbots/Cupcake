package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeandWristConstants;


public class Intake extends SubsystemBase {
    
    public static CANSparkMax intakeMotor = new CANSparkMax(Constants.IntakeandWristConstants.kIntakeMotorCanId, MotorType.kBrushless);

    public Intake() {
        intakeMotor.restoreFactoryDefaults();

        intakeMotor.setInverted(IntakeandWristConstants.kIntakeMotorInverted);
        intakeMotor.setIdleMode(IdleMode.kBrake);
        intakeMotor.setSmartCurrentLimit(IntakeandWristConstants.kCurrentLimitStall, IntakeandWristConstants.kCurrentLimitFree);
        intakeMotor.setOpenLoopRampRate(0.08);

        intakeMotor.burnFlash();
    }

    public CommandBase IntakeIn() {
        return this.runOnce(() -> {
            intakeMotor.set(1);
        });
    }

    public CommandBase IntakeOut() {
        return this.runOnce(() -> {
            intakeMotor.set(-1);
        });
    }

    public CommandBase IntakeIdle() {
        return this.runOnce(() -> {
            intakeMotor.set(0.2);
        });
    }
}
