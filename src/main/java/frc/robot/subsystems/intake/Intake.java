package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

public class Intake extends SubsystemBase {
    IntakeIOInputsAutoLogged inputs;
    IntakeIO io;

    public Intake(IntakeIO io) {
        this.io = io;
        inputs = new IntakeIOInputsAutoLogged();
    }

    public void intakeDownAndRunRollers() {
        io.setTargetIntakePositionDegrees(0); 
        io.setRollerTopVolts(5);
        io.setRollerBottomVolts(5);
    }

    public void intakeUp() {
        io.setTargetIntakePositionDegrees(30);
    }

    public Command intakeDownAndRunRollersCommand() {
        return this.startEnd(() -> intakeDownAndRunRollers(),() -> intakeUp());
    }

}

