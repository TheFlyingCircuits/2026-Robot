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

    public void intakeRunRollers() {
        io.setTargetRollerBottomVelocity(5);
        io.setTargetRollerTopVelocity(5);
    }

    public void intakeDown() {
        io.setTargetIntakePositionDegrees(0); 
    }

    public void intakeUp() {
        io.setTargetIntakePositionDegrees(30);
    }

    public void intakeDefault() {
        io.setIntakeVolts(0);
    }

    public Command intakeDefaultCommand() {
        return this.run(() -> intakeDefault());
    }

    public Command intakeDownCommand() {
        System.out.print("IntakeDown");
        return this.runOnce(() -> intakeDown());
    }

    
    public Command intakeUpCommand() {
        return this.runOnce(() -> intakeUp());
    }

    public Command intakeRunRollersCommand() {
        return this.run(() -> intakeRunRollers());
    }

}

