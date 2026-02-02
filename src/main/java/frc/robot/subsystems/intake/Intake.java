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

    public void intakeDown() {
        io.setTargetIntakePositionDegrees(0); 
    }

    public void intakeUp() {
        io.setTargetIntakePositionDegrees(30);
    }

    public void suck() {
        io.setRollerTopVolts(5);
        io.setRollerBottomVolts(5);
    }

    public Command intakeDownCommand() {
        return this.runOnce(() -> intakeDown());
    }

    public Command intakeUpCommand() {
        return this.runOnce(() -> intakeUp());
    }

    public Command suckCommand() {
        return this.run(() -> suck());
    }

}

