package frc.robot.subsystems.intake;


public class Intake {
   IntakeIOInputsAutoLogged inputs;
   IntakeIO io;




   public Intake(IntakeIO io) {
       this.io = io;
       inputs = new IntakeIOInputsAutoLogged();
      
   }




}

