package frc.robot.subsystems.intake;


import org.littletonrobotics.junction.AutoLog;

import frc.robot.Constants.IntakeConstants;


public interface IntakeIO {
   @AutoLog
   public class IntakeIOInputs {
        public double intakePositionDegrees = IntakeConstants.intakePositionDegrees;

        public double intakeVelocityRPM = 0.0;
        public double rollerTopVelocityRPM = 0.0;
        public double rollerBottomVelocityRPM = 0.0;

        public double intakeVelocityDegreesPerSecond = 0.0;
   }
  
   public default void updateInputs(IntakeIOInputs inputs) {
   };
      
   public default void setRollerTopVolts(double volts) {
   };


   public default void setRollerBottomVolts(double volts) {
   };


   public default void setIntakeVolts(double volts) {
   };


   public default void setTargetRollerTopVolts(double volts) {
   };


   public default void setTargetRollerBottomVolts(double volts) {
   };


   public default void setTargetIntakePositionDegrees(double degrees) {
   };


}
