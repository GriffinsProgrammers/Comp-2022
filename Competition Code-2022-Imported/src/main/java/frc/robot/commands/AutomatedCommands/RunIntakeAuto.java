package frc.robot.commands.AutomatedCommands;

import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunIntakeAuto extends CommandBase {
  boolean run;
  Intake intake;
  private long startTime;
  private double time;

  public RunIntakeAuto(Intake intake, boolean run, double time) {
    this.intake = intake;
    this.run = run;
    this.time = time;
    addRequirements(intake);
  }

  public void initialize() {
    startTime = System.currentTimeMillis();
    if (run) {
      intake.pistonForward();
      intake.intake();
    } else if (!run) {
      intake.pistonReverse();
      intake.off();
    }
  }

  public void execute() {}

  public void end(boolean interrupted) {
    System.out.println("Intake done");
  }

  public boolean isFinished() {
    return (System.currentTimeMillis() - startTime > 1000 * time);
  }
}
