package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

  private RobotContainer container;

  @Override
  public void robotInit() {
    container = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    var auto = container.getAutonomousCommand();
    if (auto != null) {
      CommandScheduler.getInstance().schedule(auto);
    }/*else {
      container.setAutonomousCommand();
      CommandScheduler.getInstance().schedule(container.getAutonomousCommand());
    }*/
  }

  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();
  }
}
