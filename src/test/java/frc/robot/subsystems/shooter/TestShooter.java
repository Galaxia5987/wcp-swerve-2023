package frc.robot.subsystems.shooter;

import org.junit.Test;
import org.junit.runner.RunWith;
import org.junit.runners.JUnit4;

@RunWith(JUnit4.class)
public class TestShooter {
    private final Shooter shooter = Shooter.getInstance();

    @Test
    public void setPower() {
        shooter.setPower(1);
    }
}
