package frc.robot;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

/**
 * Unit tests for shooter TA → RPS interpolation (Constants.Shooter.getShooterTargetRPSFromTA).
 * TA values are 0–100 (percentage of image area) as returned by Limelight.
 */
class ShooterLookupTest {

    @Test
    void noTargetReturnsZero() {
        assertEquals(0.0, Constants.Shooter.getShooterTargetRPSFromTA(0.0));
        assertEquals(0.0, Constants.Shooter.getShooterTargetRPSFromTA(-0.1));
    }

    @Test
    void exactBandPoints() {
        assertEquals(25.0, Constants.Shooter.getShooterTargetRPSFromTA(60.0));
        assertEquals(25.0, Constants.Shooter.getShooterTargetRPSFromTA(100.0));
        assertEquals(35.0, Constants.Shooter.getShooterTargetRPSFromTA(40.0));
        assertEquals(45.0, Constants.Shooter.getShooterTargetRPSFromTA(20.0));
    }

    @Test
    void interpolationBetweenBands() {
        // 30 is halfway between 20 (45) and 40 (35) → 40
        double rps = Constants.Shooter.getShooterTargetRPSFromTA(30.0);
        assertEquals(40.0, rps, 0.01);
        // 50 is halfway between 40 (35) and 60 (25) → 30
        rps = Constants.Shooter.getShooterTargetRPSFromTA(50.0);
        assertEquals(30.0, rps, 0.01);
    }

    @Test
    void taClampedToValidRange() {
        // TA=1 interpolates between 0→55 and 20→45: 55 - (1/20)*10 = 54.5
        assertEquals(54.5, Constants.Shooter.getShooterTargetRPSFromTA(1.0), 0.01);
        assertEquals(25.0, Constants.Shooter.getShooterTargetRPSFromTA(99.0));
        assertEquals(25.0, Constants.Shooter.getShooterTargetRPSFromTA(150.0));
    }
}
