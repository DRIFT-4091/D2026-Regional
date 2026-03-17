package frc.robot;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

/**
 * Unit tests for shooter TA → RPS interpolation (Constants.Shooter.getShooterTargetRPSFromTA).
 * TA values are 0–1 (fraction of image area) as returned by Limelight.
 */
class ShooterLookupTest {

    @Test
    void noTargetReturnsZero() {
        assertEquals(0.0, Constants.Shooter.getShooterTargetRPSFromTA(0.0));
        assertEquals(0.0, Constants.Shooter.getShooterTargetRPSFromTA(-0.1));
    }

    @Test
    void exactBandPoints() {
        assertEquals(20.0, Constants.Shooter.getShooterTargetRPSFromTA(1.0));
        assertEquals(40.0, Constants.Shooter.getShooterTargetRPSFromTA(0.7));
        assertEquals(55.0, Constants.Shooter.getShooterTargetRPSFromTA(0.4));
        assertEquals(75.0, Constants.Shooter.getShooterTargetRPSFromTA(0.2));
        // TA=0 is treated as no target and returns 0 (see noTargetReturnsZero)
    }

    @Test
    void interpolationBetweenBands() {
        // 0.3 is halfway between 0.2 (75) and 0.4 (55) → 65
        double rps = Constants.Shooter.getShooterTargetRPSFromTA(0.3);
        assertEquals(65.0, rps, 0.01);
        // 0.55 is halfway between 0.4 (55) and 0.7 (40) → 47.5
        rps = Constants.Shooter.getShooterTargetRPSFromTA(0.55);
        assertEquals(47.5, rps, 0.01);
    }

    @Test
    void taClampedToValidRange() {
        // TA=0.01 interpolates between 0→85 and 0.2→75: 85 - (0.01/0.2)*10 = 84.5
        assertEquals(84.5, Constants.Shooter.getShooterTargetRPSFromTA(0.01), 0.01);
        // 0.99 interpolates between 0.85 (30) and 1.0 (20) → 30 - (0.14/0.15)*10 ≈ 20.67
        assertEquals(20.67, Constants.Shooter.getShooterTargetRPSFromTA(0.99), 0.02);
        assertEquals(20.0, Constants.Shooter.getShooterTargetRPSFromTA(1.5));
    }
}
