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
        assertEquals(25.0, Constants.Shooter.getShooterTargetRPSFromTA(1.0));
        assertEquals(35.0, Constants.Shooter.getShooterTargetRPSFromTA(0.7));
        assertEquals(55.0, Constants.Shooter.getShooterTargetRPSFromTA(0.4));
        assertEquals(60.0, Constants.Shooter.getShooterTargetRPSFromTA(0.2));
        // TA=0 is treated as no target and returns 0 (see noTargetReturnsZero)
    }

    @Test
    void interpolationBetweenBands() {
        // 0.3 is halfway between 0.2 (60) and 0.4 (55) → 57.5
        double rps = Constants.Shooter.getShooterTargetRPSFromTA(0.3);
        assertEquals(57.5, rps, 0.01);
        // 0.55 is halfway between 0.4 (55) and 0.7 (35) → 45
        rps = Constants.Shooter.getShooterTargetRPSFromTA(0.55);
        assertEquals(45.0, rps, 0.01);
    }

    @Test
    void taClampedToValidRange() {
        // TA=0.01 interpolates between 0→70 and 0.2→60: 70 - (0.01/0.2)*10 = 69.5
        assertEquals(69.5, Constants.Shooter.getShooterTargetRPSFromTA(0.01), 0.01);
        // 0.99 interpolates between 0.7 (35) and 1.0 (25) → ~25.3
        assertEquals(25.33, Constants.Shooter.getShooterTargetRPSFromTA(0.99), 0.02);
        assertEquals(25.0, Constants.Shooter.getShooterTargetRPSFromTA(1.5));
    }
}
