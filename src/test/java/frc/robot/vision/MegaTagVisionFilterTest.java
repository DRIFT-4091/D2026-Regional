package frc.robot.vision;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Unit tests for MegaTag vision pose acceptance (MegaTag.isVisionPoseAcceptable).
 */
class MegaTagVisionFilterTest {

    @Test
    void acceptablePosePasses() {
        assertTrue(MegaTag.isVisionPoseAcceptable(1, 50.0, 0.0));
        assertTrue(MegaTag.isVisionPoseAcceptable(2, 50.0, 100.0));
        assertTrue(MegaTag.isVisionPoseAcceptable(1, 99.0, 359.0));
    }

    @Test
    void zeroTagsRejected() {
        assertFalse(MegaTag.isVisionPoseAcceptable(0, 50.0, 0.0));
    }

    @Test
    void highLatencyRejected() {
        assertFalse(MegaTag.isVisionPoseAcceptable(1, 101.0, 0.0));
        assertFalse(MegaTag.isVisionPoseAcceptable(2, 150.0, 0.0));
    }

    @Test
    void highAngularVelocityRejected() {
        assertFalse(MegaTag.isVisionPoseAcceptable(1, 50.0, 361.0));
        assertFalse(MegaTag.isVisionPoseAcceptable(1, 50.0, -400.0));
    }

    @Test
    void boundaryAtMaxAngularVelocity() {
        assertTrue(MegaTag.isVisionPoseAcceptable(1, 50.0, 360.0));
        assertTrue(MegaTag.isVisionPoseAcceptable(1, 50.0, -360.0));
    }

    @Test
    void boundaryAtMaxLatency() {
        assertTrue(MegaTag.isVisionPoseAcceptable(1, 100.0, 0.0));
    }
}
