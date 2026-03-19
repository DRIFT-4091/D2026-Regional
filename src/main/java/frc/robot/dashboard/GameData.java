package frc.robot.dashboard;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Reads the FMS game data string each loop and publishes hub active/inactive status to
 * NetworkTables for display in Elastic.
 *
 * 2026 REBUILT game data format (update parseHubActive() if the manual differs):
 *   First character 'A' -> alliance hub is ACTIVE
 *   Anything else / empty string -> alliance hub is INACTIVE
 *
 * NetworkTables path:  GameData/hubActive  (boolean)
 *                      GameData/hubStatus  (string: "ACTIVE" | "INACTIVE")
 */
public class GameData {

    private final BooleanPublisher hubActivePublisher;
    private final StringPublisher  hubStatusPublisher;

    public GameData() {
        var table = NetworkTableInstance.getDefault().getTable("GameData");
        hubActivePublisher = table.getBooleanTopic("hubActive").publish();
        hubStatusPublisher = table.getStringTopic("hubStatus").publish();
    }

    /** Call from robotPeriodic() to keep the dashboard values current. */
    public void update() {
        boolean active = isHubActive();
        hubActivePublisher.set(active);
        hubStatusPublisher.set(active ? "ACTIVE" : "INACTIVE");
    }

    /**
     * Returns true when the FMS game data string signals that the alliance hub is active.
     *
     * 2026 REBUILT: FMS pushes a single-character string.
     *   'A' = hub active, 'I' (or empty before match start) = hub inactive.
     */
    public static boolean isHubActive() {
        String msg = DriverStation.getGameSpecificMessage();
        return msg != null && !msg.isEmpty() && msg.charAt(0) == 'A';
    }
}
