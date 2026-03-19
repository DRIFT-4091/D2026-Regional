package frc.robot.assist;

/** Driver-selectable alignment target type. Chosen on the Elastic Dashboard before pressing RB. */
public enum AlignTarget {
    HUB("Hub"),
    TRENCH("Trench"),
    HUMAN_PLAYER("Human Player");

    public final String label;

    AlignTarget(String label) {
        this.label = label;
    }
}
