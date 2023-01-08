package frc.robot.limelight;

public enum CamMode {
    VISION(0),
    DRIVERCAM(1);

    private final int mode;

    private CamMode(final int mode) {
        this.mode = mode;
    }

    public int getNetworkTableValue() {
        return mode;
    }

    public static CamMode getFromNetworkTableValue(int value) {
        return CamMode.values()[value];
    }
}
