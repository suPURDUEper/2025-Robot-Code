package org.supurdueper.robot2025;

public enum CanId {

    // Drive
    TALONFX_DRIVE_FL(1, "canivore"),
    TALONFX_STEER_FL(2, "canivore"),
    TALONFX_DRIVE_FR(3, "canivore"),
    TALONFX_STEER_FR(4, "canivore"),
    TALONFX_DRIVE_BL(5, "canivore"),
    CANCODER_STEER_FL(0, "canivore"),
    CANCODER_STEER_FR(0, "canivore"),
    CANCODER_STEER_BL(0, "canivore"),
    CANCODER_STEER_BR(0, "canivore"),

    // Elevator
    TALONFX_ELEVATOR_LEADER(9, "canivore"),
    TALONFX_ELEVATOR_FOLLOWER(10, "canivore"),

    // Climber
    TALONFX_CLIMBER_LEADER(11, "canivore"),
    TALONFX_CLIMBER_FOLLOWER(12, "canivore"),
    TALONFX_CLIMBER_GRAB(13, "canivore"),

    // Funnel
    TALONFX_FUNNEL(14, "canivore"),
    TALONFX_FUNNEL_TILT(15, "canivore"),
    CANCODER_FUNNEL_TILT(0, "canivore"),

    // Wrist
    TALONFX_WRIST(0, "rio"),
    CANCODER_WRIST(0, "rio"),

    TALONFX_CORAL(0, "rio"),
    TALONFX_ALGAE(0, "rio");

    private final int mDeviceNumber;
    private final String mBus;

    CanId(int mDeviceNumber, String mBus) {
        this.mDeviceNumber = mDeviceNumber;
        this.mBus = mBus;
    }

    public int getDeviceNumber() {
        return mDeviceNumber;
    }

    public String getBus() {
        return mBus;
    }
}
