package org.supurdueper.robot2025;

public enum CanId {

    // Drive
    TALONFX_DRIVE_FL(1, "canivore"),
    TALONFX_STEER_FL(2, "canivore"),
    TALONFX_DRIVE_FR(3, "canivore"),
    TALONFX_STEER_FR(4, "canivore"),
    TALONFX_DRIVE_BL(5, "canivore"),
    TALONFX_STEER_BL(6, "canivore"),
    TALONFX_DRIVE_BR(7, "canivore"),
    TALONFX_STEER_BR(8, "canivore"),
    CANCODER_STEER_FL(21, "canivore"),
    CANCODER_STEER_FR(22, "canivore"),
    CANCODER_STEER_BL(23, "canivore"),
    CANCODER_STEER_BR(24, "canivore"),
    PIGEON(25, "canivore"),

    // Elevator
    TALONFX_ELEVATOR_LEADER(9, "rio"),
    TALONFX_ELEVATOR_FOLLOWER(10, "rio"),

    // Climber
    TALONFX_CLIMBER_LEADER(11, "rio"),
    TALONFX_CLIMBER_FOLLOWER(12, "rio"),
    TALONFX_CLIMBER_GRAB(13, "rio"),

    // Funnel
    TALONFX_FUNNEL(14, "rio"),
    TALONFX_FUNNEL_TILT(15, "rio"),
    CANCODER_FUNNEL_TILT(27, "rio"),

    // Wrist
    TALONFX_WRIST(16, "rio"),
    CANCODER_WRIST(26, "rio"),

    TALONFX_CORAL(17, "rio"),
    TALONFX_ALGAE(18, "rio"),

    CANRANGE_CORAL(28, "rio");

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
