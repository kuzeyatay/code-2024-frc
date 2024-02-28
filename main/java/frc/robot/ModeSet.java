package frc.robot;

/** class for setting modes. DONT FORGET WHILE SWITCHING FROM REAL TO SIM */
public final class ModeSet {
  public static final Mode currentMode = Mode.SIM;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
