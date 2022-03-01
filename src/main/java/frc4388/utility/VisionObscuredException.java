package frc4388.utility;

/** Exception that occurs if the limelight can't see enough points
 * @author Daniel Thomas McGrath
 */
public class VisionObscuredException extends RuntimeException {
    public VisionObscuredException() {
        super();
    }

    public VisionObscuredException(String message) {
        super(message);
    }

    public VisionObscuredException(String message, Throwable cause) {
        super(message, cause);
    }

    public VisionObscuredException(Throwable cause) {
        super(cause);
    }
}