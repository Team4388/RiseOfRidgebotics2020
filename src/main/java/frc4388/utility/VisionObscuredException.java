package frc4388.utility;

/** Exception that occurs if the limelight can't see enough points
 * @author Daniel Thomas McGrath
 */
public class VisionObscuredException extends Exception {
    /**
     * Creates new VisionObscuredException with error text 'null'
     */
    public VisionObscuredException() {
        super("Unable to see sufficient vision points");
    }

    /** Creates new VisionObscuredException with error text message
     * 
     * @param message Error text message
     */
    public VisionObscuredException(String message) {
        super(message);
    }

    /** Creates new VisionObscuredException with error text message and detailed stack trace
     * 
     * @param message Error text message
     * @param cause Root cause of error
     */
    public VisionObscuredException(String message, Throwable cause) {
        super(message, cause);
    }

    /** Creates new VisionObscuredException with error text 'null' and detailed stack trace
     * 
     * @param cause Root cause of error
     */
    public VisionObscuredException(Throwable cause) {
        super("Unable to see sufficient vision points", cause);
    }
}
