package plugin.trackmate.detector;

/**
 * A class to store key names for parameters of the current {@link SpotDetector}
 * s.
 */
public class ThresholdDetectorKeys
{
	
	/**
	 * The key identifying the parameter that sets the threshold method for the threshold-based
	 * detector. Expected values are {@link String}s.
	 * <p>
	 * Currently used by:
	 * <ul>
	 * <li>{@link ThresholdDetector}
	 * </ul>
	 */
	public static final String KEY_THRESHOLD_METHOD = "THRESHOLD_METHOD";

	/** A default value for the {@link #KEY_THRESHOLD_METHOD} parameter. */
	public static final String DEFAULT_THRESHOLD_METHOD = "Default";
	
	/**
	 * The key identifying the parameter that sets whether the threshold for the
	 * ThresholdDetector should be above or below.
	 * <p>
	 * Currently used by:
	 * <ul>
	 * <li>{@link ThresholdDetector}
	 * </ul>
	 */
	public static final String KEY_ABOVE = "ABOVE";

	/** A default value for the {@link #KEY_THRESHOLD} parameter. */
	public static final boolean DEFAULT_ABOVE = true;

	/**
	 * The key identifying the parameter setting whether we output information helpful for debugging.
	 * <p>
	 * Currently used by:
	 * <ul>
	 * <li> {@link ThresholdDetector}
	 * </ul>
	 */
	public static final String KEY_DEBUG_MODE = "DEBUG_MODE";

	/** A default value for the {@link #KEY_DEBUG} parameter. */
	public static final boolean DEFAULT_DEBUG_MODE = false;
	
	/**
	 * The key identifying the parameter setting that determines where debug output is saved.
	 * <p>
	 * Currently used by:
	 * <ul>
	 * <li> {@link ThresholdDetector}
	 * </ul>
	 */
	public static final String KEY_DEBUG_OUTPATH = "DEBUG_OUTPATH";

	/** A default value for the {@link #KEY_DEBUG_OUTPATH} parameter. */
	public static final String DEFAULT_DEBUG_OUTPATH = "/tmp";
	
}
