package plugin.trackmate.detector;

import static fiji.plugin.trackmate.detection.DetectorKeys.KEY_THRESHOLD;
import static fiji.plugin.trackmate.detector.DetectorKeys.KEY_DEBUG_MODE;
import static fiji.plugin.trackmate.detector.DetectorKeys.KEY_DEBUG_OUTPATH;
import static fiji.plugin.trackmate.util.TMUtils.checkMapKeys;
import static fiji.plugin.trackmate.util.TMUtils.checkParameter;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import net.imglib2.Interval;
import net.imglib2.RandomAccessible;
import net.imglib2.type.NativeType;
import net.imglib2.type.numeric.RealType;
import net.imglib2.type.numeric.integer.UnsignedByteType;

import org.scijava.plugin.Plugin;

import fiji.plugin.trackmate.Model;
import fiji.plugin.trackmate.Settings;
import fiji.plugin.trackmate.detection.SpotDetector;
import fiji.plugin.trackmate.detection.SpotDetectorFactory;
import fiji.plugin.trackmate.gui.ConfigurationPanel;
import fiji.plugin.trackmate.util.TMUtils;

@Plugin( type = SpotDetectorFactory.class )
public class LocalThresholdDetectorFactory< T extends RealType< T > & NativeType< T >> extends ThresholdDetectorFactory< T >
{

	/*
	 * CONSTANTS
	 */

	/** A string key identifying this factory. */
	public static final String THIS_DETECTOR_KEY = "LOCAL_THRESHOLD_DETECTOR";

	/** The pretty name of the target detector. */
	public static final String THIS_NAME = "Local Threshold detector";

	/** An html information text. */
	public static final String THIS_INFO_TEXT = "<html>" + "This segmenter is based on local image thresholding <br> " + "to create a binary mask. A threshold value is passed and blobs are detected in the binary mask. </html>";
	
	/*
	 * METHODS
	 */

	@SuppressWarnings("unchecked")
	@Override
	public SpotDetector< T > getDetector( final Interval interval, final int frame )
	{
		T threshold;
		if (img.firstElement() instanceof UnsignedByteType) {
			threshold = (T)(new UnsignedByteType(((Number)settings.get( KEY_THRESHOLD )).intValue()));
		} else {
			return null;
		}

		final RandomAccessible< T > imFrame = prepareFrameImg( frame );
		final double[] calibration = TMUtils.getSpatialCalibration( img );
		final LocalThresholdDetector< T > detector = new LocalThresholdDetector<>(imFrame, interval, calibration, threshold, frame);

		// Enable debug mode, if set
		if (settings.containsKey(KEY_DEBUG_MODE) && (Boolean)settings.get(KEY_DEBUG_MODE)) {
			detector.setDebugMode(true);
			if (!settings.containsKey(KEY_DEBUG_OUTPATH)) {
				errorMessage = "Debug mode set but not debug outpath is specified!";
				return null;
			}
			detector.setDebugPath((String)settings.get(KEY_DEBUG_OUTPATH));
		}

		return detector;
	}

	@Override
	public ConfigurationPanel getDetectorConfigurationPanel( final Settings lSettings, final Model model )
	{
		return new ThresholdDetectorConfigurationPanel( lSettings, model, THIS_INFO_TEXT, THIS_NAME );
	}
	
	@Override
	public boolean checkSettings( final Map< String, Object > lSettings )
	{
		boolean ok = true;
		final StringBuilder errorHolder = new StringBuilder();
		ok = ok & checkParameter( lSettings, KEY_THRESHOLD, Double.class, errorHolder );
		final List< String > mandatoryKeys = new ArrayList<>();
		mandatoryKeys.add( KEY_THRESHOLD );
		ok = ok & checkMapKeys( lSettings, mandatoryKeys, null, errorHolder );
		if ( !ok )
		{
			errorMessage = errorHolder.toString();
		}
		return ok;
	}

}
