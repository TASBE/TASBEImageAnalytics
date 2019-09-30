package plugin.trackmate.detector;

import static fiji.plugin.trackmate.detection.DetectorKeys.KEY_THRESHOLD;
import static fiji.plugin.trackmate.detection.DetectorKeys.KEY_TARGET_CHANNEL;
import static plugin.trackmate.detector.ThresholdDetectorKeys.KEY_ABOVE;
import static plugin.trackmate.detector.ThresholdDetectorKeys.DEFAULT_ABOVE;
import static plugin.trackmate.detector.ThresholdDetectorKeys.KEY_THRESHOLD_METHOD;
import static plugin.trackmate.detector.ThresholdDetectorKeys.DEFAULT_THRESHOLD_METHOD;
import static plugin.trackmate.detector.ThresholdDetectorKeys.KEY_DEBUG_MODE;
import static plugin.trackmate.detector.ThresholdDetectorKeys.DEFAULT_DEBUG_MODE;
import static plugin.trackmate.detector.ThresholdDetectorKeys.KEY_DEBUG_OUTPATH;
import static fiji.plugin.trackmate.util.TMUtils.checkMapKeys;
import static fiji.plugin.trackmate.util.TMUtils.checkParameter;
import static fiji.plugin.trackmate.io.IOUtils.readBooleanAttribute;
import static fiji.plugin.trackmate.io.IOUtils.readDoubleAttribute;
import static fiji.plugin.trackmate.io.IOUtils.writeTargetChannel;
import static fiji.plugin.trackmate.io.IOUtils.writeThreshold;
import static fiji.plugin.trackmate.io.IOUtils.writeAttribute;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import javax.swing.ImageIcon;

import net.imagej.ImgPlus;
import net.imglib2.Interval;
import net.imglib2.RandomAccessible;
import net.imglib2.type.NativeType;
import net.imglib2.type.numeric.RealType;
import net.imglib2.type.numeric.integer.UnsignedByteType;
import net.imglib2.type.numeric.integer.UnsignedShortType;
import net.imglib2.view.Views;

import org.scijava.plugin.Plugin;

import fiji.plugin.trackmate.Model;
import fiji.plugin.trackmate.Settings;
import fiji.plugin.trackmate.detection.SpotDetector;
import fiji.plugin.trackmate.detection.SpotDetectorFactory;
import fiji.plugin.trackmate.gui.ConfigurationPanel;
import fiji.plugin.trackmate.util.TMUtils;

import org.jdom2.Element;

@Plugin( type = SpotDetectorFactory.class )
public class ThresholdDetectorFactory< T extends RealType< T > & NativeType< T >> implements SpotDetectorFactory< T >
{

	/*
	 * CONSTANTS
	 */

	/** A string key identifying this factory. */
	public static final String THIS_DETECTOR_KEY = "THRESHOLD_DETECTOR";

	/** The pretty name of the target detector. */
	public static final String THIS_NAME = "Threshold detector";

	/** An html information text. */
	public static final String THIS_INFO_TEXT = "<html>" + "This segmenter is based on an image thresholding <br> " + "to create a binary mask. A threshold value is passed and blobs are detected in the binary mask.  if the threshold parameter is 0, then the threshold is computed automatically. </html>";
	
	protected ImgPlus< T > img;

	protected Map< String, Object > settings;
	
	protected String errorMessage;
	
	/*
	 * METHODS
	 */
	
	protected RandomAccessible< T > prepareFrameImg( final int frame )
	{
		final double[] calibration = TMUtils.getSpatialCalibration( img );
		RandomAccessible< T > imFrame;
		final int cDim = TMUtils.findCAxisIndex( img );
		if ( cDim < 0 )
		{
			imFrame = img;
		}
		else
		{
			// In ImgLib2, dimensions are 0-based.
			final int channel = ( Integer ) settings.get( KEY_TARGET_CHANNEL ) - 1;
			imFrame = Views.hyperSlice( img, cDim, channel );
		}

		int timeDim = TMUtils.findTAxisIndex( img );
		if ( timeDim >= 0 )
		{
			if ( cDim >= 0 && timeDim > cDim )
			{
				timeDim--;
			}
			imFrame = Views.hyperSlice( imFrame, timeDim, frame );
		}

		// In case we have a 1D image.
		if ( img.dimension( 0 ) < 2 )
		{ // Single column image, will be rotated internally.
			calibration[ 0 ] = calibration[ 1 ]; // It gets NaN otherwise
			calibration[ 1 ] = 1;
			imFrame = Views.hyperSlice( imFrame, 0, 0 );
		}
		if ( img.dimension( 1 ) < 2 )
		{ // Single line image
			imFrame = Views.hyperSlice( imFrame, 1, 0 );
		}

		return imFrame;
	}

	@SuppressWarnings("unchecked")
	@Override
	public SpotDetector< T > getDetector( final Interval interval, final int frame )
	{
		T threshold;
		boolean above = DEFAULT_ABOVE;
		if (settings.containsKey(KEY_ABOVE)) {
			above = (Boolean)settings.get( KEY_ABOVE );
		}
		if (img.firstElement() instanceof UnsignedByteType) {
			threshold = (T)(new UnsignedByteType(((Number)settings.get( KEY_THRESHOLD )).intValue()));
		} else if (img.firstElement() instanceof UnsignedShortType) {
			threshold = (T)(new UnsignedShortType(((Number)settings.get( KEY_THRESHOLD )).intValue()));
		} else {
			return null;
		}
		String thresholdMethod = DEFAULT_THRESHOLD_METHOD;
		if (settings.containsKey( KEY_THRESHOLD_METHOD )) {
			thresholdMethod = (String)settings.get( KEY_THRESHOLD_METHOD);
		}
		final RandomAccessible< T > imFrame = prepareFrameImg( frame );
		final double[] calibration = TMUtils.getSpatialCalibration( img );
		final ThresholdDetector< T > detector = new ThresholdDetector<>(imFrame, interval, calibration, threshold, frame, above, thresholdMethod);

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
	public String getKey()
	{
		return THIS_DETECTOR_KEY;
	}

	@Override
	public String getName()
	{
		return THIS_NAME;
	}

	@Override
	public String getInfoText()
	{
		return THIS_INFO_TEXT;
	}

	@Override
	public boolean marshall( final Map< String, Object > lSettings, final Element element )
	{
		final StringBuilder errorHolder = new StringBuilder();
		final boolean ok = writeTargetChannel( lSettings, element, errorHolder ) 
				&& writeThreshold( lSettings, element, errorHolder ) 
				&& writeAttribute( lSettings, element, KEY_ABOVE, Boolean.class, errorHolder )
				&& writeAttribute( lSettings, element, KEY_THRESHOLD_METHOD, String.class, errorHolder );;
		if ( !ok )
		{
			errorMessage = errorHolder.toString();
		}
		return ok;
	}

	@Override
	public boolean unmarshall( final Element element, final Map< String, Object > lSettings )
	{
		lSettings.clear();
		final StringBuilder errorHolder = new StringBuilder();
		boolean ok = true;
		ok = ok & readDoubleAttribute( element, lSettings, KEY_THRESHOLD, errorHolder );
		ok = ok & readBooleanAttribute( element, lSettings, KEY_ABOVE, errorHolder );
		// Reading string attributes is unsupported, apparently
		//ok = ok & readStringAttribute( element, lSettings, KEY_THRESHOLD_METHOD, errorHolder );
		if ( !ok )
		{
			errorMessage = errorHolder.toString();
			return false;
		}
		return checkSettings( lSettings );
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
		final List< String > optionalKeys = new ArrayList<>();
		optionalKeys.add( KEY_ABOVE );
		optionalKeys.add( KEY_THRESHOLD_METHOD );
		optionalKeys.add( KEY_DEBUG_MODE );
		optionalKeys.add( KEY_DEBUG_OUTPATH );
		ok = ok & checkMapKeys( lSettings, mandatoryKeys, optionalKeys, errorHolder );
		if ( !ok )
		{
			errorMessage = errorHolder.toString();
		}
		return ok;
	}

	@Override
	public ImageIcon getIcon() {
		return null;
	}

	@Override
	public Map<String, Object> getDefaultSettings() {
		final Map< String, Object > lSettings = new HashMap<>();
		lSettings.put( KEY_THRESHOLD_METHOD, DEFAULT_THRESHOLD_METHOD );
		lSettings.put( KEY_THRESHOLD, 0 );
		lSettings.put( KEY_ABOVE, DEFAULT_ABOVE );
		lSettings.put( KEY_DEBUG_MODE, DEFAULT_DEBUG_MODE );
		return lSettings;
	}

	@Override
	public String getErrorMessage() {
		return errorMessage;
	}

	@Override
	public boolean setTarget( final ImgPlus< T > img, final Map< String, Object > settings ) {
		this.img = img;
		this.settings = settings;
		return checkSettings( settings );
	}

}
