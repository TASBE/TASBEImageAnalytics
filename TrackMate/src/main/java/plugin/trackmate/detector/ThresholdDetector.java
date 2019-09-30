package plugin.trackmate.detector;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import fiji.plugin.trackmate.Spot;
import fiji.plugin.trackmate.detection.DetectionUtils;
import fiji.plugin.trackmate.detection.SpotDetector;
import ij.IJ;
import ij.ImagePlus;
import ij.measure.Measurements;
import ij.measure.ResultsTable;
import ij.plugin.filter.Analyzer;
import ij.plugin.filter.ParticleAnalyzer;
import net.imglib2.Interval;
import net.imglib2.RandomAccessible;
import net.imglib2.RandomAccessibleInterval;
import net.imglib2.algorithm.binary.Thresholder;
import net.imglib2.algorithm.neighborhood.RectangleShape;
import net.imglib2.algorithm.neighborhood.Shape;
import net.imglib2.img.Img;
import net.imglib2.img.ImgFactory;
import net.imglib2.img.ImgView;
import net.imglib2.img.display.imagej.ImageJFunctions;
import net.imglib2.type.NativeType;
import net.imglib2.type.logic.BitType;
import net.imglib2.type.numeric.RealType;
import net.imglib2.util.Util;
import net.imglib2.view.Views;

public class ThresholdDetector< T extends RealType< T > & NativeType< T >> implements SpotDetector< T >
{

	/*
	 * FIELDS
	 */
	
	private final static String BASE_ERROR_MESSAGE = "ThresholdDetector: ";

	/** The image to segment. Will not modified. */
	protected RandomAccessible< T > img;

	protected T threshold;
	
	protected boolean above;

	protected String baseErrorMessage;

	protected String errorMessage;

	/** The list of {@link Spot} that will be populated by this detector. */
	protected List< Spot > spots = new ArrayList<>();

	/** The processing time in ms. */
	protected long processingTime;

	protected final Interval interval;
	
	protected final double[] calibration;
	
	boolean debug = false;
	
	String debugPath;
	
	int frame;
	
	protected String thresholdMethod;

	/*
	 * CONSTRUCTOR
	 */

	public ThresholdDetector(final RandomAccessible<T> img, final Interval interval, final double[] calibration,
			final T threshold, int frame, boolean above, String thresholdMethod)
	{
		this.img = img;
		this.calibration = calibration;
		this.interval = DetectionUtils.squeeze( interval );
		this.threshold = threshold;
		this.baseErrorMessage = BASE_ERROR_MESSAGE;
		this.above = above;
		this.frame = frame;
		this.thresholdMethod = thresholdMethod;
	}

	/*
	 * METHODS
	 */
	
	@Override
	public boolean checkInput()
	{
		if ( null == img )
		{
			errorMessage = baseErrorMessage + "Image is null.";
			return false;
		}
		if ( img.numDimensions() > 3 )
		{
			errorMessage = baseErrorMessage + "Image must be 1D, 2D or 3D, got " + img.numDimensions() + "D.";
			return false;
		}
		return true;
	}

	@Override
	public boolean process()
	{

		final long start = System.currentTimeMillis();

		RandomAccessibleInterval< T > view = Views.interval( img, interval );

		final T type = view.randomAccess().get().createVariable();
		final ImgFactory< T > factory = Util.getArrayOrCellImgFactory( view, type );
		Img<T> inputImg = ImgView.wrap(view, factory);

		//System.out.println("Thresholding value: " + threshold);
		T localThreshold = inputImg.firstElement().createVariable();
		if (threshold.getRealDouble() == 0) {
			ImagePlus origImg = ImageJFunctions.wrap(inputImg, "Original");
			origImg.getProcessor().setAutoThreshold(thresholdMethod);
			String desc = "";
			if (isDebug() && debugPath != null) {
				String[] result = debugPath.split(File.separator);
				desc = result[result.length - 2];
			}
			System.out.println("Thresholding (frame, min, max, desc): " + frame + ", " + origImg.getProcessor().getMinThreshold() + ", " + origImg.getProcessor().getMaxThreshold() + ", " + desc);
			localThreshold.setReal(origImg.getProcessor().getMaxThreshold());
		} else {
			localThreshold = threshold;
		}

		Img<BitType> threshImg = Thresholder.threshold(inputImg, localThreshold, above, 1);

		List<Shape> strels = new ArrayList<Shape>();
		strels.add(new RectangleShape(3, false));
		
		//Img<BitType> closedImg = Closing.close(threshImg, strels, 1);

		int minSize = 5;
		double minCircularity = 0.0001;
		ResultsTable table = new ResultsTable();
		int paFlags = ParticleAnalyzer.IN_SITU_SHOW | ParticleAnalyzer.SHOW_MASKS | ParticleAnalyzer.CLEAR_WORKSHEET;
		/*
		int measurements = Measurements.AREA | Measurements.MEAN | Measurements.STD_DEV | Measurements.MODE
				| Measurements.MIN_MAX | Measurements.CENTROID | Measurements.RECT
				| Measurements.INTEGRATED_DENSITY | Measurements.MEDIAN;
		*/
		ParticleAnalyzer pa = new ParticleAnalyzer(paFlags, Measurements.ALL_STATS, table, minSize, Double.POSITIVE_INFINITY, minCircularity, 1.0);
        
		ImagePlus origImg = ImageJFunctions.wrap(inputImg, "Original");
		ImagePlus binaryMask = ImageJFunctions.wrap(threshImg, "Binary Mask");
		binaryMask.getProcessor().invertLut(); // This is necessary for the mask to match expectations within ParticleAnalyzer
		
		//origImg.show("Orig");
		//binaryMask.show("Binary Mask");
		
		Analyzer.setRedirectImage(origImg);
		if (!pa.analyze(binaryMask)) {
			errorMessage = baseErrorMessage + " error with ParticleAnalyzer!";
			return false;
		}
		
		//System.out.println("isDebug: " + isDebug());
		if (isDebug()) {
			//System.out.println("\tgetDebugPath: " + getDebugPath() + "/segMasks_fr" + frame + ".png");
			IJ.save(binaryMask, getDebugPath() + "/segMasks_fr" + frame + ".png");
			
			IJ.save(origImg, getDebugPath() + "/orig_fr" + frame + ".png");
		}
		//binaryMask.show("Binary Mask After");
		//pa.getOutputImage();
		spots = new ArrayList<Spot>();
		
		for (int row = 0; row < table.size(); row++) {
			final double x = table.getValueAsDouble(ResultsTable.X_CENTROID, row ) * calibration[ 0 ];
			final double y = table.getValueAsDouble(ResultsTable.Y_CENTROID, row ) * calibration[ 1 ];
			final double z = 0 * calibration[ 2 ];
			final double quality = table.getValueAsDouble(ResultsTable.AREA, row );
			//final double largestSize = Math.max(table.getValueAsDouble(ResultsTable.ROI_WIDTH, row ), table.getValueAsDouble(ResultsTable.ROI_HEIGHT, row ));
			final double radius = table.getValueAsDouble(ResultsTable.MAJOR, row ) / 2;
			Spot newSpot = new Spot( x, y, z, radius, quality );
			for (String col : table.getHeadings()) {
				// Can't serialize this in XML, so skip it
				if (col.equals("%Area")) {
					newSpot.putFeature("PercentArea", table.getValue(col, row));
				} else {
					newSpot.putFeature(col, table.getValue(col, row));
				}
			}
			spots.add(newSpot);
		}

		final long end = System.currentTimeMillis();
		processingTime = end - start;

		return true;
	}
	
	@Override
	public List< Spot > getResult()
	{
		return spots;
	}

	@Override
	public String getErrorMessage()
	{
		return errorMessage;
	}

	@Override
	public long getProcessingTime()
	{
		return processingTime;
	}

	public boolean isDebug() {
		return debug;
	}

	public void setDebugMode(boolean debug) {
		this.debug = debug;
	}

	public String getDebugPath() {
		return debugPath;
	}

	public void setDebugPath(String debugPath) {
		this.debugPath = debugPath;
	}

}
