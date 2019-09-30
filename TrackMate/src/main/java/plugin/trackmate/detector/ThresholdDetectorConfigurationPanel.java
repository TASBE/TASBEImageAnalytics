package plugin.trackmate.detector;

import static fiji.plugin.trackmate.detection.DetectorKeys.KEY_THRESHOLD;
import static plugin.trackmate.detector.ThresholdDetectorKeys.KEY_THRESHOLD_METHOD;
import static plugin.trackmate.detector.ThresholdDetectorKeys.DEFAULT_THRESHOLD_METHOD;
import static plugin.trackmate.detector.ThresholdDetectorKeys.KEY_ABOVE;
import static plugin.trackmate.detector.ThresholdDetectorKeys.DEFAULT_ABOVE;
import static fiji.plugin.trackmate.gui.TrackMateWizard.BIG_FONT;
import static fiji.plugin.trackmate.gui.TrackMateWizard.FONT;
import static fiji.plugin.trackmate.gui.TrackMateWizard.SMALL_FONT;

import java.awt.Font;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;

import javax.swing.ImageIcon;
import javax.swing.JButton;
import javax.swing.JLabel;
import javax.swing.JTextField;
import javax.swing.SpringLayout;
import javax.swing.SwingConstants;

import fiji.plugin.trackmate.Dimension;
import fiji.plugin.trackmate.Logger;
import fiji.plugin.trackmate.Model;
import fiji.plugin.trackmate.Settings;
import fiji.plugin.trackmate.Spot;
import fiji.plugin.trackmate.SpotCollection;
import fiji.plugin.trackmate.TrackMate;
import fiji.plugin.trackmate.detection.LogDetectorFactory;
import fiji.plugin.trackmate.detection.SpotDetectorFactory;
import fiji.plugin.trackmate.gui.ConfigurationPanel;
import fiji.plugin.trackmate.gui.TrackMateGUIController;
import fiji.plugin.trackmate.gui.panels.components.JNumericTextField;
import fiji.plugin.trackmate.util.JLabelLogger;
import fiji.util.NumberParser;
import ij.ImagePlus;

public class ThresholdDetectorConfigurationPanel extends ConfigurationPanel {

	private static final long serialVersionUID = 1L;

	private static final String TOOLTIP_PREVIEW = "<html>" + "Preview the current settings on the current frame." + "<p>" + "Advice: change the settings until you get at least <br>" + "<b>all</b> the spots you want, and do not mind the <br>" + "spurious spots too much. You will get a chance to <br>" + "get rid of them later." + "</html>";

	private static final String TOOLTIP_REFRESH = "<html>" + "Will read the threshold from the current <br>" + "ImagePlus and use its value here.</html>";
	
	private static final String TOOLTIP_THRESHOLD = "<html>" + "Set the threshold used to create a binary image.<br>A value of 0 will cause the threshold to be computed automatically.</html>";
	
	private static final String TOOLTIP_THRESHOLD_METHOD = "<html>" + "Set the threshold method used <br>" + "to create a binary image, if a threshold value of 0 is passed.</html>";
	
	private static final String TOOLTIP_ABOVE = "<html>" + "If true, black objects on white background <br>" + "otherwise white objects on black background.</html>";

	private static final ImageIcon ICON_REFRESH = new ImageIcon( TrackMateGUIController.class.getResource( "images/arrow_refresh_small.png" ) );

	private static final ImageIcon ICON_PREVIEW = new ImageIcon( TrackMateGUIController.class.getResource( "images/flag_checked.png" ) );

	private JLabel jLabel1;

	protected JLabel jLabelSegmenterName;

	protected JButton jButtonRefresh;

	protected JButton btnPreview;

	protected JTextField jTextFieldThreshold;

	protected JLabel jLabelThreshold;
	
	protected JTextField jTextFieldThresholdMethod;
	
	protected JLabel jLabelThresholdMethod;
	
	protected JTextField jTextFieldThresholdAbove;
	
	protected JLabel jLabelThresholdAbove;

	protected JLabel jLabelHelpText;

	protected final String infoText;

	protected final String spaceUnits;

	protected final String detectorName;

	protected final ImagePlus imp;

	protected final Model model;

	private Logger localLogger;

	/** The layout in charge of laying out this panel content. */
	protected SpringLayout layout;

	protected final Settings settings;

	/*
	 * CONSTRUCTOR
	 */

	/**
	 * Creates a new {@link LogDetectorConfigurationPanel}, a GUI able to
	 * configure settings suitable to {@link LogDetectorFactory} and derived
	 * implementations.
	 *
	 * @param settings
	 *            the {@link Settings} object to get the source image from as
	 *            well as physical calibration date and target interval.
	 * @param model
	 *            the {@link Model} that will be fed with the preview results.
	 *            It is the responsibility of the views registered to listen to
	 *            model change to display the preview results.
	 * @param infoText
	 *            the detector info text, will be displayed on the panel.
	 * @param detectorName
	 *            the detector name, will be displayed on the panel.
	 */
	public ThresholdDetectorConfigurationPanel( final Settings settings, final Model model, final String infoText, final String detectorName )
	{
		this.settings = settings;
		this.imp = settings.imp;
		this.infoText = infoText;
		this.detectorName = detectorName;
		this.model = model;
		this.spaceUnits = model.getSpaceUnits();
		initGUI();
	}

	/*
	 * METHODS
	 */

	@Override
	public Map< String, Object > getSettings()
	{
		final HashMap< String, Object > lSettings = new HashMap< >( 5 );
		final double threshold = NumberParser.parseDouble( jTextFieldThreshold.getText() );
		final String thresholdMethod = jTextFieldThresholdMethod.getText();
		final boolean thresholdAbove = Boolean.parseBoolean(jTextFieldThresholdAbove.getText());
		lSettings.put( KEY_THRESHOLD, threshold );
		lSettings.put( KEY_THRESHOLD_METHOD, thresholdMethod );
		lSettings.put( KEY_ABOVE, thresholdAbove );
		return lSettings;
	}

	@Override
	public void setSettings( final Map< String, Object > settings )
	{
		jTextFieldThreshold.setText( "" + settings.get( KEY_THRESHOLD ) );
		if (settings.containsKey(KEY_THRESHOLD_METHOD)) {
			jTextFieldThresholdMethod.setText("" + settings.get( KEY_THRESHOLD_METHOD));
		} else {
			jTextFieldThresholdMethod.setText(DEFAULT_THRESHOLD_METHOD);
		}
		if (settings.containsKey(KEY_ABOVE)) {
			jTextFieldThresholdAbove.setText("" + settings.get( KEY_ABOVE));
		} else {
			jTextFieldThresholdAbove.setText(Boolean.toString(DEFAULT_ABOVE));
		}
		
		// The ThresholdDetector adds a bunch of features from the ParticleAnalyzer
		// We need to declare all of these features
		final String[] pa_features = {"Area", "Percent Area", "Mean", "StdDev", "Mode", "Min", 
		                              "Max", "X", "Y", "XM", "YM", "Perim.", 
		                              "BX", "BY", "Width", "Height", "Major", 
		                              "Minor", "Angle", "Circ.", "Feret", 
		                              "IntDen", "Median", "Skew", "Kurt", 
		                              "RawIntDen", "FeretX", "FeretY", 
		                              "FeretAngle", "MinFeret", "AR", 
		                              "Round", "Solidity"};

		Collection< String > features = new ArrayList<>( pa_features.length );
		Map< String, String > featureNames = new HashMap< >( pa_features.length );
		Map< String, String > featureShortNames = new HashMap< >( pa_features.length );
		Map< String, Dimension > featureDimensions = new HashMap< >( pa_features.length );
		Map< String, Boolean > isInt = new HashMap< >( pa_features.length );
		for (String feature : pa_features) {
			features.add(feature);
			featureNames.put(feature, feature);
			featureShortNames.put(feature, feature);
			featureDimensions.put(feature, Dimension.STRING);
			isInt.put(feature, false);
		}
		if (model != null) {
			System.out.println("Test test test test test");
			System.out.println("Test test test test test");
			System.out.println("Test test test test test");
			System.out.println("Test test test test test");
			model.getFeatureModel().declareSpotFeatures(features, featureNames, featureShortNames, featureDimensions, isInt);
		}
	}

	/**
	 * Returns a new instance of the {@link SpotDetectorFactory} that this
	 * configuration panels configures. The new instance will in turn be used
	 * for the preview mechanism. Therefore, classes extending this class are
	 * advised to return a suitable implementation of the factory.
	 * 
	 * @return a new {@link SpotDetectorFactory}.
	 */
	@SuppressWarnings( "rawtypes" )
	protected SpotDetectorFactory< ? > getDetectorFactory()
	{
		return new ThresholdDetectorFactory();
	}

	/*
	 * PRIVATE METHODS
	 */

	/**
	 * Launch detection on the current frame.
	 */
	protected void preview()
	{
		btnPreview.setEnabled( false );
		new Thread( "TrackMate preview detection thread" )
		{
			@Override
			public void run()
			{
				final Settings lSettings = new Settings();
				lSettings.setFrom( imp );
				final int frame = imp.getFrame() - 1;
				lSettings.tstart = frame;
				lSettings.tend = frame;

				lSettings.detectorFactory = getDetectorFactory();
				lSettings.detectorSettings = getSettings();

				final TrackMate trackmate = new TrackMate( lSettings );
				trackmate.getModel().setLogger( localLogger );

				final boolean detectionOk = trackmate.execDetection();
				if ( !detectionOk )
				{
					localLogger.error( trackmate.getErrorMessage() );
					return;
				}
				localLogger.log( "Found " + trackmate.getModel().getSpots().getNSpots( false ) + " spots." );

				// Wrap new spots in a list.
				final SpotCollection newspots = trackmate.getModel().getSpots();
				final Iterator< Spot > it = newspots.iterator( frame, false );
				final ArrayList< Spot > spotsToCopy = new ArrayList< >( newspots.getNSpots( frame, false ) );
				while ( it.hasNext() )
				{
					spotsToCopy.add( it.next() );
				}
				// Pass new spot list to model.
				model.getSpots().put( frame, spotsToCopy );
				// Make them visible
				for ( final Spot spot : spotsToCopy )
				{
					spot.putFeature( SpotCollection.VISIBLITY, SpotCollection.ONE );
				}
				// Generate event for listener to reflect changes.
				model.setSpots( model.getSpots(), true );

				btnPreview.setEnabled( true );

			}
		}.start();
	}

	/**
	 * Fill the text fields with parameters grabbed from stored ImagePlus.
	 */
	private void refresh()
	{
		if ( null == imp ) {
			return;
		}
		double threshold = imp.getProcessor().getMinThreshold();
		if ( threshold < 0 )
		{
			threshold = 0;
		}
		jTextFieldThreshold.setText( String.format( "%.0f", threshold ) );
	}

	protected void initGUI()
	{
		try
		{
			this.setPreferredSize( new java.awt.Dimension( 300, 461 ) );
			layout = new SpringLayout();
			setLayout( layout );
			{
				jLabel1 = new JLabel();
				layout.putConstraint( SpringLayout.NORTH, jLabel1, 10, SpringLayout.NORTH, this );
				layout.putConstraint( SpringLayout.WEST, jLabel1, 5, SpringLayout.WEST, this );
				layout.putConstraint( SpringLayout.EAST, jLabel1, -5, SpringLayout.EAST, this );
				this.add( jLabel1 );
				jLabel1.setText( "Settings for detector:" );
				jLabel1.setFont( FONT );
			}
			{
				jLabelSegmenterName = new JLabel();
				layout.putConstraint( SpringLayout.NORTH, jLabelSegmenterName, 33, SpringLayout.NORTH, this );
				layout.putConstraint( SpringLayout.WEST, jLabelSegmenterName, 11, SpringLayout.WEST, this );
				layout.putConstraint( SpringLayout.EAST, jLabelSegmenterName, -11, SpringLayout.EAST, this );
				this.add( jLabelSegmenterName );
				jLabelSegmenterName.setFont( BIG_FONT );
				jLabelSegmenterName.setText( detectorName );
			}
			{
				jLabelHelpText = new JLabel();
				layout.putConstraint( SpringLayout.NORTH, jLabelHelpText, 60, SpringLayout.NORTH, this );
				layout.putConstraint( SpringLayout.WEST, jLabelHelpText, 10, SpringLayout.WEST, this );
				layout.putConstraint( SpringLayout.SOUTH, jLabelHelpText, 164, SpringLayout.NORTH, this );
				layout.putConstraint( SpringLayout.EAST, jLabelHelpText, -10, SpringLayout.EAST, this );
				this.add( jLabelHelpText );
				jLabelHelpText.setFont( FONT.deriveFont( Font.ITALIC ) );
				jLabelHelpText.setText( infoText.replace( "<br>", "" ).replace( "<p>", "<p align=\"justify\">" ).replace( "<html>", "<html><p align=\"justify\">" ) );
			}
			{
				jLabelThreshold = new JLabel();
				jLabelThreshold.setToolTipText(TOOLTIP_THRESHOLD);
				layout.putConstraint( SpringLayout.NORTH, jLabelThreshold, 247, SpringLayout.NORTH, this );
				layout.putConstraint( SpringLayout.WEST, jLabelThreshold, 16, SpringLayout.WEST, this );
				this.add( jLabelThreshold );
				jLabelThreshold.setText( "Threshold:" );
				jLabelThreshold.setFont( FONT );
			}
			{
				jTextFieldThreshold = new JNumericTextField();
				jTextFieldThreshold.setToolTipText(TOOLTIP_THRESHOLD);		
				layout.putConstraint( SpringLayout.EAST, jLabelThreshold, -6, SpringLayout.WEST, jTextFieldThreshold );
				layout.putConstraint( SpringLayout.WEST, jTextFieldThreshold, 168, SpringLayout.WEST, this );
				layout.putConstraint( SpringLayout.NORTH, jTextFieldThreshold, 247, SpringLayout.NORTH, this );
				layout.putConstraint( SpringLayout.SOUTH, jTextFieldThreshold, 263, SpringLayout.NORTH, this );
				jTextFieldThreshold.setHorizontalAlignment( SwingConstants.CENTER );
				jTextFieldThreshold.setColumns( 6 );
				jTextFieldThreshold.setText( "0" );
				this.add( jTextFieldThreshold );
				jTextFieldThreshold.setFont( FONT );
			}
			{
				jLabelThresholdMethod = new JLabel();
				jLabelThresholdMethod.setToolTipText(TOOLTIP_THRESHOLD_METHOD);
				layout.putConstraint( SpringLayout.NORTH, jLabelThresholdMethod, 263, SpringLayout.NORTH, this );
				layout.putConstraint( SpringLayout.WEST, jLabelThresholdMethod, 16, SpringLayout.WEST, this );
				this.add( jLabelThresholdMethod );
				jLabelThresholdMethod.setText( "Threshold Method:" );
				jLabelThresholdMethod.setFont( FONT );
			}
			{
				jTextFieldThresholdMethod = new JTextField();
				jTextFieldThresholdMethod.setToolTipText(TOOLTIP_THRESHOLD_METHOD);		
				layout.putConstraint( SpringLayout.EAST, jLabelThresholdMethod, -6, SpringLayout.WEST, jTextFieldThresholdMethod );
				layout.putConstraint( SpringLayout.WEST, jTextFieldThresholdMethod, 168, SpringLayout.WEST, this );
				layout.putConstraint( SpringLayout.NORTH, jTextFieldThresholdMethod, 263, SpringLayout.NORTH, this );
				layout.putConstraint( SpringLayout.SOUTH, jTextFieldThresholdMethod, 279, SpringLayout.NORTH, this );
				jTextFieldThresholdMethod.setHorizontalAlignment( SwingConstants.CENTER );
				jTextFieldThresholdMethod.setColumns( 10 );
				jTextFieldThresholdMethod.setText( "Default" );
				this.add( jTextFieldThresholdMethod );
				jTextFieldThresholdMethod.setFont( FONT );
			}
			{
				jLabelThresholdAbove = new JLabel();
				jLabelThresholdAbove.setToolTipText(TOOLTIP_ABOVE);
				layout.putConstraint( SpringLayout.NORTH, jLabelThresholdAbove, 263+16, SpringLayout.NORTH, this );
				layout.putConstraint( SpringLayout.WEST, jLabelThresholdAbove, 16, SpringLayout.WEST, this );
				this.add( jLabelThresholdAbove );
				jLabelThresholdAbove.setText( "White objects, black background:" );
				jLabelThresholdAbove.setFont( FONT );
			}
			{
				jTextFieldThresholdAbove = new JTextField();
				jTextFieldThresholdAbove.setToolTipText(TOOLTIP_ABOVE);		
				layout.putConstraint( SpringLayout.EAST, jLabelThresholdAbove, -6, SpringLayout.WEST, jTextFieldThresholdAbove );
				layout.putConstraint( SpringLayout.WEST, jTextFieldThresholdAbove, 168, SpringLayout.WEST, this );
				layout.putConstraint( SpringLayout.NORTH, jTextFieldThresholdAbove, 263+16, SpringLayout.NORTH, this );
				layout.putConstraint( SpringLayout.SOUTH, jTextFieldThresholdAbove, 279+16, SpringLayout.NORTH, this );
				jTextFieldThresholdAbove.setHorizontalAlignment( SwingConstants.CENTER );
				jTextFieldThresholdAbove.setColumns( 8 );
				jTextFieldThresholdAbove.setText( "True" );
				this.add( jTextFieldThresholdAbove );
				jTextFieldThresholdAbove.setFont( FONT );
			}
			{
				jButtonRefresh = new JButton( "Refresh treshold", ICON_REFRESH );
				layout.putConstraint( SpringLayout.NORTH, jButtonRefresh, 370, SpringLayout.NORTH, this );
				layout.putConstraint( SpringLayout.WEST, jButtonRefresh, 11, SpringLayout.WEST, this );
				layout.putConstraint( SpringLayout.SOUTH, jButtonRefresh, 395, SpringLayout.NORTH, this );
				layout.putConstraint( SpringLayout.EAST, jButtonRefresh, 131, SpringLayout.WEST, this );
				// this.add( jButtonRefresh );
				jButtonRefresh.setToolTipText( TOOLTIP_REFRESH );
				jButtonRefresh.setFont( SMALL_FONT );
				jButtonRefresh.addActionListener( new ActionListener()
				{
					@Override
					public void actionPerformed( final ActionEvent e )
					{
						refresh();
					}
				} );
			}
			{
				btnPreview = new JButton( "Preview", ICON_PREVIEW );
				layout.putConstraint( SpringLayout.NORTH, btnPreview, 370, SpringLayout.NORTH, this );
				layout.putConstraint( SpringLayout.WEST, btnPreview, -141, SpringLayout.EAST, this );
				layout.putConstraint( SpringLayout.SOUTH, btnPreview, 395, SpringLayout.NORTH, this );
				layout.putConstraint( SpringLayout.EAST, btnPreview, -10, SpringLayout.EAST, this );
				btnPreview.setToolTipText( TOOLTIP_PREVIEW );
				this.add( btnPreview );
				btnPreview.setFont( SMALL_FONT );
				btnPreview.addActionListener( new ActionListener()
				{
					@Override
					public void actionPerformed( final ActionEvent e )
					{
						preview();
					}
				} );
			}

			{
				final JLabelLogger labelLogger = new JLabelLogger();
				layout.putConstraint( SpringLayout.NORTH, labelLogger, 407, SpringLayout.NORTH, this );
				layout.putConstraint( SpringLayout.WEST, labelLogger, 10, SpringLayout.WEST, this );
				layout.putConstraint( SpringLayout.SOUTH, labelLogger, 431, SpringLayout.NORTH, this );
				layout.putConstraint( SpringLayout.EAST, labelLogger, -10, SpringLayout.EAST, this );
				add( labelLogger );
				localLogger = labelLogger.getLogger();
			}
		}
		catch ( final Exception e )
		{
			e.printStackTrace();
		}
	}

	@Override
	public void clean()
	{}

}
