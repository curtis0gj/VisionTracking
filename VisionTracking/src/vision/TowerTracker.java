package vision;

import java.util.ArrayList;
import java.util.Iterator;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;

import edu.wpi.first.wpilibj.networktables.NetworkTable;

public class TowerTracker {
	static {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
	}

	public static NetworkTable table;

	public static final Scalar RED = new Scalar(0, 0, 255), BLUE = new Scalar(255, 0, 0), GREEN = new Scalar(0, 255, 0),
			BLACK = new Scalar(0, 0, 0), YELLOW = new Scalar(0, 255, 255),

			LOWER_BOUNDS = new Scalar(62, 122, 75), UPPER_BOUNDS = new Scalar(108, 255, 150);

	public static final Size resize = new Size(320, 240);

	public static VideoCapture videoCapture;

	public static Mat matOriginal = new Mat();
	public static Mat matHSV = new Mat();
	public static Mat matThresh = new Mat();
	public static Mat clusters = new Mat();
	public static Mat matHeirarchy = new Mat();

	public static final int TOP_TARGET_HEIGHT = 97;
	public static final int TOP_CAMERA_HEIGHT = 5; // 12 inches on the robot.
	public static final double VERTICAL_FOV = 51;
	public static final double HORIZONTAL_FOV = 67;
	public static final double CAMERA_ANGLE = 30; // 15 degrees on the robot.

	public static boolean shouldRun = true;
	long startTime = 0;
	long endTime = 0;

	public static void main(String[] args) {
		initializeNetworkTables();
		openVideoCapture();
		mainLoop();
		videoCapture.release();
	}

	private static void initializeNetworkTables() {
		NetworkTable.setClientMode();
		NetworkTable.setIPAddress("roboRIO-5033-FRC.local"); // 10.50.33.75
		table = NetworkTable.getTable("SmartDashboard");

		while (!table.isConnected()) {
			try {
				System.out.println("no network connection");
				Thread.sleep(500);
			} catch (InterruptedException ex) {
				Thread.currentThread().interrupt();
			}
		}
	}

	private static void openVideoCapture() {
		videoCapture.open("http://10.50.33.29/mjpg/video.mjpg");
		while (!videoCapture.isOpened()) {
			try {
				System.out.println("no camera connection");
				Thread.sleep(500);
			} catch (InterruptedException ex) {
				Thread.currentThread().interrupt();
			}
		}
	}

	private static void mainLoop() {
		while (shouldRun) {
			long startTime = System.currentTimeMillis();

			try {
				processImage();
			} catch (Exception e) {
				e.printStackTrace();
				break;
			}
			long endTime = System.currentTimeMillis();

			if (endTime - startTime < 75) {
				try {
					Thread.sleep(100);
				} catch (InterruptedException ex) {
					Thread.currentThread().interrupt();
				}
			}
		}
	}

	public static void processImage() {
		ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();
		double y, targetX, distance, azimuth;
		int frames = 0;

		contours.clear();
		videoCapture.read(matOriginal);
		Imgproc.cvtColor(matOriginal, matHSV, Imgproc.COLOR_BGR2HSV);
		Core.inRange(matHSV, LOWER_BOUNDS, UPPER_BOUNDS, matThresh);
		Imgproc.findContours(matThresh, contours, matHeirarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

		for (Iterator<MatOfPoint> iterator = contours.iterator(); iterator.hasNext();) {
			MatOfPoint matOfPoint = (MatOfPoint) iterator.next();
			Rect rec = Imgproc.boundingRect(matOfPoint);
			if (rec.height < 25 || rec.width < 25) {
				iterator.remove();
				continue;
			}
			float aspect = (float) rec.width / (float) rec.height;
			if (aspect < 1.0)
				iterator.remove();
		}
		for (MatOfPoint mop : contours) {
			Rect rec = Imgproc.boundingRect(mop);
			Imgproc.rectangle(matOriginal, rec.br(), rec.tl(), BLACK);
		}

		if (contours.size() == 1) {
			Rect rec = Imgproc.boundingRect(contours.get(0));
			y = rec.br().y + rec.height / 2;
			y = -((2 * (y / matOriginal.height())) - 1);
			distance = (TOP_TARGET_HEIGHT - TOP_CAMERA_HEIGHT)
					/ Math.tan((y * VERTICAL_FOV / 2.0 + CAMERA_ANGLE) * Math.PI / 180);

			targetX = rec.tl().x + rec.width / 2;
			targetX = (2 * (targetX / matOriginal.width())) - 1;
			azimuth = normalize360(targetX * HORIZONTAL_FOV / 2.0 + 0);

			String distanceAsString = Double.toString(distance);
			String azimuthAsString = Double.toString(azimuth);

			String visionData = distanceAsString + ":" + azimuthAsString;

			table.putString("distance and azimuth", visionData);
			System.out.println("Distance : Azimuth = " + visionData);

			frames = 0;
		} else {
			frames++;

			if (frames > 15) {
				String targetLost = "3.14:-1";
				table.putString("distance and azimuth", targetLost);
				System.out.println("Target Lost = " + targetLost);
			}
		}
	}

	public static double normalize360(double angle) {
		while (angle >= 360.0) {
			angle -= 360.0;
		}
		while (angle < 0.0) {
			angle += 360.0;
		}
		return angle;
	}
}
