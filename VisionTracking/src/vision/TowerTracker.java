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
	public static Boolean networkConnected = false;

	public static final Scalar RED = new Scalar(0, 0, 255), BLUE = new Scalar(255, 0, 0), GREEN = new Scalar(0, 255, 0),
			BLACK = new Scalar(0, 0, 0), YELLOW = new Scalar(0, 255, 255),

			LOWER_BOUNDS = new Scalar(65, 119, 154), UPPER_BOUNDS = new Scalar(158, 255, 255);

	public static final Size resize = new Size(320, 240);

	public static VideoCapture videoCapture;

	public static Mat matOriginal = new Mat();
	public static Mat matHSV = new Mat();
	public static Mat matThresh = new Mat();
	public static Mat clusters = new Mat();
	public static Mat matHeirarchy = new Mat();

	public static final int TOP_TARGET_HEIGHT = 97;
	public static final int TOP_CAMERA_HEIGHT = 12;
	public static final double VERTICAL_FOV = 51;
	public static final double HORIZONTAL_FOV = 67;
	public static final double CAMERA_ANGLE = 28;

	public static boolean shouldRun = true;
	long startTime = 0;
	long endTime = 0;
	public static int frames;

	public static void main(String[] args) {
		initializeNetworkTables();
		videoCapture = new VideoCapture();
		openVideoCapture();
		mainLoop();
		videoCapture.release();
	}

	private static void initializeNetworkTables() {
		String rioIP = "10.50.33.75";
		NetworkTable.setClientMode();
		NetworkTable.setIPAddress(rioIP);
		table = NetworkTable.getTable("SmartDashboard");

		while (!table.isConnected()) {
			try {
				System.out.println("no network connection");
				Thread.sleep(250);
			} catch (InterruptedException ex) {
				Thread.currentThread().interrupt();
			}
		}
		networkConnected = true;
	}

	private static void openVideoCapture() {
		String camIP = "http://axis-camera.local/mjpg/video.mjpg";
		videoCapture.open(camIP);

		while (!videoCapture.isOpened()) {
			try {
				System.out.println("no camera connection");
				Thread.sleep(250);
				videoCapture.open(camIP);
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

			if (endTime - startTime < 38) {
				try {
					Thread.sleep(50);
				} catch (InterruptedException ex) {
					Thread.currentThread().interrupt();
				}
			}
		}
	}

	public static void processImage() {
		ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();
		double y, targetX, distance, azimuth;

		contours.clear();
		videoCapture.read(matOriginal);
		Imgproc.cvtColor(matOriginal, matHSV, Imgproc.COLOR_BGR2HSV);
		Core.inRange(matHSV, LOWER_BOUNDS, UPPER_BOUNDS, matThresh);
		Imgproc.findContours(matThresh, contours, matHeirarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

		for (MatOfPoint mop : contours) {
			Rect rec = Imgproc.boundingRect(mop);
			Imgproc.rectangle(matOriginal, rec.br(), rec.tl(), RED);
		}

		for (Iterator<MatOfPoint> iterator = contours.iterator(); iterator.hasNext();) {
			MatOfPoint matOfPoint = (MatOfPoint) iterator.next();
			Rect rec = Imgproc.boundingRect(matOfPoint);
			if (rec.height < 15 || rec.width < 15) {
				iterator.remove();
				continue;
			}
			float aspect = (float) rec.width / (float) rec.height;
			if (aspect < 1.0) {
				iterator.remove();
				continue;
			}

			if (rec.y > 100) {
				iterator.remove();
				continue;
			}
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

			String smartDashBoardVisionData = distanceAsString + ":" + azimuthAsString;

			if (networkConnected) {
				table.putNumber("distance", distance);
				table.putNumber("azimuth", azimuth);

				table.putString("distance and azimuth", smartDashBoardVisionData);

				NetworkTable.flush();
			}

			System.out.println("distance: " + distance + " azimuth: " + azimuth);
			frames = 0;
		} else {
			frames += 1;

			if (frames > 4) {
				String targetLost = "3.14:-1";
				if (networkConnected) {
					table.putString("distance and azimuth", targetLost);
					table.putNumber("distance", 0);
					table.putNumber("azimuth", 0);
				}
				System.out.println("target lost");
			} else {
				System.out.println("frames " + frames);
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
