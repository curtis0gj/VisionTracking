package vision;

import java.sql.Blob;
import java.util.ArrayList;
import java.util.Iterator;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;

import edu.wpi.first.wpilibj.networktables.NetworkTable;

public class TowerTracker {
	static {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
		NetworkTable.setClientMode();
		NetworkTable.setIPAddress("roboRIO-5033-FRC.local");
	}

	static NetworkTable table;

	public static final Scalar RED = new Scalar(0, 0, 255), BLUE = new Scalar(255, 0, 0), GREEN = new Scalar(0, 255, 0),
			BLACK = new Scalar(0, 0, 0), YELLOW = new Scalar(0, 255, 255),

			LOWER_BOUNDS = new Scalar(62, 122, 75), UPPER_BOUNDS = new Scalar(108, 255, 150);
	// 58, 0, 109 : 93, 255, 240 : 75
	public static final Size resize = new Size(320, 240);

	public static VideoCapture videoCapture;
	public static Mat matOriginal, matHSV, matThresh, clusters, matHeirarchy;

	public static final int TOP_TARGET_HEIGHT = 97;
	// 12 inches on robot.
	public static final int TOP_CAMERA_HEIGHT = 5;
	public static final double VERTICAL_FOV = 51;
	public static final double HORIZONTAL_FOV = 67;
	// 15 degrees on robot.
	public static final double CAMERA_ANGLE = 30;

	public static boolean shouldRun = true;
	long startTime = 0;
	long endTime = 0;

	public static void main(String[] args) {
		NetworkTable.setClientMode();
		// roborio-TEAM-frc.local
		NetworkTable.setIPAddress("10.50.33.75");
		table = NetworkTable.getTable("SmartDashboard");

		while (!table.isConnected()) {
			try {
				System.out.println("no network connection");
				Thread.sleep(500);
			} catch (InterruptedException ex) {
				Thread.currentThread().interrupt();
			}
		}

		matOriginal = new Mat();
		matHSV = new Mat();
		matThresh = new Mat();
		clusters = new Mat();
		matHeirarchy = new Mat();

		videoCapture = new VideoCapture();
		videoCapture.open("http://10.50.33.29/mjpg/video.mjpg");
		while (!videoCapture.isOpened()) {
			try {
				System.out.println("no camera connection");
				Thread.sleep(500);
			} catch (InterruptedException ex) {
				Thread.currentThread().interrupt();
			}
		}

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
		videoCapture.release();
		System.exit(0);
	}

	public static void processImage() {
		ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();
		double x, y, targetX, targetY, distance, azimuth;

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

			table.putNumber("distance", distance);
			table.putNumber("azimuth", azimuth);
			System.out.println("Distance: " + distance + ", Azimuth: " + azimuth);
		} else {
			table.putNumber("azimuth", -1);
			System.out.println("Target Lost");
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
