// Kalibracja.cpp : Defines the entry point for the console application.
//
#include "stdafx.h"
#include <iostream>
#include <sstream>
#include <string>
#include <ctime>
#include <cstdio>

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <fstream>
//#include "popt_pp1.h"

using namespace cv;
using namespace std;

static void help()
{
	cout << "This is a camera calibration sample." << endl
		<< "Usage: camera_calibration [configuration_file -- default ./default.xml]" << endl
		<< "Near the sample file you'll find the configuration file, which has detailed help of "
		"how to edit it.  It may be any OpenCV supported file format XML/YAML." << endl;
}
class Settings
{
public:
	Settings() : goodInput(false) {}
	enum Pattern { NOT_EXISTING, CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };
	enum InputType { INVALID, CAMERA, VIDEO_FILE, IMAGE_LIST };

	void write(FileStorage& fs) const                        //Write serialization for this class
	{
		fs << "{"
			<< "BoardSize_Width" << boardSize.width
			<< "BoardSize_Height" << boardSize.height
			<< "Square_Size" << squareSize
			<< "Calibrate_Pattern" << patternToUse
			<< "Calibrate_NrOfFrameToUse" << nrFrames
			<< "Calibrate_FixAspectRatio" << aspectRatio
			<< "Calibrate_AssumeZeroTangentialDistortion" << calibZeroTangentDist
			<< "Calibrate_FixPrincipalPointAtTheCenter" << calibFixPrincipalPoint

			<< "Write_DetectedFeaturePoints" << writePoints
			<< "Write_extrinsicParameters" << writeExtrinsics
			<< "Write_outputFileName" << outputFileName

			<< "Show_UndistortedImage" << showUndistorsed

			<< "Input_FlipAroundHorizontalAxis" << flipVertical
			<< "Input_Delay" << delay
			<< "Input" << input
			<< "}";
	}
	void read(const FileNode& node)                          //Read serialization for this class
	{
		node["BoardSize_Width"] >> boardSize.width;
		node["BoardSize_Height"] >> boardSize.height;
		node["Calibrate_Pattern"] >> patternToUse;
		node["Square_Size"] >> squareSize;
		node["Calibrate_NrOfFrameToUse"] >> nrFrames;
		node["Calibrate_FixAspectRatio"] >> aspectRatio;
		node["Write_DetectedFeaturePoints"] >> writePoints;
		node["Write_extrinsicParameters"] >> writeExtrinsics;
		node["Write_outputFileName"] >> outputFileName;
		node["Calibrate_AssumeZeroTangentialDistortion"] >> calibZeroTangentDist;
		node["Calibrate_FixPrincipalPointAtTheCenter"] >> calibFixPrincipalPoint;
		node["Calibrate_UseFisheyeModel"] >> useFisheye;
		node["Input_FlipAroundHorizontalAxis"] >> flipVertical;
		node["Show_UndistortedImage"] >> showUndistorsed;
		node["Input"] >> input;
		node["Input_Delay"] >> delay;
		node["Fix_K1"] >> fixK1;
		node["Fix_K2"] >> fixK2;
		node["Fix_K3"] >> fixK3;
		node["Fix_K4"] >> fixK4;
		node["Fix_K5"] >> fixK5;

		validate();
	}
	void validate()
	{
		goodInput = true;
		if (boardSize.width <= 0 || boardSize.height <= 0)
		{
			cerr << "Invalid Board size: " << boardSize.width << " " << boardSize.height << endl;
			goodInput = false;
		}
		if (squareSize <= 10e-6)
		{
			cerr << "Invalid square size " << squareSize << endl;
			goodInput = false;
		}
		if (nrFrames <= 0)
		{
			cerr << "Invalid number of frames " << nrFrames << endl;
			goodInput = false;
		}

		if (input.empty())      // Check for valid input
			inputType = INVALID;
		else
		{
			if (input[0] >= '0' && input[0] <= '9')
			{
				stringstream ss(input);
				ss >> cameraID;
				inputType = CAMERA;
			}
			else
			{
				if (isListOfImages(input) && readStringList(input, imageList))
				{
					inputType = IMAGE_LIST;
					nrFrames = (nrFrames < (int)imageList.size()) ? nrFrames : (int)imageList.size();
				}
				else
					inputType = VIDEO_FILE;
			}
			if (inputType == CAMERA)
				inputCapture.open(cameraID);
			if (inputType == VIDEO_FILE)
				inputCapture.open(input);
			if (inputType != IMAGE_LIST && !inputCapture.isOpened())
				inputType = INVALID;
		}
		if (inputType == INVALID)
		{
			cerr << " Input does not exist: " << input;
			goodInput = false;
		}

		flag = 0;
		if (calibFixPrincipalPoint) flag |= CALIB_FIX_PRINCIPAL_POINT;
		if (calibZeroTangentDist)   flag |= CALIB_ZERO_TANGENT_DIST;
		if (aspectRatio)            flag |= CALIB_FIX_ASPECT_RATIO;
		if (fixK1)                  flag |= CALIB_FIX_K1;
		if (fixK2)                  flag |= CALIB_FIX_K2;
		if (fixK3)                  flag |= CALIB_FIX_K3;
		if (fixK4)                  flag |= CALIB_FIX_K4;
		if (fixK5)                  flag |= CALIB_FIX_K5;

		if (useFisheye) {
			// the fisheye model has its own enum, so overwrite the flags
			flag = fisheye::CALIB_FIX_SKEW | fisheye::CALIB_RECOMPUTE_EXTRINSIC;
			if (fixK1)                   flag |= fisheye::CALIB_FIX_K1;
			if (fixK2)                   flag |= fisheye::CALIB_FIX_K2;
			if (fixK3)                   flag |= fisheye::CALIB_FIX_K3;
			if (fixK4)                   flag |= fisheye::CALIB_FIX_K4;
			if (calibFixPrincipalPoint) flag |= fisheye::CALIB_FIX_PRINCIPAL_POINT;
		}

		calibrationPattern = NOT_EXISTING;
		if (!patternToUse.compare("CHESSBOARD")) calibrationPattern = CHESSBOARD;
		if (!patternToUse.compare("CIRCLES_GRID")) calibrationPattern = CIRCLES_GRID;
		if (!patternToUse.compare("ASYMMETRIC_CIRCLES_GRID")) calibrationPattern = ASYMMETRIC_CIRCLES_GRID;
		if (calibrationPattern == NOT_EXISTING)
		{
			cerr << " Camera calibration mode does not exist: " << patternToUse << endl;
			goodInput = false;
		}
		atImageList = 0;

	}
	Mat nextImage()
	{
		Mat result;
		if (inputCapture.isOpened())
		{
			Mat view0;
			inputCapture >> view0;
			view0.copyTo(result);
		}
		else if (atImageList < imageList.size())
			result = imread(imageList[atImageList++], IMREAD_COLOR);

		return result;
	}

	static bool readStringList(const string& filename, vector<string>& l)
	{
		l.clear();
		FileStorage fs(filename, FileStorage::READ);
		if (!fs.isOpened())
			return false;
		FileNode n = fs.getFirstTopLevelNode();
		if (n.type() != FileNode::SEQ)
			return false;
		FileNodeIterator it = n.begin(), it_end = n.end();
		for (; it != it_end; ++it)
			l.push_back((string)*it);
		return true;
	}

	static bool isListOfImages(const string& filename)
	{
		string s(filename);
		// Look for file extension
		if (s.find(".xml") == string::npos && s.find(".yaml") == string::npos && s.find(".yml") == string::npos)
			return false;
		else
			return true;
	}
public:
	Size boardSize;              // The size of the board -> Number of items by width and height
	Pattern calibrationPattern;  // One of the Chessboard, circles, or asymmetric circle pattern
	float squareSize;            // The size of a square in your defined unit (point, millimeter,etc).
	int nrFrames;                // The number of frames to use from the input for calibration
	float aspectRatio;           // The aspect ratio
	int delay;                   // In case of a video input
	bool writePoints;            // Write detected feature points
	bool writeExtrinsics;        // Write extrinsic parameters
	bool calibZeroTangentDist;   // Assume zero tangential distortion
	bool calibFixPrincipalPoint; // Fix the principal point at the center
	bool flipVertical;           // Flip the captured images around the horizontal axis
	string outputFileName;       // The name of the file where to write
	bool showUndistorsed;        // Show undistorted images after calibration
	string input;                // The input ->
	bool useFisheye;             // use fisheye camera model for calibration
	bool fixK1;                  // fix K1 distortion coefficient
	bool fixK2;                  // fix K2 distortion coefficient
	bool fixK3;                  // fix K3 distortion coefficient
	bool fixK4;                  // fix K4 distortion coefficient
	bool fixK5;                  // fix K5 distortion coefficient

	int cameraID;
	vector<string> imageList;
	size_t atImageList;
	VideoCapture inputCapture;
	InputType inputType;
	bool goodInput;
	int flag;

private:
	string patternToUse;


};

static inline void read(const FileNode& node, Settings& x, const Settings& default_value = Settings())
{
	if (node.empty())
		x = default_value;
	else
		x.read(node);
}

static inline void write(FileStorage& fs, const String&, const Settings& s)
{
	s.write(fs);
}

enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };

bool runCalibrationAndSave(Settings& s, Size imageSize, Mat&  cameraMatrix, Mat& distCoeffs,
	vector<vector<Point2f> > imagePoints);

//program
Mat img, gray;
Size im_size;


int main(int argc, char* argv[])
{
	help();

	//! [file_read]
	Settings s;
	const string inputSettingsFile = argc > 1 ? argv[1] : "default.xml";
	FileStorage fs(inputSettingsFile, FileStorage::READ); // Read the settings
	if (!fs.isOpened())
	{
		cout << "Could not open the configuration file: \"" << inputSettingsFile << "\"" << endl;
		return -1;
	}
	fs["Settings"] >> s;
	fs.release();                                       
	// close Settings file
    //! [file_read]

	//FileStorage fout("settings.yml", FileStorage::WRITE); // write config as YAML
	 //fout << "Settings" << s;

	if (!s.goodInput)
	{
		cout << "Invalid input detected. Application stopping. " << endl;
		return -1;
	}

	vector<vector<Point2f> > imagePoints;
	Mat cameraMatrix, distCoeffs;
	Size imageSize;
	int mode = s.inputType == Settings::IMAGE_LIST ? CAPTURING : DETECTION;
	clock_t prevTimestamp = 0;
	const Scalar RED(0, 0, 255), GREEN(0, 255, 0);
	const char ESC_KEY = 27;

	//! [get_input]
	for (;;)
	{
		Mat view;
		bool blinkOutput = false;

		view = s.nextImage();

		//-----  If no more image, or got enough, then stop calibration and show result -------------
		if (mode == CAPTURING && imagePoints.size() >= (size_t)s.nrFrames)
		{
			if (runCalibrationAndSave(s, imageSize, cameraMatrix, distCoeffs, imagePoints))
				mode = CALIBRATED;
			else
				mode = DETECTION;
		}
		if (view.empty())          // If there are no more images stop the loop
		{
			// if calibration threshold was not reached yet, calibrate now
			if (mode != CALIBRATED && !imagePoints.empty())
				runCalibrationAndSave(s, imageSize, cameraMatrix, distCoeffs, imagePoints);
			break;
		}
		//! [get_input]

		imageSize = view.size();  // Format input image.
		if (s.flipVertical)    flip(view, view, 0);

		//! [find_pattern]
		vector<Point2f> pointBuf;

		bool found;

		int chessBoardFlags = CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE;

		if (!s.useFisheye) {
			// fast check erroneously fails with high distortions like fisheye
			chessBoardFlags |= CALIB_CB_FAST_CHECK;
		}

		switch (s.calibrationPattern) // Find feature points on the input format
		{
		case Settings::CHESSBOARD:
			found = findChessboardCorners(view, s.boardSize, pointBuf, chessBoardFlags);
			break;
		case Settings::CIRCLES_GRID:
			found = findCirclesGrid(view, s.boardSize, pointBuf);
			break;
		case Settings::ASYMMETRIC_CIRCLES_GRID:
			found = findCirclesGrid(view, s.boardSize, pointBuf, CALIB_CB_ASYMMETRIC_GRID);
			break;
		default:
			found = false;
			break;
		}
		//! [find_pattern]
		//! [pattern_found]
		if (found)                // If done with success,
		{
			// improve the found corners' coordinate accuracy for chessboard
			if (s.calibrationPattern == Settings::CHESSBOARD)
			{
				Mat viewGray;
				cvtColor(view, viewGray, COLOR_BGR2GRAY);
				cornerSubPix(viewGray, pointBuf, Size(11, 11),
					Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
			}

			if (mode == CAPTURING &&  // For camera only take new samples after delay time
				(!s.inputCapture.isOpened() || clock() - prevTimestamp > s.delay*1e-3*CLOCKS_PER_SEC))
			{
				imagePoints.push_back(pointBuf);
				prevTimestamp = clock();
				blinkOutput = s.inputCapture.isOpened();
			}

			// Draw the corners.
			drawChessboardCorners(view, s.boardSize, Mat(pointBuf), found);
		}
		//! [pattern_found]
		//----------------------------- Output Text ------------------------------------------------
		//! [output_text]
		string msg = (mode == CAPTURING) ? "100/100" :
			mode == CALIBRATED ? "Calibrated" : "Press 'g' to start";
		int baseLine = 0;
		Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);
		Point textOrigin(view.cols - 2 * textSize.width - 10, view.rows - 2 * baseLine - 10);

		if (mode == CAPTURING)
		{
			if (s.showUndistorsed)
				msg = format("%d/%d Undist", (int)imagePoints.size(), s.nrFrames);
			else
				msg = format("%d/%d", (int)imagePoints.size(), s.nrFrames);
		}

		putText(view, msg, textOrigin, 1, 1, mode == CALIBRATED ? GREEN : RED);

		if (blinkOutput)
			bitwise_not(view, view);
		//! [output_text]
		//------------------------- Video capture  output  undistorted ------------------------------
		//! [output_undistorted]
		if (mode == CALIBRATED && s.showUndistorsed)
		{
			Mat temp = view.clone();
			if (s.useFisheye)
				cv::fisheye::undistortImage(temp, view, cameraMatrix, distCoeffs);
			else
				undistort(temp, view, cameraMatrix, distCoeffs);
		}
		//! [output_undistorted]
		//------------------------------ Show image and check for input commands -------------------
		//! [await_input]
		imshow("Image View", view);
		char key = (char)waitKey(s.inputCapture.isOpened() ? 50 : s.delay);

		if (key == ESC_KEY)
			break;

		if (key == 'u' && mode == CALIBRATED)
			s.showUndistorsed = !s.showUndistorsed;

		if (s.inputCapture.isOpened() && key == 'g')
		{
			mode = CAPTURING;
			imagePoints.clear();
		}
		//! [await_input]
	}

	// -----------------------Show the undistorted image for the image list ------------------------
	//! [show_results]
	if (s.inputType == Settings::IMAGE_LIST && s.showUndistorsed)
	{
		Mat view, rview, map1, map2;

		if (s.useFisheye)
		{
			Mat newCamMat;
			fisheye::estimateNewCameraMatrixForUndistortRectify(cameraMatrix, distCoeffs, imageSize,
				Matx33d::eye(), newCamMat, 1);
			fisheye::initUndistortRectifyMap(cameraMatrix, distCoeffs, Matx33d::eye(), newCamMat, imageSize,
				CV_16SC2, map1, map2);
		}
		else
		{
			initUndistortRectifyMap(
				cameraMatrix, distCoeffs, Mat(),
				getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0), imageSize,
				CV_16SC2, map1, map2);
		}

		for (size_t i = 0; i < s.imageList.size(); i++)
		{
			view = imread(s.imageList[i], IMREAD_COLOR);
			if (view.empty())
				continue;
			remap(view, rview, map1, map2, INTER_LINEAR);
			imshow("Image View", rview);
			char c = (char)waitKey();
			if (c == ESC_KEY || c == 'q' || c == 'Q')
				break;
		}
	}
	//! [show_results]

	//program
	fstream wyniki, w;	// Tworzenie kana³ów dla plików
	const uint CAM_NUM = 2;		//Liczba kamer

	VideoCapture kanalczolo, kanaloko;  //This will hold the VideoCapture objects

	Mat RamkaCzolo, RamkaOko;  //This will hold the resulting frames from each camera
	Mat img0, hsv_img0, binary, czolo; //Miejsce na obrazki 
									   //	vector<Mat> hsv_split;        //Miejsce na kana³y hsv 

	kanalczolo.open(0);  //Otwieranie strumienia przechwytywania danych
	kanaloko.open(1);

	int frame_width0 = kanalczolo.get(CV_CAP_PROP_FRAME_WIDTH); //do zapisu wymiary
	int frame_height0 = kanalczolo.get(CV_CAP_PROP_FRAME_HEIGHT);
	int frame_width1 = kanaloko.get(CV_CAP_PROP_FRAME_WIDTH); //do zapisu wymiary
	int frame_height1 = kanaloko.get(CV_CAP_PROP_FRAME_HEIGHT);
	cout << "Czolo szer. " << frame_width0 << ", wys. " << frame_height0 << endl;
	cout << "Oko szer. " << frame_width1 << ", wys. " << frame_height1 << endl;
	// Define the codec and create VideoWriter object.The output is stored in 'outcpp.avi' file. 
	VideoWriter video0("Czolo.avi", CV_FOURCC('M', 'J', 'P', 'G'), 5, Size(frame_width0, frame_height0));
	VideoWriter video1("Oko.avi", CV_FOURCC('M', 'J', 'P', 'G'), 5, Size(frame_width1, frame_height1));
	VideoWriter video2("Czolo_z_okregami.avi", CV_FOURCC('M', 'J', 'P', 'G'), 5, Size(frame_width0, frame_height0));
	VideoWriter video3("Analiza.avi", CV_FOURCC('M', 'J', 'P', 'G'), 5, Size(frame_width0, frame_height0));
	// ustawienie najbardziej 

	int ldx = 0, ldy = 0, lgx, lgy, pdx, pdy, pgx, pgy, ro;
	int **tp;
	tp = new int *[10];
	//int i = 0;
	for (int i = 0; i < 10; i++)
		tp[i] = new int[2];
	int nr = 0;
	wyniki.open("Wyniki.csv", ios::out);	//Tworzenie pliku z wynikami
	wyniki << "ramka; x; y; r" << endl;


	while (waitKey(1) != 27) {			//continous loop until 'Esc' key is pressed

		kanalczolo >> RamkaCzolo;
		video0.write(RamkaCzolo);
		imshow("Obraz z czola", RamkaCzolo);

		kanaloko >> RamkaOko;  // przechwtywanie klatka po klatce z ka¿dego ujêcia

							   //Obrót obrazu
		Mat MacierzRotacji = getRotationMatrix2D(Point(RamkaOko.cols / 2, RamkaOko.rows / 2), 260, 1);
		//Mat MacierzRotacji = getRotationMatrix2D(Point(RamkaOko.cols, RamkaOko.rows), 270, 1);
		Mat RamkaRotacji;
		warpAffine(RamkaOko, RamkaRotacji, MacierzRotacji, RamkaOko.size());
		RamkaOko = RamkaRotacji;

		video1.write(RamkaOko);
		nr++;

		RamkaOko.copyTo(img0); // Skopiowanie klatki do img
		Mat src_gray;
		cvtColor(img0, src_gray, CV_BGR2GRAY);

		GaussianBlur(src_gray, src_gray, Size(9, 9), 2, 2);  /// Reduce the noise so we avoid false circle detection
		vector<Vec3f> circles;

		HoughCircles(src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows / 8, 200, 20, 0, 0); /// Apply the Hough Transform to find the circles

		RamkaCzolo.copyTo(czolo);

		// Wykrywanie i rysowanie okrêgów
		for (size_t i = 0; i < circles.size(); i++)
		{
			Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
			int radius = cvRound(circles[i][2]);

			wyniki << nr << "; " << cvRound(circles[i][0]) << "; " << cvRound(circles[i][1]) << "; " << cvRound(circles[i][2]) << endl;
			img = img0;
			flip(img, img0, 1);//odwracanie obrazu wzglêdem y

			circle(img0, center, 3, Scalar(0, 255, 0), -1, 8, 0); //œrodek okrêgu
			circle(img0, center, radius, Scalar(0, 0, 255), 3, 8, 0); // kontur okrêgu
																	  //czolo
			circle(czolo, center, 30, Scalar(0, 255, 0), -1, 8, 0);
			circle(czolo, center, radius, Scalar(0, 0, 255), 3, 8, 0);
			//ellipse(img0, center, s, double angle, double startAngle, double endAngle, const Scalar& color, int thickness = 1, int lineType = 8, int shift = 0)¶
		}
		// Rysowanie okrêgów na kamerze
		namedWindow("Okregi", CV_WINDOW_AUTOSIZE);
		imshow("Okregi", img0);
		video2.write(czolo);

	}
	wyniki.close();

	video2.release();//Zwolnienie wszystkich zasobów VideoCapture
	kanalczolo.release();
	video0.release();
	kanaloko.release();
	video1.release();

	//Zliczanie liczby lini pliku
	ifstream plik;
	string  wiersz;
	int linia;
	int licznik = 0;
	plik.open("Wyniki.csv");
	while (getline(plik, wiersz)) licznik++;
	plik.close();

	//Utworzenie tablicy wyników
	int nr_lini;
	plik.open("Wyniki.csv");

	if (plik.good() == false)
	{
		cout << "Nie uda³o siê otworzyæ pliku";
		exit(0);
	}

	string sram, sx, sy, sr;
	int tab[3];
	int **tabwy;
	tabwy = new int *[licznik];
	for (int i = 0; i < licznik; i++)
		tabwy[i] = new int[3];

	int ram, x, y, r, ipom = 0;

	while (getline(plik, wiersz))
	{
		int i = 0;
		int znalezionaPozycja = wiersz.find(";");
		cout << endl;
		do
		{
			tab[i] = znalezionaPozycja + 1;
			i++;
			znalezionaPozycja = wiersz.find(";", znalezionaPozycja + 1);

		} while (znalezionaPozycja != std::string::npos);

		sram = wiersz.substr(0, tab[0] - 1);
		ram = atoi(sram.c_str());
		sx = wiersz.substr(tab[0], tab[1] - tab[0] - 1);
		x = atoi(sx.c_str());
		sy = wiersz.substr(tab[1], tab[2] - tab[1] - 1);
		y = atoi(sy.c_str());
		sr = wiersz.substr(tab[2], wiersz.length());
		r = atoi(sr.c_str());
		tabwy[ipom][0] = ram;
		tabwy[ipom][1] = x;
		tabwy[ipom][2] = y;
		tabwy[ipom][3] = r;
		ipom++;
	}
	plik.close();

	for (int i = 0; i < licznik; i++)
	{
		for (int j = 0; j < 4; j++)
			cout << tabwy[i][j] << " ";

		cout << endl;
	}


	CvCapture* vid = cvCreateFileCapture("C:/Users/patrycja/Desktop/Kalibracja/Kalibracja/x64/Debug/Czolo.avi"); // Odczytanie pliku avi
																										  //CvCapture* vid2 = cvCreateFileCapture("E:/Okulografia/Okulografia/Eye_tracker/Eye_tracker/Oko.avi");
																										  // tworzymy okno wyswietlajace obraz
																										  //cvNamedWindow("Kamera1", 0);

	cvQueryFrame(vid); // odczytanie pierwszej klatki - niezbedne do prawidlowego odczytania wlasciwosci pliku przy uzyciu funkcji cvGetCaptureProperty
	nr = 1;

	//	cvQueryFrame(vid2);

	double fps = cvGetCaptureProperty(vid, CV_CAP_PROP_FPS); // odczytujemy z wlasciwosci pliku liczbe klatek na sekunde
															 //		double fps2 = cvGetCaptureProperty(vid2, CV_CAP_PROP_FP
	int odstep_miedzy_klatkami = 1000 / fps; // wyliczamy czas potrzebny do odtwarzania pliku z prawidlowa prêdkoscia

	int srx = -50, sry = -50, srx5 = -50, srx4 = -50, srx3 = -50, srx2 = -50, srx1 = -50, sry5 = -50, sry4 = -50, sry3 = -50, sry2 = -50, sry1 = -50;
	//Point srodek(srx, sry);//tworzenie punktu do zczytania z wyniku
	Mat analiza;
	while (true)
	{

		// pobranie kolejnej ramki
		IplImage* ramka = cvQueryFrame(vid);
		for (int i = 0; i < licznik; i++)
		{
			if (nr == tabwy[i][0])
			{
				srx = 640 - tabwy[i][1];
				sry = tabwy[i][2];
			}
		}


		srx5 = srx4;
		sry5 = sry4;
		srx4 = srx3;
		sry4 = sry3;
		srx3 = srx2;
		sry3 = sry2;
		srx2 = srx1;
		sry2 = sry1;
		srx1 = srx;
		sry1 = sry;

		analiza = cvarrToMat(ramka);
		Point srodek(srx, sry);
		Point srodek1(srx1, sry1);
		Point srodek2(srx2, sry2);
		Point srodek3(srx3, sry3);
		Point srodek4(srx4, sry4);
		Point srodek5(srx5, sry5);

		circle(analiza, srodek5, 10, Scalar(0, 100, 0), 4, 0, 0);
		circle(analiza, srodek4, 14, Scalar(0, 125, 0), 4, 0, 0);
		circle(analiza, srodek3, 18, Scalar(0, 150, 0), 4, 0, 0);
		circle(analiza, srodek2, 22, Scalar(0, 175, 0), 4, 0, 0);
		circle(analiza, srodek1, 25, Scalar(0, 215, 0), 4, 0, 0);
		circle(analiza, srodek, 30, Scalar(0, 255, 0), 4, 0, 0);

		if (ramka != 0) // Je¿eli nie jest pusta to wyœwietlamy
		{
			//cvShowImage("plik wideo", ramka);
			//do³o¿one
			video3.write(analiza);
			imshow("Obraz z czola i okregi", analiza);
			nr++;
		}
		else
			break;

		int c = cvWaitKey(odstep_miedzy_klatkami); // Czekamy przez okreœlony czas

		if (c == 'k') // Je¿eli naciœniêto klawisz 'k' koñczymy wyswietlanie
			break;
	}

	video3.release();
	// zwolnienie zasobów
	cvDestroyAllWindows();
	cvReleaseCapture(&vid);
	system("pause");
	return 0;
}

//! [compute_errors]
static double computeReprojectionErrors(const vector<vector<Point3f> >& objectPoints,
	const vector<vector<Point2f> >& imagePoints,
	const vector<Mat>& rvecs, const vector<Mat>& tvecs,
	const Mat& cameraMatrix, const Mat& distCoeffs,
	vector<float>& perViewErrors, bool fisheye)
{
	vector<Point2f> imagePoints2;
	size_t totalPoints = 0;
	double totalErr = 0, err;
	perViewErrors.resize(objectPoints.size());

	for (size_t i = 0; i < objectPoints.size(); ++i)
	{
		if (fisheye)
		{
			fisheye::projectPoints(objectPoints[i], imagePoints2, rvecs[i], tvecs[i], cameraMatrix,
				distCoeffs);
		}
		else
		{
			projectPoints(objectPoints[i], rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imagePoints2);
		}
		err = norm(imagePoints[i], imagePoints2, NORM_L2);

		size_t n = objectPoints[i].size();
		perViewErrors[i] = (float)std::sqrt(err*err / n);
		totalErr += err*err;
		totalPoints += n;
	}

	return std::sqrt(totalErr / totalPoints);
}
//! [compute_errors]
//! [board_corners]
static void calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f>& corners,
	Settings::Pattern patternType /*= Settings::CHESSBOARD*/)
{
	corners.clear();

	switch (patternType)
	{
	case Settings::CHESSBOARD:
	case Settings::CIRCLES_GRID:
		for (int i = 0; i < boardSize.height; ++i)
			for (int j = 0; j < boardSize.width; ++j)
				corners.push_back(Point3f(j*squareSize, i*squareSize, 0));
		break;

	case Settings::ASYMMETRIC_CIRCLES_GRID:
		for (int i = 0; i < boardSize.height; i++)
			for (int j = 0; j < boardSize.width; j++)
				corners.push_back(Point3f((2 * j + i % 2)*squareSize, i*squareSize, 0));
		break;
	default:
		break;
	}
}
//! [board_corners]
static bool runCalibration(Settings& s, Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
	vector<vector<Point2f> > imagePoints, vector<Mat>& rvecs, vector<Mat>& tvecs,
	vector<float>& reprojErrs, double& totalAvgErr)
{
	//! [fixed_aspect]
	cameraMatrix = Mat::eye(3, 3, CV_64F);
	if (s.flag & CALIB_FIX_ASPECT_RATIO)
		cameraMatrix.at<double>(0, 0) = s.aspectRatio;
	//! [fixed_aspect]
	if (s.useFisheye) {
		distCoeffs = Mat::zeros(4, 1, CV_64F);
	}
	else {
		distCoeffs = Mat::zeros(8, 1, CV_64F);
	}

	vector<vector<Point3f> > objectPoints(1);
	calcBoardCornerPositions(s.boardSize, s.squareSize, objectPoints[0], s.calibrationPattern);

	objectPoints.resize(imagePoints.size(), objectPoints[0]);

	//Find intrinsic and extrinsic camera parameters
	double rms;

	if (s.useFisheye) {
		Mat _rvecs, _tvecs;
		rms = fisheye::calibrate(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, _rvecs,
			_tvecs, s.flag);

		rvecs.reserve(_rvecs.rows);
		tvecs.reserve(_tvecs.rows);
		for (int i = 0; i < int(objectPoints.size()); i++) {
			rvecs.push_back(_rvecs.row(i));
			tvecs.push_back(_tvecs.row(i));
		}
	}
	else {
		rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs,
			s.flag);
	}

	cout << "Re-projection error reported by calibrateCamera: " << rms << endl;

	bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

	totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints, rvecs, tvecs, cameraMatrix,
		distCoeffs, reprojErrs, s.useFisheye);

	return ok;
}

// Print camera parameters to the output file
static void saveCameraParams(Settings& s, Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
	const vector<Mat>& rvecs, const vector<Mat>& tvecs,
	const vector<float>& reprojErrs, const vector<vector<Point2f> >& imagePoints,
	double totalAvgErr)
{
	FileStorage fs(s.outputFileName, FileStorage::WRITE);

	time_t tm;
	time(&tm);
	struct tm *t2 = localtime(&tm);
	char buf[1024];
	strftime(buf, sizeof(buf), "%c", t2);

	fs << "calibration_time" << buf;

	if (!rvecs.empty() || !reprojErrs.empty())
		fs << "nr_of_frames" << (int)std::max(rvecs.size(), reprojErrs.size());
	fs << "image_width" << imageSize.width;
	fs << "image_height" << imageSize.height;
	fs << "board_width" << s.boardSize.width;
	fs << "board_height" << s.boardSize.height;
	fs << "square_size" << s.squareSize;

	if (s.flag & CALIB_FIX_ASPECT_RATIO)
		fs << "fix_aspect_ratio" << s.aspectRatio;

	if (s.flag)
	{
		std::stringstream flagsStringStream;
		if (s.useFisheye)
		{
			flagsStringStream << "flags:"
				<< (s.flag & fisheye::CALIB_FIX_SKEW ? " +fix_skew" : "")
				<< (s.flag & fisheye::CALIB_FIX_K1 ? " +fix_k1" : "")
				<< (s.flag & fisheye::CALIB_FIX_K2 ? " +fix_k2" : "")
				<< (s.flag & fisheye::CALIB_FIX_K3 ? " +fix_k3" : "")
				<< (s.flag & fisheye::CALIB_FIX_K4 ? " +fix_k4" : "")
				<< (s.flag & fisheye::CALIB_RECOMPUTE_EXTRINSIC ? " +recompute_extrinsic" : "");
		}
		else
		{
			flagsStringStream << "flags:"
				<< (s.flag & CALIB_USE_INTRINSIC_GUESS ? " +use_intrinsic_guess" : "")
				<< (s.flag & CALIB_FIX_ASPECT_RATIO ? " +fix_aspectRatio" : "")
				<< (s.flag & CALIB_FIX_PRINCIPAL_POINT ? " +fix_principal_point" : "")
				<< (s.flag & CALIB_ZERO_TANGENT_DIST ? " +zero_tangent_dist" : "")
				<< (s.flag & CALIB_FIX_K1 ? " +fix_k1" : "")
				<< (s.flag & CALIB_FIX_K2 ? " +fix_k2" : "")
				<< (s.flag & CALIB_FIX_K3 ? " +fix_k3" : "")
				<< (s.flag & CALIB_FIX_K4 ? " +fix_k4" : "")
				<< (s.flag & CALIB_FIX_K5 ? " +fix_k5" : "");
		}
		fs.writeComment(flagsStringStream.str());
	}

	fs << "flags" << s.flag;

	fs << "fisheye_model" << s.useFisheye;

	fs << "camera_matrix" << cameraMatrix;
	fs << "distortion_coefficients" << distCoeffs;

	fs << "avg_reprojection_error" << totalAvgErr;
	if (s.writeExtrinsics && !reprojErrs.empty())
		fs << "per_view_reprojection_errors" << Mat(reprojErrs);

	if (s.writeExtrinsics && !rvecs.empty() && !tvecs.empty())
	{
		CV_Assert(rvecs[0].type() == tvecs[0].type());
		Mat bigmat((int)rvecs.size(), 6, CV_MAKETYPE(rvecs[0].type(), 1));
		bool needReshapeR = rvecs[0].depth() != 1 ? true : false;
		bool needReshapeT = tvecs[0].depth() != 1 ? true : false;

		for (size_t i = 0; i < rvecs.size(); i++)
		{
			Mat r = bigmat(Range(int(i), int(i + 1)), Range(0, 3));
			Mat t = bigmat(Range(int(i), int(i + 1)), Range(3, 6));

			if (needReshapeR)
				rvecs[i].reshape(1, 1).copyTo(r);
			else
			{
				//*.t() is MatExpr (not Mat) so we can use assignment operator
				CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
				r = rvecs[i].t();
			}

			if (needReshapeT)
				tvecs[i].reshape(1, 1).copyTo(t);
			else
			{
				CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
				t = tvecs[i].t();
			}
		}
		fs.writeComment("a set of 6-tuples (rotation vector + translation vector) for each view");
		fs << "extrinsic_parameters" << bigmat;
	}

	if (s.writePoints && !imagePoints.empty())
	{
		Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
		for (size_t i = 0; i < imagePoints.size(); i++)
		{
			Mat r = imagePtMat.row(int(i)).reshape(2, imagePtMat.cols);
			Mat imgpti(imagePoints[i]);
			imgpti.copyTo(r);
		}
		fs << "image_points" << imagePtMat;
	}
}

//! [run_and_save]
bool runCalibrationAndSave(Settings& s, Size imageSize, Mat& cameraMatrix, Mat& distCoeffs,
	vector<vector<Point2f> > imagePoints)
{
	vector<Mat> rvecs, tvecs;
	vector<float> reprojErrs;
	double totalAvgErr = 0;

	bool ok = runCalibration(s, imageSize, cameraMatrix, distCoeffs, imagePoints, rvecs, tvecs, reprojErrs,
		totalAvgErr);
	cout << (ok ? "Calibration succeeded" : "Calibration failed")
		<< ". avg re projection error = " << totalAvgErr << endl;

	if (ok)
		saveCameraParams(s, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, reprojErrs, imagePoints,
			totalAvgErr);
	return ok;
}
//! [run_and_save]
