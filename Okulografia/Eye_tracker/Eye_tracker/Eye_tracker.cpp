#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2\videoio.hpp>
#include <opencv2\highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp" 
#include <string> 
#include <iostream>
#include <fstream>
#include "popt_pp1.h"
#include "stdafx.h"
#include <opencv2/core/utility.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
using namespace cv;
using namespace std;


Mat img, gray;
Size im_size;
/// w dół kalibracja poprawna
/*
#include <cctype>
#include <stdio.h>
//#include <string.h>
#include <time.h>

using namespace cv;
using namespace std;

const char * usage =
" \nexample command line for calibration from a live feed.\n"
"   calibration  -w=4 -h=5 -s=0.025 -o=camera.yml -op -oe\n"
" \n"
" example command line for calibration from a list of stored images:\n"
"   imagelist_creator image_list.xml *.png\n"
"   calibration -w=4 -h=5 -s=0.025 -o=camera.yml -op -oe image_list.xml\n"
" where image_list.xml is the standard OpenCV XML/YAML\n"
" use imagelist_creator to create the xml or yaml list\n"
" file consisting of the list of strings, e.g.:\n"
" \n"
"<?xml version=\"1.0\"?>\n"
"<opencv_storage>\n"
"<images>\n"
"view000.png\n"
"view001.png\n"
"<!-- view002.png -->\n"
"view003.png\n"
"view010.png\n"
"one_extra_view.jpg\n"
"</images>\n"
"</opencv_storage>\n";




const char* liveCaptureHelp =
"When the live video from camera is used as input, the following hot-keys may be used:\n"
"  <ESC>, 'q' - quit the program\n"
"  'g' - start capturing images\n"
"  'u' - switch undistortion on/off\n";

static void help()
{
printf("This is a camera calibration sample.\n"
"Usage: calibration\n"
"     -w=<board_width>         # the number of inner corners per one of board dimension\n"
"     -h=<board_height>        # the number of inner corners per another board dimension\n"
"     [-pt=<pattern>]          # the type of pattern: chessboard or circles' grid\n"
"     [-n=<number_of_frames>]  # the number of frames to use for calibration\n"
"                              # (if not specified, it will be set to the number\n"
"                              #  of board views actually available)\n"
"     [-d=<delay>]             # a minimum delay in ms between subsequent attempts to capture a next view\n"
"                              # (used only for video capturing)\n"
"     [-s=<squareSize>]       # square size in some user-defined units (1 by default)\n"
"     [-o=<out_camera_params>] # the output filename for intrinsic [and extrinsic] parameters\n"
"     [-op]                    # write detected feature points\n"
"     [-oe]                    # write extrinsic parameters\n"
"     [-zt]                    # assume zero tangential distortion\n"
"     [-a=<aspectRatio>]      # fix aspect ratio (fx/fy)\n"
"     [-p]                     # fix the principal point at the center\n"
"     [-v]                     # flip the captured images around the horizontal axis\n"
"     [-V]                     # use a video file, and not an image list, uses\n"
"                              # [input_data] string for the video file name\n"
"     [-su]                    # show undistorted images after calibration\n"
"     [input_data]             # input data, one of the following:\n"
"                              #  - text file with a list of the images of the board\n"
"                              #    the text file can be generated with imagelist_creator\n"
"                              #  - name of video file with a video of the board\n"
"                              # if input_data not specified, a live view from the camera is used\n"
"\n");
printf("\n%s", usage);
printf("\n%s", liveCaptureHelp);
}

enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };
enum Pattern { CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };

static double computeReprojectionErrors(
const vector<vector<Point3f> >& objectPoints,
const vector<vector<Point2f> >& imagePoints,
const vector<Mat>& rvecs, const vector<Mat>& tvecs,
const Mat& cameraMatrix, const Mat& distCoeffs,
vector<float>& perViewErrors)
{
vector<Point2f> imagePoints2;
int i, totalPoints = 0;
double totalErr = 0, err;
perViewErrors.resize(objectPoints.size());

for (i = 0; i < (int)objectPoints.size(); i++)
{
projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i],
cameraMatrix, distCoeffs, imagePoints2);
err = norm(Mat(imagePoints[i]), Mat(imagePoints2), NORM_L2);
int n = (int)objectPoints[i].size();
perViewErrors[i] = (float)std::sqrt(err*err / n);
totalErr += err*err;
totalPoints += n;
}

return std::sqrt(totalErr / totalPoints);
}

static void calcChessboardCorners(Size boardSize, float squareSize, vector<Point3f>& corners, Pattern patternType = CHESSBOARD)
{
corners.resize(0);

switch (patternType)
{
case CHESSBOARD:
case CIRCLES_GRID:
for (int i = 0; i < boardSize.height; i++)
for (int j = 0; j < boardSize.width; j++)
corners.push_back(Point3f(float(j*squareSize),
float(i*squareSize), 0));
break;

case ASYMMETRIC_CIRCLES_GRID:
for (int i = 0; i < boardSize.height; i++)
for (int j = 0; j < boardSize.width; j++)
corners.push_back(Point3f(float((2 * j + i % 2)*squareSize),
float(i*squareSize), 0));
break;

default:
CV_Error(Error::StsBadArg, "Unknown pattern type\n");
}
}

static bool runCalibration(vector<vector<Point2f> > imagePoints,
Size imageSize, Size boardSize, Pattern patternType,
float squareSize, float aspectRatio,
int flags, Mat& cameraMatrix, Mat& distCoeffs,
vector<Mat>& rvecs, vector<Mat>& tvecs,
vector<float>& reprojErrs,
double& totalAvgErr)
{
cameraMatrix = Mat::eye(3, 3, CV_64F);
if (flags & CALIB_FIX_ASPECT_RATIO)
cameraMatrix.at<double>(0, 0) = aspectRatio;

distCoeffs = Mat::zeros(8, 1, CV_64F);

vector<vector<Point3f> > objectPoints(1);
calcChessboardCorners(boardSize, squareSize, objectPoints[0], patternType);

objectPoints.resize(imagePoints.size(), objectPoints[0]);

double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
distCoeffs, rvecs, tvecs, flags | CALIB_FIX_K4 | CALIB_FIX_K5);
//|CALIB_FIX_K3|CALIB_FIX_K4|CALIB_FIX_K5);
printf("RMS error reported by calibrateCamera: %g\n", rms);

bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

return ok;
}


static void saveCameraParams(const string& filename,
Size imageSize, Size boardSize,
float squareSize, float aspectRatio, int flags,
const Mat& cameraMatrix, const Mat& distCoeffs,
const vector<Mat>& rvecs, const vector<Mat>& tvecs,
const vector<float>& reprojErrs,
const vector<vector<Point2f> >& imagePoints,
double totalAvgErr)
{
FileStorage fs(filename, FileStorage::WRITE);

time_t tt;
time(&tt);
struct tm *t2 = localtime(&tt);
char buf[1024];
strftime(buf, sizeof(buf) - 1, "%c", t2);

fs << "calibration_time" << buf;

if (!rvecs.empty() || !reprojErrs.empty())
fs << "nframes" << (int)std::max(rvecs.size(), reprojErrs.size());
fs << "image_width" << imageSize.width;
fs << "image_height" << imageSize.height;
fs << "board_width" << boardSize.width;
fs << "board_height" << boardSize.height;
fs << "square_size" << squareSize;

if (flags & CALIB_FIX_ASPECT_RATIO)
fs << "aspectRatio" << aspectRatio;

if (flags != 0)
{
sprintf(buf, "flags: %s%s%s%s",
flags & CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
flags & CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
flags & CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
flags & CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "");
//cvWriteComment( *fs, buf, 0 );
}

fs << "flags" << flags;

fs << "camera_matrix" << cameraMatrix;
fs << "distortion_coefficients" << distCoeffs;

fs << "avg_reprojection_error" << totalAvgErr;
if (!reprojErrs.empty())
fs << "per_view_reprojection_errors" << Mat(reprojErrs);

if (!rvecs.empty() && !tvecs.empty())
{
CV_Assert(rvecs[0].type() == tvecs[0].type());
Mat bigmat((int)rvecs.size(), 6, rvecs[0].type());
for (int i = 0; i < (int)rvecs.size(); i++)
{
Mat r = bigmat(Range(i, i + 1), Range(0, 3));
Mat t = bigmat(Range(i, i + 1), Range(3, 6));

CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
//*.t() is MatExpr (not Mat) so we can use assignment operator
r = rvecs[i].t();
t = tvecs[i].t();
}
//cvWriteComment( *fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0 );
fs << "extrinsic_parameters" << bigmat;
}

if (!imagePoints.empty())
{
Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
for (int i = 0; i < (int)imagePoints.size(); i++)
{
Mat r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
Mat imgpti(imagePoints[i]);
imgpti.copyTo(r);
}
fs << "image_points" << imagePtMat;
}
}

static bool readStringList(const string& filename, vector<string>& l)
{
l.resize(0);
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


static bool runAndSave(const string& outputFilename,
const vector<vector<Point2f> >& imagePoints,
Size imageSize, Size boardSize, Pattern patternType, float squareSize,
float aspectRatio, int flags, Mat& cameraMatrix,
Mat& distCoeffs, bool writeExtrinsics, bool writePoints)
{
vector<Mat> rvecs, tvecs;
vector<float> reprojErrs;
double totalAvgErr = 0;

bool ok = runCalibration(imagePoints, imageSize, boardSize, patternType, squareSize,
aspectRatio, flags, cameraMatrix, distCoeffs,
rvecs, tvecs, reprojErrs, totalAvgErr);
printf("%s. avg reprojection error = %.2f\n",
ok ? "Calibration succeeded" : "Calibration failed",
totalAvgErr);

if (ok)
saveCameraParams(outputFilename, imageSize,
boardSize, squareSize, aspectRatio,
flags, cameraMatrix, distCoeffs,
writeExtrinsics ? rvecs : vector<Mat>(),
writeExtrinsics ? tvecs : vector<Mat>(),
writeExtrinsics ? reprojErrs : vector<float>(),
writePoints ? imagePoints : vector<vector<Point2f> >(),
totalAvgErr);
return ok;
}
*/

int main(int argc, char* argv[])
{

	fstream wyniki;	// Tworzenie kanałów dla plików
	const uint CAM_NUM = 2;		//Liczba kamer

	VideoCapture kanalczolo, kanaloko;  //Obiekty, w których przetrzymujemy dane dla obu kamer

	Mat RamkaCzolo, RamkaOko;  //This will hold the resulting frames from each camera
	Mat img0, hsv_img0, binary, czolo; //Miejsce na klatki

	kanalczolo.open(0);  //Otwieranie strumienia przechwytywania danych
	kanaloko.open(1);

	int frame_width0 = kanalczolo.get(CV_CAP_PROP_FRAME_WIDTH); //wymiary do zapisu
	int frame_height0 = kanalczolo.get(CV_CAP_PROP_FRAME_HEIGHT);
	int frame_width1 = kanaloko.get(CV_CAP_PROP_FRAME_WIDTH);
	int frame_height1 = kanaloko.get(CV_CAP_PROP_FRAME_HEIGHT);
	cout << "Czolo szer. " << frame_width0 << ", wys. " << frame_height0 << endl;
	cout << "Oko szer. " << frame_width1 << ", wys. " << frame_height1 << endl;
	// Define the codec and create VideoWriter object.The output is stored in 'outcpp.avi' file. 
	VideoWriter video0("Czolo.avi", CV_FOURCC('M', 'J', 'P', 'G'), 5, Size(frame_width0, frame_height0));
	VideoWriter video1("Oko.avi", CV_FOURCC('M', 'J', 'P', 'G'), 5, Size(frame_width1, frame_height1));
	VideoWriter video2("Czolo_z_okregami.avi", CV_FOURCC('M', 'J', 'P', 'G'), 5, Size(frame_width0, frame_height0));
	VideoWriter video3("Analiza.avi", CV_FOURCC('M', 'J', 'P', 'G'), 5, Size(frame_width0, frame_height0));
	/*
	int **tp;
	tp = new int *[10];
	for (int i = 0; i < 10; i++)
		tp[i] = new int[2];
	*/
	/*
	//znalezienie parametrów filtra
	namedWindow("Control", CV_WINDOW_AUTOSIZE);
	int iLowH = 0;
	int iHighH = 179;

	int iLowS = 0;
	int iHighS = 255;

	int iLowV = 0;
	int iHighV = 255;

	//Create trackbars in "Control" window
	cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
	cvCreateTrackbar("HighH", "Control", &iHighH, 179);

	cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
	cvCreateTrackbar("HighS", "Control", &iHighS, 255);

	cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
	cvCreateTrackbar("HighV", "Control", &iHighV, 255);
	*/

	int nr = 0;
	wyniki.open("Wyniki.csv", ios::out);	//Tworzenie pliku z wynikami
	wyniki << "ramka; x; y; r" << endl;

	while (waitKey(1) != 27) {			//continous loop until 'Esc' key is pressed

		kanalczolo >> RamkaCzolo;
		imshow("Obraz z czola", RamkaCzolo);

		kanaloko >> RamkaOko;  // przechwtywanie klatka po klatce z każdego ujęcia

		//Obrót obrazu
		Mat MacierzRotacji = getRotationMatrix2D(Point(RamkaOko.cols / 2, RamkaOko.rows / 2), 260, 1);
		Mat RamkaRotacji;
		warpAffine(RamkaOko, RamkaRotacji, MacierzRotacji, RamkaOko.size());
		RamkaOko = RamkaRotacji;

		/*
		RamkaOko.copyTo(img0); // Skopiowanie klatki do img
		cvtColor(img0, hsv_img0, CV_BGR2HSV);        //Konwrsja do HSV
		split(hsv_img0, hsv_split);        //Podzial HSV na poszczegolne kanaly
		//inRange(hsv_split[0], 80, 255, binary);  //Progowanie zgodnie z wartosciami lowerb, i upperb
		//inRange(hsv_split[0], Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), binary);
		inRange(hsv_split[0], Scalar(150, 182, 65), Scalar(163, 215, 111), binary);
		//usuwanie małych obiektów z planu
		erode(binary, binary, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		dilate(binary, binary, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

		//wypełnianie małych dziur na pierwszym planie
		dilate(binary, binary, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		erode(binary, binary, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		*/

		//Kopiowanie do szarości i szukanie okręgów
		RamkaOko.copyTo(img0); // Skopiowanie klatki do img
		Mat src_gray;
		cvtColor(img0, src_gray, CV_BGR2GRAY);

		GaussianBlur(src_gray, src_gray, Size(9, 9), 2, 2);  // Zmniejszenie hałasów w celu uniknięcie wykrywania fałszywych okręgów
		vector<Vec3f> circles;

		HoughCircles(src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows / 8, 200, 20, 0, 0); // Wykonanie transformacji Hough w celu wykrycia okręgów
		
		nr++;

		video1.write(RamkaOko);
		video0.write(RamkaCzolo);
		RamkaCzolo.copyTo(czolo);

		// Wykrywanie i rysowanie okręgów
		for (size_t i = 0; i < circles.size(); i++)
		{
			Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
			int radius = cvRound(circles[i][2]);

			wyniki << nr << "; " << 640-cvRound(circles[i][0]) << "; " << cvRound(circles[i][1]) << "; " << cvRound(circles[i][2]) << endl;
		//	img = img0;
		//	flip(img, img0, 1);//odwracanie obrazu względem y

			circle(img0, center, 3, Scalar(0, 255, 0), -1, 8, 0); //środek okręgu
			circle(img0, center, radius, Scalar(0, 0, 255), 3, 8, 0); // kontur okręgu
																	  //czolo
			circle(czolo, center, 30, Scalar(0, 255, 0), -1, 8, 0);
			circle(czolo, center, radius, Scalar(0, 0, 255), 3, 8, 0);
			//ellipse(img0, center, s, double angle, double startAngle, double endAngle, const Scalar& color, int thickness = 1, int lineType = 8, int shift = 0)¶
		}

		// Rysowanie okręgów na kamerze
		namedWindow("Okregi", CV_WINDOW_AUTOSIZE);
		imshow("Okregi", img0);
		video2.write(czolo);
		imshow("Czolo", czolo);
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
		cout << "Nie udało się otworzyć pliku";
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


	CvCapture* vid = cvCreateFileCapture("E:/Okulografia/Okulografia/Eye_tracker/Eye_tracker/Czolo.avi"); // Odczytanie pliku avi
	cvQueryFrame(vid); // odczytanie pierwszej klatki - niezbedne do prawidlowego odczytania wlasciwosci pliku przy uzyciu funkcji cvGetCaptureProperty
	nr = 1;

	double fps = cvGetCaptureProperty(vid, CV_CAP_PROP_FPS); // odczytujemy z wlasciwosci pliku liczbe klatek na sekunde													
	int odstep_miedzy_klatkami = 1000 / fps; // wyliczamy czas potrzebny do odtwarzania pliku z prawidlowa prędkoscia
	float srx = -100.00, sry = -100.00, srx5 = -100.00, srx4 = -100.00, srx3 = -100.00, srx2 = -100.00, srx1 = -100.00, sry5 = -100.00, sry4 = -100.00, sry3 = -100.00, sry2 = -100.00, sry1 = -100.00;

	Mat analiza;
	while (true)
	{

		// pobranie kolejnej ramki
		IplImage* ramka = cvQueryFrame(vid);
		for (int i = 0; i < licznik; i++)
		{
			if (nr == tabwy[i][0])
			{
				srx = tabwy[i][1];
				sry = tabwy[i][2];

			}
		}

		if ((srx > 320) && (srx<350)) srx = 1.1*srx;
		if ((srx > 215)&& (srx<250)) srx = 0.8*srx;
		if ((srx > 480) && (srx<550)) srx = 1.2*srx;

		if (sry < 200) sry = 0.82*sry;
		if ((sry > 200) && (sry < 238)) sry = 1.05*sry;
		if ((sry > 249)&& (sry<329)) sry = 1.15*sry;
		if (sry > 329) sry = 1.35*sry;
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

		if (ramka != 0) // Jeżeli nie jest pusta to wyświetlamy
		{
			video3.write(analiza);
			imshow("Obraz z czola i okregi", analiza);
			nr++;
		}
		else
			break;

		int c = cvWaitKey(odstep_miedzy_klatkami); // Czekamy przez określony czas

		if (c == 'k') // Jeżeli naciśnięto klawisz 'k' kończymy wyswietlanie
			break;
	}

	video3.release();
	// zwolnienie zasobów
	cvDestroyAllWindows();
	cvReleaseCapture(&vid);

	system("pause");
}
