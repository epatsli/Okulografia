// Eye_tracker.cpp : Defines the entry point for the console application.
//
/*
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
using namespace cv;
using namespace std;
#include "stdafx.h"


int main()
{
	Mat img(650, 600, CV_16UC3, Scalar(0, 50000, 50000)); //create an image ( 3 channels, 16 bit image depth, 650 high, 600 wide, (0, 50000, 50000) assigned for Blue, Green and Red plane respectively. )

	if (img.empty()) //check whether the image is loaded or not
	{
		cout << "ERROR : Image cannot be loaded..!!" << endl;
		//system("pause"); //wait for a key press
		return -1;
	}

	vector<int> compression_params; //vector that stores the compression parameters of the image

	compression_params.push_back(CV_IMWRITE_JPEG_QUALITY); //specify the compression technique

	compression_params.push_back(98); //specify the compression quality



	bool bSuccess = imwrite("C:/Users/patrycja/Desktop/20171015_141307.jpg", img, compression_params); //write the image to file



	if (!bSuccess)

	{

		cout << "ERROR : Failed to save the image" << endl;

		//system("pause"); //wait for a key press

	}

	namedWindow("MyWindow", CV_WINDOW_AUTOSIZE); //create a window with the name "MyWindow"
	imshow("MyWindow", img); //display the image which is stored in the 'img' in the "MyWindow" window

	waitKey(0);  //wait for a keypress

	destroyWindow("MyWindow"); //destroy the window with the name, "MyWindow"

    return 0;
}
*/
/**
* @file Morphology_1.cpp
* @brief Erosion and Dilation sample code
* @author OpenCV team
*/

/*
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

using namespace cv;

/// Global variables
Mat src, erosion_dst, dilation_dst;

int erosion_elem = 0;
int erosion_size = 0;
int dilation_elem = 0;
int dilation_size = 0;
int const max_elem = 2;
int const max_kernel_size = 21;

void Erosion(int, void*);
void Dilation(int, void*);

int main(int, char** argv)
{
	/// Load an image
	src = imread(argv[1], IMREAD_COLOR);

	if (src.empty())
	{
		return -1;
	}

	/// Create windows
	namedWindow("Erosion Demo", WINDOW_AUTOSIZE);
	namedWindow("Dilation Demo", WINDOW_AUTOSIZE);
	moveWindow("Dilation Demo", src.cols, 0);

	/// Create Erosion Trackbar
	createTrackbar("Element:\n 0: Rect \n 1: Cross \n 2: Ellipse", "Erosion Demo",
		&erosion_elem, max_elem,
		Erosion);

	createTrackbar("Kernel size:\n 2n +1", "Erosion Demo",
		&erosion_size, max_kernel_size,
		Erosion);

	/// Create Dilation Trackbar
	createTrackbar("Element:\n 0: Rect \n 1: Cross \n 2: Ellipse", "Dilation Demo",
		&dilation_elem, max_elem,
		Dilation);

	createTrackbar("Kernel size:\n 2n +1", "Dilation Demo",
		&dilation_size, max_kernel_size,
		Dilation);

	/// Default start
	Erosion(0, 0);
	Dilation(0, 0);

	waitKey(0);
	return 0;
}


void Erosion(int, void*)
{
	int erosion_type = 0;
	if (erosion_elem == 0) { erosion_type = MORPH_RECT; }
	else if (erosion_elem == 1) { erosion_type = MORPH_CROSS; }
	else if (erosion_elem == 2) { erosion_type = MORPH_ELLIPSE; }

	//![kernel]
	Mat element = getStructuringElement(erosion_type,
		Size(2 * erosion_size + 1, 2 * erosion_size + 1),
		Point(erosion_size, erosion_size));
	//![kernel]

	/// Apply the erosion operation
	erode(src, erosion_dst, element);
	imshow("Erosion Demo", erosion_dst);
}

void Dilation(int, void*)
{
	int dilation_type = 0;
	if (dilation_elem == 0) { dilation_type = MORPH_RECT; }
	else if (dilation_elem == 1) { dilation_type = MORPH_CROSS; }
	else if (dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }

	Mat element = getStructuringElement(dilation_type,
		Size(2 * dilation_size + 1, 2 * dilation_size + 1),
		Point(dilation_size, dilation_size));

	/// Apply the dilation operation
	dilate(src, dilation_dst, element);
	imshow("Dilation Demo", dilation_dst);
}
//![dilation]
*/

/*
* File:   main.cpp
* Author: sagar
*
* Created on 10 September, 2012, 7:48 PM
*/



/*OK
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/videoio.hpp>  // Video write
#include <iostream>
using namespace cv;
using namespace std;

int main() {
	VideoCapture stream1(0);   //0 is the id of video device.0 if you have only one camera.
	VideoCapture stream2(1);

	if (!stream1.isOpened()) { //check if video device has been initialised
		cout << "cannot open camera1";
	}

	if (!stream2.isOpened()) { //check if video device has been initialised
		cout << "cannot open camera2";
	}

	//stream1.set(CV_CAP_PROP_FRAME_WIDTH, 320);
	//stream1.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
	int frame_width1 = stream1.get(CV_CAP_PROP_FRAME_WIDTH); //do zapisu wymiary
	int frame_height1 = stream1.get(CV_CAP_PROP_FRAME_HEIGHT);
	int frame_width2 = stream2.get(CV_CAP_PROP_FRAME_WIDTH); //do zapisu wymiary
	int frame_height2 = stream2.get(CV_CAP_PROP_FRAME_HEIGHT);
	// Define the codec and create VideoWriter object.The output is stored in 'outcpp.avi' file. 
	VideoWriter video1("Kamera1.avi", CV_FOURCC('M', 'J', 'P', 'G'), 10, Size(frame_width1, frame_height1));
	VideoWriter video2("Kamera1.avi", CV_FOURCC('M', 'J', 'P', 'G'), 10, Size(frame_width2, frame_height2));
	//unconditional loop
	while (true) {
		Mat cameraFrame, cameraFrame1;
		stream1.read(cameraFrame);
		video1.write(cameraFrame);
		imshow("kamera1", cameraFrame);

		stream2.read(cameraFrame);
		imshow("kamera2", cameraFrame1);

		if (waitKey(30) >= 0)
			break;
	}

	// When everything done, release the video capture and write object
	stream1.release();
	video1.release();
	stream2.release();
	video2.release();


	return 0;
}


*/

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
//#include <Windows.h>
using namespace cv;
using namespace std;

//kali

vector< vector< Point3f > > object_points;
vector< vector< Point2f > > image_points;
vector< Point2f > corners;
vector< vector< Point2f > > left_img_points;

Mat img, gray;
Size im_size;

void setup_calibration(int board_width, int board_height, int num_imgs,
	float square_size, char* imgs_directory, char* imgs_filename,
	char* extension) {
	Size board_size = Size(board_width, board_height);
	int board_n = board_width * board_height;

	for (int k = 1; k <= num_imgs; k++) {
		char img_file[100];
		sprintf(img_file, "%s%s%d.%s", imgs_directory, imgs_filename, k, extension);
		img = imread(img_file, CV_LOAD_IMAGE_COLOR);
		cv::cvtColor(img, gray, CV_BGR2GRAY);

		bool found = false;
		found = cv::findChessboardCorners(img, board_size, corners,
			CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
		if (found)
		{
			cornerSubPix(gray, corners, cv::Size(5, 5), cv::Size(-1, -1),
				TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
			drawChessboardCorners(gray, board_size, corners, found);
		}

		vector< Point3f > obj;
		for (int i = 0; i < board_height; i++)
			for (int j = 0; j < board_width; j++)
				obj.push_back(Point3f((float)j * square_size, (float)i * square_size, 0));

		if (found) {
			cout << k << ". Found corners!" << endl;
			image_points.push_back(corners);
			object_points.push_back(obj);
		}
	}
}

double computeReprojectionErrors(const vector< vector< Point3f > >& objectPoints,
	const vector< vector< Point2f > >& imagePoints,
	const vector< Mat >& rvecs, const vector< Mat >& tvecs,
	const Mat& cameraMatrix, const Mat& distCoeffs) {
	vector< Point2f > imagePoints2;
	int i, totalPoints = 0;
	double totalErr = 0, err;
	vector< float > perViewErrors;
	perViewErrors.resize(objectPoints.size());

	for (i = 0; i < (int)objectPoints.size(); ++i) {
		projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix,
			distCoeffs, imagePoints2);
		err = norm(Mat(imagePoints[i]), Mat(imagePoints2), CV_L2);
		int n = (int)objectPoints[i].size();
		perViewErrors[i] = (float)std::sqrt(err*err / n);
		totalErr += err*err;
		totalPoints += n;
	}
	return std::sqrt(totalErr / totalPoints);
}
//kali

//void main(
	int main(int argc, char* argv[])
{
		/*
		//kali
		int board_width, board_height, num_imgs;
		float square_size;
		char* imgs_directory;
		char* imgs_filename;
		char* out_file;
		char* extension;

		static struct poptOption options[] = {
			{ "board_width",'w',POPT_ARG_INT,&board_width,0,"Checkerboard width","NUM" },
			{ "board_height",'h',POPT_ARG_INT,&board_height,0,"Checkerboard height","NUM" },
			{ "num_imgs",'n',POPT_ARG_INT,&num_imgs,0,"Number of checkerboard images","NUM" },
			{ "square_size",'s',POPT_ARG_FLOAT,&square_size,0,"Size of checkerboard square","NUM" },
			{ "imgs_directory",'d',POPT_ARG_STRING,&imgs_directory,0,"Directory containing images","STR" },
			{ "imgs_filename",'i',POPT_ARG_STRING,&imgs_filename,0,"Image filename","STR" },
			{ "extension",'e',POPT_ARG_STRING,&extension,0,"Image extension","STR" },
			{ "out_file",'o',POPT_ARG_STRING,&out_file,0,"Output calibration filename (YML)","STR" },
			POPT_AUTOHELP
		{ NULL, 0, 0, NULL, 0, NULL, NULL }
		};

		POpt popt(NULL, argc, argv, options, 0);
		int c;
		while ((c = popt.getNextOpt()) >= 0) {}

		setup_calibration(board_width, board_height, num_imgs, square_size,imgs_directory, imgs_filename, extension);

		printf("Starting Calibration\n");
		Mat K;
		Mat D;
		vector< Mat > rvecs, tvecs;
		int flag = 0;
		flag |= CV_CALIB_FIX_K4;
		flag |= CV_CALIB_FIX_K5;
		calibrateCamera(object_points, image_points, img.size(), K, D, rvecs, tvecs, flag);

		cout << "Calibration error: " << computeReprojectionErrors(object_points, image_points, rvecs, tvecs, K, D) << endl;

		FileStorage fs(out_file, FileStorage::WRITE);
		fs << "K" << K;
		fs << "D" << D;
		fs << "board_width" << board_width;
		fs << "board_height" << board_height;
		fs << "square_size" << square_size;
		printf("Done Calibration\n");

		//kali
		*/

	fstream wyniki,w;	// Tworzenie kana³ów dla plików
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

		int nr=0;
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
			/*
			RamkaOko.copyTo(img0); // Skopiowanie klatki do img
			cvtColor(img0, hsv_img0, CV_BGR2HSV);        //Konwrsja do HSV
			split(hsv_img0, hsv_split);        //Podzial HSV na poszczegolne kanaly
			//inRange(hsv_split[0], 80, 255, binary);  //Progowanie zgodnie z wartosciami lowerb, i upperb
			//inRange(hsv_split[0], Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), binary);
			inRange(hsv_split[0], Scalar(150, 182, 65), Scalar(163, 215, 111), binary);
			//morphological opening (remove small objects from the foreground)
			erode(binary, binary, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
			dilate(binary, binary, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

			//morphological closing (fill small holes in the foreground)
			dilate(binary, binary, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
			erode(binary, binary, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
			*/
		
			//Kopiowanie do szaroœci i szukanie okrêgów
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

/*
			vector<vector<Point> > contours;
			//vector<RotatedRect> rotRecs;
			findContours(img0, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

			RotatedRect rotRecs[contours.size()];

			for (int i = 0; i < contours.size(); i++) {
				rotRecs[i] = fitEllipse(contours[i]);
			}
*/

			// Rysowanie okrêgów na kamerze
			namedWindow("Okregi", CV_WINDOW_AUTOSIZE);
			imshow("Okregi", img0);
			video2.write(czolo);

			/*
			cv::Mat element(3, 3, CV_8U, cv::Scalar(1));    //Okreslenie opcji erozji
			blur(binary, binary, cv::Size(3, 3));        //Rozmycie
			erode(binary, binary, element);            //Erozja
			imshow("zwykly", img0);            //Obrazek Orginalny
			imshow("hcv", binary);            //Obraz binarny
			*/
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
	
		
		CvCapture* vid = cvCreateFileCapture("E:/Okulografia/Okulografia/Eye_tracker/Eye_tracker/Czolo.avi"); // Odczytanie pliku avi
		//CvCapture* vid2 = cvCreateFileCapture("E:/Okulografia/Okulografia/Eye_tracker/Eye_tracker/Oko.avi");
		// tworzymy okno wyswietlajace obraz
		//cvNamedWindow("Kamera1", 0);

		cvQueryFrame(vid); // odczytanie pierwszej klatki - niezbedne do prawidlowego odczytania wlasciwosci pliku przy uzyciu funkcji cvGetCaptureProperty
		nr = 1;
	//	cvQueryFrame(vid2);
		
		double fps = cvGetCaptureProperty(vid, CV_CAP_PROP_FPS); // odczytujemy z wlasciwosci pliku liczbe klatek na sekunde
//		double fps2 = cvGetCaptureProperty(vid2, CV_CAP_PROP_FPS);
		/*
		//Liczba ramek w pliku
		w.open("Wy.csv", ios::out);
		w << "f; fps; fps2 " << endl;
		double f = cvGetCaptureProperty(vid, CV_CAP_PROP_FRAME_COUNT);
		double ff = cvGetCaptureProperty(vid2, CV_CAP_PROP_FRAME_COUNT);
		w << f << " ; " << fps << ";" << ff << ";"<< fps2;
		w.close();
		*/
		
		int odstep_miedzy_klatkami = 1000 / fps; // wyliczamy czas potrzebny do odtwarzania pliku z prawidlowa prêdkoscia
		
		int srx=-50, sry=-50, srx5=-50, srx4=-50, srx3=-50, srx2=-50, srx1=-50, sry5 = -50, sry4 = -50, sry3 = -50, sry2 = -50, sry1 = -50;
		//Point srodek(srx, sry);//tworzenie punktu do zczytania z wyniku
		Mat analiza;
		while (true)
		{
			
			// pobranie kolejnej ramki
			IplImage* ramka = cvQueryFrame(vid);
			for (int i = 0; i <licznik; i++)
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
}
