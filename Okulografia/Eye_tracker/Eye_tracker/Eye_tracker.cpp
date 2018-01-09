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
	VideoWriter video0("Czolo.avi", CV_FOURCC('M', 'J', 'P', 'G'), 5, Size(frame_width0, frame_height0));
	VideoWriter video1("Oko.avi", CV_FOURCC('M', 'J', 'P', 'G'), 5, Size(frame_width1, frame_height1));
	VideoWriter video3("Analiza.avi", CV_FOURCC('M', 'J', 'P', 'G'), 5, Size(frame_width0, frame_height0));

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

			wyniki << nr << "; " << 640 - cvRound(circles[i][0]) << "; " << cvRound(circles[i][1]) << "; " << cvRound(circles[i][2]) << endl;

			circle(img0, center, 3, Scalar(0, 255, 0), -1, 8, 0); //środek okręgu
			circle(img0, center, radius, Scalar(0, 0, 255), 3, 8, 0); // kontur okręgu
																	  //czolo
			circle(czolo, center, 30, Scalar(0, 255, 0), -1, 8, 0);
			circle(czolo, center, radius, Scalar(0, 0, 255), 3, 8, 0);
		}

		// Rysowanie okręgów na kamerze
		namedWindow("Obraz gałki ocznej", CV_WINDOW_AUTOSIZE);
		imshow("Obraz gałki ocznej", img0);

	}
	wyniki.close();
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
	cvDestroyAllWindows();
	cvReleaseCapture(&vid);
	system("pause");
}
