//
//#define _CRT_SECURE_NO_WARNINGS 
//#define _USE_MATH_DEFINES
//
//#include <opencv2/opencv.hpp>
//#include <opencv2/opencv_lib.hpp>
//
//#include "opencv2/video/tracking.hpp"
//#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/highgui/highgui.hpp"
//
//#include <tesseract/baseapi.h>
//#include <leptonica/allheaders.h>
//
//#include <Tchar.h>
//#include <iostream>
//#include <fstream>
//#include <string>
//#include <time.h>
//#include <direct.h>
//#include <math.h>
//
//#include "stdafx.h"
//#include "BingTranslate.h"
//
//using namespace cv;
//using namespace std;
//
//#define search_area 200 /*傾き探索範囲*/
//#define rough_cut_area_width 200
//#define rough_cut_area_hight 100
//
//#define MY_ID  "spwork"
//#define MY_KEY "/GHlsseZh16I8hFW0ao2SowLu81Wqm4OzgoSc2yLwHo="
//
//typedef struct _data{/*切り取り範囲の構造体*/
//	int event, x = 0, y = 0, flag;
//}Data;
//
//typedef struct _cutdata{/*切り取り範囲の構造体*/
//	int x = 0, y = 0, width = 0, hight = 0;
//}cutData;
//
//typedef struct _totaldata{/*切り取り範囲の構造体*/
//	int x = 0, total = 0;
//}totalData;
//
////追跡する色のデータ
//struct track_color_data {
//	int trackObject;
//	bool selectObject;
//	cv::Point origin;
//	cv::Rect selection;
//};
//
//struct HSV_Threshold{
//	int V_max;
//	int V_min;
//	int S_max;
//	int S_min;
//};
//
//void CAM(char* filename);
//int histogram(int argc, char **argv);
//int histogram2(IplImage* img);
//void get_imageData(char* filename, IplImage* img);
//void Mouse(int event, int x, int y, int flags, void *param);
//void threshold(IplImage* img, IplImage* t_img, char* filename);
//void cutImage(IplImage* img, IplImage* cut_img);
//void makeDirectory(const char* dirName);
//int slope_table(int angle);
//int slope_revision(IplImage* img, Rect data);
//double inc_y_table(int angle);
//int Bresenhams_line_algorithm(IplImage* img, Point data);
//int cutout(IplImage* threshold_img, cutData *cut);
//char *ocr(IplImage* result_img);
//
///*時間の取得*/
//time_t now = time(NULL);
//struct tm *pnow = localtime(&now);
//
//int main()
//{
//	cv::VideoCapture cap;
//	cv::Size cap_size(640, 480);
//	cap.set(CV_CAP_PROP_FRAME_WIDTH, cap_size.width);
//	cap.set(CV_CAP_PROP_FRAME_HEIGHT, cap_size.height);
//
//	IplImage image, *gray_image, *AfterRotation, *threshold_image, *point_image, *cutOut_image, eximg;
//	//Data data;
//	cutData cut;
//	const char *directoryName;
//	char fileName[100], gray_fileName[100], threshold_fileName[100], point_fileName[100], result_fileName[100];
//	Rect roi;
//	Point  Notice_coordinates;
//	CvMat* rotationMat;
//	int ans_angle = 0.0;
//
//	Point2f be_center;
//
//	char *result_word;
//	const char *ansText = " ";
//
//	Mat channels[3];
//	Mat dst;
//	Rect mask_size(0, 0, 640, 480);
//	cv::Rect trackWindow(0, 0, 640, 480);
//	int	color_check = 0;
//	int count;
//	ofstream ofs("ans_word.txt");
//
//	//int * ch = {1};
//	// 最初に見つかったカメラを開く
//	cap.open(1);
//	if (!cap.isOpened()) {
//		std::cout << "カメラの初期化に失敗しました" << std::endl;
//		return -1;
//	}
//
//	//閾値
//	HSV_Threshold hsv_threshold;
//	hsv_threshold.S_min = 30;
//	hsv_threshold.V_max = 240;
//	hsv_threshold.V_min = 30;
//	cv::namedWindow("Histogram");
//
//
//	//追跡画像の設定
//	track_color_data data = { 0 };
//	Mat track_color_img = (cv::Mat_<float>(16, 1) << 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
//
//	//ヒストグラム
//
//	cv::Mat hist;
//	cv::Mat histimg = cv::Mat::zeros(200, 320, CV_8UC3);
//	int hsize = 16;
//	float hranges[] = { 0, 180 };
//	const float* phranges = hranges;
//
//	//// ヒストグラムを計算
//	hist = track_color_img;
//	cout << hist << endl;
//
//	// 表示用にRGBに変換
//	histimg = cv::Scalar::all(0);
//	int binW = histimg.cols / hsize;
//	cv::Mat buf(1, hsize, CV_8UC3);
//	for (int i = 0; i < hsize; i++){
//		buf.at<cv::Vec3b>(i) = cv::Vec3b(cv::saturate_cast<uchar>(i*180. / hsize), 255, 255);
//	}
//	cv::cvtColor(buf, buf, cv::COLOR_HSV2BGR);
//
//	// ヒストグラムを描画
//	for (int i = 0; i < hsize; i++) {
//		int val = cv::saturate_cast<int>(hist.at<float>(i)*histimg.rows / 255);
//		cv::rectangle(histimg, cv::Point(i*binW, histimg.rows), cv::Point((i + 1)*binW, histimg.rows - val), cv::Scalar(buf.at<cv::Vec3b>(i)), -1, 8);
//	}
//
//	cv::imshow("Histogram", histimg);
//	waitKey(0);
//	// ビデオライタ
//	int fps = 30;
//	cv::VideoWriter writer("capture1.wmv", CV_FOURCC('X', 'V', 'I', 'D'), fps, cap_size);
//	cv::namedWindow("Capture");
//
//	while (1) {
//		color_check = 0;
//		// 画像を取得
//		cv::Mat frame, bef;
//		cap >> frame;
//		frame.copyTo(bef);
//		if (frame.empty()) break;
//
//		// HSVに変換
//		cv::Mat hsv;
//		cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
//		// 閾値処理
//		cv::Mat mask;
//		cv::Scalar lower(0, hsv_threshold.S_min, MIN(hsv_threshold.V_min, hsv_threshold.V_max));
//		cv::Scalar upper(180, 256, MAX(hsv_threshold.V_min, hsv_threshold.V_max));
//		cv::inRange(hsv, lower, upper, mask);
//
//		// Hueだけ抽出
//		int ch[] = { 0, 0 };
//		cv::Mat hue(hsv.size(), hsv.depth());
//		mixChannels(&hsv, 1, &hue, 1, ch, 1);
//
//		//cout << hsv.cols << " " << hsv.rows << endl;
//		for (int i = 0; i < hsv.rows; i++){
//			for (int j = 0; j < hsv.cols; j++){
//				int index = hsv.step*i + (j * 3);
//				if (hsv.data[index]<10 &&   //Hの範囲指定
//					120<hsv.data[index + 1] && //Sの範囲指定
//					120<hsv.data[index + 2]){ //Vの範囲指定
//					color_check = 1;
//
//					break;
//				}
//				//cout << "ok" << endl;
//				/*ofs << int(channels[0].data[i, j]) << " ";*/
//			}
//			/*ofs << " "<< endl;*/
//		}
//		//cout << channels[0].cols << " " << channels[0].rows << endl;
//		//cout << color_check << endl;
//
//		if (color_check){
//			//バックプロジェクション
//			cv::Mat backproj;
//			cv::calcBackProject(&hue, 1, 0, hist, backproj, &phranges);
//			backproj &= mask;
//
//			//// CamShiftアルゴリズム
//			cv::RotatedRect trackBox = cv::CamShift(backproj, trackWindow, cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 10, 1));
//
//			/*cout << trackBox.angle << " " << trackBox.size << endl;
//			cout << Notice_coordinates.x <<" "<<Notice_coordinates.y << endl;*/
//			if (be_center == trackBox.center){
//				count++;
//			}
//			else {
//				count = 0;
//			}
//			be_center = trackBox.center;
//
//
//			if (count == 2){
//
//				/*Notice_coordinates.x = trackBox.center.x + ((trackBox.size.height / 2) * cos(trackBox.angle*(M_PI / 180)+(M_PI)/2));
//				Notice_coordinates.y = trackBox.center.y + ((trackBox.size.height / 2) * sin(trackBox.angle*(M_PI / 180)+(M_PI) / 2));*/
//				cout << trackBox.angle << " x." << Notice_coordinates.x << " y." << Notice_coordinates.y << " c.x" << trackBox.center.x << " c.y" << trackBox.center.y << " " << trackBox.size.height / 2 << endl;
//				/*保存ファイルの作成*/
//				sprintf(fileName, "Outimage_%d%02d%02d%02d%02d.jpg",
//					pnow->tm_year + 1900, pnow->tm_mon + 1, pnow->tm_mday, pnow->tm_hour, pnow->tm_min);
//				printf("%s\n", fileName);
//				/*ディレクトリの作成*/
//				directoryName = "Image";
//				makeDirectory(directoryName);
//				/*画像の代入*/
//				image = frame;
//				/*画像保存*/
//				cvSaveImage(fileName, &image);
//				/*グレースケールの画像作成，表示，保存*/
//				gray_image = cvCreateImage(cvGetSize(&image), IPL_DEPTH_8U, 1);
//				gray_image = cvLoadImage(fileName, CV_LOAD_IMAGE_GRAYSCALE);
//				cvNamedWindow("Gray_Image", CV_WINDOW_AUTOSIZE);
//				cvShowImage("Gray_Image", gray_image);
//				sprintf(gray_fileName, "Gray_%s", fileName);
//				printf("%s\n", gray_fileName);
//				cvSaveImage(gray_fileName, gray_image);
//				/*傾き補正*/
//				ans_angle = Bresenhams_line_algorithm(gray_image, Notice_coordinates);
//				std::cout << "修正角度" << ans_angle << "°" << std::endl;
//				/*傾きに応じて画像を回転*/
//				AfterRotation = cvCreateImage(cvGetSize(gray_image), IPL_DEPTH_8U, 1);
//				rotationMat = cvCreateMat(2, 3, CV_32FC1);
//				cv2DRotationMatrix(cvPoint2D32f(Notice_coordinates.x, Notice_coordinates.y), ans_angle, 1, rotationMat);
//				cvWarpAffine(gray_image, AfterRotation, rotationMat);
//				/*回転後画像の表示*/
//				cvNamedWindow("rotation", CV_WINDOW_AUTOSIZE);
//				cvShowImage("rotation", AfterRotation);
//				cvSaveImage("rotation.bmp", AfterRotation);
//
//				/*大まかな切り出し処理*/
//				/*ディレクトリの作成*/
//				directoryName = "PointImage";
//				makeDirectory(directoryName);
//				/*大まかな切り出し処理*/
//				//cvSetImageROI(AfterRotation, cvRect(data.x - 50, data.y - 25, 100, 50));
//				cvSetImageROI(AfterRotation, cvRect(Notice_coordinates.x - 100, Notice_coordinates.y - 50, 200, 100));
//				point_image = cvCreateImage(cvGetSize(AfterRotation), IPL_DEPTH_8U, 1);
//				printf("%d:%d\n", point_image->height, point_image->width);
//				cvCopy(AfterRotation, point_image);
//				cvResetImageROI(AfterRotation);
//				/*保存ファイルの作成*/
//				sprintf(point_fileName, "Point_Outimage_%d%02d%02d%02d%02d.jpg",
//					pnow->tm_year + 1900, pnow->tm_mon + 1, pnow->tm_mday, pnow->tm_hour, pnow->tm_min);
//				cvSaveImage(point_fileName, point_image);
//
//				/*輝度の取得（切り出し後）*/
//				directoryName = "PointImageData";
//				makeDirectory(directoryName);
//				get_imageData(point_fileName, point_image);
//				/*histogram2(point_image);*/
//
//				/*切り出し後の二値化*/
//				directoryName = "ThresholdPointImage";
//				makeDirectory(directoryName);
//				threshold_image = cvCreateImage(cvGetSize(point_image), IPL_DEPTH_8U, 1);
//				threshold(point_image, threshold_image, point_fileName);
//				cvNamedWindow("threshold_Image", CV_WINDOW_AUTOSIZE);
//				cvShowImage("threshold_Image", threshold_image);
//
//				/*輝度を用いて単語切り出し*/
//				cutOut_image = cvCreateImage(cvGetSize(threshold_image), IPL_DEPTH_8U, 1);
//				cutout(threshold_image, &cut);
//				cvSetImageROI(threshold_image, cvRect(cut.x, cut.y, cut.width, cut.hight));
//				cutOut_image = cvCreateImage(cvGetSize(threshold_image), IPL_DEPTH_8U, 1);
//				cvCopy(threshold_image, cutOut_image);
//				cvResetImageROI(threshold_image);
//				/*切り出し後画像を表示*/
//				cvNamedWindow("cutOut", CV_WINDOW_AUTOSIZE);
//				cvShowImage("cutOut", cutOut_image);
//				/*画像を保存*/
//				sprintf(result_fileName, "result_Outimage_%d%02d%02d%02d%02d.jpg",
//					pnow->tm_year + 1900, pnow->tm_mon + 1, pnow->tm_mday, pnow->tm_hour, pnow->tm_min);
//				cvSaveImage(result_fileName, cutOut_image);
//
//				/*文字認識*/
//				result_word = ocr(cutOut_image);
//
//				/*翻訳*/
//				// Can connect to the Bing Translator?
//				BingTranslate::RESULT Result;
//				if (BingTranslate::Translator::CanConnect(Result))
//				{
//					printf("Connected via: \"%s\".\n", Result.Info.c_str());
//
//					// Instance translator
//					BingTranslate::Translator BingX(MY_ID, MY_KEY, Result);
//					if (Result.IsSuccess())
//					{
//						// Translate string			
//						if (BingX.Translate(result_word, -1, "en", "ja", Result))
//						{
//							printf("\nResult: \"%s\".\n", Result.Info.c_str());
//							ofs << "tesseract : " << result_word << endl;
//							ofs << "Microsoft Translator : " << Result.Info.c_str() << endl;
//							ansText = Result.Info.c_str();
//						}
//					}
//
//					if (!Result.IsSuccess())
//						printf("\nError:\n%s\n", Result.Info.c_str());
//				}
//				else
//				{
//					printf("Error! Can't connect to the Bing Translator.\n");
//					printf("Reason: \"%s\".\n", Result.Info.c_str());
//				}
//			}
//
//			// 表示
//			//ellipse(frame, trackBox, cv::Scalar(0, 0, 255), 3, 16); // cv::LINE_AA=16
//			//矢印の終点計算
//			Point end;
//			Notice_coordinates.x = trackBox.center.x + ((50 + trackBox.size.height / 2) * cos(trackBox.angle*(M_PI / 180) + (M_PI) / 2));
//			Notice_coordinates.y = trackBox.center.y + ((50 + trackBox.size.height / 2) * sin(trackBox.angle*(M_PI / 180) + (M_PI) / 2));
//			cv::putText(bef, ansText, cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 0, 200), 2, CV_AA);
//			/*Notice_coordinates.x = (50 + trackBox.size.height) * cos(trackBox.angle*(M_PI / 180) + (M_PI) / 2) + trackBox.center.x;
//			Notice_coordinates.y = (50 + trackBox.size.height)* cos(trackBox.angle*(M_PI / 180) + (M_PI) / 2) + trackBox.center.y;*/
//			arrowedLine(bef, trackBox.center, Notice_coordinates, cv::Scalar(200, 0, 0), 5, CV_AA);
//			eximg = bef;
//			cvSaveImage("example_cup.bmp", &eximg);
//
//			// 選択領域を表示
//			if (data.selectObject && data.selection.width > 0 && data.selection.height > 0) {
//				cv::Rect roi = data.selection & cv::Rect(0, 0, frame.cols, frame.rows);
//				cv::Mat tmp(frame, roi);
//				cv::bitwise_not(tmp, tmp);
//			}
//		}
//		//表示
//		writer << bef;
//		cv::imshow("Capture", bef);
//		if (cv::waitKey(30) >= 0) {
//			cv::imwrite("cap.png", bef);
//			break;
//		}
//	}
//
//	return 0;
//}
//
//void makeDirectory(const char *dirName)
//{
//	/*日付からディレクトリ作成*/
//	char directoryName[100];
//	sprintf(directoryName, "%s_%d%02d%02d",
//		dirName, pnow->tm_year + 1900, pnow->tm_mon + 1, pnow->tm_mday);
//	_mkdir(directoryName);
//	_chdir(directoryName);
//}
//
//void CAM(char* filename)
//{
//	const double WIDTH = 640;  // 幅
//	const double HEIGHT = 480; // 高さ
//	const int CAMERANUM = 0; // カメラ番号
//	/*画像関係*/
//	CvCapture *capture = NULL;
//	IplImage *frame = 0;
//	int ch;
//
//	// カメラ接続、幅と高さの設定
//	capture = cvCreateCameraCapture(CAMERANUM);
//	cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, WIDTH);
//	cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, HEIGHT);
//	namedWindow("Capture", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
//	while (1) {
//		frame = cvQueryFrame(capture);
//		cvShowImage("Capture", frame);
//		ch = cvWaitKey(1); // 0 はディレイ時間 (ミリ秒単位)
//		if (ch == 's'){
//			cvSaveImage(filename, frame);
//		}
//		if (ch == '\x1b') {
//			// ESC キー
//			break;
//		}
//	}
//	cvReleaseCapture(&capture);
//	cvDestroyWindow("Capture");
//}
//
//void get_imageData(char* filename, IplImage* img)
//{
//	int x, y, i = 0;
//	char out_FileName[100];
//	sprintf(out_FileName, "Data_%s.txt", filename);
//	std::ofstream ofs(out_FileName);
//
//	uchar p[3];
//
//	uchar* bank = new uchar[img->height*img->width];
//	for (y = 0; y < img->height; y++) {
//		for (x = 0; x < img->width; x++) {
//			/* 画素値を直接操作する一例 */
//			//p[0] = img->imageData[img->widthStep * y + x * 3];        // B
//			//p[1] = img->imageData[img->widthStep * y + x * 3 + 1];    // G
//			//p[2] = img->imageData[img->widthStep * y + x * 3 + 2];    // R
//			i++;
//			//bank[i] = 0.144*p[0] + 0.587*p[1] + 0.299*p[2];
//			bank[i] = img->imageData[img->widthStep * y + x];
//			//ofs << std::dec << static_cast<int>(bank[i]) << std::endl;
//		}
//	}
//}
//
//void Mouse(int event, int x, int y, int flags, void *param)
//{
//	Data* data = (Data*)param;
//	switch (event){
//	case CV_EVENT_LBUTTONDOWN:
//	{
//								 std::cout << x << "," << y << "\n";
//								 data->x = x;
//								 data->y = y;
//								 break;
//	}
//	default:
//	{
//			   break;
//	}
//	}
//}
//
//void threshold(IplImage* img, IplImage* t_img, char* filename)
//{
//	IplImage *src_img = 0, *dst_img;
//	int x;
//	char exfilename[100];/*拡張子付きファイル名*/
//
//	//cvSmooth (src_img, src_img, CV_GAUSSIAN, 5);//平均化
//
//	//cvThreshold(img, t_img, 100, 255, CV_THRESH_BINARY);
//	//sprintf(exfilename, "100_%s", filename);
//	//cvSaveImage(exfilename, t_img);
//
//	//cvThreshold(img, t_img, 50, 255, CV_THRESH_BINARY);
//	//sprintf(exfilename, "50%s", filename);
//	//cvSaveImage(exfilename, t_img);
//
//	//cvThreshold(img, t_img, 75, 255, CV_THRESH_BINARY);
//	//sprintf(exfilename, "75%s", filename);
//	//cvSaveImage(exfilename, t_img);
//
//	// (1)輝度平均
//	//x = cvThreshold(img, t_img, 127, 255, CV_THRESH_BINARY);
//	//sprintf(exfilename, "ave%d%s", x, filename);
//	//cvSaveImage(exfilename, t_img);
//
//	// (2)大津の手法
//	x = cvThreshold(img, t_img, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
//	sprintf(exfilename, "Otsu%d%s", x, filename);
//	cvSaveImage(exfilename, t_img);
//}
//
//int slope_table(int angle){
//	int table[11] = { 0, 57, 28, 19, 14,
//		11, 9, 8, 7, 6, 5 };
//	if (angle < 0){
//		return table[abs(angle)] * -1;
//	}
//	else {
//		return table[angle];
//	}
//}
//
//double inc_y_table(int angle){
//	double table[21] = { 0, 57.29, 28.64, 19.08, 14.30,
//		11.43, 9.51, 8.14, 7.12, 6.31,
//		5.67, 5.14, 4.70, 4.33, 4.01,
//		3.73, 3.49, 3.27, 3.08, 2.90, 2.75 };
//	if (angle < 0){
//		return table[abs(angle)] * -1;
//	}
//	else {
//		return table[angle];
//	}
//}
//
//int slope_revision(IplImage* img, Data data){ /*いくつすすんだら1上がるか*/
//	int angle = 0;
//	int i = 0, j = 0;
//	int p = 0, m = 0;
//	int plusSearchArea = search_area / 2, minusSearchArea = search_area / 2;
//	unsigned int sum = 0, bank = 0;
//	int slope = 0;
//	double ave = 0.0;
//	double max = 0.0, min = 0.0;
//	double ans = 0.0, max_ans = 0.0;
//	int slopeAngle = 0;
//	std::ofstream ofs("slopecheack.txt");
//
//	/*傾き探索範囲の調整*/
//	if (data.x - search_area / 2 < 0){/*画像サイズよりはみ出たら*/
//		plusSearchArea = abs(data.x - search_area / 2) + search_area / 2;
//		minusSearchArea = data.x;
//	}
//	else if ((data.x + search_area / 2) > 640){ /*画像サイズよりはみ出たら*/
//		plusSearchArea = 640 - data.x;
//		minusSearchArea = ((data.x + search_area / 2) - 640) + search_area / 2;
//	}
//	/*平均輝度計算*/
//	for (angle = -10; angle < 11; angle++){
//		//ofs << "傾き" << angle << std::endl;
//		max = 0;
//		min = 0;
//		/*y軸 切片移動*/
//		for (i = -10; i < 11; i++){
//			//ofs << "切片" << i << std::endl;
//			sum = 0;
//			/*x軸－方向*/
//			for (m = 1; m < minusSearchArea; m++){
//				if (angle == 0){
//					slope = 0;
//				}
//				else {
//					slope = m / slope_table(angle);/*傾き計算*/
//				}
//				bank = (uchar)img->imageData[img->widthStep *(data.y + i - slope) + data.x - m];
//				sum += (uchar)img->imageData[img->widthStep *(data.y + i - slope) + data.x - m];
//				//std::cout << "a" << std::endl;	
//			}
//			/*x軸＋方向*/
//			for (p = 1; p < plusSearchArea; p++){
//				if (angle == 0){
//					slope = 0;
//				}
//				else {
//					slope = p / slope_table(angle);/*傾き計算*/
//				}
//				bank = (uchar)img->imageData[img->widthStep *(data.y + i + slope) + data.x + p];
//				sum += (uchar)img->imageData[img->widthStep *(data.y + i + slope) + data.x + p];
//				//std::cout << "b" << std::endl;
//			}
//			//std::cout << sum << std::endl;
//			//ofs << sum << std::endl;
//			ave = sum / search_area;
//			if (i == -10){
//				max = ave;
//				min = ave;
//			}
//			else{
//
//				if (max<ave) max = ave;
//				if (min>ave) min = ave;
//			}
//		}
//		ans = abs(max - min);
//		if (ans>max_ans) {
//			max_ans = ans;
//			slopeAngle = angle;
//		}
//	}
//
//	return slopeAngle;
//}
//
//int Bresenhams_line_algorithm(IplImage* img, Point data){
//
//	int plusSearchArea = search_area / 2, minusSearchArea = search_area / 2;
//	int angle = 0;
//	int i, j;
//	int sum = 0, sub = 0;
//	int over = 0;
//	int dx = 0, dy = 0;
//	int x = 0, y = 0;
//	int D = 0;/*関数値*/
//	int quadrant = 0;/*傾き*/
//	double ave = 0.0, max = 0.0, min = 0.0;
//	double ans = 0.0, max_ans = 0.0;
//	int slopeAngle = 0;
//	std::ofstream ofs("BMsum.txt");
//
//	CvPoint startPoint = { 0 }, endPoint;
//
//
//	/*傾き探索範囲の調整*/
//	/*始点の決定(x座標)*/
//	sub = data.x - search_area / 2;
//	if (sub < 0){/*画像サイズよりはみ出たら*/
//		startPoint.x = 0;
//		over = sub*-1;
//	}
//	else{
//		startPoint.x = data.x - search_area / 2;
//		over = 0;
//	}
//	/*終点の決定(x座標)*/
//	endPoint.x = startPoint.x + search_area + over;
//	if (endPoint.x>640){/*画像サイズよりはみ出たら*/
//		endPoint.x = 640;
//	}
//
//	for (angle = -20; angle <21; angle++){
//		//ofs << "傾き" << angle << std::endl;
//		max = 0;
//		min = 0;
//		for (j = -10; j < 11; j++){/*切片の移動*/
//			sum = 0;
//			/*始点の決定(y座標)*/
//			if (angle == 0){
//				startPoint.y = data.y + j;
//			}
//			else{
//				startPoint.y = data.y + ((startPoint.x - data.x) / inc_y_table(angle)) + j;
//			}
//			/*終点の決定(y座標)*/
//			if (angle == 0){
//				endPoint.y = data.y + j;
//			}
//			else{
//				endPoint.y = data.y + ((endPoint.x - data.x) / inc_y_table(angle)) + j;
//			}
//			//std::cout << "始点(" << startPoint.x - data.x << "," << startPoint.y - data.y << ")" 
//			//<< " " << "終点(" << endPoint.x - data.x << "," << endPoint.y - data.y<< ")" << std::endl;
//			/*傾きの方向決定*/
//			dx = (endPoint.x - data.x) - (startPoint.x - data.x);
//			dy = (endPoint.y - data.y) - (startPoint.y - data.y);
//			//std::cout <<dy<<","<<dx<<std::endl;
//			if (((double)dy / (double)dx)>0){
//				quadrant = 1;
//				/*std::cout << "傾き正" << std::endl;*/
//			}
//			else {
//				quadrant = 2;
//				/*std::cout << "傾き負" << std::endl;*/
//			}
//			/*std::cout << "quadrant=" << quadrant << std::endl;*/
//			/*ブレゼンハムのアルゴリズム*/
//			/*始点時の計算*/
//			D = 2 * dy - dx;
//			y = startPoint.y;
//			cout << startPoint.x << " " << startPoint.y << endl;
//			sum += (uchar)img->imageData[img->widthStep *startPoint.y + startPoint.x];
//			/*傾きに応じた処理*/
//			switch (quadrant)
//			{
//			case 1:/*傾き正*/
//				for (x = startPoint.x + 1; x < endPoint.x + 1; x++){
//					if (D>0){
//						y = y + 1;
//						sum += (uchar)img->imageData[img->widthStep *(y)+x];
//						D = D + (2 * dy - 2 * dx);
//					}
//					else {
//						sum += (uchar)img->imageData[img->widthStep *(y)+x];
//						D = D + (2 * dy);
//					}
//					//std::cout << "D=" << D << "x=" << x << "y=" << y << std::endl;
//				}
//				break;
//			case 2:/*傾き負*/
//				for (x = startPoint.x + 1; x < endPoint.x + 1; x++){
//					if (D>0){
//						y = y - 1;
//						sum += (uchar)img->imageData[img->widthStep *(y)+x];
//						D = D + (2 * (-1)* dy - 2 * dx);
//					}
//					else {
//						sum += (uchar)img->imageData[img->widthStep *(y)+x];
//						D = D + (2 * (-1)* dy);
//					}
//					//std::cout << "D=" << D << "x=" << x << "y=" << y << std::endl;
//				}
//				break;
//			default:
//				break;
//			}
//			//ofs << sum << std::endl;
//			ave = sum;
//			if (j == -10){
//				max = ave;
//				min = ave;
//			}
//			else{
//				if (max<ave) max = ave;
//				if (min>ave) min = ave;
//			}
//		}
//		ans = abs(max - min);
//		//ofs  << ans << std::endl;
//		if (ans>max_ans) {
//			max_ans = ans;
//			slopeAngle = angle;
//		}
//		//std::cout << "slope" << slopeAngle << std::endl;
//	}
//
//	return slopeAngle;
//}
//
//void bobsort(totalData x[], int n){
//	totalData temp;
//	for (int i = 0; i < n - 1; i++) {
//		for (int j = n - 1; j > i; j--) {
//			if (x[j - 1].total < x[j].total) {  /* 前の要素の方が大きかったら */
//				temp = x[j];        /* 交換する */
//				x[j] = x[j - 1];
//				x[j - 1] = temp;
//			}
//		}
//	}
//}
//
//int cutout(IplImage* threshold_img, cutData *cut)
//{
//	unsigned int sum = 0;
//	char cutData_fileName[100];
//	double ave = 0.0;
//	double last_ave = 0.0;
//	double sub = 0.0, half = 0.0;
//	int i = 0, j = 0, k = 0;
//	int flag = 0;
//	int s = 0, m = 0, l = 0, n = 0;
//	int count = 0, bcount = 0;
//	int num = 0, brank_num = 0;
//	int lineStart[10];
//	int maxBrightness = 0;
//	int maxBrightness_EndPoint = 0;
//	int y_th = 0;
//	int pre_sum;
//	totalData bank[300], start, end, temp, edge[2];
//	Rect y_axis_brank[10];
//
//	sprintf(cutData_fileName, "cutData_%d%02d%02d%02d%02d.txt",
//		pnow->tm_year + 1900, pnow->tm_mon + 1, pnow->tm_mday, pnow->tm_hour, pnow->tm_min);
//	std::ofstream ofs(cutData_fileName);
//
//	/*y方向*/
//	y_th = threshold_img->width * 255;
//	for (int j = 0; j < threshold_img->width; j++){
//		sum += (uchar)threshold_img->imageData[threshold_img->widthStep * 0 + j];
//	}
//	cout << sum << endl;
//	for (int i = 0; i < rough_cut_area_hight; i++){
//		sum = 0;
//		for (int j = 0; j < threshold_img->width; j++){
//			sum += (uchar)threshold_img->imageData[threshold_img->widthStep * i + j];
//		}
//		cout << sum << endl;
//		if (sum >= y_th && count == 0){
//			y_axis_brank[brank_num].y = i;
//			pre_sum = sum;
//			cout << "okokokokokokok" << endl;
//		}
//		if (sum >= y_th){
//			count++;
//			cout << count << endl;
//		}
//		if (sum<y_th && count>1){
//			y_axis_brank[brank_num].height = count;
//			cout << "brank_num=" << brank_num << "y=" << y_axis_brank[brank_num].y << "hight=" << y_axis_brank[brank_num].height << endl;
//			brank_num++;
//			count = 0;
//		}
//	}
//	for (int num = 0; num < brank_num - 1; num++){
//		if (y_axis_brank[num].y < rough_cut_area_hight / 2 && rough_cut_area_hight / 2  < (y_axis_brank[num + 1].y + y_axis_brank[num + 1].height)){
//			cut->y = y_axis_brank[num].y + (y_axis_brank[num].height / 2);
//			cut->hight = y_axis_brank[num + 1].y + (y_axis_brank[num + 1].height / 2) - cut->y;
//			break;
//		}
//	}
//	//for (int i=0;i<rough_cut_area_hight/2;i++){/*50は大まかに切り出したときのサイズ200x100の中心座標(100,50)*/
//	//	for (int j = 0; j < threshold_img->width; j++){
//	//		sum += (uchar)threshold_img->imageData[threshold_img->widthStep * (rough_cut_area_hight / 2 - i) + j];
//	//	}
//	//	if (sum>maxBrightness){
//	//		maxBrightness = sum;
//	//		maxBrightness_EndPoint = i;
//	//	}
//	//	else if (maxBrightness == sum){
//	//		count++;
//	//	}
//	//	else if (sum < maxBrightness / 2){
//	//		break;
//	//	}
//	//	sum = 0;
//	//}
//	//cut->y = (rough_cut_area_hight / 2) - (maxBrightness_EndPoint+count/2);
//	//count = 0;
//	//maxBrightness = 0;
//	//sum = 0;
//	//for (int i = 0; i < rough_cut_area_hight / 2; i++){
//	//	for (int j = 0; j < threshold_img->width; j++){
//	//		sum += (uchar)threshold_img->imageData[threshold_img->widthStep * (rough_cut_area_hight / 2 + i) + j];
//	//	}
//	//	if (sum>maxBrightness){
//	//		maxBrightness = sum;
//	//		maxBrightness_EndPoint = i;
//	//	}
//	//	else if (maxBrightness == sum){
//	//		count++;
//	//	}
//	//	else if (sum < maxBrightness / 2){
//	//		break;
//	//	}
//	//	std::cout << maxBrightness_EndPoint << " " << maxBrightness << std::endl;
//	//	sum = 0;
//	//}
//	//cut->hight = (rough_cut_area_hight / 2) + (maxBrightness_EndPoint+count/2) - cut->y;
//	//count = 0;
//	//sum = 0;
//	std::cout << "y=" << cut->y << " , " << "hight=" << cut->hight << std::endl;
//	std::cout << "aaaaaaaaaaaaaaaaaaaaaaaaaaaaa" << std::endl;
//
//	cvSetImageROI(threshold_img, cvRect(0, cut->y, rough_cut_area_hight, cut->hight));
//
//	//	/*画像内の枠線（余分）の除去*/
//	//	/*直線の開始地点の探索*/
//	//	for (int i = threshold_img->widthStep * cut->y; i < threshold_img->widthStep * cut->y + 100; i++){
//	//		if (threshold_img->imageData[i] == 255){
//	//			lineStart[num] = i;
//	//			num++;
//	//		}
//	//	}
//	//	std::cout << "lineNum=" << num << std::endl;
//	//
//	//	/*for (int i = 0; i < num; i++){
//	//		for ()
//	//	}
//	//*/
//
//	/*x方向*/
//	for (int i = 0; i < threshold_img->width; i++){
//		sum = 0;
//		for (int j = cut->y; j < cut->hight + cut->y; j++){
//			sum += (uchar)threshold_img->imageData[threshold_img->widthStep * j + i];
//		}
//		ofs << sum << std::endl;
//		if (sum == 255 * cut->hight){
//			count++;
//			flag = 1;
//		}
//		else if (sum != 255 * cut->hight  && flag == 1){
//			bank[bcount].total = count;
//			count = 0;
//			bank[bcount].x = i;
//			bcount++;
//			flag = 0;
//		}
//		//右端の処理
//		if (i == threshold_img->width - 1 && flag == 1){
//			bank[bcount].total = count;
//			count = 0;
//			bank[bcount].x = i;
//			bcount++;
//		}
//	}
//	cvResetImageROI(threshold_img);
//	//std::cout << bcount << std::endl;
//	for (int i = 0; i < bcount; i++){
//		std::cout << "x=" << bank[i].x << " , " << "total=" << bank[i].total << std::endl;
//		ofs << bank[i].x << " " << bank[i].total << std::endl;
//	}
//
//	//端を除外
//	if (bank[0].x - bank[0].total == 0) {
//		edge[0] = bank[0];
//		bank[0].total = 0;
//		bank[0].x = 0;
//	}
//	if (bank[bcount - 1].x == 199) {
//		edge[1] = bank[bcount - 1];
//		bank[bcount - 1].total = 0;
//		bank[bcount - 1].x = 0;
//	}
//
//	//バブルソート
//	bobsort(bank, bcount);
//
//	std::cout << "AfterSort" << std::endl;
//	ofs << "afterSort" << std::endl;
//	for (int i = 0; i < bcount; i++){
//		std::cout << "x=" << bank[i].x << " , " << "total=" << bank[i].total << std::endl;
//		ofs << bank[i].x << " " << bank[i].total << std::endl;
//	}
//	//startの決定
//	start.total = 0;
//	start.x = 0;
//	if (bank[0].x < rough_cut_area_width / 2){
//		start = bank[0];
//	}
//	for (i = 1; i < bcount; i++){
//		sub = bank[i - 1].total - bank[i].total;
//		half = bank[i].total / 2.0;
//		std::cout << sub << "," << half << "," << start.x << "," << bank[i].x << std::endl;
//		if (sub < half && start.x < bank[i].x && rough_cut_area_width / 2> bank[i].x){
//			std::cout << "1" << std::endl;
//			start = bank[i];
//		}
//		else if (sub >= half){
//			break;
//		}
//	}
//	std::cout << "start=" << start.x << " , " << "end=" << end.x << std::endl;
//
//	if (start.x == 0 || start.x>rough_cut_area_width / 2) start = edge[0];
//	//endの決定
//	end.total = 0;
//	end.x = rough_cut_area_width;
//	if (bank[0].x > rough_cut_area_width / 2){
//		end = bank[0];
//	}
//	for (i = 1; i < bcount; i++){
//		sub = bank[i - 1].total - bank[i].total;
//		half = bank[i].total / 2.0;
//		std::cout << sub << "," << half << "," << start.x << "," << bank[i].x << std::endl;
//		if (sub < half && end.x > bank[i].x && rough_cut_area_width / 2< bank[i].x){
//			std::cout << "1" << std::endl;
//			end = bank[i];
//		}
//		else if (sub >= half){
//			break;
//		}
//	}
//	std::cout << "start=" << start.x << " , " << "end=" << end.x << std::endl;
//
//	if (end.x == rough_cut_area_width || end.x<rough_cut_area_width / 2) end = edge[1];
//	std::cout << "start=" << start.x << " , " << "end=" << end.x << std::endl;
//
//	cut->x = start.x - 3;
//	cut->width = end.x - start.x + 3;
//	std::cout << "aaaaaaaaaaaaaaaaaaaaaaaaaaaaa" << std::endl;
//
//	std::cout << "CutRange " << "x=" << cut->x << " , " << "y=" << cut->y << " , " << "width=" << cut->width << " , " << "hight=" << cut->hight << std::endl;
//
//	return 0;
//}
//
//char *ocr(IplImage* result_img){
//	char *outText;
//	cv::Mat result = cv::cvarrToMat(result_img);
//	tesseract::TessBaseAPI *api = new tesseract::TessBaseAPI();
//	// Initialize tesseract-ocr with English, without specifying tessdata path
//	if (api->Init(NULL, "eng")) {
//		fprintf(stderr, "Could not initialize tesseract.\n");
//		exit(1);
//	}
//
//	api->SetImage(result.data, result.size().width, result.size().height, result.channels(), result.step1());
//	api->Recognize(0);
//	// Get OCR result
//	outText = api->GetUTF8Text();
//	//printf("OCR output: %s\n", outText);
//	std::cout << outText << std::endl;
//
//	// Destroy used object and release memory
//	api->End();
//
//	return outText;
//}