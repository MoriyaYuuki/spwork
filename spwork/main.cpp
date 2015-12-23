
#define _CRT_SECURE_NO_WARNINGS 
#define _USE_MATH_DEFINES

#include <opencv2/opencv.hpp>
#include <opencv2/opencv_lib.hpp>

#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv/cv.h"
#include "opencv/highgui.h"

#include <tesseract/baseapi.h>
#include <leptonica/allheaders.h>

#include <cvPutTextJP_Win2.h>

#include <Tchar.h>
#include <iostream>
#include <sstream> 
#include <fstream>
#include <string>
#include <string.h>
#include <time.h>
#include <direct.h>
#include <math.h>

#include "stdafx.h"
#include "BingTranslate.h"

using namespace cv;
using namespace std;

#define search_area 200 /*傾き探索範囲*/
#define rough_cut_area_width 200
#define rough_cut_area_hight 100

#define MY_ID  "spwork"
#define MY_KEY "/GHlsseZh16I8hFW0ao2SowLu81Wqm4OzgoSc2yLwHo="

#define camera_num 1

//#define MARKER_SIZE (20)       /* マーカーの外側の1辺のサイズ[mm] */

typedef struct _data{/*切り取り範囲の構造体*/
	int event, x = 0, y = 0, flag;
}Data;

typedef struct _cutdata{/*切り取り範囲の構造体*/
	int x = 0, y = 0, width = 0, hight = 0;
}cutData;

typedef struct _totaldata{/*切り取り範囲の構造体*/
	int x = 0, total = 0;
}totalData;

//追跡する色のデータ
struct track_color_data {
	int trackObject;
	bool selectObject;
	cv::Point origin;
	cv::Rect selection;
};

struct HSV_Threshold{
	int V_max;
	int V_min;
	int S_max;
	int S_min;
};

void CAM(char* filename);
int histogram(int argc, char **argv);
int histogram2(IplImage* img);
void get_imageData(char* filename, IplImage* img);
void Mouse(int event, int x, int y, int flags, void *param);
void threshold(IplImage* img, IplImage* t_img);//, char* filename);
void cutImage(IplImage* img, IplImage* cut_img);
void makeDirectory(const char* dirName);
int slope_table(int angle);
int slope_revision(IplImage* img, Rect data);
double inc_y_table(int angle);
int Bresenhams_line_algorithm(IplImage* img, Point data);
int cutout(IplImage* threshold_img, cutData *cut);
char *ocr(IplImage* result_img);
void meanShift_init();
void meanShift(Mat in_frame, Point *Notice_coordinates);
void Bing_Translator(char * in_Text ,char *out_Text);
BOOL ConvUtf8toSJis(BYTE* pSource, BYTE* pDist, int* pSize);
void check_OCR();
void testAR();
int Calibrate();
void take_pic();
void ARtracking(Mat in_frame, Point *Notice_coordinates);
void test_th();
void cut_img();
void on_mouse(int event, int x, int y, int flags, void* param);
vector<Mat> calculateIntegralHOG(const Mat& image);
void calculateHOGInCell(Mat& hogCell, Rect roi, const vector<Mat>& integrals);
Mat getHOG(Point pt, const vector<Mat>& integrals);
int test_Hog();
int init_Hogdata();
void get_Hogdata(Mat input_img, float *Hogdata[]);
void resize_img();
void get_Boost();
void getHog_another(Mat inImg, float* vec_tmp);

/*時間の取得*/
time_t now = time(NULL);
struct tm *pnow = localtime(&now);

/*meanshift用グローバル関数*/
HSV_Threshold hsv_threshold;
Mat meanshift_hist;
track_color_data data = { 0 };
float meanshift_hranges[] = { 0, 180 };

/*AR用*/
CvMat *intrinsic, *distortion;
CvMat *rotation = cvCreateMat(1, 3, CV_32FC1);
CvMat *translation = cvCreateMat(1, 3, CV_32FC1);
IplImage * mask0 = cvLoadImage("mask0deg.bmp", 0);
IplImage * mask90 = cvLoadImage("mask90deg.bmp", 0);
IplImage * mask180 = cvLoadImage("mask180deg.bmp", 0);
IplImage * mask270 = cvLoadImage("mask270deg.bmp", 0);
IplImage * tempmask = cvCloneImage(mask0);//作業用  

/*cut_img used*/
Rect selection;
int select_object;


/*Hog計算用*/
// ヒストグラムのビン数
#define N_BIN 9
// 何度ずつに分けて投票するか（分解能）
#define THETA (180 / N_BIN)
// セルの大きさ（ピクセル数）
#define CELL_SIZE 12
// ブロックの大きさ（セル数）奇数
#define BLOCK_SIZE 3
// ブロックの大きさの半分（ピクセル数）
#define R (CELL_SIZE*(BLOCK_SIZE)*0.5)

//Hog要素数
#define Hog_num 43740
//pos+neg data_num
#define Hog_data_num 198

//拡大倍率
#define Resize_Rate 4


int main()
{
	//動作確認用関数
	//check_OCR();//画像処理部
	//testAR();
	//take_pic();
	//Calibrate();
	//test_th();
	//cut_img();
	//resize_img();
	//test_Hog();
	//init_Hogdata();
	//get_Boost();


	Data mouse_Data;
	cutData cut;
	CvMat object_points;
	CvMat image_points;
	CvMat point_counts;
	char *translate_Text_View = " ";
	char total_Text_View[50] = " ";
	char str[100];
	Point  *Notice_coordinates;//特定単語座標
	Point  *preNotice_coordinates;//１つ前の特定単語座標
	Notice_coordinates = new Point;
	preNotice_coordinates = new Point;
	int stopCount = 0;
	int loopCount = 0;
	int ch;
	

	//ARの初期設定
	CvFileStorage *fs;
	CvFileNode *param;
	fs = cvOpenFileStorage("camera.xml", 0, CV_STORAGE_READ);
	param = cvGetFileNodeByName(fs, NULL, "intrinsic");
	intrinsic = (CvMat *)cvRead(fs, param);
	param = cvGetFileNodeByName(fs, NULL, "distortion");
	distortion = (CvMat *)cvRead(fs, param);
	cvReleaseFileStorage(&fs);

	//カメラの初期設定(画像サイズの設定)
	cv::VideoCapture cap;
	cv::Size cap_size(640, 480);
	cap.set(CV_CAP_PROP_FRAME_WIDTH, cap_size.width);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, cap_size.height);

	// 動画保存設定
	int fps = 8;
	//cvGetCaptureProperty((CvCapture *)cap, CV_CAP_PROP_FPS);

	//保存先の入力
	//ディレクトリ
	char saveDirectoryName[100];
	char saveInDirectoryName[100];
	cout << "Save_Directory_Name : " ;
	scanf_s("%s", saveDirectoryName,100);
	//結果.txt
	char saveResultWordName[100];
	cout << "Save_resultWord_Name(.txt) : " ;
	scanf_s("%s", saveResultWordName, 100);
	ofstream ofs(saveResultWordName);
	//動画の名前.avi
	char saveCaptureName[100];
	cout << "Save_Capture_Name(.avi) : ";
	scanf_s("%s", saveCaptureName, 100);
	VideoWriter writer(saveCaptureName, CV_FOURCC('X', 'V', 'I', 'D'), fps, cap_size);
	//保存画像の名前
	char saveImageName[100];
	cout << "Save_Image_Name : ";
	scanf_s("%s", saveImageName, 100);
	
	// 最初に見つかったカメラを開く
	cap.open(0);
	if (!cap.isOpened()) {
		cout << "カメラの初期化に失敗しました" << endl;
		return -1;
	}

	//色追跡の設定
	meanShift_init();

	//日本語表示の設定
	cvPutTextJP putTextJP;
	Point2i pos; //結果の表字座標
	pos.x = 0;
	pos.y = 0;
	putTextJP.setLocate(pos);

	cout << "動作開始" << endl;
	cvWaitKey();

	cv::namedWindow("Capture");
	while (1) {
		// 画像を取得
		Mat original_frame, copy_frame;
		Mat gray_Mat;
		cap >> original_frame;
		if (original_frame.empty()) break;
		original_frame.copyTo(copy_frame);
		//色追跡
		meanShift(copy_frame,Notice_coordinates);
		//ARマーカー追跡
		//ARtracking(copy_frame, Notice_coordinates);
		if (abs(Notice_coordinates->x - preNotice_coordinates->x) < 5  && abs(Notice_coordinates->y - preNotice_coordinates->y) <5){
			stopCount++;
		}
		/*cout << "stopCount = " <<stopCount << endl;
		cout << "座標x" << Notice_coordinates->x << "座標y" << Notice_coordinates->y << endl;*/
		if (stopCount ==100 ){
			sprintf(saveInDirectoryName, "%02d", loopCount+1);
			makeDirectory(saveInDirectoryName);
			//グレースケール化
			cvtColor(original_frame, gray_Mat, CV_BGR2GRAY);
			imwrite("original.png", original_frame);
			imwrite("gray.png", gray_Mat);
			IplImage gray_img = gray_Mat;
			//表示(デバック用)
			/*cvNamedWindow("Copy");
			cvShowImage("Copy", &gray_img);*/
			////マウスから座標入力
			//std::cout << "マウスで座標入力" << std::endl;
			//cvSetMouseCallback("Copy", Mouse, (void*)&mouse_Data);
			//cvWaitKey(0);
			//Point coordinates;
			//coordinates.x = mouse_Data.x;
			//coordinates.y = mouse_Data.y;
			//傾き補正
			int slopeValue = Bresenhams_line_algorithm(&gray_img, *Notice_coordinates);
			//std::cout << "修正角度" << slopeValue << "°" << std::endl;
			//画像の回転
			IplImage *afterRotation_img = cvCreateImage(cvGetSize(&gray_img), IPL_DEPTH_8U, 1);
			CvMat *rotationMat = cvCreateMat(2, 3, CV_32FC1);
			cv2DRotationMatrix(cvPoint2D32f(Notice_coordinates->x, Notice_coordinates->y), slopeValue, 1, rotationMat);
			cvWarpAffine(&gray_img, afterRotation_img, rotationMat, CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS, cvScalarAll(255));
			imwrite("rotaion.png", (Mat)afterRotation_img);
			/*大まかな切り出し処理*/
			cvSetImageROI(afterRotation_img, cvRect(Notice_coordinates->x - rough_cut_area_width / 2, Notice_coordinates->y - rough_cut_area_hight / 2, rough_cut_area_width, rough_cut_area_hight));
			IplImage *rough_cut_img = cvCreateImage(cvGetSize(afterRotation_img), IPL_DEPTH_8U, 1);
			cvCopy(afterRotation_img, rough_cut_img);
			cvResetImageROI(afterRotation_img);
			imwrite("rough_cut.png", (Mat)rough_cut_img);
			//cvWaitKey(0);
			//二値化
			IplImage *threshold_img = cvCreateImage(cvGetSize(rough_cut_img), IPL_DEPTH_8U, 1);
			//threshold(rough_cut_img, threshold_img);
			adaptiveThreshold((Mat)rough_cut_img, (Mat)threshold_img, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 7, 8);
			imwrite("threshold.png", (Mat)threshold_img);
			//cvNamedWindow("rough_th");
			//cvShowImage("rough_th", threshold_img);
			//waitKey();
			//単語の切り出し
			IplImage *cutOut_img = cvCreateImage(cvGetSize(threshold_img), IPL_DEPTH_8U, 1);
			cutout(threshold_img, &cut);
			cvSetImageROI(threshold_img, cvRect(cut.x, cut.y, cut.width, cut.hight));
			cutOut_img = cvCreateImage(cvGetSize(threshold_img), IPL_DEPTH_8U, 1);
			cvCopy(threshold_img, cutOut_img);
			cvResetImageROI(threshold_img);
			imwrite("cutOut.png", (Mat)cutOut_img);
			//cvNamedWindow("cutOut_img");
			//cvShowImage("cutOut_img", cutOut_img);
			//waitKey();
			//画像補間+鮮鋭化
			//4倍に拡大
			IplImage *cutOutResize_img = cvCreateImage(cvSize(cutOut_img->width * Resize_Rate, cutOut_img->height * Resize_Rate), IPL_DEPTH_8U, 1);
			resize((Mat)cutOut_img, (Mat)cutOutResize_img, cvSize(cutOut_img->width * Resize_Rate, cutOut_img->height * Resize_Rate), INTER_NEAREST);
			//鮮鋭化
			IplImage * cutOutResizeGFilter_img = cvCreateImage(cvSize(cutOutResize_img->width, cutOutResize_img->height), IPL_DEPTH_8U, 1);//縦横4倍;
			GaussianBlur((Mat)cutOutResize_img, (Mat)cutOutResizeGFilter_img, cv::Size(11, 11), 6, 6);
			IplImage *subTmp_img = cvCreateImage(cvSize(cutOutResize_img->width, cutOutResize_img->height ), IPL_DEPTH_8U, 1);
			IplImage *cutOutResizeUnsharp_img = cvCreateImage(cvSize(cutOutResize_img->width, cutOutResize_img->height), IPL_DEPTH_8U, 1);
			(Mat)subTmp_img = (Mat)cutOutResize_img - (Mat)cutOutResizeGFilter_img;
			(Mat)cutOutResizeUnsharp_img = (Mat)cutOutResize_img + ((Mat)subTmp_img * 60);

			//文字認識
			//char* result_Text = ocr(cutOut_img);
			char* result_Text = ocr(cutOutResizeUnsharp_img);
			//char* result_Text1 = ocr(unsharp_test_img2);
			//cout << "Normal : " << result_Text << "Nearest : " << result_Text1 << /*"Linear : " <<*/ /*result_Text2 <<
			//	"Cubic : " << result_Text3 << "Lanczos : " << result_Text4 <<*/ endl;
			//ofs << "Normal : " << result_Text << "Nearest : " << result_Text1 << /*"Linear : " << *//*result_Text2 <<
			//	"Cubic : " << result_Text3 << "Lanczos : " << result_Text4 << */endl;
			//sprintf(str, "%2d.bmp", loopCount+1);
			//cvSaveImage(str, unsharp_test_img2);
			
			//翻訳
			char translate_Text[100] = { " " };
			Bing_Translator(result_Text, translate_Text);
			//cout << translate_Text << endl;
			//標示用型変換
			int nSize = 0;
			ConvUtf8toSJis((BYTE*)(translate_Text), NULL, &nSize);
			BYTE* translate_Text_sjis = new BYTE[nSize + 1];
			ZeroMemory(translate_Text_sjis, nSize + 1);
			ConvUtf8toSJis((BYTE*)(translate_Text), translate_Text_sjis, &nSize);
			translate_Text_View = (char *)translate_Text_sjis;
			cout << translate_Text_View << endl;
			strtok(result_Text, "\n\0");
			strtok(translate_Text_View, "\n\0");
			sprintf_s(total_Text_View,50,"%s : %s",result_Text,translate_Text_View);
			//waitKey();

			ofs << "tesseract : " << result_Text << " , " << "Bing Translator : " << translate_Text << endl;
			stopCount = 0;
			loopCount++;
			_chdir("spwork");
		}
		//表示
		//cout << translate_Text_view << endl;
		rectangle(copy_frame, Point(0, 0), Point(270, 30), Scalar(255, 255, 255), -1, CV_AA);
		putTextJP.setLocate(pos);
		putTextJP.putText(copy_frame,total_Text_View, cvScalar(0, 0, 255));
		cv::imshow("Capture", copy_frame);
		writer << copy_frame;
		preNotice_coordinates->x = Notice_coordinates->x;
		preNotice_coordinates->y = Notice_coordinates->y;
		ch = cvWaitKey(1); // 0 はディレイ時間 (ミリ秒単位)
		//if (ch == '\x1b') {
		//	// ESC キー
		//	break;
		//}
		if (cv::waitKey(30) >= 0) {
			cv::imwrite("cap.png",original_frame);
			break;
		}
	}
	cvDestroyWindow("Capture");
	cap.release();
	writer.release();
	return 0;
}


void makeDirectory(const char *dirName)
{
	/*日付からディレクトリ作成*/
	char directoryName[100];
	sprintf(directoryName, "%s_%d%02d%02d",
		dirName, pnow->tm_year + 1900, pnow->tm_mon + 1, pnow->tm_mday);
	_mkdir(directoryName);
	_chdir(directoryName);
}

void CAM(char* filename)
{
	const double WIDTH = 640;  // 幅
	const double HEIGHT = 480; // 高さ
	const int CAMERANUM = 0; // カメラ番号
	/*画像関係*/
	CvCapture *capture = NULL;
	IplImage *frame = 0;
	int ch;

	// カメラ接続、幅と高さの設定
	capture = cvCreateCameraCapture(CAMERANUM);
	cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, WIDTH);
	cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, HEIGHT);
	namedWindow("Capture", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
	while (1) {
		frame = cvQueryFrame(capture);
		cvShowImage("Capture", frame);
		ch = cvWaitKey(1); // 0 はディレイ時間 (ミリ秒単位)
		if (ch == 's'){
			cvSaveImage(filename, frame);
		}
		if (ch == '\x1b') {
			// ESC キー
			break;
		}
	}
	cvReleaseCapture(&capture);
	cvDestroyWindow("Capture");

}

void get_imageData(char* filename, IplImage* img)
{
	int x, y, i = 0;
	char out_FileName[100];
	sprintf(out_FileName, "Data_%s.txt", filename);
	ofstream ofs(out_FileName);
	uchar* bank = new uchar[img->height*img->width];

	for (y = 0; y < img->height; y++) {
		for (x = 0; x < img->width; x++) {
			/* 画素値を直接操作する一例 */
			//p[0] = img->imageData[img->widthStep * y + x * 3];        // B
			//p[1] = img->imageData[img->widthStep * y + x * 3 + 1];    // G
			//p[2] = img->imageData[img->widthStep * y + x * 3 + 2];    // R
			i++;
			//bank[i] = 0.144*p[0] + 0.587*p[1] + 0.299*p[2];
			bank[i] = img->imageData[img->widthStep * y + x];
			//ofs << dec << static_cast<int>(bank[i]) << endl;
		}
	}
}

void Mouse(int event, int x, int y, int flags, void *param)
{
	Data* data = (Data*)param;
	switch (event){
	case CV_EVENT_LBUTTONDOWN:
	{
		cout << x << "," << y << "\n";
		data->x = x;
		data->y = y;
		break;
	}
	default:
	{
		break;
	}
	}
}

void threshold(IplImage* img, IplImage* t_img)//, char* filename)
{
	IplImage *src_img = 0;
	int x;
	char exfilename[100];/*拡張子付きファイル名*/

	//cvSmooth (src_img, src_img, CV_GAUSSIAN, 5);//平均化

	//cvThreshold(img, t_img, 100, 255, CV_THRESH_BINARY);
	//sprintf(exfilename, "100_%s", filename);
	//cvSaveImage(exfilename, t_img);

	//cvThreshold(img, t_img, 50, 255, CV_THRESH_BINARY);
	//sprintf(exfilename, "50%s", filename);
	//cvSaveImage(exfilename, t_img);

	//cvThreshold(img, t_img, 75, 255, CV_THRESH_BINARY);
	//sprintf(exfilename, "75%s", filename);
	//cvSaveImage(exfilename, t_img);

	// (1)輝度平均
	//x = cvThreshold(img, t_img, 127, 255, CV_THRESH_BINARY);
	//sprintf(exfilename, "ave%d%s", x, filename);
	//cvSaveImage(exfilename, t_img);

	// (2)大津の手法
	x = cvThreshold(img, t_img, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
	//sprintf(exfilename, "Otsu%d%s", x, filename);
	//cvSaveImage(exfilename, t_img);
}

int slope_table(int angle){
	int table[11] = { 0, 57, 28, 19, 14,
		11, 9, 8, 7, 6, 5 };
	if (angle < 0){
		return table[abs(angle)] * -1;
	}
	else {
		return table[angle];
	}
}

double inc_y_table(int angle){
	double table[21] = { 0, 57.29, 28.64, 19.08, 14.30,
		11.43, 9.51, 8.14, 7.12, 6.31,
		5.67, 5.14, 4.70, 4.33, 4.01,
		3.73, 3.49, 3.27, 3.08, 2.90, 2.75 };
	if (angle < 0){
		return table[abs(angle)] * -1;
	}
	else {
		return table[angle];
	}
}

int slope_revision(IplImage* img, Data data){ /*いくつすすんだら1上がるか*/
	int angle = 0;
	int i = 0, j = 0;
	int p = 0, m = 0;
	int plusSearchArea = search_area / 2, minusSearchArea = search_area / 2;
	unsigned int sum = 0, bank = 0;
	int slope = 0;
	double ave = 0.0;
	double max = 0.0, min = 0.0;
	double ans = 0.0, max_ans = 0.0;
	int slopeAngle = 0;
	ofstream ofs("slopecheack.txt");

	/*傾き探索範囲の調整*/
	if (data.x - search_area / 2 < 0){/*画像サイズよりはみ出たら*/
		plusSearchArea = abs(data.x - search_area / 2) + search_area / 2;
		minusSearchArea = data.x;
	}
	else if ((data.x + search_area / 2) > 640){ /*画像サイズよりはみ出たら*/
		plusSearchArea = 640 - data.x;
		minusSearchArea = ((data.x + search_area / 2) - 640) + search_area / 2;
	}
	/*平均輝度計算*/
	for (angle = -10; angle < 11; angle++){
		//ofs << "傾き" << angle << endl;
		max = 0;
		min = 0;
		/*y軸 切片移動*/
		for (i = -10; i < 11; i++){
			//ofs << "切片" << i << endl;
			sum = 0;
			/*x軸－方向*/
			for (m = 1; m < minusSearchArea; m++){
				if (angle == 0){
					slope = 0;
				}
				else {
					slope = m / slope_table(angle);/*傾き計算*/
				}
				bank = (uchar)img->imageData[img->widthStep *(data.y + i - slope) + data.x - m];
				sum += (uchar)img->imageData[img->widthStep *(data.y + i - slope) + data.x - m];
				//cout << "a" << endl;	
			}
			/*x軸＋方向*/
			for (p = 1; p < plusSearchArea; p++){
				if (angle == 0){
					slope = 0;
				}
				else {
					slope = p / slope_table(angle);/*傾き計算*/
				}
				bank = (uchar)img->imageData[img->widthStep *(data.y + i + slope) + data.x + p];
				sum += (uchar)img->imageData[img->widthStep *(data.y + i + slope) + data.x + p];
				//cout << "b" << endl;
			}
			//cout << sum << endl;
			//ofs << sum << endl;
			ave = sum / search_area;
			if (i == -10){
				max = ave;
				min = ave;
			}
			else{

				if (max<ave) max = ave;
				if (min>ave) min = ave;
			}
		}
		ans = abs(max - min);
		if (ans>max_ans) {
			max_ans = ans;
			slopeAngle = angle;
		}
	}

	return slopeAngle;
}

int Bresenhams_line_algorithm(IplImage* img, Point data){

	int plusSearchArea = search_area / 2, minusSearchArea = search_area / 2;
	int angle = 0;
	int j;
	int sum = 0, sub = 0;
	int over = 0;
	int dx = 0, dy = 0;
	int x = 0, y = 0;
	int D = 0;/*関数値*/
	int quadrant = 0;/*傾き*/
	double ave = 0.0, max = 0.0, min = 0.0;
	double ans = 0.0, max_ans = 0.0;
	int slopeAngle = 0;
	ofstream ofs("BMsum.txt");

	CvPoint startPoint = {0}, endPoint;

	
	/*傾き探索範囲の調整*/
	/*始点の決定(x座標)*/
	sub = data.x - search_area / 2;
	if (sub < 0){/*画像サイズよりはみ出たら*/
		startPoint.x = 0;
		over = sub*-1;
	}
	else{
		startPoint.x = data.x - search_area / 2;
		over = 0;
	}
	/*終点の決定(x座標)*/
	endPoint.x = startPoint.x + search_area + over;
	if (endPoint.x>640){/*画像サイズよりはみ出たら*/
		endPoint.x = 640;
	}

	for (angle = -20; angle <21; angle++){
		//ofs << "傾き" << angle << endl;
		max = 0;
		min = 0;
		for (j = -10; j < 11; j++){/*切片の移動*/
			sum = 0;
			/*始点の決定(y座標)*/
			if (angle == 0){
				startPoint.y = data.y + j;
			}
			else{
				startPoint.y = data.y + ((startPoint.x - data.x) / inc_y_table(angle)) + j;
			}
			/*終点の決定(y座標)*/
			if (angle == 0){
				endPoint.y = data.y + j;
			}
			else{
				endPoint.y = data.y + ((endPoint.x - data.x) / inc_y_table(angle)) + j;
			}
			//cout << "始点(" << startPoint.x - data.x << "," << startPoint.y - data.y << ")" 
			//<< " " << "終点(" << endPoint.x - data.x << "," << endPoint.y - data.y<< ")" << endl;
			/*傾きの方向決定*/
			dx = (endPoint.x - data.x) - (startPoint.x - data.x);
			dy = (endPoint.y - data.y) - (startPoint.y - data.y);
			//cout <<dy<<","<<dx<<endl;
			if (((double)dy / (double)dx)>0){
				quadrant = 1;
				/*cout << "傾き正" << endl;*/
			}
			else {
				quadrant = 2;
				/*cout << "傾き負" << endl;*/
			}
			/*cout << "quadrant=" << quadrant << endl;*/
			/*ブレゼンハムのアルゴリズム*/
			/*始点時の計算*/
			D = 2 * dy - dx;
			y = startPoint.y;
			//cout << startPoint.x << " " << startPoint.y << endl;
			sum += (uchar)img->imageData[img->widthStep *startPoint.y + startPoint.x];
			/*傾きに応じた処理*/
			switch (quadrant)
			{
			case 1:/*傾き正*/
				for (x = startPoint.x + 1; x < endPoint.x + 1; x++){
					if (D>0){
						y = y + 1;
						sum += (uchar)img->imageData[img->widthStep *(y)+x];
						D = D + (2 * dy - 2 * dx);
					}
					else {
						sum += (uchar)img->imageData[img->widthStep *(y)+x];
						D = D + (2 * dy);
					}
					//cout << "D=" << D << "x=" << x << "y=" << y << endl;
				}
				break;
			case 2:/*傾き負*/
				for (x = startPoint.x + 1; x < endPoint.x + 1; x++){
					if (D>0){
						y = y - 1;
						sum += (uchar)img->imageData[img->widthStep *(y)+x];
						D = D + (2 * (-1)* dy - 2 * dx);
					}
					else {
						sum += (uchar)img->imageData[img->widthStep *(y)+x];
						D = D + (2 * (-1)* dy);
					}
					//cout << "D=" << D << "x=" << x << "y=" << y << endl;
				}
				break;
			default:
				break;
			}
			//ofs << sum << endl;
			ave = sum;
			if (j == -10){
				max = ave;
				min = ave;
			}
			else{
				if (max<ave) max = ave;
				if (min>ave) min = ave;
			}
		}
		ans = abs(max - min);
		//ofs  << ans << endl;
		if (ans>max_ans) {
			max_ans = ans;
			slopeAngle = angle;
		}
		//cout << "slope" << slopeAngle << endl;
	}

	return slopeAngle;
}

void bobsort(totalData x[], int n){
	totalData temp;
	for (int i = 0; i < n - 1; i++) {
		for (int j = n - 1; j > i; j--) {
			if (x[j - 1].total < x[j].total) {  /* 前の要素の方が大きかったら */
				temp = x[j];        /* 交換する */
				x[j] = x[j - 1];
				x[j - 1] = temp;
			}
		}
	}
}

int cutout(IplImage* threshold_img, cutData *cut)
{
	unsigned int sum = 0;
	char cutData_fileName[100];
	double ave = 0.0;
	double last_ave = 0.0;
	double sub = 0.0, half = 0.0;
	int i = 0, j = 0, k = 0;
	int flag = 0;
	int s = 0, m = 0, l = 0, n = 0;
	int count = 0, bcount = 0;
	int num = 0, brank_num = 0;
	int maxBrightness = 0;
	int maxBrightness_EndPoint = 0;
	unsigned  int y_th = 0;
	int pre_sum;
	totalData bank[300], start, end, temp, edge[2];
	Rect y_axis_brank[10];

	sprintf(cutData_fileName, "cutData_%d%02d%02d%02d%02d.txt",
		pnow->tm_year + 1900, pnow->tm_mon + 1, pnow->tm_mday, pnow->tm_hour, pnow->tm_min);
	ofstream ofs(cutData_fileName);

	/*y方向*/
	y_th = threshold_img->width * 255;
	for (int j = 0; j < threshold_img->width; j++){
		sum += (uchar)threshold_img->imageData[threshold_img->widthStep * 0 + j];
	}
	//cout << sum << endl;
	for (int i = 0; i < rough_cut_area_hight; i++){
		sum = 0;
		for (int j = 0; j < threshold_img->width; j++){
			sum += (uchar)threshold_img->imageData[threshold_img->widthStep * i + j];
		}
		//cout << sum << endl;
		if (sum >= y_th && count == 0){
			y_axis_brank[brank_num].y = i;
			pre_sum = sum;
			//cout << "start" << endl;
		}
		if (sum >= y_th){
			count++;
			//cout << "count=" << count << endl;
			if (i == (rough_cut_area_hight - 1)){
				y_axis_brank[brank_num].height = count;
				//cout << "brank_num=" << brank_num << "y=" << y_axis_brank[brank_num].y << "hight=" << y_axis_brank[brank_num].height << endl;
				brank_num++;
				count = 0;
			}
		}
		if (sum<y_th && count>1){
			y_axis_brank[brank_num].height = count;
			//cout << "brank_num=" << brank_num << "y=" << y_axis_brank[brank_num].y << "hight=" << y_axis_brank[brank_num].height << endl;
			brank_num++;
			count = 0;
		}
	}
	for (int num = 0; num < brank_num - 1; num++){
		if (y_axis_brank[num].y < (rough_cut_area_hight / 2) && (rough_cut_area_hight / 2)  < (y_axis_brank[num + 1].y + y_axis_brank[num + 1].height)){
			cut->y = y_axis_brank[num].y + (y_axis_brank[num].height / 2);
			cut->hight = y_axis_brank[num + 1].y + (y_axis_brank[num + 1].height / 2) - cut->y;
			break;
		}
	}

	//cout << "y=" << y_axis_brank[num].y + (y_axis_brank[num].height / 2) << " , " << "hight=" << cut->hight << endl;
	//cout << "aaaaaaaaaaaaaaaaaaaaaaaaaaaaa" << endl;

	cvSetImageROI(threshold_img, cvRect(0, cut->y, rough_cut_area_hight, cut->hight));

	/*x方向*/
	for (int i = 0; i < threshold_img->width; i++){
		sum = 0;
		for (int j = cut->y; j < cut->hight + cut->y; j++){
			sum += (uchar)threshold_img->imageData[threshold_img->widthStep * j + i];
		}
		ofs << sum << endl;
		if (sum == 255 * cut->hight){
			count++;
			flag = 1;
		}
		else if (sum != 255 * cut->hight  && flag == 1){
			bank[bcount].total = count;
			count = 0;
			bank[bcount].x = i;
			bcount++;
			flag = 0;
		}
		//右端の処理
		if (i == threshold_img->width - 1 && flag == 1){
			bank[bcount].total = count;
			count = 0;
			bank[bcount].x = i;
			bcount++;
		}
	}
	cvResetImageROI(threshold_img);
	//cout << bcount << endl;
	for (int i = 0; i < bcount; i++){
		//cout << "x=" << bank[i].x << " , " << "total=" << bank[i].total << endl;
		ofs << bank[i].x << " " << bank[i].total << endl;
	}

	//端を除外
	if (bank[0].x - bank[0].total == 0) {
		edge[0] = bank[0];
		bank[0].total = 0;
		bank[0].x = 0;
	}
	if (bank[bcount - 1].x == 199) {
		edge[1] = bank[bcount - 1];
		bank[bcount - 1].total = 0;
		bank[bcount - 1].x = 0;
	}

	//バブルソート
	bobsort(bank, bcount);

	//cout << "AfterSort" << endl;
	ofs << "afterSort" << endl;
	for (int i = 0; i < bcount; i++){
		//cout << "x=" << bank[i].x << " , " << "total=" << bank[i].total << endl;
		ofs << bank[i].x << " " << bank[i].total << endl;
	}
	//startの決定
	start.total = 0;
	start.x = 0;
	if (bank[0].x < rough_cut_area_width / 2){
		start = bank[0];
	}
	for (i = 1; i < bcount; i++){
		sub = bank[i - 1].total - bank[i].total;
		half = bank[i].total / 2.0;
		//cout << sub << "," << half << "," << start.x << "," << bank[i].x << endl;
		if (sub < half && start.x < bank[i].x && rough_cut_area_width / 2> bank[i].x){
			//cout << "1" << endl;
			start = bank[i];
		}
		else if (sub >= half){
			break;
		}
	}
	//cout << "start=" << start.x << " , " << "end=" << end.x << endl;

	if (start.x == 0 || start.x>rough_cut_area_width / 2) start = edge[0];
	//endの決定
	end.total = 0;
	end.x = rough_cut_area_width;
	if (bank[0].x > rough_cut_area_width / 2){
		end = bank[0];
	}
	for (i = 1; i < bcount; i++){
		sub = bank[i - 1].total - bank[i].total;
		half = bank[i].total / 2.0;
		//cout << sub << "," << half << "," << start.x << "," << bank[i].x << endl;
		if (sub < half && end.x > bank[i].x && rough_cut_area_width / 2< bank[i].x){
			//cout << "1" << endl;
			end = bank[i];
		}
		else if (sub >= half){
			break;
		}
	}
	//cout << "start=" << start.x << " , " << "end=" << end.x << endl;

	if (end.x == rough_cut_area_width || end.x<rough_cut_area_width / 2) end = edge[1];
	//cout << "start=" << start.x << " , " << "end=" << end.x << endl;

	cut->x = start.x-3;
	cut->width = end.x - start.x+3;
	//cout << "aaaaaaaaaaaaaaaaaaaaaaaaaaaaa" << endl;

	//cout << "CutRange " << "x=" << cut->x << " , " << "y=" << cut->y << " , " << "width=" << cut->width << " , " << "hight=" << cut->hight << endl;

	return 0;
}

char *ocr(IplImage* result_img){
	char *outText = " ";
	cv::Mat result = cv::cvarrToMat(result_img);
	tesseract::TessBaseAPI *api = new tesseract::TessBaseAPI();
	// Initialize tesseract-ocr with English, without specifying tessdata path
	if (api->Init(NULL, "eng")) {
		fprintf(stderr, "Could not initialize tesseract.\n");
		exit(1);
	}
	api->SetImage(result.data,result.size().width,result.size().height,result.channels(),result.step1());
	api->Recognize(0);
	// Get OCR result
	outText = api->GetUTF8Text();
	//printf("OCR output: %s\n", outText);
	//cout << outText << endl;
	// Destroy used object and release memory
	api->End();
	//Boxa* boxes = api->GetComponentImages(tesseract::RIL_TEXTLINE, true, NULL, NULL);
	//printf("Found %d textline image components.\n", boxes->n);
	//for (int i = 0; i < boxes->n; i++) {
	//	BOX* box = boxaGetBox(boxes, i, L_CLONE);
	//	api->SetRectangle(box->x, box->y, box->w, box->h);
	//	outText = api->GetUTF8Text();
	//	int conf = api->MeanTextConf();
	//	printf("Box[%d]: x=%d, y=%d, w=%d, h=%d, confidence: %d, text: %s",
	//		i, box->x, box->y, box->w, box->h, conf, outText);
	//}
	return outText;
}

void Bing_Translator(char * in_Text, char *outText)
{
	const char *ansText = " ";
	BingTranslate::RESULT Result;
	if (BingTranslate::Translator::CanConnect(Result))
	{
		//printf("Connected via: \"%s\".\n", Result.Info.c_str());

		// Instance translator
		BingTranslate::Translator BingX(MY_ID, MY_KEY, Result);
		if (Result.IsSuccess())
		{
			// Translate string			
			if (BingX.Translate(in_Text, -1, "en", "ja", Result))
			{
			/*	printf("\nResult: \"%s\".\n", Result.Info.c_str());
				ofs << "tesseract : " << trans_word << endl;
				ofs << "Microsoft Translator : " << Result.Info.c_str() << endl;*/
				ansText = Result.Info.c_str();
			}
		}

		if (!Result.IsSuccess())
			printf("\nError:\n%s\n", Result.Info.c_str());
	}
	else
	{
		printf("Error! Can't connect to the Bing Translator.\n");
		printf("Reason: \"%s\".\n", Result.Info.c_str());
	}
	strcpy_s(outText, 100, ansText);
}


void meanShift_init(){
	//閾値
	hsv_threshold.S_min = 30;
	hsv_threshold.V_max = 240;
	hsv_threshold.V_min = 30;
	cv::namedWindow("Histogram");
	
	//色追跡の設定
	Mat track_color_img = (cv::Mat_<float>(16, 1) << 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
	
	//追跡色のヒストグラム
	Mat histimg = cv::Mat::zeros(200, 320, CV_8UC3);
	int hsize = 16;
	
	//ヒストグラムを計算
	meanshift_hist = track_color_img;
	//cout << meanshift_hist << endl;
	
	//表示用にRGBに変換
	histimg = cv::Scalar::all(0);
	int binW = histimg.cols / hsize;
	cv::Mat buf(1, hsize, CV_8UC3);
	for (int i = 0; i < hsize; i++){
		buf.at<cv::Vec3b>(i) = cv::Vec3b(cv::saturate_cast<uchar>(i*180. / hsize), 255, 255);
	}
	cv::cvtColor(buf, buf, cv::COLOR_HSV2BGR);
	
	//ヒストグラムを描画
	for (int i = 0; i < hsize; i++) {
		int val = cv::saturate_cast<int>(meanshift_hist.at<float>(i)*histimg.rows / 255);
		cv::rectangle(histimg, cv::Point(i*binW, histimg.rows), cv::Point((i + 1)*binW, histimg.rows - val), cv::Scalar(buf.at<cv::Vec3b>(i)), -1, 8);
	}
	cout << "追跡色ヒストグラム" << endl;
	cv::imshow("Histogram", histimg);
	cv::waitKey(0);
}

void meanShift(Mat in_frame,Point *Notice_coordinates){
	Point2f before_center;
	
	Rect mask_size(0, 0, 640, 480);
	Rect trackWindow(0, 0, 640, 480);
	int	color_check = 0;
	static int count=0;
	const float* phranges = meanshift_hranges;
	
	//HSVに変換
	cv::Mat hsv;
	cv::cvtColor(in_frame, hsv, cv::COLOR_BGR2HSV);
	// 閾値処理
	cv::Mat mask;
	cv::Scalar lower(0, hsv_threshold.S_min, MIN(hsv_threshold.V_min, hsv_threshold.V_max));
	cv::Scalar upper(180, 256, MAX(hsv_threshold.V_min, hsv_threshold.V_max));
	cv::inRange(hsv, lower, upper, mask);
	// Hueだけ抽出
	int ch[] = { 0, 0 };
	cv::Mat hue(hsv.size(), hsv.depth());
	mixChannels(&hsv, 1, &hue, 1, ch, 1);
	//cout << hsv.cols << " " << hsv.rows << endl;
	////バックプロジェクション
	cv::Mat backproj;
	cv::calcBackProject(&hue, 1, 0, meanshift_hist, backproj, &phranges);
	backproj &= mask;
	/*cv::namedWindow("test");
	cout << meanshift_hist << endl;
	cv::imshow("test", mask);
	waitKey(0);*/

	// CamShiftアルゴリズム
	cv::RotatedRect trackBox = cv::CamShift(backproj, trackWindow, cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 10, 1));
	/*cout << trackBox.angle << " " << trackBox.size << endl;
	cout << Notice_coordinates.x <<" "<<Notice_coordinates.y << endl;*/
	if (before_center == trackBox.center){
		count++;
	}
	else {
		count = 0;
	}
	before_center = trackBox.center;
	// 表示
	ellipse(in_frame, trackBox, cv::Scalar(0, 0, 255), 3, 16); // cv::LINE_AA=16
	//矢印の終点計算
	Point end;
	Notice_coordinates->x = trackBox.center.x + ((100 + trackBox.size.height / 2) * cos(trackBox.angle*(M_PI / 180) + (M_PI) / 2));
	Notice_coordinates->y = trackBox.center.y + ((100 + trackBox.size.height / 2) * sin(trackBox.angle*(M_PI / 180) + (M_PI) / 2));
	arrowedLine(in_frame, trackBox.center, *Notice_coordinates, cv::Scalar(200, 0, 0), 5, CV_AA);	
	// 選択領域を表示
	if (data.selectObject && data.selection.width > 0 && data.selection.height > 0) {
		cv::Rect roi = data.selection & cv::Rect(0, 0, in_frame.cols, in_frame.rows);
		cv::Mat tmp(in_frame, roi);
		cv::bitwise_not(tmp, tmp);
	}
}

BOOL ConvUtf8toSJis(BYTE* pSource, BYTE* pDist, int* pSize)
{
	*pSize = 0;

	//UTF-8からUTF-16へ変換
	const int nSize = ::MultiByteToWideChar(CP_UTF8, 0, (LPCSTR)pSource, -1, NULL, 0);

	BYTE* buffUtf16 = new BYTE[nSize * 2 + 2];
	::MultiByteToWideChar(CP_UTF8, 0, (LPCSTR)pSource, -1, (LPWSTR)buffUtf16, nSize);

	//UTF-16からShift-JISへ変換
	const int nSizeSJis = ::WideCharToMultiByte(CP_ACP, 0, (LPCWSTR)buffUtf16, -1, NULL, 0, NULL, NULL);
	if (!pDist){
		*pSize = nSizeSJis;
		delete buffUtf16;
		return TRUE;
	}

	BYTE* buffSJis = new BYTE[nSizeSJis * 2];
	ZeroMemory(buffSJis, nSizeSJis * 2);
	::WideCharToMultiByte(CP_ACP, 0, (LPCWSTR)buffUtf16, -1, (LPSTR)buffSJis, nSizeSJis, NULL, NULL);

	*pSize = lstrlen((char*)buffSJis);
	memcpy(pDist, buffSJis, *pSize);

	delete buffUtf16;
	delete buffSJis;

	return TRUE;
}

void check_OCR()
{
	IplImage *test_img, *gaussTest_img, *gaussTest_img2;
	IplImage *unsharp_test_img, *unsharp_gaussTest_img;
	IplImage *sub_test_img;
	IplImage *resize_img1, *resize_img2, *resize_img3, *resize_img4;
	IplImage *g_resize_img1, *g_resize_img2, *g_resize_img3, *g_resize_img4;
	IplImage *g_resize_img1_2;
	IplImage *unsharp_resize_img1, *unsharp_resize_img2, *unsharp_resize_img3, *unsharp_resize_img4;
	IplImage *unsharp_g_resize_img1, *unsharp_g_resize_img2, *unsharp_g_resize_img3, *unsharp_g_resize_img4;
	ofstream ofs("result_word.txt");

	int k = 10;

	//テストデータ
	test_img = cvLoadImage("example1.bmp", CV_LOAD_IMAGE_ANYCOLOR);
	/*画像表示*/
	//cvNamedWindow("Image", CV_WINDOW_AUTOSIZE);
	//cvShowImage("Image", test_img);
	//cvWaitKey(0);
	//cvDestroyWindow("Image");

	//ディレクトリの作成
	makeDirectory("check_OCR");

	//画像拡大
	resize_img1 = cvCreateImage(cvSize(test_img->width * 4, test_img->height * 4), IPL_DEPTH_8U, 1);//縦横4倍
	resize_img2 = cvCreateImage(cvSize(test_img->width * 4, test_img->height * 4), IPL_DEPTH_8U, 1);//縦横4倍
	resize_img3 = cvCreateImage(cvSize(test_img->width * 4, test_img->height * 4), IPL_DEPTH_8U, 1);//縦横4倍
	resize_img4 = cvCreateImage(cvSize(test_img->width * 4, test_img->height * 4), IPL_DEPTH_8U, 1);//縦横4倍

	cv::resize((Mat)test_img, (Mat)resize_img1, cvSize(test_img->width * 4, test_img->height * 4), INTER_NEAREST);
	cv::resize((Mat)test_img, (Mat)resize_img2, cvSize(test_img->width * 4, test_img->height * 4), INTER_LINEAR);
	cv::resize((Mat)test_img, (Mat)resize_img3, cvSize(test_img->width * 4, test_img->height * 4), INTER_CUBIC);
	cv::resize((Mat)test_img, (Mat)resize_img4, cvSize(test_img->width * 4, test_img->height * 4), INTER_LANCZOS4);

	cvSaveImage("result.bmp", test_img);
	cvSaveImage("result1.bmp", resize_img1);
	cvSaveImage("result2.bmp", resize_img2);
	cvSaveImage("result3.bmp", resize_img3);
	cvSaveImage("result4.bmp", resize_img4);

	char* result_Text = ocr(test_img);
	char* result_Text1 = ocr(resize_img1);
	char* result_Text2 = ocr(resize_img2);
	char* result_Text3 = ocr(resize_img3);
	char* result_Text4 = ocr(resize_img4);

	ofs << result_Text << " , " << result_Text1 << " , " << result_Text2 << " , " << result_Text3 << " , " << result_Text4  << endl;

	//ぼかし
	gaussTest_img = cvCreateImage(cvSize(test_img->width, test_img->height), IPL_DEPTH_8U, 1);
	cv::GaussianBlur((Mat)test_img, (Mat)gaussTest_img, cv::Size(5, 5), 1.5, 1.5);

	g_resize_img1 = cvCreateImage(cvSize(gaussTest_img->width * 4, gaussTest_img->height * 4), IPL_DEPTH_8U, 1);//縦横4倍
	//g_resize_img2 = cvCreateImage(cvSize(gaussTest_img->width * 4, gaussTest_img->height * 4), IPL_DEPTH_8U, 1);//縦横4倍
	//g_resize_img3 = cvCreateImage(cvSize(gaussTest_img->width * 4, gaussTest_img->height * 4), IPL_DEPTH_8U, 1);//縦横4倍
	//g_resize_img4 = cvCreateImage(cvSize(gaussTest_img->width * 4, gaussTest_img->height * 4), IPL_DEPTH_8U, 1);//縦横4倍

	cv::resize((Mat)gaussTest_img, (Mat)g_resize_img1, cvSize(gaussTest_img->width * 4, gaussTest_img->height * 4), INTER_NEAREST);
	//cv::resize((Mat)gaussTest_img, (Mat)g_resize_img2, cvSize(gaussTest_img->width * 4, gaussTest_img->height * 4), INTER_LINEAR);
	//cv::resize((Mat)gaussTest_img, (Mat)g_resize_img3, cvSize(gaussTest_img->width * 4, gaussTest_img->height * 4), INTER_CUBIC);
	//cv::resize((Mat)gaussTest_img, (Mat)g_resize_img4, cvSize(gaussTest_img->width * 4, gaussTest_img->height * 4), INTER_LANCZOS4);

	cvSaveImage("g_result.bmp", gaussTest_img);
	cvSaveImage("g_result1.bmp", g_resize_img1);
	//cvSaveImage("g_result2.bmp", g_resize_img2);
	//cvSaveImage("g_result3.bmp", g_resize_img3);
	//cvSaveImage("g_result4.bmp", g_resize_img4);

	char* g_result_Text = ocr(gaussTest_img);
	char* g_result_Text1 = ocr(g_resize_img1);
	//char* g_result_Text2 = ocr(g_resize_img2);
	//char* g_result_Text3 = ocr(g_resize_img3);
	//char* g_result_Text4 = ocr(g_resize_img4);

	ofs << g_result_Text << " , " << g_result_Text1  << endl;

	//鮮鋭化
	gaussTest_img2 = cvCreateImage(cvSize(test_img->width, test_img->height), IPL_DEPTH_8U, 1);
	cv::GaussianBlur((Mat)gaussTest_img, (Mat)gaussTest_img2, cv::Size(3, 3), 6,6);
	sub_test_img = cvCreateImage(cvSize(test_img->width, test_img->height), IPL_DEPTH_8U, 1);
	unsharp_test_img = cvCreateImage(cvSize(test_img->width, test_img->height), IPL_DEPTH_8U, 1);
	(Mat)sub_test_img = (Mat)gaussTest_img - (Mat)gaussTest_img2;
	(Mat)unsharp_test_img = (Mat)gaussTest_img + ((Mat)sub_test_img * 15);

	g_resize_img1_2=cvCreateImage(cvSize(gaussTest_img->width * 4, gaussTest_img->height * 4), IPL_DEPTH_8U, 1);//縦横4倍;
	cv::GaussianBlur((Mat)g_resize_img1, (Mat)g_resize_img1_2, cv::Size(11, 11), 6, 6);
	IplImage *sub_test_img2 = cvCreateImage(cvSize(gaussTest_img->width * 4, gaussTest_img->height * 4), IPL_DEPTH_8U, 1);
	IplImage *unsharp_test_img2 = cvCreateImage(cvSize(gaussTest_img->width * 4, gaussTest_img->height * 4), IPL_DEPTH_8U, 1);
	(Mat)sub_test_img2 = (Mat)g_resize_img1 - (Mat)g_resize_img1_2;
	(Mat)unsharp_test_img2 = (Mat)g_resize_img1 + ((Mat)sub_test_img2 * 60);
	//unsharp_resize_img1 = cvCreateImage(cvSize(test_img->width * 4, test_img->height * 4), IPL_DEPTH_8U, 1);//縦横4倍
	//unsharp_resize_img2 = cvCreateImage(cvSize(test_img->width * 4, test_img->height * 4), IPL_DEPTH_8U, 1);//縦横4倍
	//unsharp_resize_img3 = cvCreateImage(cvSize(test_img->width * 4, test_img->height * 4), IPL_DEPTH_8U, 1);//縦横4倍
	//unsharp_resize_img4 = cvCreateImage(cvSize(test_img->width * 4, test_img->height * 4), IPL_DEPTH_8U, 1);//縦横4倍
	//unsharp_gaussTest_img = cvCreateImage(cvSize(test_img->width, test_img->height), IPL_DEPTH_8U, 1);
	//unsharp_g_resize_img1 = cvCreateImage(cvSize(gaussTest_img->width * 4, gaussTest_img->height * 4), IPL_DEPTH_8U, 1);//縦横4倍
	//unsharp_g_resize_img2 = cvCreateImage(cvSize(gaussTest_img->width * 4, gaussTest_img->height * 4), IPL_DEPTH_8U, 1);//縦横4倍
	//unsharp_g_resize_img3 = cvCreateImage(cvSize(gaussTest_img->width * 4, gaussTest_img->height * 4), IPL_DEPTH_8U, 1);//縦横4倍
	//unsharp_g_resize_img4 = cvCreateImage(cvSize(gaussTest_img->width * 4, gaussTest_img->height * 4), IPL_DEPTH_8U, 1);//縦横4倍

	//cv_UnsharpMasking(test_img, unsharp_test_img,k);
	//cv_UnsharpMasking(resize_img1, unsharp_resize_img1, k);
	//cv_UnsharpMasking(resize_img2, unsharp_resize_img2, k);
	//cv_UnsharpMasking(resize_img3, unsharp_resize_img3, k);
	//cv_UnsharpMasking(resize_img4, unsharp_resize_img4, k);
	//cv_UnsharpMasking(gaussTest_img, unsharp_gaussTest_img, k);
	//cv_UnsharpMasking(g_resize_img1, unsharp_g_resize_img1, k);
	//cv_UnsharpMasking(g_resize_img2, unsharp_g_resize_img2, k);
	//cv_UnsharpMasking(g_resize_img3, unsharp_g_resize_img3, k);
	//cv_UnsharpMasking(g_resize_img4, unsharp_g_resize_img4, k);

	cvSaveImage("unsharp_result.bmp", unsharp_test_img);
	cvSaveImage("unsharp_result2.bmp", unsharp_test_img2);
	//cvSaveImage("unsharp_result1.bmp", unsharp_resize_img1);
	//cvSaveImage("unsharp_result2.bmp", unsharp_resize_img2);
	//cvSaveImage("unsharp_result3.bmp", unsharp_resize_img3);
	//cvSaveImage("unsharp_result4.bmp", unsharp_resize_img4);

	//cvSaveImage("unsharp_g_result.bmp", unsharp_gaussTest_img);
	//cvSaveImage("unsharp_g_result1.bmp", unsharp_g_resize_img1);
	//cvSaveImage("unsharp_g_result2.bmp", unsharp_g_resize_img2);
	//cvSaveImage("unsharp_g_result3.bmp", unsharp_g_resize_img3);
	//cvSaveImage("unsharp_g_result4.bmp", unsharp_g_resize_img4);

	char* unsharp_result_Text = ocr(unsharp_test_img);
	char* unsharp_result_Text2 = ocr(unsharp_test_img2);
	//char* unsharp_result_Text1 = ocr(unsharp_resize_img1);
	//char* unsharp_result_Text2 = ocr(unsharp_resize_img2);
	//char* unsharp_result_Text3 = ocr(unsharp_resize_img3);
	//char* unsharp_result_Text4 = ocr(unsharp_resize_img4);

	//char* unsharp_g_result_Text = ocr(unsharp_gaussTest_img);
	//char* unsharp_g_result_Text1 = ocr(unsharp_g_resize_img1);
	//char* unsharp_g_result_Text2 = ocr(unsharp_g_resize_img2);
	//char* unsharp_g_result_Text3 = ocr(unsharp_g_resize_img3);
	//char* unsharp_g_result_Text4 = ocr(unsharp_g_resize_img4);

	//ofs << unsharp_result_Text << " , " << unsharp_result_Text1 << " , " << unsharp_result_Text2 << " , " << unsharp_result_Text3 << " , " << unsharp_result_Text4 << endl;
	//ofs << unsharp_g_result_Text << " , " << unsharp_g_result_Text1 << " , " << unsharp_g_result_Text2 << " , " << unsharp_g_result_Text3 << " , " << unsharp_g_result_Text4 << endl;
	ofs << unsharp_result_Text << "," << unsharp_result_Text2 << endl;
	cout << "終了" << endl;
}

void testAR()
{
	int cap_count = 0;
	///////////////////////////////////////////  
	//画像に表示させる立方体の準備。  

	#define MARKER_SIZE (20)       /* マーカーの外側の1辺のサイズ[mm] */  
	//float in_data[] = {
	//	0, 0, 0, 
	//	0,0, 0,
	//	0,0, 0, };
	//float dis_data[] = {
	//	0,
	//	0, 
	//	0, 
	//	0
	//};
	//CvMat intrinsic_b = cvMat(3, 3, CV_32FC1,in_data);
	//CvMat distortion_b = cvMat(1, 4, CV_32FC1,dis_data);
	//CvMat *intrinsic = cvCreateMat(3, 3, CV_32FC1);
	//CvMat *distortion = cvCreateMat(1, 4, CV_32FC1);
	//intrinsic = &intrinsic_b;
	//distortion = &distortion_b;
	int i, j, k;

	CvMat *intrinsic, *distortion;
	CvFileStorage *fs;
	CvFileNode *param;

	fs = cvOpenFileStorage("camera.xml", 0, CV_STORAGE_READ);
	param = cvGetFileNodeByName(fs, NULL, "intrinsic");
	intrinsic = (CvMat *)cvRead(fs, param);
	param = cvGetFileNodeByName(fs, NULL, "distortion");
	distortion = (CvMat *)cvRead(fs, param);
	cvReleaseFileStorage(&fs);


	CvMat object_points;
	CvMat image_points;
	CvMat point_counts;

	CvMat *rotation = cvCreateMat(1, 3, CV_32FC1);
	CvMat *translation = cvCreateMat(1, 3, CV_32FC1);
	//立方体生成用  
	CvMat *srcPoints3D = cvCreateMat(4, 1, CV_32FC3);//元の3次元座標  
	CvMat *dstPoints2D = cvCreateMat(4, 1, CV_32FC2);//画面に投影したときの2次元座標  
	CvPoint2D32f *corners = (CvPoint2D32f *)cvAlloc(sizeof (CvPoint2D32f)* 4);//四角形  

	CvPoint3D32f baseMarkerPoints[4];
	//四角が物理空間上ではどの座標になるかを指定する。  
	//コーナー      実際の座標(mm)  
	//   X   Y     X    Y    
	//   0   0   = 0    0  
	//   0   1   = 0    20  
	//   1   0   = 20   0  
	baseMarkerPoints[0].x = (float)0 * MARKER_SIZE;
	baseMarkerPoints[0].y = (float)0 * MARKER_SIZE;
	baseMarkerPoints[0].z = 0.0;

	baseMarkerPoints[1].x = (float)0 * MARKER_SIZE;
	baseMarkerPoints[1].y = (float)1 * MARKER_SIZE;
	baseMarkerPoints[1].z = 0.0;

	baseMarkerPoints[2].x = (float)1 * MARKER_SIZE;
	baseMarkerPoints[2].y = (float)1 * MARKER_SIZE;
	baseMarkerPoints[2].z = 0.0;

	baseMarkerPoints[3].x = (float)1 * MARKER_SIZE;
	baseMarkerPoints[3].y = (float)0 * MARKER_SIZE;
	baseMarkerPoints[3].z = 0.0;

	//軸の基本座標を求める。  
	for (i = 0; i< 4; i++)
	{
		switch (i)
		{
		case 0: srcPoints3D -> data.fl[0] = 0;
			srcPoints3D -> data.fl[1] = 0;
			srcPoints3D -> data.fl[2] = 0;
			break;
		case 1: srcPoints3D -> data.fl[0 + i * 3] = (float)MARKER_SIZE;
			srcPoints3D -> data.fl[1 + i * 3] = 0;
			srcPoints3D -> data.fl[2 + i * 3] = 0;
			break;
		case 2: srcPoints3D -> data.fl[0 + i * 3] = 0;
			srcPoints3D -> data.fl[1 + i * 3] = (float)MARKER_SIZE;
			srcPoints3D -> data.fl[2 + i * 3] = 0;
			break;
		case 3: srcPoints3D -> data.fl[0 + i * 3] = 0;
			srcPoints3D -> data.fl[1 + i * 3] = 0;
			srcPoints3D -> data.fl[2 + i * 3] = -(float)MARKER_SIZE;;
			break;

		}
	}
	//矢印の基本座標を求める。
	CvMat *arrow_srcPoints3D = cvCreateMat(4, 1, CV_32FC3);//元の3次元座標  
	CvMat *arrow_dstPoints2D = cvCreateMat(4, 1, CV_32FC2);//画面に投影したときの2次元座標  
	for (i = 0; i< 2; i++)
	{
		switch (i)
		{
		case 0: arrow_srcPoints3D->data.fl[0] = 10;
			arrow_srcPoints3D->data.fl[1] = 10;
			arrow_srcPoints3D->data.fl[2] = 0;
			break;
		case 1: arrow_srcPoints3D->data.fl[0 + i * 3] = 10;
			arrow_srcPoints3D->data.fl[1 + i * 3] = -100;
			arrow_srcPoints3D->data.fl[2 + i * 3] = 0;
			break;
		}
	}

	///軸の準備　ここまで  
	//////////////////////////////////// 

	IplImage img;
	IplImage gray_img;
	IplImage* gray_img_Contour;

	//フォントの設定  
	CvFont dfont;
	float hscale = 0.5f;
	float vscale = 0.5f;
	float italicscale = 0.0f;
	int  thickness = 1;
	char text[255] = " ";
	cvInitFont(&dfont, CV_FONT_HERSHEY_SIMPLEX, hscale, vscale, italicscale, thickness, CV_AA);

	CvFont axisfont;
	float axhscale = 0.8f;
	float axvscale = 0.8f;
	cvInitFont(&axisfont, CV_FONT_HERSHEY_SIMPLEX, axhscale, axvscale, italicscale, thickness, CV_AA);
	//輪郭保存用のストレージを確保  
	CvMemStorage *storage = cvCreateMemStorage(0);//輪郭用  
	CvMemStorage *storagepoly = cvCreateMemStorage(0);//輪郭近似ポリゴン用  

	CvSeq *firstcontour = NULL;
	CvSeq *polycontour = NULL;

	IplImage *marker_inside = cvCreateImage(cvSize(57, 57), IPL_DEPTH_8U, 1);
	IplImage *marker_inside_zoom = cvCreateImage(cvSize(marker_inside->width * 2, marker_inside->height * 2), IPL_DEPTH_8U, 1);
	IplImage *tmp_img = cvCloneImage(marker_inside);

	CvMat *map_matrix;
	map_matrix = cvCreateMat(3, 3, CV_32FC1);

	CvPoint2D32f src_pnt[4], dst_pnt[4], tmp_pnt[4];

	dst_pnt[0] = cvPoint2D32f(0, 0);
	dst_pnt[1] = cvPoint2D32f(marker_inside->width, 0);
	dst_pnt[2] = cvPoint2D32f(marker_inside->width, marker_inside->height);
	dst_pnt[3] = cvPoint2D32f(0, marker_inside->height);
	map_matrix = cvCreateMat(3, 3, CV_32FC1);

	cvNamedWindow(" marker_inside", CV_WINDOW_AUTOSIZE);
	cvNamedWindow(" capture_image", CV_WINDOW_AUTOSIZE);

	//マスク画像の読み込み。  
	//検出したマーカーと、この画像のANDを取り、cvCountNonZeroが一番大きかったものをマーカーの向きとする。  
	IplImage * mask0 = cvLoadImage("mask0deg.bmp", 0);
	IplImage * mask90 = cvLoadImage("mask90deg.bmp", 0);
	IplImage * mask180 = cvLoadImage("mask180deg.bmp", 0);
	IplImage * mask270 = cvLoadImage("mask270deg.bmp", 0);
	IplImage * tempmask = cvCloneImage(mask0);//作業用  

	cv::VideoCapture cap;
	cv::Size cap_size(640, 480);
	cap.set(CV_CAP_PROP_FRAME_WIDTH, cap_size.width);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, cap_size.height);

	cap.open(0);
	if (!cap.isOpened()) {
		cout << "カメラの初期化に失敗しました" << endl;
		//return -1;
	}
	waitKey(1000);

	cv::Mat original_frame, copy_frame;
	cv::Mat gray_Mat;
	cap >> original_frame;
	if (original_frame.empty()) cout << "カメラの初期化に失敗しました" << endl;;
	original_frame.copyTo(copy_frame);
	cv::cvtColor(original_frame, gray_Mat, CV_BGR2GRAY);
	gray_img = gray_Mat;
	gray_img_Contour = cvCreateImage(cvGetSize(&gray_img), IPL_DEPTH_8U, 1);

	

	cv::namedWindow("test");
	cv::namedWindow("Capture");
	while (1)
	{
		cap >> original_frame;
		original_frame.copyTo(copy_frame);
		//グレースケール化
		cv::cvtColor(original_frame, gray_Mat, CV_BGR2GRAY);
		img = original_frame;
		gray_img = gray_Mat;
		//平滑化
		cvSmooth(&gray_img, &gray_img, CV_GAUSSIAN, 3);
		//二値化
		cvThreshold(&gray_img, &gray_img, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
		//反転  
		cvNot(&gray_img, &gray_img);
		//輪郭抽出
		int contourCount = 0;
		cvCopy(&gray_img, gray_img_Contour);
		contourCount = cvFindContours(gray_img_Contour, storage, &firstcontour, sizeof (CvContour), CV_RETR_CCOMP);
		//輪郭に近似しているポリゴンを求める（最小直線距離3ピクセルに設定）
		polycontour = cvApproxPoly(firstcontour, sizeof(CvContour), storagepoly, CV_POLY_APPROX_DP, 3, 1);
		cout << polycontour->total << endl;
		//waitKey();
		for (CvSeq* c = polycontour; c != NULL; c = c->h_next){
			if ((cvContourPerimeter(c)< 2000) && (cvContourPerimeter(c)> 60) && (c->total == 4)){
				if (c->v_next != NULL){
					if (c->v_next->total == 4){
						int nearestindex = 0;
						CvSeq* c_vnext = c->v_next;
						//c_vnext = c_vnext->v_next;
						cvDrawContours(&img, c, CV_RGB(255, 255, 0), CV_RGB(255, 0, 0), 0);
						cvDrawContours(&img, c_vnext, CV_RGB(255, 0, 0), CV_RGB(0, 0, 255), 0);
						float xlist[4];
						float ylist[4];
						for (int n = 0; n < 4; n++){
							CvPoint* p = CV_GET_SEQ_ELEM(CvPoint, c->v_next, n);
							tmp_pnt[n].x = (float)p->x;
							tmp_pnt[n].y = (float)p->y;
							xlist[n] = (float)p->x;
							ylist[n] = (float)p->y;
						}
						//for (int i = 0; i < 4; i++) {
						//	cout << "x=" << tmp_pnt[i].x << "y= " << tmp_pnt[i].y << endl;
						//}

						//四角の情報を渡す。どちらを向いているかはまだわからない  
						cvGetPerspectiveTransform(tmp_pnt, dst_pnt, map_matrix);
						//marker_inside（マーカーの内側だけを抽出し、正方形に透視変換したもの）  
						//を、マスク画像を指定して一時イメージにコピー。  
						//一時イメージに白い点が多数あれば、マスク画像と同じ方向を向いていることになる。  
						cvWarpPerspective(&gray_img, marker_inside, map_matrix, CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS, cvScalarAll(0));
						cv::imshow("marker_inside", (Mat)marker_inside);
						int notzeroCount = 0;

						int maxCount = 0;
						int markerDirection = 0;//基本は0deg  
						cvResize(marker_inside, marker_inside_zoom);
						//cout << "width=" << tempmask->width << "height=" << tempmask->height << endl;
						//waitKey();
						cvCopy(marker_inside, tempmask, mask0);
						notzeroCount = cvCountNonZero(tempmask);
						if (maxCount< notzeroCount)
						{
						maxCount = notzeroCount;
						markerDirection = 0;
						sprintf(text,"0deg", notzeroCount);

						}
						cvZero(tempmask);
						cvCopy(marker_inside, tempmask, mask90);
						notzeroCount = cvCountNonZero(tempmask);
						if (maxCount< notzeroCount)
						{
							maxCount = notzeroCount;
							markerDirection = 90;
							sprintf(text, " 90deg");
						}
						cvZero(tempmask);
						cvCopy(marker_inside, tempmask, mask180);
						notzeroCount = cvCountNonZero(tempmask);
						if (maxCount< notzeroCount)
						{
							maxCount = notzeroCount;
							markerDirection = 180;
							sprintf(text, " 180deg");
						}
						cvZero(tempmask);

						cvCopy(marker_inside, tempmask, mask270);
						notzeroCount = cvCountNonZero(tempmask);
						if (maxCount< notzeroCount)
						{
							maxCount = notzeroCount;
							markerDirection = 270;
							sprintf(text, " 270deg");
						}
						cvPutText(marker_inside_zoom, text, cvPoint(70, 70), &dfont, cvScalarAll(255));
						cvZero(tempmask);

						cvShowImage(" marker_inside" , marker_inside_zoom);
						//四角の向きを反映させる。  
						if (markerDirection == 0) {
							src_pnt[0].x = tmp_pnt[0].x;
							src_pnt[0].y = tmp_pnt[0].y;
							src_pnt[1].x = tmp_pnt[3].x;
							src_pnt[1].y = tmp_pnt[3].y;
							src_pnt[2].x = tmp_pnt[2].x;
							src_pnt[2].y = tmp_pnt[2].y;
							src_pnt[3].x = tmp_pnt[1].x;
							src_pnt[3].y = tmp_pnt[1].y;
						}
						if (markerDirection == 90) {
							src_pnt[0].x = tmp_pnt[1].x;
							src_pnt[0].y = tmp_pnt[1].y;
							src_pnt[1].x = tmp_pnt[0].x;
							src_pnt[1].y = tmp_pnt[0].y;
							src_pnt[2].x = tmp_pnt[3].x;
							src_pnt[2].y = tmp_pnt[3].y;
							src_pnt[3].x = tmp_pnt[2].x;
							src_pnt[3].y = tmp_pnt[2].y;
						}
						if (markerDirection == 180) {
							src_pnt[0].x = tmp_pnt[2].x;
							src_pnt[0].y = tmp_pnt[2].y;
							src_pnt[1].x = tmp_pnt[1].x;
							src_pnt[1].y = tmp_pnt[1].y;
							src_pnt[2].x = tmp_pnt[0].x;
							src_pnt[2].y = tmp_pnt[0].y;
							src_pnt[3].x = tmp_pnt[3].x;
							src_pnt[3].y = tmp_pnt[3].y;
						}
						if (markerDirection == 270)
						{
							src_pnt[0].x = tmp_pnt[3].x;
							src_pnt[0].y = tmp_pnt[3].y;
							src_pnt[1].x = tmp_pnt[2].x;
							src_pnt[1].y = tmp_pnt[2].y;
							src_pnt[2].x = tmp_pnt[1].x;
							src_pnt[2].y = tmp_pnt[1].y;
							src_pnt[3].x = tmp_pnt[0].x;
							src_pnt[3].y = tmp_pnt[0].y;
						}
						//cvPutText(&img,"0", cvPoint((int)src_pnt[0].x,(int)src_pnt[0].y), &dfont, CV_RGB(255, 0, 255));  
						//cvPutText(&img,"1", cvPoint((int)src_pnt[1].x,(int)src_pnt[1].y), &dfont, CV_RGB(255, 0, 255)); 
						//マーカーのイメージ上での座標を設定。  
						cvInitMatHeader(&image_points, 4, 1, CV_32FC2, src_pnt);
						//矢印のイメージ上での座標を設定

						//マーカーの基本となる座標を設定  
						cvInitMatHeader(&object_points, 4, 3, CV_32FC1, baseMarkerPoints);
						
						//カメラの内部定数(intrinsticとdistortion)から、rotationとtranslationを求める   
						cvFindExtrinsicCameraParams2(&object_points, &image_points, intrinsic, distortion, rotation, translation);

						
						//求めたものを使用して、現実空間上の座標が画面上だとどの位置に来るかを計算  
						cvProjectPoints2(srcPoints3D, rotation, translation, intrinsic, distortion, dstPoints2D);
						cvProjectPoints2(arrow_srcPoints3D, rotation, translation, intrinsic, distortion, arrow_dstPoints2D);
						//軸を描画  
						CvPoint startpoint;
						CvPoint endpoint;
						//for (int as = 0; as < 12;as++) {
						//	cout <<as<<"="<< dstPoints2D->data.fl[as] << endl;
						//}
					
						//startpoint = cvPoint((int)dstPoints2D -> data.fl[0], (int)dstPoints2D -> data.fl[1]);
						//for (j = 1; j< 4; j++) {
						//	endpoint = cvPoint((int)dstPoints2D -> data.fl[(j)* 2], (int)dstPoints2D -> data.fl[1 + (j)* 2]);
						//	if (j == 1){
						//		cvLine(&img, startpoint, endpoint, CV_RGB(255, 0, 0), 2, 8, 0);
						//		cvPutText(&img, " X" , endpoint, &axisfont, CV_RGB(255, 0, 0));
						//	}
						//	if (j == 2){
						//		cvLine(&img, startpoint, endpoint, CV_RGB(0, 255, 0), 2, 8, 0);
						//		cvPutText(&img, " Y", endpoint, &axisfont, CV_RGB(0, 255, 0));
						//	}
						//	if (j == 3){
						//		cvLine(&img, startpoint, endpoint, CV_RGB(0, 0, 255), 2, 8, 0);
						//		cvPutText(&img, " Z", endpoint, &axisfont, CV_RGB(0, 0, 255));
						//	}
						//}
						CvPoint arrow_startPoint;
						CvPoint arrow_endPoint;
						arrow_startPoint= cvPoint((int)arrow_dstPoints2D->data.fl[0], (int)arrow_dstPoints2D->data.fl[1]);
						arrow_endPoint = cvPoint((int)arrow_dstPoints2D->data.fl[2], (int)arrow_dstPoints2D->data.fl[3]);
						arrowedLine((Mat)&img, arrow_startPoint, arrow_endPoint, cv::Scalar(200, 0, 0), 5, CV_AA);
					}
				}
			}
		}
		char str[100];
		int c = cvWaitKey(2);
		if (c == 0x73)
		{ // 's'キー入力
			sprintf(str, "AR_img%2d.png", cap_count);
			cv::imwrite(str, (Mat)&img);
			cap_count++;
		}
		//cvNamedWindow(" capture_image", CV_WINDOW_AUTOSIZE);
		cv::imshow("Capture", (Mat)&gray_img);
		cv::imshow(" capture_image", (Mat)&img);
		cv::waitKey(2);
	}
}

int Calibrate()
{
	#define IMAGE_NUM  (25)         /* 画像数 */
	#define PAT_ROW    (7)          /* パターンの行数 */
	#define PAT_COL    (10)         /* パターンの列数 */
	#define PAT_SIZE   (PAT_ROW*PAT_COL)
	#define ALL_POINTS (IMAGE_NUM*PAT_SIZE)
	#define CHESS_SIZE (24.0)       /* パターン1マスの1辺サイズ[mm] */

	int i, j, k;
	int corner_count, found;
	int p_count[IMAGE_NUM];
	IplImage *src_img[IMAGE_NUM];
	CvSize pattern_size = cvSize(PAT_COL, PAT_ROW);
	CvPoint3D32f objects[ALL_POINTS];
	CvPoint2D32f *corners = (CvPoint2D32f *)cvAlloc(sizeof (CvPoint2D32f)* ALL_POINTS);
	CvMat object_points;
	CvMat image_points;
	CvMat point_counts;
	CvMat *intrinsic = cvCreateMat(3, 3, CV_32FC1);
	CvMat *rotation = cvCreateMat(1, 3, CV_32FC1);
	CvMat *translation = cvCreateMat(1, 3, CV_32FC1);
	CvMat *distortion = cvCreateMat(1, 4, CV_32FC1);

	// (1)キャリブレーション画像の読み込み
	for (i = 0; i < IMAGE_NUM; i++) {
		char buf[32];
		sprintf(buf, "calib_img/%02d.png", i);
		if ((src_img[i] = cvLoadImage(buf, CV_LOAD_IMAGE_COLOR)) == NULL) {
			fprintf(stderr, "cannot load image file : %s\n", buf);
		}
	}

	// (2)3次元空間座標の設定
	for (i = 0; i < IMAGE_NUM; i++) {
		for (j = 0; j < PAT_ROW; j++) {
			for (k = 0; k < PAT_COL; k++) {
				objects[i * PAT_SIZE + j * PAT_COL + k].x = j * CHESS_SIZE;
				objects[i * PAT_SIZE + j * PAT_COL + k].y = k * CHESS_SIZE;
				objects[i * PAT_SIZE + j * PAT_COL + k].z = 0.0;
			}
		}
	}
	cvInitMatHeader(&object_points, ALL_POINTS, 3, CV_32FC1, objects);

	// (3)チェスボード（キャリブレーションパターン）のコーナー検出
	int found_num = 0;
	cvNamedWindow("Calibration", CV_WINDOW_AUTOSIZE);
	for (i = 0; i < IMAGE_NUM; i++) {
		found = cvFindChessboardCorners(src_img[i], pattern_size, &corners[i * PAT_SIZE], &corner_count);
		fprintf(stderr, "%02d...", i);
		if (found) {
			fprintf(stderr, "ok\n");
			found_num++;
		}
		else {
			fprintf(stderr, "fail\n");
		}
		// (4)コーナー位置をサブピクセル精度に修正，描画
		IplImage *src_gray = cvCreateImage(cvGetSize(src_img[i]), IPL_DEPTH_8U, 1);
		cvCvtColor(src_img[i], src_gray, CV_BGR2GRAY);
		cvFindCornerSubPix(src_gray, &corners[i * PAT_SIZE], corner_count,
			cvSize(3, 3), cvSize(-1, -1), cvTermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03));
		cvDrawChessboardCorners(src_img[i], pattern_size, &corners[i * PAT_SIZE], corner_count, found);
		p_count[i] = corner_count;
		cvShowImage("Calibration", src_img[i]);
		cvWaitKey(0);
	}
	cvDestroyWindow("Calibration");

	if (found_num != IMAGE_NUM)
		return -1;
	cvInitMatHeader(&image_points, ALL_POINTS, 1, CV_32FC2, corners);
	cvInitMatHeader(&point_counts, IMAGE_NUM, 1, CV_32SC1, p_count);

	// (5)内部パラメータ，歪み係数の推定
	cvCalibrateCamera2(&object_points, &image_points, &point_counts, cvSize(640, 480), intrinsic, distortion);

	// (6)外部パラメータの推定
	CvMat sub_image_points, sub_object_points;
	int base = 0;
	cvGetRows(&image_points, &sub_image_points, base * PAT_SIZE, (base + 1) * PAT_SIZE);
	cvGetRows(&object_points, &sub_object_points, base * PAT_SIZE, (base + 1) * PAT_SIZE);
	cvFindExtrinsicCameraParams2(&sub_object_points, &sub_image_points, intrinsic, distortion, rotation, translation);

	// (7)XMLファイルへの書き出し
	CvFileStorage *fs;
	fs = cvOpenFileStorage("camera.xml", 0, CV_STORAGE_WRITE);
	cvWrite(fs, "intrinsic", intrinsic);
	cvWrite(fs, "rotation", rotation);
	cvWrite(fs, "translation", translation);
	cvWrite(fs, "distortion", distortion);
	cvReleaseFileStorage(&fs);

	for (i = 0; i < IMAGE_NUM; i++) {
		cvReleaseImage(&src_img[i]);
	}

	return 0;
}

void take_pic()
{
	
	char cut_imgName[100];
	char filePass[200];
	cv::VideoCapture cap;
	cv::Size cap_size(640, 480);
	cap.set(CV_CAP_PROP_FRAME_WIDTH, cap_size.width);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, cap_size.height);
	
	cap.open(1);
	if (!cap.isOpened()) {
		cout << "カメラの初期化に失敗しました" << endl;
		//return -1;
	}
	cv::namedWindow("Capture");
	cv::waitKey(1000);

	makeDirectory("Eng_txt");
	ofstream ofs("Eng_txt_data.txt");
	cout << "ok" << endl;
	cv::Mat original_frame;

	int c;
	int pic_count = 1;
	int while_count = 0;
	while (1){
		c = cvWaitKey(2);
		cap >> original_frame;
		cv::imshow("Capture", original_frame);
		if (c == 0x73)// 's'キー入力
		{ 
			//for (int i = 0; i < 100;i++){
				sprintf(cut_imgName, "%02d.bmp", pic_count);
				cvtColor(original_frame, original_frame, CV_BGR2GRAY);
				imwrite(cut_imgName, original_frame);
				sprintf(filePass, "C:/dev/rawdata/%02d.bmp", pic_count);
				ofs << filePass << " " << "1" << " " << "0" << " " << "0" << " " << "640" << " " << "480" << endl;
				pic_count++;
			//}			
		}

	}	
}

void ARtracking(Mat in_frame, Point *Notice_coordinates)
{
	int cap_count = 0;
	///////////////////////////////////////////  
	//画像に表示させる立方体の準備。  

	#define MARKER_SIZE (20)       /* マーカーの外側の1辺のサイズ[mm] */  
	int i, j, k;

	//立方体生成用  
	CvMat *srcPoints3D = cvCreateMat(4, 1, CV_32FC3);//元の3次元座標  
	CvMat *dstPoints2D = cvCreateMat(4, 1, CV_32FC2);//画面に投影したときの2次元座標  
	CvPoint2D32f *corners = (CvPoint2D32f *)cvAlloc(sizeof (CvPoint2D32f)* 4);//四角形  

	CvMat object_points;
	CvMat image_points;
	CvMat point_counts;

	CvPoint3D32f baseMarkerPoints[4];
	//四角が物理空間上ではどの座標になるかを指定する。  
	//コーナー      実際の座標(mm)  
	//   X   Y     X    Y    
	//   0   0   = 0    0  
	//   0   1   = 0    20  
	//   1   0   = 20   0  
	baseMarkerPoints[0].x = (float)0 * MARKER_SIZE;
	baseMarkerPoints[0].y = (float)0 * MARKER_SIZE;
	baseMarkerPoints[0].z = 0.0;

	baseMarkerPoints[1].x = (float)0 * MARKER_SIZE;
	baseMarkerPoints[1].y = (float)1 * MARKER_SIZE;
	baseMarkerPoints[1].z = 0.0;

	baseMarkerPoints[2].x = (float)1 * MARKER_SIZE;
	baseMarkerPoints[2].y = (float)1 * MARKER_SIZE;
	baseMarkerPoints[2].z = 0.0;

	baseMarkerPoints[3].x = (float)1 * MARKER_SIZE;
	baseMarkerPoints[3].y = (float)0 * MARKER_SIZE;
	baseMarkerPoints[3].z = 0.0;

	//軸の基本座標を求める。  
	for (i = 0; i< 4; i++)
	{
		switch (i)
		{
		case 0: srcPoints3D->data.fl[0] = 0;
			srcPoints3D->data.fl[1] = 0;
			srcPoints3D->data.fl[2] = 0;
			break;
		case 1: srcPoints3D->data.fl[0 + i * 3] = (float)MARKER_SIZE;
			srcPoints3D->data.fl[1 + i * 3] = 0;
			srcPoints3D->data.fl[2 + i * 3] = 0;
			break;
		case 2: srcPoints3D->data.fl[0 + i * 3] = 0;
			srcPoints3D->data.fl[1 + i * 3] = (float)MARKER_SIZE;
			srcPoints3D->data.fl[2 + i * 3] = 0;
			break;
		case 3: srcPoints3D->data.fl[0 + i * 3] = 0;
			srcPoints3D->data.fl[1 + i * 3] = 0;
			srcPoints3D->data.fl[2 + i * 3] = -(float)MARKER_SIZE;;
			break;

		}
	}
	//矢印の基本座標を求める。
	CvMat *arrow_srcPoints3D = cvCreateMat(4, 1, CV_32FC3);//元の3次元座標  
	CvMat *arrow_dstPoints2D = cvCreateMat(4, 1, CV_32FC2);//画面に投影したときの2次元座標  
	for (i = 0; i< 2; i++)
	{
		switch (i)
		{
		case 0: arrow_srcPoints3D->data.fl[0] = 10;
			arrow_srcPoints3D->data.fl[1] = 10;
			arrow_srcPoints3D->data.fl[2] = 0;
			break;
		case 1: arrow_srcPoints3D->data.fl[0 + i * 3] = 10;
			arrow_srcPoints3D->data.fl[1 + i * 3] = -100;
			arrow_srcPoints3D->data.fl[2 + i * 3] = 0;
			break;
		}
	}

	//軸の準備　ここまで  
	////////////////////////////////// 

	IplImage img;
	IplImage gray_img;
	IplImage* gray_img_Contour;

	//輪郭保存用のストレージを確保  
	CvMemStorage *storage = cvCreateMemStorage(0);//輪郭用  
	CvMemStorage *storagepoly = cvCreateMemStorage(0);//輪郭近似ポリゴン用  

	CvSeq *firstcontour = NULL;
	CvSeq *polycontour = NULL;

	IplImage *marker_inside = cvCreateImage(cvSize(57, 57), IPL_DEPTH_8U, 1);
	IplImage *marker_inside_zoom = cvCreateImage(cvSize(marker_inside->width * 2, marker_inside->height * 2), IPL_DEPTH_8U, 1);
	IplImage *tmp_img = cvCloneImage(marker_inside);

	CvMat *map_matrix;
	map_matrix = cvCreateMat(3, 3, CV_32FC1);

	CvPoint2D32f src_pnt[4], dst_pnt[4], tmp_pnt[4];

	dst_pnt[0] = cvPoint2D32f(0, 0);
	dst_pnt[1] = cvPoint2D32f(marker_inside->width, 0);
	dst_pnt[2] = cvPoint2D32f(marker_inside->width, marker_inside->height);
	dst_pnt[3] = cvPoint2D32f(0, marker_inside->height);
	map_matrix = cvCreateMat(3, 3, CV_32FC1);

	//cvNamedWindow(" marker_inside", CV_WINDOW_AUTOSIZE);
	//cvNamedWindow(" capture_image", CV_WINDOW_AUTOSIZE);

	//マスク画像の読み込み。  
	//検出したマーカーと、この画像のANDを取り、cvCountNonZeroが一番大きかったものをマーカーの向きとする。  

	cv::Mat copy_frame;
	cv::Mat gray_Mat;
	cv::cvtColor(in_frame, gray_Mat, CV_BGR2GRAY);
	gray_img = gray_Mat;
	gray_img_Contour = cvCreateImage(cvGetSize(&gray_img), IPL_DEPTH_8U, 1);

	//cv::namedWindow("test");
	//cv::namedWindow("Capture");
	in_frame.copyTo(copy_frame);
	//グレースケール化
	cv::cvtColor(in_frame, gray_Mat, CV_BGR2GRAY);
	img = in_frame;
	gray_img = gray_Mat;
	//平滑化
	cvSmooth(&gray_img, &gray_img, CV_GAUSSIAN, 3);
	//二値化
	cvThreshold(&gray_img, &gray_img, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
	//反転  
	cvNot(&gray_img, &gray_img);
	//輪郭抽出
	int contourCount = 0;
	cvCopy(&gray_img, gray_img_Contour);
	contourCount = cvFindContours(gray_img_Contour, storage, &firstcontour, sizeof (CvContour), CV_RETR_CCOMP);
	//輪郭に近似しているポリゴンを求める（最小直線距離3ピクセルに設定）
	polycontour = cvApproxPoly(firstcontour, sizeof(CvContour), storagepoly, CV_POLY_APPROX_DP, 3, 1);
	//cout << polycontour->total << endl;

	for (CvSeq* c = polycontour; c != NULL; c = c->h_next){
		if ((cvContourPerimeter(c)< 2000) && (cvContourPerimeter(c)> 60) && (c->total == 4)){
			if (c->v_next != NULL){
				if (c->v_next->total == 4){
					int nearestindex = 0;
					CvSeq* c_vnext = c->v_next;
					//c_vnext = c_vnext->v_next;
					cvDrawContours(&img, c, CV_RGB(255, 255, 0), CV_RGB(255, 0, 0), 0);
					cvDrawContours(&img, c_vnext, CV_RGB(255, 0, 0), CV_RGB(0, 0, 255), 0);
					float xlist[4];
					float ylist[4];
					for (int n = 0; n < 4; n++){
						CvPoint* p = CV_GET_SEQ_ELEM(CvPoint, c->v_next, n);
						tmp_pnt[n].x = (float)p->x;
						tmp_pnt[n].y = (float)p->y;
						xlist[n] = (float)p->x;
						ylist[n] = (float)p->y;
					}
					//四角の情報を渡す。どちらを向いているかはまだわからない  
					cvGetPerspectiveTransform(tmp_pnt, dst_pnt, map_matrix);
					//marker_inside（マーカーの内側だけを抽出し、正方形に透視変換したもの）  
					//を、マスク画像を指定して一時イメージにコピー。  
					//一時イメージに白い点が多数あれば、マスク画像と同じ方向を向いていることになる。  
					cvWarpPerspective(&gray_img, marker_inside, map_matrix, CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS, cvScalarAll(0));
					//cv::imshow("marker_inside", (Mat)marker_inside);
					int notzeroCount = 0;
					int maxCount = 0;
					int markerDirection = 0;//基本は0deg  
					cvResize(marker_inside, marker_inside_zoom);
					//cout << "width=" << tempmask->width << "height=" << tempmask->height << endl;
					//waitKey();
					cvCopy(marker_inside, tempmask, mask0);
					notzeroCount = cvCountNonZero(tempmask);
					if (maxCount < notzeroCount)
					{
						maxCount = notzeroCount;
						markerDirection = 0;
						cout << 0 << endl;
					}
					cvZero(tempmask);
					cvCopy(marker_inside, tempmask, mask90);
					notzeroCount = cvCountNonZero(tempmask);
					if (maxCount < notzeroCount)
					{
						maxCount = notzeroCount;
						markerDirection = 90;
						cout << 90 << endl;
					}
					cvZero(tempmask);
					cvCopy(marker_inside, tempmask, mask180);
					notzeroCount = cvCountNonZero(tempmask);
					if (maxCount < notzeroCount)
					{
						maxCount = notzeroCount;
						markerDirection = 180;
						cout << 180 << endl;
					}
					cvZero(tempmask);

					cvCopy(marker_inside, tempmask, mask270);
					notzeroCount = cvCountNonZero(tempmask);
					if (maxCount < notzeroCount)
					{
						maxCount = notzeroCount;
						markerDirection = 270;
						cout << 270 << endl;
					}
					cvZero(tempmask);
					//四角の向きを反映させる。  
					if (markerDirection == 0) {
						src_pnt[0].x = tmp_pnt[0].x;
						src_pnt[0].y = tmp_pnt[0].y;
						src_pnt[1].x = tmp_pnt[3].x;
						src_pnt[1].y = tmp_pnt[3].y;
						src_pnt[2].x = tmp_pnt[2].x;
						src_pnt[2].y = tmp_pnt[2].y;
						src_pnt[3].x = tmp_pnt[1].x;
						src_pnt[3].y = tmp_pnt[1].y;
					}
					if (markerDirection == 90) {
						src_pnt[0].x = tmp_pnt[1].x;
						src_pnt[0].y = tmp_pnt[1].y;
						src_pnt[1].x = tmp_pnt[0].x;
						src_pnt[1].y = tmp_pnt[0].y;
						src_pnt[2].x = tmp_pnt[3].x;
						src_pnt[2].y = tmp_pnt[3].y;
						src_pnt[3].x = tmp_pnt[2].x;
						src_pnt[3].y = tmp_pnt[2].y;
					}
					if (markerDirection == 180) {
						src_pnt[0].x = tmp_pnt[2].x;
						src_pnt[0].y = tmp_pnt[2].y;
						src_pnt[1].x = tmp_pnt[1].x;
						src_pnt[1].y = tmp_pnt[1].y;
						src_pnt[2].x = tmp_pnt[0].x;
						src_pnt[2].y = tmp_pnt[0].y;
						src_pnt[3].x = tmp_pnt[3].x;
						src_pnt[3].y = tmp_pnt[3].y;
					}
					if (markerDirection == 270)
					{
						src_pnt[0].x = tmp_pnt[3].x;
						src_pnt[0].y = tmp_pnt[3].y;
						src_pnt[1].x = tmp_pnt[2].x;
						src_pnt[1].y = tmp_pnt[2].y;
						src_pnt[2].x = tmp_pnt[1].x;
						src_pnt[2].y = tmp_pnt[1].y;
						src_pnt[3].x = tmp_pnt[0].x;
						src_pnt[3].y = tmp_pnt[0].y;
					}
					//cvPutText(&img,"0", cvPoint((int)src_pnt[0].x,(int)src_pnt[0].y), &dfont, CV_RGB(255, 0, 255));  
					//cvPutText(&img,"1", cvPoint((int)src_pnt[1].x,(int)src_pnt[1].y), &dfont, CV_RGB(255, 0, 255)); 
					//マーカーのイメージ上での座標を設定。  
					cvInitMatHeader(&image_points, 4, 1, CV_32FC2, src_pnt);
					//矢印のイメージ上での座標を設定
					//マーカーの基本となる座標を設定  
					cvInitMatHeader(&object_points, 4, 3, CV_32FC1, baseMarkerPoints);
					//カメラの内部定数(intrinsticとdistortion)から、rotationとtranslationを求める   
					cvFindExtrinsicCameraParams2(&object_points, &image_points, intrinsic, distortion, rotation, translation);
					//求めたものを使用して、現実空間上の座標が画面上だとどの位置に来るかを計算  
					cvProjectPoints2(srcPoints3D, rotation, translation, intrinsic, distortion, dstPoints2D);
					cvProjectPoints2(arrow_srcPoints3D, rotation, translation, intrinsic, distortion, arrow_dstPoints2D);
					//軸を描画  
					CvPoint startpoint;
					CvPoint endpoint;
					CvPoint arrow_startPoint;
					CvPoint arrow_endPoint;
					arrow_startPoint = cvPoint((int)arrow_dstPoints2D->data.fl[0], (int)arrow_dstPoints2D->data.fl[1]);
					arrow_endPoint = cvPoint((int)arrow_dstPoints2D->data.fl[2], (int)arrow_dstPoints2D->data.fl[3]);
					arrowedLine(in_frame, arrow_startPoint, arrow_endPoint, cv::Scalar(200, 0, 0), 5, CV_AA);
					Notice_coordinates->x = arrow_endPoint.x;
					Notice_coordinates->y = arrow_endPoint.y;
				}
			}
		}
	}
}

void test_th()
{
	cv::VideoCapture cap;
	cv::Size cap_size(640, 480);
	cap.set(CV_CAP_PROP_FRAME_WIDTH, cap_size.width);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, cap_size.height);

	int fps = 8;
	//cvGetCaptureProperty((CvCapture *)cap, CV_CAP_PROP_FPS);

	cv::VideoWriter ad_th_writer("adth_capture1.avi", CV_FOURCC('X', 'V', 'I', 'D'), fps, cap_size);
	cv::VideoWriter th_writer("th_capture1.avi", CV_FOURCC('X', 'V', 'I', 'D'), fps, cap_size);

	cv::Mat original_frame, copy_frame;
	cv::Mat gray_Mat,threshold_Mat, ad_threshold_Mat;
	IplImage img;
	IplImage gray_img;
	IplImage *threshold_img,* ad_threshold_img;

	cv::namedWindow("ad_th");
	cv::namedWindow("th");
	waitKey(0);

	cap.open(0);
	if (!cap.isOpened()) {
		cout << "カメラの初期化に失敗しました" << endl;
		//return -1;
	}
	waitKey(1000);

	cap >> original_frame;
	if (original_frame.empty()) cout << "カメラの初期化に失敗しました" << endl;;
	original_frame.copyTo(copy_frame);
	cv::cvtColor(original_frame, gray_Mat, CV_BGR2GRAY);
	gray_img = gray_Mat;
	
	while (1)
	{
		cap >> original_frame;
		original_frame.copyTo(copy_frame);
		//グレースケール化
		cv::cvtColor(original_frame, gray_Mat, CV_BGR2GRAY);
		img = original_frame;
		gray_img = gray_Mat;
		threshold_img = cvCreateImage(cvGetSize(&gray_img), IPL_DEPTH_8U, 1);
		//二値化
		cv::threshold(gray_Mat, threshold_Mat, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
		adaptiveThreshold(gray_Mat, ad_threshold_Mat, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 7, 8);

		cv::imshow("ad_th", ad_threshold_Mat);
		cv::imshow("th", threshold_Mat);
		ad_th_writer << ad_threshold_Mat;
		th_writer << threshold_Mat;
		cv::waitKey(2);
	}
}

void cut_img()
{
	ofstream ofs("data.txt");
	// (1)load a source image as is
	char imgName[100], cut_imgName[100];
	char filePass[200];
	int img_num = 1;
	int num=1;
	while (img_num < 100){
		sprintf(imgName, "negative/%02d.bmp", img_num);
		cout << imgName << endl;
		Mat src_img = imread(imgName, -1);
		if (!src_img.data){
			cout << "error" << endl;;
		}
		// (2)create a window and set the callback function for mouse events
		namedWindow("Image", 1);
		cvSetMouseCallback("Image", (CvMouseCallback)(&on_mouse), &src_img);
		// (3)show the source image with an invert area, and quit when 'esc' pressed
		while (1) {
			Mat dst_img = src_img.clone();

			imshow("Image", dst_img);
			int key = waitKey(10);
			if ((char)key == 27)//Esc key
				break;
		}
		cout << "ok" << endl;
		sprintf(cut_imgName, "re_negative/%02d.bmp", img_num);
		Mat cut_img(src_img, selection);
		//cvtColor(cut_img, cut_img, CV_BGR2GRAY);
		imwrite(cut_imgName,cut_img);
		sprintf(filePass, "C:/dev/index_finger/%2d.bmp", img_num);
		//cout << "指の本数を入力＝" << endl;
		//scanf_s("%d", &num);
		//ofs << filePass << " " << num << " " << selection.x << " " << selection.y << " " << selection.width << " " << selection.height << endl; 
		ofs << filePass << endl; 
		img_num++;
	}
}

void resize_img()
{
	char imgName[100], resize_imgName[100];
	char filePass[200];
	int img_num = 1;
	int num = 1;
	Size resize_size(200, 450);
	while (img_num < 100){
		//sprintf(imgName, "re_negative/%02d.bmp", img_num);
		sprintf(imgName, "dog.jpg");
		cout << imgName << endl;
		Mat src_img = imread(imgName, CV_LOAD_IMAGE_GRAYSCALE);
		if (!src_img.data){
			cout << "error" << endl;;
		}
		
		namedWindow("Image", 1);
		//sprintf(resize_imgName, "resize_negative/%02d.bmp", img_num);
		sprintf(resize_imgName, "resize_negative/dog.bmp");
		Mat resize_img(200, 450, src_img.type());
		cv::resize(src_img, resize_img, resize_size);
		imwrite(resize_imgName, resize_img);
		img_num++;
	}
}


void on_mouse(int event, int x, int y, int flags, void* param)
{
	static Point2i origin;
	Mat *img = static_cast<Mat*>(param);

	// (4)calculate coordinates of selected area (by Click and Drag)
	if (select_object) {
		selection.x = CV_IMIN(x, origin.x);
		selection.y = CV_IMIN(y, origin.y);
		selection.width = selection.x + CV_IABS(x - origin.x);
		selection.height = selection.y + CV_IABS(y - origin.y);

		selection.x = CV_IMAX(selection.x, 0);
		selection.y = CV_IMAX(selection.y, 0);
		selection.width = CV_IMIN(selection.width, img->cols);
		selection.height = CV_IMIN(selection.height, img->rows);
		selection.width -= selection.x;
		selection.height -= selection.y;
	}

	// (5)process a start and a finish selecting events (i.e. button-up and -down)
	switch (event) {
	case CV_EVENT_LBUTTONDOWN:
		origin = Point2i(x, y);
		selection = Rect(x, y, 0, 0);
		select_object = 1;
		break;
	case CV_EVENT_LBUTTONUP:
		select_object = 0;
		break;
	}
}

// 積分画像生成
vector<Mat> calculateIntegralHOG(const Mat& image) {
	// X, Y方向に微分
	Mat xsobel, ysobel;
	Sobel(image, xsobel, CV_32F, 1, 0);
	Sobel(image, ysobel, CV_32F, 0, 1);

	// 角度別の画像を生成しておく
	vector<Mat> bins(N_BIN);
	for (int i = 0; i < N_BIN; i++)
		bins[i] = Mat::zeros(image.size(), CV_32F);

	// X, Y微分画像を勾配方向と強度に変換
	Mat Imag, Iang;
	cartToPolar(xsobel, ysobel, Imag, Iang, true);
	// 勾配方向を[0, 180)にする
	add(Iang, Scalar(180), Iang, Iang < 0);
	add(Iang, Scalar(-180), Iang, Iang >= 180);
	// 勾配方向を[0, 1, ..., 8]にする準備（まだfloat）
	Iang /= THETA;

	// 勾配方向を強度で重みをつけて、角度別に投票する
	for (int y = 0; y < image.rows; y++) {
		for (int x = 0; x < image.cols; x++) {
			int ind = Iang.at<float>(y, x);
			bins[ind].at<float>(y, x) += Imag.at<float>(y, x);
		}
	}

	// 角度別に積分画像生成
	vector<Mat> integrals(N_BIN);
	for (int i = 0; i < N_BIN; i++) {
		// 積分画像をつくる、OpenCVの関数がある
		integral(bins[i], integrals[i]);
	}

	return integrals;
}

// ある矩形領域の勾配ヒストグラムを求める
// ここでいう矩形はHOG特徴量のセルに該当
void calculateHOGInCell(Mat& hogCell, Rect roi, const vector<Mat>& integrals) {
	int x0 = roi.x, y0 = roi.y;
	int x1 = x0 + roi.width, y1 = y0 + roi.height;
	for (int i = 0; i < N_BIN; i++) {
		Mat integral = integrals[i];
		float a = integral.at<double>(y0, x0);
		float b = integral.at<double>(y1, x1);
		float c = integral.at<double>(y0, x1);
		float d = integral.at<double>(y1, x0);
		hogCell.at<float>(0, i) = (a + b) - (c + d);
	}
}

// HOG特徴量を計算する
// pt: ブロックの中心点
Mat getHOG(Point pt, const vector<Mat>& integrals) {
	// ブロックが画像からはみ出していないか確認
	if (pt.x - R < 0 ||
		pt.y - R < 0 ||
		pt.x + R >= integrals[0].cols ||
		pt.y + R >= integrals[0].rows
		) {
		return Mat();
	}

	// 与点を中心としたブロックで、
	// セルごとに勾配ヒストグラムを求めて連結
	Mat hist(Size(N_BIN*BLOCK_SIZE*BLOCK_SIZE, 1), CV_32F);
	Point tl(0, pt.y - R);
	int c = 0;
	for (int i = 0; i < BLOCK_SIZE; i++) {
		tl.x = pt.x - R;
		for (int j = 0; j < BLOCK_SIZE; j++) {
			calculateHOGInCell(hist.colRange(c, c + N_BIN),
				Rect(tl, tl + Point(CELL_SIZE, CELL_SIZE)),
				integrals);
			tl.x += CELL_SIZE;
			c += N_BIN;
		}
		tl.y += CELL_SIZE;
	}
	// L2ノルムで正規化
	normalize(hist, hist, 1, 0, NORM_L2);
	return hist;
}

int test_Hog() {
	// 画像をグレイスケールで読み込む
	string fileName = "re_rawdata/01.bmp";
	Mat originalImage = imread(fileName, CV_LOAD_IMAGE_GRAYSCALE);
	ofstream ofs("Hog_data.csv");

	// 積分画像生成
	vector<Mat> integrals = calculateIntegralHOG(originalImage);
	// ある点(x, y)のHOG特徴量を求めるには
	// Mat hist = getHOG(Point(x, y), integrals);
	// とする。histはSize(81, 1) CV_32FのMat


	/* ****************** *
	* 以下、表示のための処理
	* ****************** */

	// 表示用画像を用意（半分の輝度に）
	Mat image = originalImage.clone();
	image *= 0.5;

	// 格子点でHOG計算
	Mat meanHOGInBlock(Size(N_BIN, 1), CV_32F);

	for (int y = CELL_SIZE / 2; y < image.rows; y += CELL_SIZE) {
		for (int x = CELL_SIZE / 2; x < image.cols; x += CELL_SIZE) {
			// (x, y)でのHOGを取得
			Mat hist = getHOG(Point(x, y), integrals);
			// ブロックが画像からはみ出ていたら continue
			if (hist.empty()) continue;

			// ブロックごとに勾配方向ヒストグラム生成
			meanHOGInBlock = Scalar(0);
			for (int i = 0; i < N_BIN; i++) {
				for (int j = 0; j < BLOCK_SIZE*BLOCK_SIZE; j++) {
					meanHOGInBlock.at<float>(0, i) += hist.at<float>(0, i + j*N_BIN);
				}
			}
			// L2ノルムで正規化（強い方向が強調される）
			normalize(meanHOGInBlock, meanHOGInBlock, 1, 0, CV_L2);
			// 角度ごとに線を描画
			Point center(x, y);
			for (int i = 0; i < N_BIN; i++) {
				double theta = (i * THETA + 90.0) * CV_PI / 180.0;
				Point rd(CELL_SIZE*0.5*cos(theta), CELL_SIZE*0.5*sin(theta));
				Point rp = center - rd;
				Point lp = center - -rd;
				line(image, rp, lp, Scalar(255 * meanHOGInBlock.at<float>(0, i), 255, 255));
			}
			for (int row = 0; row < (int)meanHOGInBlock.rows; row++){
				for (int col = 0; col < (int)meanHOGInBlock.cols; col++){
					ofs << meanHOGInBlock.at<float>(row, col) << endl;
				}
			}
		}
	}


	// 表示
	imshow("out", image);
	waitKey(0);

	return 0;
}

int init_Hogdata()
{
	int rawdata_num = 0;
	char fileName[100], outFileName[100];

	cout << "get_Hogdata" << endl;
	cout << "rawdata_num = " ;
	scanf_s("%d", &rawdata_num);
	cout << "outFileName : " ;
	scanf_s("%s", outFileName,100);
	ofstream ofs(outFileName, std::ios::out | std::ios::app);
	float * result_vec;
	int count=0;
	for (int num = 1; num < rawdata_num+1; num++){
		// 画像をグレイスケールで読み込む
		//string fileName = "re_rawdata/01.bmp";
		sprintf(fileName, "resize_negative/%02d.bmp", num);
		Mat originalImage = imread(fileName, CV_LOAD_IMAGE_GRAYSCALE);
		int HogSize = floor((originalImage.rows - CELL_SIZE) / CELL_SIZE)*floor((originalImage.cols - CELL_SIZE) / CELL_SIZE)*BLOCK_SIZE*BLOCK_SIZE*N_BIN;
		//cout << HogSize << endl;
		result_vec = (float *)malloc(sizeof(double)* (HogSize+1));
		getHog_another(originalImage, result_vec);
		for (int i = 0; i<(int)ceil((float)originalImage.rows / (float)CELL_SIZE) - (BLOCK_SIZE - 1); i++){
			for (int j = 0; j<(int)ceil((float)originalImage.cols / (float)CELL_SIZE) - (BLOCK_SIZE - 1); j++){
				count = 0;
				for (count = 0; count<BLOCK_SIZE*BLOCK_SIZE*N_BIN; count++){
					ofs << result_vec[count] << endl;
				}
			}
		}
		free(result_vec);
		//// 積分画像生成
		//vector<Mat> integrals = calculateIntegralHOG(originalImage);
		//// ある点(x, y)のHOG特徴量を求めるには
		//// Mat hist = getHOG(Point(x, y), integrals);
		//// とする。histはSize(81, 1) CV_32FのMat


		///* ****************** *
		//* 以下、表示のための処理
		//* ****************** */

		//// 表示用画像を用意（半分の輝度に）
		//Mat image = originalImage.clone();
		//image *= 0.5;

		//// 格子点でHOG計算
		//Mat meanHOGInBlock(Size(N_BIN, 1), CV_32F);

		//for (int y = CELL_SIZE / 2; y < image.rows; y += CELL_SIZE) {
		//	for (int x = CELL_SIZE / 2; x < image.cols; x += CELL_SIZE) {
		//		// (x, y)でのHOGを取得
		//		Mat hist = getHOG(Point(x, y), integrals);
		//		// ブロックが画像からはみ出ていたら continue
		//		if (hist.empty()) continue;

		//		// ブロックごとに勾配方向ヒストグラム生成
		//		meanHOGInBlock = Scalar(0);
		//		for (int i = 0; i < N_BIN; i++) {
		//			for (int j = 0; j < BLOCK_SIZE*BLOCK_SIZE; j++) {
		//				meanHOGInBlock.at<float>(0, i) += hist.at<float>(0, i + j*N_BIN);
		//			}
		//		}
		//		// L2ノルムで正規化（強い方向が強調される）
		//		normalize(meanHOGInBlock, meanHOGInBlock, 1, 0, CV_L2);
		//		// 角度ごとに線を描画
		//		Point center(x, y);
		//		for (int i = 0; i < N_BIN; i++) {
		//			double theta = (i * THETA + 90.0) * CV_PI / 180.0;
		//			Point rd(CELL_SIZE*0.5*cos(theta), CELL_SIZE*0.5*sin(theta));
		//			Point rp = center - rd;
		//			Point lp = center - -rd;
		//			line(image, rp, lp, Scalar(255 * meanHOGInBlock.at<float>(0, i), 255, 255));
		//		}
		//		for (int row = 0; row < (int)meanHOGInBlock.rows; row++){
		//			for (int col = 0; col < (int)meanHOGInBlock.cols; col++){
		//				ofs << meanHOGInBlock.at<float>(row, col) << endl;
		//			}
		//		}
		//	}
		//}
		//// 表示
		//imshow("out", image);
		//imwrite("ppt_use.bmp", image);
		//waitKey(0);
	}
	return 0;
}

void get_Hogdata(Mat input_img, float *Hogdata)
{
	// 積分画像生成
	vector<Mat> integrals = calculateIntegralHOG(input_img);
	// ある点(x, y)のHOG特徴量を求めるには
	// Mat hist = getHOG(Point(x, y), integrals);
	// とする。histはSize(81, 1) CV_32FのMat

	/* ****************** *
	* 以下、表示のための処理
	* ****************** */
	// 表示用画像を用意（半分の輝度に）
	Mat image = input_img.clone();
	image *= 0.5;
	// 格子点でHOG計算
	Mat meanHOGInBlock(Size(N_BIN, 1), CV_32F);
	for (int y = CELL_SIZE / 2; y < image.rows; y += CELL_SIZE) {
		for (int x = CELL_SIZE / 2; x < image.cols; x += CELL_SIZE) {
			// (x, y)でのHOGを取得
			Mat hist = getHOG(Point(x, y), integrals);
			// ブロックが画像からはみ出ていたら continue
			if (hist.empty()) continue;
			// ブロックごとに勾配方向ヒストグラム生成
			meanHOGInBlock = Scalar(0);
			for (int i = 0; i < N_BIN; i++) {
				for (int j = 0; j < BLOCK_SIZE*BLOCK_SIZE; j++) {
					meanHOGInBlock.at<float>(0, i) += hist.at<float>(0, i + j*N_BIN);
				}
			}
			// L2ノルムで正規化（強い方向が強調される）
			int num=0;
			normalize(meanHOGInBlock, meanHOGInBlock, 1, 0, CV_L2);
			for (int row = 0; row < (int)meanHOGInBlock.rows; row++){
				for (int col = 0; col < (int)meanHOGInBlock.cols; col++){
					Hogdata[num] = meanHOGInBlock.at<float>(row, col) ;
					num++;
				}
			}
		}
	}
}

void get_Boost()
{
	float training_Hogdata[Hog_data_num][Hog_num];
	float training_labels[Hog_data_num] = {-1};
	ifstream ifs("Hogdata_ano2.csv");
	if (!ifs) {
		cout << "Error:Input data file not found" << endl;
	}
	stringstream ss;
	string tmp;
	char true_imgName[100], false_imgName[100];
	CvBoost boost;

	//Hogdataの準備
	for (int i = 0;i < Hog_data_num; i++){
		for (int j = 0;j< Hog_num; j++){
			getline(ifs, tmp);
			ss.str(tmp);
			ss >> training_Hogdata[i][j];
			ss.str("");
			ss.clear(stringstream::goodbit);
		}
	}
	cout << "data load end" << endl;
	for (int i = 0; i <  100; i++){
		training_labels[i] = 1;
	}
	Mat Hogdata_Mat(Hog_data_num, Hog_num, CV_32FC1, training_Hogdata);
	Mat training_labels_Mat(1,Hog_data_num,CV_32FC1, training_labels);
	cout << "Boost training start" << endl;
	boost.train(Hogdata_Mat, CV_ROW_SAMPLE, training_labels_Mat);
	cout << "Boost training end" << endl; 
	waitKey();
	sprintf(true_imgName, "resize_positive/%02d.bmp", 16);
	cout << true_imgName << endl;
	Mat true_img = imread(true_imgName, -1);
	float true_Hogdata[Hog_num];
	//get_Hogdata(true_img,true_Hogdata);
	getHog_another(true_img, true_Hogdata);
	sprintf(false_imgName, "resize_negative/01.bmp");
	cout << false_imgName << endl;
	Mat false_img = imread(false_imgName, -1);
	float false_Hogdata[Hog_num];
	//getHog_another(false_img, false_Hogdata);
	getHog_another(false_img, false_Hogdata);

	Mat true_Hogdata_Mat(1, Hog_num, CV_32FC1, true_Hogdata);
	Mat false_Hogdata_Mat(1, Hog_num, CV_32FC1, false_Hogdata);

	float response1 = boost.predict(true_Hogdata_Mat);
	if (response1 == 1.0f) {
		cout <<"true : " <<response1 << endl;
	}
	else {
		cout << "false : "<<response1 << endl;
	}
	float response2 = boost.predict(false_Hogdata_Mat);
	if (response2 == 1.0f) {
		cout << "true : "<<response2 << endl;
	}
	else {
		cout << "false : " <<response1<< endl;
	}

	//waitKey(0);
}

int minimum(int a, int b)
{
	if (a<b){
		return a;
	}
	return b;
}

void getHog_another(Mat inImg,float* vec_tmp){
	int x, y, i, j, k, m, n, count;// cell_size = 12, rot_res = 9, block_size = 3;
	float dx, dy, ***hist, norm; //*vec_tmp,
    IplImage im;
    CvMat *mag = NULL, *theta = NULL;
	/*ofstream ofs(outFileName, std::ios::out | std::ios::app);
	if ((im = cvLoadImage(fileName, CV_LOAD_IMAGE_GRAYSCALE)) == 0){
		fprintf(stderr, "No such file %s", fileName);
		exit(0);
	}*/
	// magnitude and direction
	im = (IplImage )inImg;
	mag = cvCreateMat(im.height, im.width, CV_32F);
	theta = cvCreateMat(im.height, im.width, CV_32F);
	for (y = 0; y<im.height; y++){
	    for (x = 0; x<im.width; x++){
	        if (x == 0 || x == im.width - 1 || y == 0 || y == im.height - 1){
	            cvmSet(mag, y, x, 0.0);
	            cvmSet(theta, y, x, 0.0);
			}
	        else{
	            dx = double((uchar)im.imageData[y*im.widthStep + x + 1]) - double((uchar)im.imageData[y*im.widthStep + x - 1]);
				dy = double((uchar)im.imageData[(y + 1)*im.widthStep + x]) - double((uchar)im.imageData[(y - 1)*im.widthStep + x]);
				cvmSet(mag, y, x, sqrt(dx*dx + dy*dy));
				cvmSet(theta, y, x, atan(dy / (dx + 0.01)));
			}
		}
	}
	// histogram generation for each cell
	hist = (float***)malloc(sizeof(float**)* (int)ceil((float)im.height / (float)CELL_SIZE));
	if (hist == NULL) exit(1);
	for (i = 0; i<(int)ceil((float)im.height / (float)CELL_SIZE); i++){
		hist[i] = (float**)malloc(sizeof(float*)*(int)ceil((float)im .width / (float)CELL_SIZE));
		if (hist[i] == NULL) exit(1);
		for (j = 0; j<(int)ceil((float)im.width / (float)CELL_SIZE); j++){
			hist[i][j] = (float *)malloc(sizeof(float)*N_BIN);
			if (hist[i][j] == NULL) exit(1);
		}
	}
	for (i = 0; i<(int)ceil((float)im.height / (float)CELL_SIZE); i++){
		for (j = 0; j<(int)ceil((float)im.width / (float)CELL_SIZE); j++){
			for (k = 0; k < N_BIN; k++){
				hist[i][j][k] = 0.0;
			}
		}
	}
	for (i = 0; i<(int)ceil((float)im.height / (float)CELL_SIZE); i++){
		for (j = 0; j<(int)ceil((float)im.width / (float)CELL_SIZE); j++){
			for (m = i*CELL_SIZE; m<minimum((i + 1)*CELL_SIZE, im.height); m ++){
				for (n = j*CELL_SIZE; n<minimum((j + 1)*CELL_SIZE, im.width); n++){
					hist[i][j][(int)floor((cvmGet(theta, m, n) + CV_PI / 2)*N_BIN / CV_PI)] += cvmGet(mag, m, n);
				}
			}
		}
	}
	//normalization for each block & generate vector 
	//vec_tmp = (float *)malloc(sizeof(float)*BLOCK_SIZE*BLOCK_SIZE*N_BIN);
	for (i = 0; i<(int)ceil((float)im.height / (float)CELL_SIZE) - (BLOCK_SIZE - 1); i++){
		for (j = 0; j<(int)ceil((float)im.width / (float)CELL_SIZE) - (BLOCK_SIZE - 1); j++){
			count = 0;
			norm = 0.0;
			for (m = i; m<i + BLOCK_SIZE; m++){
				for (n = j; n<j + BLOCK_SIZE; n++){
					for (k = 0; k<N_BIN; k++){
						vec_tmp[count++] = hist[m][n][k];
						norm += hist[m][n][k] * hist[m][n][k];
					}
				}
			}
			for (count = 0; count<BLOCK_SIZE*BLOCK_SIZE*N_BIN; count++){
				vec_tmp[count] = vec_tmp[count] / (sqrt(norm + 1));
				if (vec_tmp[count]>0.2) vec_tmp[count] = 0.2;
				//printf("%.4f ", vec_tmp[count]);
				//ofs << vec_tmp[count] << endl;
			}
		}
	}
	for (i = 0; i<(int)ceil((float)im.height / (float)CELL_SIZE); i ++) {
		for (j = 0; j <(int)ceil((float)im.width / (float)CELL_SIZE); j ++) {
			free(hist[i][j]);
		}
		free(hist[i]);
	}
	free(hist);
	//free(vec_tmp);
	cvReleaseMat(&mag);
	cvReleaseMat(&theta);
}
