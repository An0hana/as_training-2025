#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <iostream>
#include <vector>

using namespace cv;
using namespace dnn;
using namespace std;

int main(int argc, char** argv) {
    if (argc != 3) {
        cout << "Usage: " << argv[0] << " <onnx_model_path> <image_path>" << endl;
        return -1;
    }
    string modelPath = argv[1];
    string imagePath = argv[2];

    // 读取图片
    Mat image = imread(imagePath);
    if (image.empty()) {
        cerr << "Failed to read image: " << imagePath << endl;
        return -1;
    }

    // 加载模型
    dnn::DetectionModel model(modelPath);
    model.setInputParams(1.0 / 255.0, Size(960, 960), Scalar(), true, false);

    // 推理
    vector<int> classIds;
    vector<float> confidences;
    vector<Rect> boxes;
    model.detect(image, classIds, confidences, boxes, 0.5, 0.4); // 置信度阈值、NMS阈值

    // 画框
    Scalar colors[] = {Scalar(255,0,0), Scalar(0,255,0), Scalar(0,0,255), Scalar(255,255,0)};
    for (size_t i = 0; i < boxes.size(); ++i) {
        rectangle(image, boxes[i], colors[classIds[i] % 4], 2);
        string label = format("%.2f", confidences[i]);
        putText(image, label, Point(boxes[i].x, boxes[i].y - 5), FONT_HERSHEY_SIMPLEX, 0.5, colors[classIds[i] % 4], 2);
    }

    imwrite("result.png", image);
    cout << "Result saved to result.png" << endl;
    return 0;
}