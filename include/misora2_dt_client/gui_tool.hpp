#ifndef GUITOOL_H
#define GUITOOL_H

#include <opencv2/opencv.hpp>
#include <string>

class Button {
    public:
    cv::Point pos;
    cv::Size size;
    
    Button(cv::Point pos, cv::Size size);
    bool isClicked(const cv::Point& clickPos) const;
};

class DrawTool{
    private:
    cv::Mat image;
    std::vector<Button> buttons; 

    public:
    DrawTool(int width, int height, cv::Scalar backgroundColor = cv::Scalar(0,0,0));

    void drawImage(const cv::Mat& img, cv::Point pos, cv::Size size);
    void drawText(const std::string& text, cv::Point pos, double size = 1, cv::Scalar color = cv::Scalar(255,255,255), int thickness = 1);
    void drawAtText(const std::string& text, cv::Point pos, double size = 1, cv::Scalar color = cv::Scalar(255,255,255), int thickness = 1);
    void drawButton(const Button& button, const std::string& text, cv::Scalar color = cv::Scalar(255,255,255), 
                    int thickness = -1, int lineType = cv::LINE_8, double textSize = 2,
                    cv::Scalar textColor =cv::Scalar(0,0,0), int textThickness = 1);
    void drawText_new(const std::string& text, cv::Point pos, cv::Size size, cv::Scalar color = cv::Scalar(255,255,255), int thickness = 1);
    void drawButton_new(const Button& button, const std::string& text, cv::Scalar color = cv::Scalar(0,0,0), int thickness = -1, int lineType = cv::LINE_AA,
        cv::Scalar textColor = cv::Scalar(255,255,255), int textThickness = 1);
    const cv::Mat& getImage()const;
};

#endif