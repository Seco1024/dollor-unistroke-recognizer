#include <vector>
#include <iostream>
#include <cmath>
#define PI 3.14159265

using namespace std;

// point
class Point{
public:
    double x, y;
    Point(double x, double y): x(x), y(y) {}
};

//rectangle 
class Rectangle{
public:
    Rectangle(double leftx, double rightx, double topy, double bottomy){
        this->leftX_ = leftx;
        this->rightX_ = rightx;
        this->topY_ = topy;
        this->bottomY_ = bottomy;
    }
    double leftX() {return leftX_;}
    double rightX() {return rightX_;}
    double topY() {return topY_;}
    double bottomY() {return bottomY_;}
    void setRangeX(int x1, int x2);
    void setRangeY(int y1, int y2);
private:
    double leftX_;
    double rightX_;
    double topY_;
    double bottomY_;
};

void Rectangle::setRangeX(int x1, int x2){
    if(x1 < x2){
        leftX_ = x1;
        rightX_ = x2;
    }
    else{
        leftX_ = x2;
        rightX_ = x1;
    }
}

void Rectangle::setRangeY(int y1, int y2){
    if (y1 >= y2){
        topY_ = y1;
        bottomY_ = y2;
    }else{
        topY_ = y2;
        bottomY_ = y1;
    }
}

struct Result{
    string name;
    double score;
    Result(string input_name, double input_score): name (input_name), score(input_score) {}
};

//unistroke class
class Unistroke{
public:
    Unistroke(string name, vector<Point> points): name_(name), points_(points) {
        this->points_ = Resample(this->points_, 64); //resample成64個點
        int radians = IndicativeAngle(this->points_); //取角度
        this->points_ = RotateBy(this->points_, -radians); //旋轉
        this->points_ = ScaleTo(this->points_, SquareSize); //縮放
        this->points_ = TranslateTo(this->points_, Origin); //平移
    }
    string setName(string name) {this->name_ = name;}  //更改名字
    string getName() {return this->name_;} //取得名字
    vector<Point> points(){return points_;} //回傳點
private:
    string name_;
    vector<Point> points_;
};

//dolorrecognizer constant
double SquareSize = 250;
Point Origin(0, 0);
double Diagonal = sqrt(SquareSize * SquareSize + SquareSize * SquareSize);
double HalfDiagonal = 0.5 * Diagonal;
double AngleRange = 45.0 * PI / 180.0;
double AnglePrecision = 2.0 * PI / 180.0;
double Phi = 0.5 * (-1.0 + sqrt(5.0)); // 黃金比例

//dollar recognizer
class DollarRecognizer{
public:
    struct Result *Recognize(vector<Point> points){
        Unistroke candidate("", points);
        double b = +1.0e10;
        int u = -1;
        for (int i = 0; i < this->templates_.size(); i++){
            double d = DistanceAtBestAngle(candidate.points(), this->templates_[i].points(), -AngleRange, +AngleRange, AnglePrecision);
            if (d < b){
                b = d;
                u = i;
            }
        }
        return (u == -1) ? new Result("No match.", 0.0) : new Result(this->templates_[u].getName(), 1.0 / b);
    }
private:
    vector<Unistroke> templates_;
};

// $1 Unistroke Recognizer Algorithm
//resample
vector<Point> Resample(vector<Point> points, int n){
    double I = PathLength(points) / (n - 1); //理想區段長度
    double D = 0.0;
    vector<Point> newpoints; //重新取樣後的組合
    for (int i = 1; i < points.size(); i++){
        if (points[i].x == points[i-1].x && points[i].y == points[i-1].y)
            continue;
        else{
            double d = Distance(points[i-1], points[i]);
            if ((D + d) >= I){
                double qx = points[i-1].x + ((I-D) / d) * (points[i].x - points[i-1].x);
                double qy = points[i-1].y + ((I-D) / d) * (points[i].y - points[i-1].y);
                Point q(qx, qy);
                newpoints.push_back(q);
                points.insert(points.begin() + i, q);
                D = 0.0;
            }
            else D += d;
        }
    }
    if (newpoints.size() == n - 1){
        newpoints.push_back(points[points.size() - 1]);
    }
    return newpoints;
}

//indicative angle
double IndicativeAngle(vector<Point> points){
    Point *center = Centroid(points);
    return atan2(center->y - points[0].y, center->x - points[0].x);
}

//rotate by
vector<Point> RotateBy(vector<Point> points, double radians){
    Point *center = Centroid(points);
    double sin = asin(radians);
    double cos = acos(radians);
    vector<Point> newpoints;
    for (int i = 0; i < points.size(); i++){
        double qx = (points[i].x - center->x) * cos - (points[i].y - center->y) * sin + center->x;
        double qy = (points[i].x - center->x) * sin + (points[i].y - center->y) * cos + center->y;
        Point q(qx, qy);
    }
    return newpoints;
}

//centroid
Point *Centroid(vector<Point> points){
    double x = 0.0, y = 0.0;
    for (int i = 0; i < points.size(); i++) {
        x += points[i].x;
        y += points[i].y;
    }
    x /= points.size();
    y /= points.size();
    return new Point(x, y);  
}

//scale to
vector<Point> ScaleTo(vector<Point> points, double size){
    Rectangle *boundingBox = BoundingBox(points);
    vector<Point> newpoints;
    for (int i = 0; i < points.size(); i++){
        double qx = points[i].x * (size / (boundingBox->rightX() - boundingBox->leftX()));
        double qy = points[i].y * (size / (boundingBox->topY() - boundingBox->bottomY()));
        newpoints.push_back(Point(qx, qy));
    }
    return newpoints;
}

//bounding box
Rectangle *BoundingBox(vector<Point> points){
    double minX = +1.0e10;
    double maxX = -1.0e10;
    double minY = +1.0e10;
    double maxY = -1.0e10;
    for (int i = 0; i < points.size(); i++){
        if (points[i].x < minX) minX = points[i].x;
        if (points[i].x > maxX) maxX = points[i].x;
        if (points[i].y < minY) minY = points[i].y;
        if (points[i].y > maxY) maxY = points[i].y;
    }
    return new Rectangle(minX, maxX, maxY, minY);
}

//translate to
vector<Point> TranslateTo(vector<Point> points, Point pt){
    Point *center = Centroid(points);
    vector<Point> newpoints;
    for (int i = 0; i < points.size(); i++){
        double qx = points[i].x + pt.x - center->x;
        double qy = points[i].y + pt.y - center->y;
        newpoints.push_back(Point(qx, qy));
    }
    return newpoints;
}
 //path length
double PathLength(vector<Point> points){
    double d = 0.0;
    for (int i = 0; i < points.size(); i++){
        d += Distance(points[i - 1], points[i]);
    }
    return d;
}

//distance
double Distance(Point p1, Point p2){
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    return sqrt(dx * dx + dy * dy);
}

//distance at best angle
double DistanceAtBestAngle(vector<Point> points, vector<Point> T, double a, double b, double threshold){
    double x1 = Phi * a + (1.0 - Phi) * b;
    double f1 = DistanceAtAngle(points, T, x1);
    double x2 = (1.0 - Phi) * a + Phi * b;
    double f2 = DistanceAtAngle(points, T, x2);
    while(abs(b - a) > threshold){
        if (f1 < f2){
            b = x2;
            x2 = x1;
            f2 = f1;
            x1 = Phi * a + (1.0 - Phi) * b;
            f1 = DistanceAtAngle(points, T, x1);
        }else{
            a = x1;
            x1 = x2;
            f1 = f2;
            x2 = (1.0 - Phi) * a + Phi * b;
            f2 = DistanceAtAngle(points, T, x2);
        }
    }
    return min(f1, f2);
}

//distance at angle
double DistanceAtAngle(vector<Point> points, vector<Point> T, double radians){
    vector<Point> newpoints = RotateBy(points, radians);
    return PathDistance(newpoints, T);
}

//path distance
double PathDistance(vector<Point> pts1, vector<Point> pts2){
    double d = 0.0;
    for (int i = 0; i < pts1.size(); i++){
        d += Distance(pts1[i], pts2[i]);
    }
    return d / pts1.size();
}
