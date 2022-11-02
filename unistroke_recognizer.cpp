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

//dolorrecognizer constant
double SquareSize = 250;
Point Origin(0, 0);
double Diagonal = sqrt(SquareSize * SquareSize + SquareSize * SquareSize);
double HalfDiagonal = 0.5 * Diagonal;
double AngleRange = (45.0 * PI / 180.0);
double AnglePrecision = (2.0 * PI / 180.0);
double Phi = 0.5 * (-1.0 + sqrt(5.0)); // 黃金比例

//distance
double Distance(Point p1, Point p2){
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    return sqrt(dx * dx + dy * dy);
}

 //path length
double PathLength(vector<Point> points){
    double d = 0.0;
    for (int i = 1; i < points.size(); i++){
        d += Distance(points[i - 1], points[i]);
    }
    return d;
}

// $1 Unistroke Recognizer Algorithm
//resample
vector<Point> Resample(vector<Point> points, int n){
    double I = PathLength(points) / (n - 1); //理想區段長度
    double D = 0.0;
    vector<Point> newpoints; //重新取樣後的組合
    newpoints.push_back(points[0]);
    for (int i = 1; i < points.size(); i++){
        double d = Distance(points[i-1], points[i]);
        if ((D + d) >= I){
            double qx = points[i-1].x + ((I-D) / d) * (points[i].x - points[i-1].x);
            double qy = points[i-1].y + ((I-D) / d) * (points[i].y - points[i-1].y);
            newpoints.push_back(Point(qx, qy));
            points.insert(points.begin() + i, Point(qx, qy));
            D = 0.0;
        }else D += d;
    }
    if (newpoints.size() == n - 1){
        newpoints.push_back(Point(points[points.size() - 1].x, points[points.size() - 1].y));
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

//indicative angle
double IndicativeAngle(vector<Point> points){
    Point *center = Centroid(points);
    return atan2f(center->y - points[0].y, center->x - points[0].x);
}

//rotate by
vector<Point> RotateBy(vector<Point> points, double radians){
    Point *center = Centroid(points);
    double sin1 = sin(radians);
    double cos1 = cos(radians);
    vector<Point> newpoints;
    for (int i = 0; i < points.size(); i++){
        double qx = (points[i].x - center->x) * cos1 - (points[i].y - center->y) * sin1 + center->x;
        double qy = (points[i].x - center->x) * sin1 + (points[i].y - center->y) * cos1 + center->y;
        Point q(qx, qy);
        newpoints.push_back(q);
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

//path distance
double PathDistance(vector<Point> pts1, vector<Point> pts2){
    double d = 0.0;
    for (int i = 0; i < pts1.size(); i++){
        d += Distance(pts1[i], pts2[i]);
    }
    return d / pts1.size();
}

//distance at angle
double DistanceAtAngle(vector<Point> points, vector<Point> T, double radians){
    vector<Point> newpoints = RotateBy(points, radians);
    return PathDistance(newpoints, T);
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

//unistroke class
class Unistroke{
public:
    Unistroke(string name, vector<Point> points): name_(name), points_(points) {
        this->points_ = Resample(this->points_, 64); //resample成64個點
        double radians = IndicativeAngle(this->points_); //取角度
        this->points_ = RotateBy(this->points_, -radians); //旋轉
        this->points_ = ScaleTo(this->points_, SquareSize); //縮放
        this->points_ = TranslateTo(this->points_, Origin); //平移
    }
    string setName(string name) {
        this->name_ = name;
        return this->name_;
    }  //更改名字
    string getName() {return this->name_;} //取得名字
    vector<Point> points(){return this->points_;} //回傳點
private:
    string name_;
    vector<Point> points_;
};

//dollar recognizer
class DollarRecognizer{
public:
    struct Result *Recognize(vector<Point> points){
        Unistroke candidate("c", points);
        double b = 100000000000.0;
        int u = -1;
        for (int i = 0; i < this->templates_.size(); i++){
            double d = DistanceAtBestAngle(candidate.points(), this->templates_[i].points(), -AngleRange, +AngleRange, AnglePrecision);
            if (d < b){
                b = d;
                u = i;
            }
        }
        if (u == -1){
            return new Result("No match.", 0.0);
        }else{
            return new Result(this->templates_[u].getName(), 1.0 - b / HalfDiagonal);
        }
    }
private:
    vector<Unistroke> templates_ = 
    {Unistroke("triangle", {Point(137,139),Point(135,141), Point(133,144), Point(132,146), Point(130,149),Point(128,151),Point(126,155),Point(123,160),Point(120,166),Point(116,171),Point(112,177),Point(107,183),Point(102,188),Point(100,191),Point(95,195),Point(90,199),Point(86,203), Point(82,206), Point(80,209), Point(75,213),Point(67,219), Point(64,221), Point(61,223),Point(60,225), Point(62,226),Point(65,225),Point(67,226),Point(74,226),Point(77,227),Point(85,229),Point(91,230),Point(99,231),Point(108,232),Point(116,233),Point(125,233),Point(134,234),Point(145,233),Point(153,232),Point(160,233),Point(170,234),Point(177,235),Point(179,236),Point(186,237),Point(193,238),Point(198,239),Point(200,237),Point(202,239),Point(204,238),Point(206,234),Point(205,230),Point(202,222),Point(197,216),Point(192,207),Point(186,198),Point(179,189),Point(174,183),Point(170,178),Point(164,171),Point(161,168),Point(154,160),Point(148,155),Point(143,150),Point(138,148),Point(136,148)})
    , Unistroke("one", {Point(61,223),Point(60,225), Point(62,226),Point(65,225),Point(67,226),Point(74,226),Point(77,227),Point(85,229),Point(91,230),Point(99,231),Point(108,232),Point(116,233),Point(125,233),Point(134,234),Point(145,233),Point(153,232),Point(160,233),Point(170,234),Point(177,235),Point(179,236),Point(186,237),Point(193,238),Point(198,239),Point(200,237),Point(202,239),Point(204,238),Point(206,234),Point(205,230),Point(202,222),Point(197,216),Point(192,207),Point(186,198),Point(179,189),Point(174,183),Point(170,178),Point(164,171),Point(161,168),Point(154,160),Point(148,155),Point(143,150),Point(138,148),Point(136,148)})};
};

int main(void){
    vector<Point> points = {Point(61,223),Point(60,225), Point(62,226),Point(65,225),Point(67,226),Point(74,226),Point(77,227),Point(85,229),Point(91,230),Point(99,231),Point(108,232),Point(116,233),Point(125,233),Point(134,234),Point(145,233),Point(153,232),Point(160,233),Point(170,234),Point(177,235),Point(179,236),Point(186,237),Point(193,238),Point(198,239),Point(200,237),Point(202,239),Point(204,238),Point(206,234),Point(205,230),Point(202,222),Point(197,216),Point(192,207),Point(186,198),Point(179,189),Point(174,183),Point(170,178),Point(164,171),Point(161,168),Point(154,160),Point(148,155),Point(143,150),Point(138,148),Point(136,148)};
    DollarRecognizer D;
    struct Result* result = D.Recognize(points);
    cout << result->name << result->score << endl;
    return 0;
}