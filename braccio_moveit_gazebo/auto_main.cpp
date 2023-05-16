#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <torch/torch.h>
#include <ros/ros.h>
#include <gazebo_msgs/LinkState.h>
#include <gazebo_msgs/ModelState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/srv/SetModelState.h>
#include <gazebo_msgs/srv/SpawnModel.h>
#include <time.h>


using namespace std;
using namespace cv;



class AutoTargetter {
public:
    void loadCalibrate() {
        // Implementation for loading calibration
    }

    void getLinkChoose(const string& linkName) {
        // Implementation for getting link choose
    }

    void goToTarget(const string& target, const string& bowlName) {
        // Implementation for going to target
    }
};

class MeasureInference {
public:
    void loadModel() {
        // Implementation for loading the model
    }

    void readImage() {
        // Implementation for reading the image
    }

    void transImage() {
        // Implementation for transforming the image
    }

    void regInt() {
        // Implementation for region of interest
    }

    void infer() {
        // Implementation for inference
    }

    void modelCreation(int dimA, int dimB, int n, const string& classSherd) {
        // Implementation for model creation
    }

    void addLink(const string& name, int n, double x, double y) {
        // Implementation for adding a link
    }

    void printCoord(double x, double y) {
        // Implementation for printing coordinates
    }

    double confidence;
    int prediction;
    double xCenter;
    double yCenter;
    Mat thresholded;
    vector<vector<Point>> cnts;
    double pixelsPerMetric;
    double width;
    Mat ROI;
    int ROI_number;
};

class PositionPub {
public:
    void printCoord(double x, double y) {
        // Implementation for printing coordinates
    }
};

void main() {
    auto autoTargetter = AutoTargetter();
    autoTargetter.loadCalibrate();

    auto measureInference = MeasureInference();
    measureInference.loadModel();
    measureInference.readImage();
    measureInference.transImage();

    int n = 0;
    string visionPath = "../vision/";
    string positionFile = visionPath + "posizioni_mm.txt";

    if (ifstream(positionFile)) {
        remove(positionFile.c_str());
    }

    for (const auto& c : measureInference.cnts) {
        if (contourArea(c) < 100) {
            continue;
        }

        Mat orig = measureInference.thresholded.clone();
        RotatedRect box = minAreaRect(c);
        Point2f boxPoints[4];
        box.points(boxPoints);

        vector<Point> boxPointsVec;
        for (int i = 0; i < 4; i++) {
            boxPointsVec.push_back(boxPoints[i]);
        }

        int xMin = INT_MAX, yMin = INT_MAX, xMax = INT_MIN, yMax = INT_MIN;
        for (const auto& point : boxPointsVec) {
            if (point.x < xMin) xMin = point.x;
            if (point.x > xMax) xMax = point.x;
            if (point.y < yMin) yMin = point.y;
            if (point.y > yMax) yMax = point.y;
        }


Mat roi = measureInference.thresholded(Rect(xMin, yMin, xMax - xMin, yMax - yMin));
    measureInference.regInt();

    Point2f center = box.center;
    measureInference.xCenter = center.x;
    measureInference.yCenter = center.y;

    double dA = norm(boxPoints[0] - boxPoints[1]);
    double dB = norm(boxPoints[1] - boxPoints[2]);

    if (measureInference.pixelsPerMetric == 0) {
        measureInference.pixelsPerMetric = dB / measureInference.width;
    }

    double dimA = dA / measureInference.pixelsPerMetric;
    double dimB = dB / measureInference.pixelsPerMetric;

    if (n > 0) {
        measureInference.infer();

        double centX = measureInference.xCenter / measureInference.pixelsPerMetric;
        double centY = measureInference.yCenter / measureInference.pixelsPerMetric;
        string conf = to_string(measureInference.confidence);
        string lab = to_string(measureInference.prediction);
        string classSherd = "class" + lab;
        string x_mm = to_string(round(centX, 2));
        string y_mm = to_string(round(centY, 2));
        string nome = "sherd_" + to_string(n);

        ofstream position_mm(positionFile, ios::app);
        position_mm << lab << " " << conf << " " << x_mm << " " << y_mm << " " << nome << "\n";
        position_mm.close();

        measureInference.modelCreation(dimA, dimB, n, classSherd);

        PositionPub RosPub;
        RosPub.addLink(nome, n, centX / 1000, centY / 1000);

        if (im_ch == "y") {
            measureInference.imageShow(orig, box, dimA, dimB, tltrX, tltrY, trbrX, trbrY, blbrX, blbrY,
                                       tlblX, tlblY, blX, blY, brX, brY, nome);
        }
    }

    measureInference.ROI_number++;
    n++;
}

ifstream positionFileReader(positionFile);
vector<vector<string>> array;
string line;

while (getline(positionFileReader, line)) {
    istringstream iss(line);
    vector<string> row;

    copy(istream_iterator<string>(iss), istream_iterator<string>(), back_inserter(row));
    array.push_back(row);
}

sort(array.begin(), array.end(), [](const vector<string>& a, const vector<string>& b) {
    return stoi(a[0]) < stoi(b[0]);
});

ofstream stat(visionPath + "groups.txt");
vector<vector<string>> groups;
for (const auto& row : array) {
    groups.push_back(row);
    stat << row[0] << " " << row.size() << " " << accumulate(row.begin() + 1, row.end(), 0.0,
        [](const double& a, const string& b) { return a + stod(b); }) / (row.size() - 1) << " "
        << sqrt(accumulate(row.begin() + 1, row.end(), 0.0,
        [&](const double& a, const string& b) { return a + pow(stod(b) - groupStats.back()[2], 2); }) / (row.size() - 1)) << "\n";
}
stat.close();

ifstream statReader(visionPath + "groups.txt");
vector<vector<string>> statArray;
string statLine;

while (getline(statReader, statLine)) {
    istringstream iss(statLine);
    vector<string> row;

    copy(istream_iterator<string>(iss), istream_iterator<string>(), back_inserter(row));
    statArray.push_back(row);
}

vector<vector<int>> variables(statArray.size(), vector<int>(2, 0));
int idx = 0;

for (int i = 0; i < statArray.size(); i++) {
    if (stod(statArray[i][2]) > 0.30 && stod(statArray[i][3]) < 0.25) {
        variables[idx][0] = stoi(statArray[i][0]);
        variables[idx][1] = stoi(statArray[i][1]);
        idx++;
    }
}

sort(variables.begin(), variables.end(), [](const vector<int>& a, const vector<int>& b) {
    return a[1] > b[1];
});

string choosen = "[";
for (int i = 0; i < 3; i++) {
    choosen += "[" + to_string(variables[i][0]) + ", " + to_string(variables[i][1]) + "]";
    if (i != 2) {
        choosen += ", ";
    }
}
choosen += "]";

ofstream choosenWriter(visionPath + "choosen.txt");
choosenWriter << choosen;
choosenWriter.close();

ifstream posReader(positionFile);
vector<vector<string>> posizioni;
string posLine;

while (getline(posReader, posLine)) {
    istringstream iss(posLine);
    vector<string> row;

    copy(istream_iterator<string>(iss), istream_iterator<string>(), back_inserter(row));
    posizioni.push_back(row);
}

ifstream groupsReader(visionPath + "choosen.txt");
vector<vector<string>> groups;
string groupsLine;

while (getline(groupsReader, groupsLine)) {
    istringstream iss(groupsLine.substr(1, groupsLine.size() - 2));
    vector<string> row;

    copy(istream_iterator<string>(iss), istream_iterator<string>(), back_inserter(row));
    groups.push_back(row);
}

vector<tuple<int, string, string>> linkChoose;

for (int i = 0; i < posizioni.size(); i++) {
    if (find_if(groups.begin(), groups.end(), [&](const vector<string>& g) {
        return posizioni[i][0] == g[0];
    }) != groups.end()) {
        string lk = posizioni[i][0] + "::link";
        string bowlCh = to_string(find(groups.begin(), groups.end(), vector<string>{ posizioni[i][0] }) - groups.begin());

        linkChoose.emplace_back(stoi(posizioni[i][0]), posizioni[i][4], bowlCh);
    }
}

ofstream selectedWriter(visionPath + "selected.txt");
selectedWriter << "[";

for (int j = 0; j < linkChoose.size(); j++) {
    selectedWriter << "(" << get<0>(linkChoose[j]) << ", " << get<1>(linkChoose[j]) << ", " << get<2>(linkChoose[j]) << ")";
    if (j != linkChoose.size() - 1) {
        selectedWriter << ", ";
    }
}

selectedWriter << "]";
selectedWriter.close();

for (int j = 0; j < linkChoose.size(); j++) {
    string inpCh = get<1>(linkChoose[j]) + "::link";
    autoTargetter.getLinkChooseHere's the equivalent C++ code for the provided Python code:

cpp

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <torch/torch.h>
#include <ros/ros.h>
#include <gazebo_msgs/LinkState.h>
#include <gazebo_msgs/ModelState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/srv/SetModelState.h>
#include <gazebo_msgs/srv/SpawnModel.h>
#include <time.h>

using namespace std;
using namespace cv;

class AutoTargetter {
public:
    void loadCalibrate() {
        // Implementation for loading calibration
    }

    void getLinkChoose(const string& linkName) {
        // Implementation for getting link choose
    }

    void goToTarget(const string& target, const string& bowlName) {
        // Implementation for going to target
    }
};

class MeasureInference {
public:
    void loadModel() {
        // Implementation for loading the model
    }

    void readImage() {
        // Implementation for reading the image
    }

    void transImage() {
        // Implementation for transforming the image
    }

    void regInt() {
        // Implementation for region of interest
    }

    void infer() {
        // Implementation for inference
    }

    void modelCreation(int dimA, int dimB, int n, const string& classSherd) {
        // Implementation for model creation
    }

    void addLink(const string& name, int n, double x, double y) {
        // Implementation for adding a link
    }

    void printCoord(double x, double y) {
        // Implementation for printing coordinates
    }

    double confidence;
    int prediction;
    double xCenter;
    double yCenter;
    Mat thresholded;
    vector<vector<Point>> cnts;
    double pixelsPerMetric;
    double width;
    Mat ROI;
    int ROI_number;
};

class PositionPub {
public:
    void printCoord(double x, double y) {
        // Implementation for printing coordinates
    }
};

void main() {
    auto autoTargetter = AutoTargetter();
    autoTargetter.loadCalibrate();

    auto measureInference = MeasureInference();
    measureInference.loadModel();
    measureInference.readImage();
    measureInference.transImage();

    int n = 0;
    string visionPath = "../vision/";
    string positionFile = visionPath + "posizioni_mm.txt";

    if (ifstream(positionFile)) {
        remove(positionFile.c_str());
    }

    for (const auto& c : measureInference.cnts) {
        if (contourArea(c) < 100) {
            continue;
        }

        Mat orig = measureInference.thresholded.clone();
        RotatedRect box = minAreaRect(c);
        Point2f boxPoints[4];
        box.points(boxPoints);

        vector<Point> boxPointsVec;
        for (int i = 0; i < 4; i++) {
            boxPointsVec.push_back(boxPoints[i]);
        }

        int xMin = INT_MAX, yMin = INT_MAX, xMax = INT_MIN, yMax = INT_MIN;
        for (const auto& point : boxPointsVec) {
            if (point.x < xMin) xMin = point.x;
            if (point.x > xMax) xMax = point.x;
            if (point.y < yMin) yMin = point.y;
            if (point.y > yMax) yMax = point.y;
        }

scss

    Mat roi = measureInference.thresholded(Rect(xMin, yMin, xMax - xMin, yMax - yMin));
    measureInference.regInt();

    Point2f center = box.center;
    measureInference.xCenter = center.x;
    measureInference.yCenter = center.y;

    double dA = norm(boxPoints[0] - boxPoints[1]);
    double dB = norm(boxPoints[1] - boxPoints[2]);

    if (measureInference.pixelsPerMetric == 0) {
        measureInference.pixelsPerMetric = dB / measureInference.width;
    }

    double dimA = dA / measureInference.pixelsPerMetric;
    double dimB = dB / measureInference.pixelsPerMetric;

    if (n > 0) {
        measureInference.infer();

        double centX = measureInference.xCenter / measureInference.pixelsPerMetric;
        double centY = measureInference.yCenter / measureInference.pixelsPerMetric;
        string conf = to_string(measureInference.confidence);
        string lab = to_string(measureInference.prediction);
        string classSherd = "class" + lab;
        string x_mm = to_string(round(centX, 2));
        string y_mm = to_string(round(centY, 2));
        string nome = "sherd_" + to_string(n);

        ofstream position_mm(positionFile, ios::app);
        position_mm << lab << " " << conf << " " << x_mm << " " << y_mm << " " << nome << "\n";
        position_mm.close();

        measureInference.modelCreation(dimA, dimB, n, classSherd);

        PositionPub RosPub;
        RosPub.addLink(nome, n, centX / 1000, centY / 1000);

        if (im_ch == "y") {
            measureInference.imageShow(orig, box, dimA, dimB, tltrX, tltrY, trbrX, trbrY, blbrX, blbrY,
                                       tlblX, tlblY, blX, blY, brX, brY, nome);
        }
    }

    measureInference.ROI_number++;
    n++;
}

ifstream positionFileReader(positionFile);
vector<vector<string>> array;
string line;

while (getline(positionFileReader, line)) {
    istringstream iss(line);
    vector<string> row;

    copy(istream_iterator<string>(iss), istream_iterator<string>(), back_inserter(row));
    array.push_back(row);
}

sort(array.begin(), array.end(), [](const vector<string>& a, const vector<string>& b) {
    return stoi(a[0]) < stoi(b[0]);
});

ofstream stat(visionPath + "groups.txt");
vector<vector<string>> groups;
for (const auto& row : array) {
    groups.push_back(row);
    stat << row[0] << " " << row.size() << " " << accumulate(row.begin() + 1, row.end(), 0.0,
        [](const double& a, const string& b) { return a + stod(b); }) / (row.size() - 1) << " "
        << sqrt(accumulate(row.begin() + 1, row.end(), 0.0,
        [&](const double& a, const string& b) { return a + pow(stod(b) - groupStats.back()[2], 2); }) / (row.size() - 1)) << "\n";
}
stat.close();

ifstream statReader(visionPath + "

groups.txt");
vector<vector<string>> statArray;
string statLine;

scss

while (getline(statReader, statLine)) {
    istringstream iss(statLine);
    vector<string> row;

    copy(istream_iterator<string>(iss), istream_iterator<string>(), back_inserter(row));
    statArray.push_back(row);
}

vector<vector<int>> variables(statArray.size(), vector<int>(2, 0));
int idx = 0;

for (int i = 0; i < statArray.size(); i++) {
    if (stod(statArray[i][2]) > 0.30 && stod(statArray[i][3]) < 0.25) {
        variables[idx][0] = stoi(statArray[i][0]);
        variables[idx][1] = stoi(statArray[i][1]);
        idx++;
    }
}

sort(variables.begin(), variables.end(), [](const vector<int>& a, const vector<int>& b) {
    return a[1] > b[1];
});

string choosen = "[";
for (int i = 0; i < 3; i++) {
    choosen += "[" + to_string(variables[i][0]) + ", " + to_string(variables[i][1]) + "]";
    if (i != 2) {
        choosen += ", ";
    }
}
choosen += "]";

ofstream choosenWriter(visionPath + "choosen.txt");
choosenWriter << choosen;
choosenWriter.close();

ifstream posReader(positionFile);
vector<vector<string>> posizioni;
string posLine;

while (getline(posReader, posLine)) {
    istringstream iss(posLine);
    vector<string> row;

    copy(istream_iterator<string>(iss), istream_iterator<string>(), back_inserter(row));
    posizioni.push_back(row);
}

ifstream groupsReader(visionPath + "choosen.txt");
vector<vector<string>> groups;
string groupsLine;

while (getline(groupsReader, groupsLine)) {
    istringstream iss(groupsLine.substr(1, groupsLine.size() - 2));
    vector<string> row;

    copy(istream_iterator<string>(iss), istream_iterator<string>(), back_inserter(row));
    groups.push_back(row);
}

vector<tuple<int, string, string>> linkChoose;

for (int i = 0; i < posizioni.size(); i++) {
    if (find_if(groups.begin(), groups.end(), [&](const vector<string>& g) {
        return posizioni[i][0] == g[0];
    }) != groups.end()) {
        string lk = posizioni[i][0] + "::link";
        string bowlCh = to_string(find(groups.begin(), groups.end(), vector<string>{ posizioni[i][0] }) - groups.begin());

        linkChoose.emplace_back(stoi(posizioni[i][0]), posizioni[i][4], bowlCh);
    }
}

ofstream selectedWriter(visionPath + "selected.txt");
selectedWriter << "[";

for (int j = 0; j < linkChoose.size(); j++) {
    selectedWriter << "(" << get<0>(linkChoose[j]) << ", " << get<1>(linkChoose[j]) << ", " << get<2>(linkChoose[j]) << ")";
    if (j != linkChoose.size() - 1) {
        selectedWriter << ", ";
    }
}

selectedWriter << "]";
selectedWriter.close();

for (int j = 0; j < linkChoose.size(); j++) {
    string inpCh = get<1>(linkChoose[j]) + "::link";
    autoTargetter.getLinkChoose(inpCh);
        string t = get<2>(linkChoose[j]);
    string b = "Bowl " + t;
    autoTargetter.goToTarget(inpCh, b);
}

