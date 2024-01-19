
#define _USE_MATH_DEFINES
#include <iostream>
#include <cmath>
#include <fstream>
#include <string>
#include <iomanip>
#define double pi = 3.14159265358979323846;
using namespace std;
//the mapping function from plane onto sphere
void MercatorInv(float* point)
{
    point[1] = point[1] * 3.14159265358979323846;
    point[0] = atan(exp(-2 * 3.14159265358979323846 * point[0]));
    point[1] *= 360 / 3.14159265358979323846;
    point[0] *= 360 / 3.14159265358979323846;

}
class Line {
public:
    float m;
    float b;
    Line() {
        m = 0.0; b = 0.0; 
    }
    // line construction function 
    void construct(float** points, float* point1, float* point2) 
    {
        /*float k;
        if(abs(point1[1] - point2[1]) > 0.0001)
            k = (point1[0] - point2[0]) / (point1[1] - point2[1]);
        else
        {
            k = 0.0001;
        }
        float p = point1[0] - k * point1[1];*/
        float dist = sqrt(pow((point1[0] - point2[0]), 2) + pow((point1[1] - point2[1]), 2));
        float strideX = (point1[0] - point2[0]) / 100;
        float strideY = (point1[1] - point2[1]) / 100;
        float lon = point1[0] * 2 * M_PI / 360;
        float lat1 = point1[1] * 2 * M_PI / 360;
        float lon1 =  point1[0] * 2 * M_PI / 360;
        float lon2 =  point2[0] * 2 * M_PI / 360;
        float lat2 = point2[1] * 2 * M_PI / 360;
        float lat = 0.0;
        for (int i = 0; i < 100; i++)
        {
            
            lat = atan((sin(lat1) * cos(lat2) * sin(lon - lon2)
                - sin(lat2) * cos(lat1) * sin(lon - lon1)) / (cos(lat1) * cos(lat2) * sin(lon1 - lon2)));
            
            lon -= strideX/360 * 2* M_PI;
            if (lon > M_PI)
            {
                lon = lon - 2*M_PI;
            }
            points[i][0] = lon * 360 / (2 * M_PI);
            points[i][1] = lat * 360 / (2 * M_PI);
            cout << lon * 360/(2*M_PI) << ", " << lat * 360 / (2 * M_PI) << endl;
        }

        cout << strideX << ", " << strideY << endl;
        //cout << k << endl;
        //cout << p << endl;
        //float templong =point1[0];
        //float templat = point1[1];
        //points[0][0] = point1[0];
        //points[0][1] = point1[1];
        //for (int i = 1; i < 101; i++)
        //{
        //    templong = points[i-1][0] - strideX;
        //    
        //    if (k <= 0.0000001 && k >= -0.0000001)
        //    {
        //        templat -= strideY;
        //    }
        //    else
        //    {
        //        templat = (templong - p) / k;
        //    }
        //    points[i][0] = templong;
        //    points[i][1] = templat;
        //    cout << points[i][0] << "  " << points[i][1] << endl;
        //    /*MercatorInv(points[i]);*/
        //}
        //m = k;
        //b = p;
    }
};

//mapping function from sphere onto plane
void Mercator(float* point)
{
    point[1] /= 360 / 3.14159265358979323846;
    point[0] /= 360 / 3.14159265358979323846;
    point[1] = point[1] / 3.14159265358979323846;
    point[0] = log((1 + sin(point[0])) / (1 - sin(point[0]))) / (4 * 3.14159265358979323846);
    cout << point[0] << endl;
    cout << point[1] << endl;
}

int main()
{
    int datasize = 101;
    float** points = new float*[datasize];  //dataset for points to visualize in google Earth
    for (int i = 0; i < datasize; i++)
    {
        points[i] = new float[2];
    }
    float point1[2] = { -123.260032, 49.271437}; //start
    float point2[2] = { -122.9253053 ,49.280064}; //finish variable
    
    Line* traj = new Line;
    traj->construct(points, point1, point2);
    ofstream outputFile("out.csv");
    if (!outputFile) {
        std::cerr << "Failed to open the file for writing." << std::endl;
        
    }
    for (int i = 0; i < datasize; i++) {
        outputFile <<setprecision(10)<< points[i][0] << "," <<setprecision(10)<< points[i][1]<<endl;
    }

}

