#include<iostream>
#include<math.h>
#include<format>
using namespace std;
double dis(double x[], double y[])
{
    double sum = 0;
    for (int i = 0; i < 3; i++)
    {
        for (int j = i + 1; j < 3; j++)
        {
            sum += hypot((x[i] - x[j]), (y[i] - y[j]));
        }
    }
    return sum;
}
int main()
{
    double x[3], y[3];
    for (int i = 0; i < 3; i++)
    {
        cin >> x[i] >> y[i];
    }
    double sum = 0;
    sum = dis(x, y);
    cout << format("{:.2f}", sum) << endl;
    return 0;
}