#include<iostream>
#include<vector>
#include<format>
using namespace std;
int main()
{
    int x, y;
    int count = 0;
    vector<int> years;
    cin >> x >> y;
    for (int i = x; i <= y; i++)
    {
        if (i % 4 == 0 && i % 100 != 0)
        {
            years.push_back(i);
            count++;
        }
        else if (i % 400 == 0)
        {
            years.push_back(i);
            count++;
        }
    }
    cout << count << endl;
    for (int i = 0; i < count; i++)
    {
        cout << years[i] << " ";
    }
    return 0;
}