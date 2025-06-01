#include<iostream>
#include<format>
#include<vector>
using namespace std;
int isPrime(int x)
{
    if (x <= 1)
    {
        return 0;
    }
    else if (x == 2 || x == 3)
    {
        return 1;
    }
    else
    {
        for (int i = 2; i < x; i++)
        {
            if (x % i == 0)
            {
                return 0;
            }
        }
        return 1;
    }
}
int main()
{
    int n;
    cin >> n;
    int num[n];
    vector<int> anw;
    for (int i = 0; i < n; i++)
    {
        cin >> num[i];
    }
    int k = 0;
    for (int i = 0; i < n; i++)
    {
        if (isPrime(num[i]))
        {
            anw.push_back(num[i]);
            k++;
        }
    }
    for (int i = 0; i < k; i++)
    {
        cout << anw[i] << " ";
    }
    return 0;
}