class Solution {
public:
    bool isPalindrome(int x) {
        if (x < 0)
        {
            return false;
        }
        long n = 0;
        int tmp = x;
        while (tmp)
        {
            n = n * 10 + tmp % 10;
            tmp /= 10;
        }
        if (n == x)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
};