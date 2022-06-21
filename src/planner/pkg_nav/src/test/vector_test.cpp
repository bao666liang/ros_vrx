#include <iostream>
#include <vector>
using namespace std;

int main() //m*n vector
{
    int m = 3;
    int n = 4;
    vector<vector<int>> v1;
    v1.resize(m);
    for (int i = 0; i < m; i++)
    {
        v1[i].resize(n);
    }

    for (int i = 0; i < m; i++)
    {
        for (int j = 0; j < n; j++)
        {
            cout << "v1"
                 << "[" << i << "]"
                 << "[" << j << "] = " << v1[i][j] << endl;
        }
    }
    return 0;
}

int main1() //vector connect
{
    vector<string> v1;
    v1.push_back("nhooo");
    v1.push_back(".com");
    for (vector<string>::iterator itr = v1.begin(); itr != v1.end(); ++itr)
    {
        cout << *itr << endl;
    }

    return 0;
}