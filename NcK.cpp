#include<iostream>
#include<vector>

using namespace std;
void showlist(vector<int> &);

int main(int argc, char **argv)
{
    vector<int> list;
    for(int i = 1; i <= 10; i++)
        list.push_back(i);
    std::cout << "Size of list is now: " << list.size() << "\n";
    for(int j = list.size(); j > 0 ; --j)
    {
        showlist(list);
        list.resize(list.size() - 1);
        std::cout << "\n";
    }
    return 0;
}

void showlist(vector<int> &list)
{
    for(int i = 0; i < list.size(); i++)
        std::cout << list[i] << " ";
}
