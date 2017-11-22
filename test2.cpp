#include <iostream>
#include <cmath>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>// Eigen 几何模块


#include <vector>

void atest( vector<int> aa )
{
    int num;
    for(int i = 0; i < aa.size(); i++)
    {
        num = aa[i];
        cout << num << " ";
    }
}


class Foo
{

public:
      Eigen::Vector2d v;
      int a;
};





int main ( int argc, char** argv )
{
    Foo *foo = new Foo();

    foo->a = 2;
    foo->v(0) = foo->a;
    cout<<foo->v;
    cout<<"test2 finished..."<<endl;
   ///***************************************************///
    std::vector<int> v = {7, 5, 16, 8};

    // Add two more integers to vector
    v.push_back(25);
    v.push_back(13);

    // Iterate and print values of vector
    for(int n : v) {
        std::cout << n << " ";;
    }
    cout<<endl;
    atest(v);


    return 0;
}
