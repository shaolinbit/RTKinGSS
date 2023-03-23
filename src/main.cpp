#include <iostream>
#include <unistd.h>
#include "Rtkpost_ndlg.h"
using namespace std;

int main()
{
    char buffer[256];
    char *val = getcwd(buffer, sizeof(buffer));
    if (val) {
        std::cout <<"relative path："<< buffer << std::endl;
    }
    Rtkpost();

    return 0;
}
