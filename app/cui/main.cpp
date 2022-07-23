#include <QCoreApplication>
#include "stdafx.h"

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    rtksrv_cui_start();

    return a.exec();
}
