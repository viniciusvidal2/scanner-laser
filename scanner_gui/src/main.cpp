/**
 * @file /src/main.cpp
 *
 * @brief Qt based gui.
 *
 * @date November 2010
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QApplication>
#include "../include/scanner_gui/main_window.hpp"
//#include "../include/scanner_gui/registro.hpp"

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

    /*********************
    ** Qt
    **********************/
//    Registro a;
//    a.init();
//    a.process("/home/grin/Desktop/Arquivos Do Workspace laser_ws/sem_calibracao/", "jpg");

    QApplication app(argc, argv);
    scanner_gui::MainWindow w(argc,argv);
    w.show();
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();

	return result;
}
