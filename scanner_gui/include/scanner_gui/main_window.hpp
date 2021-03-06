/**
 * @file /include/scanner_gui/main_window.hpp
 *
 * @brief Qt based gui for scanner_gui.
 *
 * @date November 2010
 **/
#ifndef scanner_gui_MAIN_WINDOW_H
#define scanner_gui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "scanner.hpp"
#include <QFileDialog>
#include <QLabel>
#include <QTimer>
#include <iostream>
#include <sys/syscall.h>
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <vector>
#include <string>
#include <fstream>
#include <stdio.h>
#include <cstdlib>
#include <csignal>
#include <ctime>

/*****************************************************************************
** Namespace
*****************************************************************************/
using namespace std;

namespace scanner_gui {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
        QMutex mutex;
	~MainWindow();

        void closeEvent(QCloseEvent *event); // Overloaded function

public Q_SLOTS:

private Q_SLOTS:
        void on_pushButton_ligarscanner_clicked();
        void on_pushButton_inicio_clicked();
        void on_pushButton_visualizar_clicked();
        void on_pushButton_aquisicao_clicked();
        void on_pushButton_fimaquisicao_clicked();

        void on_dial_minmotor_sliderReleased();
        void on_dial_maxmotor_sliderReleased();
        void on_dial_minlaser_sliderReleased();
        void on_dial_maxlaser_sliderReleased();

        void on_lineEdit_minmotor_returnPressed();
        void on_lineEdit_maxmotor_returnPressed();
        void on_lineEdit_minlaser_returnPressed();
        void on_lineEdit_maxlaser_returnPressed();

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
        Scanner scan;

        double laser_min, laser_max;
};

}  // namespace scanner_gui

#endif // scanner_gui_MAIN_WINDOW_H
