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

/*****************************************************************************
** Namespace
*****************************************************************************/

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
	~MainWindow();


        void closeEvent(QCloseEvent *event); // Overloaded function

public Q_SLOTS:


private:
	Ui::MainWindowDesign ui;
	QNode qnode;
};

}  // namespace scanner_gui

#endif // scanner_gui_MAIN_WINDOW_H
