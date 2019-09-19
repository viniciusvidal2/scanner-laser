/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/scanner_gui/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace scanner_gui {

using namespace Qt;
using namespace std;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/
///////////////////////////////////////////////////////////////////////////////////////////
MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , qnode(argc,argv)
    , scan(argc, argv, &mutex)
{
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

    // Ajustes da ui
    setWindowIcon(QIcon(":/images/icon.png"));
    // Valores para o laser
    laser_min = -M_PI/4; laser_max = M_PI/4;
    // Ajuste dos Dials
    int min_motor, max_motor; // DEGREES
    scan.get_limits(min_motor, max_motor);
    ui.dial_minmotor->setMinimum(min_motor);
    ui.dial_minmotor->setMaximum(max_motor);
    ui.dial_minmotor->setValue(min_motor);
    ui.dial_maxmotor->setMinimum(min_motor);
    ui.dial_maxmotor->setMaximum(max_motor);
    ui.dial_maxmotor->setValue(max_motor);
    ui.dial_minlaser->setMinimum(-90);
    ui.dial_minlaser->setMaximum( 90);
    ui.dial_minlaser->setValue(int(RAD2DEG(laser_min)));
    ui.dial_maxlaser->setMinimum(-90);
    ui.dial_maxlaser->setMaximum( 90);
    ui.dial_maxlaser->setValue(int(RAD2DEG(laser_max)));
    // Ajuste dos lineEdits
    ui.lineEdit_minmotor->setText(QString::number(ui.dial_minmotor->value()));
    ui.lineEdit_maxmotor->setText(QString::number(ui.dial_maxmotor->value()));
    ui.lineEdit_minlaser->setText(QString::number(ui.dial_minlaser->value()));
    ui.lineEdit_maxlaser->setText(QString::number(ui.dial_maxlaser->value()));
    // Ajuste dos pushButtons
    ui.pushButton_aquisicao->setEnabled(false);
    ui.pushButton_fimaquisicao->setEnabled(false);
    ui.pushButton_inicio->setEnabled(false);
    ui.pushButton_visualizar->setEnabled(false);
    ui.pushButton_aquisicao->setStyleSheet("background-color: rgb(100, 250, 100); color: rgb(0, 0, 0)");
    ui.pushButton_fimaquisicao->setStyleSheet("background-color: rgb(70, 0, 0); color: rgb(250, 250, 250)");

}
///////////////////////////////////////////////////////////////////////////////////////////
MainWindow::~MainWindow() {}
///////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// PUSHBUTTONS ///////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
void MainWindow::on_pushButton_ligarscanner_clicked(){
    // Ligar o laser e os motores dynamixel
    std::string minl = std::to_string(DEG2RAD(ui.dial_minlaser->value()));
    std::string maxl = std::to_string(DEG2RAD(ui.dial_maxlaser->value()));
    std::replace(minl.begin(), minl.end(), ',', '.');
    std::replace(maxl.begin(), maxl.end(), ',', '.');
    std::string ligar = "gnome-terminal -x sh -c 'roslaunch scanner_gui lancar_scanner.launch ";
    ligar = ligar + "laser_min:=" + minl + " ";
    ligar = ligar + "laser_max:=" + maxl + "'";
    system(ligar.c_str());
    // Colocar dentro da classe os valores de inicio e fim de curso
    scan.set_course(double(ui.dial_minmotor->value()), double(ui.dial_maxmotor->value()));
    // Habilita outro botao
    ui.pushButton_inicio->setEnabled(true);
}
///////////////////////////////////////////////////////////////////////////////////////////
void MainWindow::on_pushButton_inicio_clicked(){
    // Manda o valor mais atual dos limites de curso
    scan.set_course(double(ui.dial_minmotor->value()), double(ui.dial_maxmotor->value()));
    // Agora pode mandar a classe enviar o comando
    scan.send_begin_course();
    // Espera chegar dando noticia
    ros::Rate r(1);
    int local;
    while(!scan.begin_reached(local)){
        ROS_INFO("Ainda estamos indo para o inicio de curso na posicao %d......", local);
        r.sleep();
    }
    ROS_WARN("Podemos comecar a aquisitar !!");
    // Habilita o outro botao
    ui.pushButton_aquisicao->setEnabled(true);
    ui.pushButton_fimaquisicao->setEnabled(true);
    ui.pushButton_visualizar->setEnabled(true);
}
///////////////////////////////////////////////////////////////////////////////////////////
void MainWindow::on_pushButton_aquisicao_clicked(){
    // Seta a variavel de controle que libera o callback funcionar
    scan.set_acquisition(true);
    // Comando para o fim de curso
    scan.start_course();

    ROS_WARN("Comecamos a aquisitar, ligue o visualizador.");
}
///////////////////////////////////////////////////////////////////////////////////////////
void MainWindow::on_pushButton_fimaquisicao_clicked(){
    // Seta a variavel de controle que fecha o callback funcionar
    scan.set_acquisition(false);
    ROS_INFO("Parou a aquisicao de forma assincrona, salvando a nuvem na area de trabalho....");
    scan.save_cloud();
    ROS_INFO("Nuvem salva.");
}
///////////////////////////////////////////////////////////////////////////////////////////
void MainWindow::on_pushButton_visualizar_clicked(){
    system("gnome-terminal -x sh -c 'rosrun rviz rviz -d $HOME/laser_ws/src/scanner-laser/scanner_gui/resources/nuvem.rviz'");
}
///////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////// DIALS /////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
void MainWindow::on_dial_minmotor_sliderReleased(){
    // Atualiza limites de fim de curso de ambos os sliders de motor
    scan.set_course(double(ui.dial_minmotor->value()), double(ui.dial_maxmotor->value()));
    // Ajusta o texto com o novo valor
    ui.lineEdit_minmotor->setText(QString::number(ui.dial_minmotor->value()));
}
///////////////////////////////////////////////////////////////////////////////////////////
void MainWindow::on_dial_maxmotor_sliderReleased(){
    // Atualiza limites de fim de curso de ambos os sliders de motor
    scan.set_course(double(ui.dial_minmotor->value()), double(ui.dial_maxmotor->value()));
    // Ajusta o texto com o novo valor
    ui.lineEdit_maxmotor->setText(QString::number(ui.dial_maxmotor->value()));
}
///////////////////////////////////////////////////////////////////////////////////////////
void MainWindow::on_dial_minlaser_sliderReleased(){
    // Atualizar a variavel local dessa classe de limite de laser
    laser_min = DEG2RAD(double(ui.dial_minlaser->value()));
    // Ajusta o texto com o novo valor
    ui.lineEdit_minlaser->setText(QString::number(ui.dial_minlaser->value()));
}
///////////////////////////////////////////////////////////////////////////////////////////
void MainWindow::on_dial_maxlaser_sliderReleased(){
    // Atualizar a variavel local dessa classe de limite de laser
    laser_max = DEG2RAD(double(ui.dial_maxlaser->value()));
    // Ajusta o texto com o novo valor
    ui.lineEdit_maxlaser->setText(QString::number(ui.dial_maxlaser->value()));
}
///////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// LINEEDITS /////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
void MainWindow::on_lineEdit_minmotor_returnPressed(){
    // Seta o valor do dial
    ui.dial_minmotor->setValue(ui.lineEdit_minmotor->text().toInt());
    // Atualiza limites de fim de curso de ambos os sliders de motor
    scan.set_course(double(ui.dial_minmotor->value()), double(ui.dial_maxmotor->value()));
}
///////////////////////////////////////////////////////////////////////////////////////////
void MainWindow::on_lineEdit_maxmotor_returnPressed(){
    // Seta o valor do dial
    ui.dial_maxmotor->setValue(ui.lineEdit_maxmotor->text().toInt());
    // Atualiza limites de fim de curso de ambos os sliders de motor
    scan.set_course(double(ui.dial_minmotor->value()), double(ui.dial_maxmotor->value()));
}
///////////////////////////////////////////////////////////////////////////////////////////
void MainWindow::on_lineEdit_minlaser_returnPressed(){
    // Seta o valor do dial
    ui.dial_minlaser->setValue(ui.lineEdit_minlaser->text().toInt());
    // Novo valor para a variavel local do laser
    laser_min = DEG2RAD(ui.lineEdit_minlaser->text().toDouble());
}
///////////////////////////////////////////////////////////////////////////////////////////
void MainWindow::on_lineEdit_maxlaser_returnPressed(){
    // Seta o valor do dial
    ui.dial_maxlaser->setValue(ui.lineEdit_maxlaser->text().toInt());
    // Novo valor para a variavel local do laser
    laser_max = DEG2RAD(ui.lineEdit_maxlaser->text().toDouble());
}
///////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////
void MainWindow::closeEvent(QCloseEvent *event)
{
    // Terminar tudo no sistema para nao conflitar com a proxima abertura
    system("gnome-terminal -x sh -c 'rosnode kill --all'");
    QMainWindow::closeEvent(event);
}
///////////////////////////////////////////////////////////////////////////////////////////
}  // namespace scanner_gui

