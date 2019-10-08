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
    , rn(argc, argv)
{
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

    // Ajustes da ui
    setWindowIcon(QIcon(":/images/icon.png"));
    // Valores para o laser
    laser_min = -65.0; laser_max = 90.0;

    /// --- ABA 1 --- ///
    // Ajuste dos Dials
    int min_motor, max_motor; // DEGREES
    scan.get_limits(min_motor, max_motor);
    ui.dial_minmotor->setMinimum(min_motor);
    ui.dial_minmotor->setMaximum(max_motor);
    ui.dial_minmotor->setValue(150);
    ui.dial_maxmotor->setMinimum(min_motor);
    ui.dial_maxmotor->setMaximum(max_motor);
    ui.dial_maxmotor->setValue(210);
    ui.dial_minlaser->setMinimum(-90);
    ui.dial_minlaser->setMaximum( 90);
    ui.dial_minlaser->setValue(int(-25));
    ui.dial_maxlaser->setMinimum(-90);
    ui.dial_maxlaser->setMaximum( 90);
    ui.dial_maxlaser->setValue(int(25));
    // Ajuste dos lineEdits
    ui.lineEdit_minmotor->setText(QString::number(ui.dial_minmotor->value()));
    ui.lineEdit_maxmotor->setText(QString::number(ui.dial_maxmotor->value()));
    ui.lineEdit_minlaser->setText(QString::number(ui.dial_minlaser->value()));
    ui.lineEdit_maxlaser->setText(QString::number(ui.dial_maxlaser->value()));
    ui.lineEdit_viagens->setEnabled(false);
    ui.label_viagens->setEnabled(false);
    // Ajuste dos pushButtons    
    ui.pushButton_inicio->setEnabled(false);
    ui.pushButton_visualizar->setEnabled(false);
    ui.pushButton_aquisicao->setStyleSheet("background-color: rgb(100, 250, 100); color: rgb(0, 0, 0)");
    ui.pushButton_fimaquisicao->setStyleSheet("background-color: rgb(70, 0, 0); color: rgb(250, 250, 250)");
    ui.pushButton_aquisicao->setEnabled(false);
    ui.pushButton_fimaquisicao->setEnabled(false);
    // Ajuste da progressBar
    ui.progressBar->setEnabled(false);

    // Liga signal e slot para a progressBar
    connect(&scan, SIGNAL(new_step()), this, SLOT(update_progressBar()));

    /// --- ABA 2 --- ///
    ui.checkBox_icp->setChecked(true); // Checkbox do icp começa a principio valendo
    // Ajustando Sliders com limites e valores iniciais
    ui.horizontalSlider_x->setMinimum(-(ui.lineEdit_limitex->text().toFloat()));
    ui.horizontalSlider_x->setMaximum(ui.lineEdit_limitex->text().toFloat());
    ui.horizontalSlider_x->setValue(0);
    ui.horizontalSlider_y->setMinimum(-(ui.lineEdit_limitey->text().toFloat()));
    ui.horizontalSlider_y->setMaximum(ui.lineEdit_limitey->text().toFloat());
    ui.horizontalSlider_y->setValue(0);
    ui.horizontalSlider_z->setMinimum(-(ui.lineEdit_limitez->text().toFloat()));
    ui.horizontalSlider_z->setMaximum(ui.lineEdit_limitez->text().toFloat());
    ui.horizontalSlider_z->setValue(0);
    // Ajustando Dials com limites e valores iniciais -> GRAUS AQUI
    ui.dial_x->setMinimum(-180);
    ui.dial_x->setMaximum(180);
    ui.dial_x->setValue(0);
    ui.dial_y->setMinimum(-180);
    ui.dial_y->setMaximum(180);
    ui.dial_y->setValue(0);
    ui.dial_z->setMinimum(-180);
    ui.dial_z->setMaximum(180);
    ui.dial_z->setValue(0);

    /// --- ABA 3 --- ///
    // Desabilitando componentes da aba 3
    ui.groupBox_filtrosforma->setEnabled(false);
    ui.frame_salvar->setEnabled(false);
    ui.pushButton_resetafiltro->setEnabled(false);
    ui.groupBox_polinomio->setEnabled(false);
}
///////////////////////////////////////////////////////////////////////////////////////////
MainWindow::~MainWindow() {}

/// --------------------------------- ABA 1 ------------------------------------------- ///

///////////////////////////////////////////////////////////////////////////////////////////
void MainWindow::update_progressBar(){
    int viagem = scan.get_current_trip(), posicao = scan.get_current_position();
    if(posicao != 0){
        int limite_min, limite_max;
        scan.get_limits(limite_min, limite_max);

        int fatia = 100 / ui.lineEdit_viagens->text().toInt();
        int valor = fatia*(viagem - 1);
        if(abs(remainder(viagem, 2)) == 1){ // Indo para o fim de curso
            valor += int(double(fatia) * double(posicao - limite_min)/double(limite_max - limite_min));
        } else {                            // Voltando para o inicio de curso
            valor += int(double(fatia) * double(limite_max - posicao)/double(limite_max - limite_min));
        }

        if(valor > 100 || viagem > ui.lineEdit_viagens->text().toInt())
            valor = 100;

        ui.progressBar->setValue(valor);
    }
}
///////////////////////////////////////////////////////////////////////////////////////////
void MainWindow::on_checkBox_intensidades_stateChanged(int v){
    if(ui.checkBox_intensidades->isChecked()){
        laser_min = -55; laser_max = 55;
        ui.dial_minlaser->setMinimum(-60);
        ui.dial_minlaser->setMaximum( 60);
        ui.dial_minlaser->setValue(int(laser_min));
        ui.dial_maxlaser->setMinimum(-60);
        ui.dial_maxlaser->setMaximum( 60);
        ui.dial_maxlaser->setValue(int(laser_max));

    } else {

        laser_min = -90; laser_max = 90;
        ui.dial_minlaser->setMinimum(-90);
        ui.dial_minlaser->setMaximum( 90);
        ui.dial_minlaser->setValue(int(laser_min));
        ui.dial_maxlaser->setMinimum(-90);
        ui.dial_maxlaser->setMaximum( 90);
        ui.dial_maxlaser->setValue(int(laser_max));
    }

    ui.lineEdit_minmotor->setText(QString::number(ui.dial_minmotor->value()));
    ui.lineEdit_maxmotor->setText(QString::number(ui.dial_maxmotor->value()));
    ui.lineEdit_minlaser->setText(QString::number(ui.dial_minlaser->value()));
    ui.lineEdit_maxlaser->setText(QString::number(ui.dial_maxlaser->value()));
}
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
    ligar = ligar + "laser_max:=" + maxl + " ";
    ligar = ligar + "set_pan_moving_speed:=" + ui.lineEdit_velocidadepan->text().toStdString() + " ";
    if(ui.checkBox_intensidades->isChecked()){
        ligar = ligar + "capt_intensidade:=true'";
    } else {
        ligar = ligar + "capt_intensidade:=false'";
    }
    system(ligar.c_str());
    // Colocar dentro da classe os valores de inicio e fim de curso
    scan.set_course(double(ui.dial_minmotor->value()), double(ui.dial_maxmotor->value()));
    // Habilita a sequencia
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
    ui.lineEdit_viagens->setEnabled(true);
    ui.label_viagens->setEnabled(true);
}
///////////////////////////////////////////////////////////////////////////////////////////
void MainWindow::on_pushButton_aquisicao_clicked(){
    // Pega a resolucao definida pelo lineEdit
    scan.set_trips(ui.lineEdit_viagens->text().toDouble());
    // Seta a variavel de controle que libera o callback funcionar
    scan.set_acquisition(true);
    // Comando para o fim de curso
    scan.start_course();

    ui.pushButton_visualizar->setEnabled(true);
    ui.progressBar->setEnabled(true);

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
    laser_min = double(ui.dial_minlaser->value());
    // Ajusta o texto com o novo valor
    ui.lineEdit_minlaser->setText(QString::number(ui.dial_minlaser->value()));
}
///////////////////////////////////////////////////////////////////////////////////////////
void MainWindow::on_dial_maxlaser_sliderReleased(){
    // Atualizar a variavel local dessa classe de limite de laser
    laser_max = double(ui.dial_maxlaser->value());
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
    laser_min = ui.lineEdit_minlaser->text().toDouble();
}
///////////////////////////////////////////////////////////////////////////////////////////
void MainWindow::on_lineEdit_maxlaser_returnPressed(){
    // Seta o valor do dial
    ui.dial_maxlaser->setValue(ui.lineEdit_maxlaser->text().toInt());
    // Novo valor para a variavel local do laser
    laser_max = ui.lineEdit_maxlaser->text().toDouble();
}
///////////////////////////////////////////////////////////////////////////////////////////

/// --------------------------------- ABA 2 ------------------------------------------- ///

/// Botao para pegar arquivo da nuvem ALVO
void MainWindow::on_pushButton_nuvemalvo_clicked(){
    QString nome_alvo;
    nome_alvo = QFileDialog::getOpenFileName(this, "Nuvem Alvo", "", "PLY Files (*.ply)");
    ui.lineEdit_nuvemalvo->setText(nome_alvo);

    rn.set_nuvem_alvo(nome_alvo);
}

/// Botao para pegar arquivo da nuvem FONTE
void MainWindow::on_pushButton_nuvemfonte_clicked(){
    QString nome_fonte;
    nome_fonte = QFileDialog::getOpenFileName(this, "Nuvem Fonte", "", "PLY Files (*.ply)");
    ui.lineEdit_nuvemfonte->setText(nome_fonte);

    rn.set_nuvem_fonte(nome_fonte);
}

/// Botao para iniciar os visualizadores para os topicos de nuvens alvo, fonte modificada e acumulada
void MainWindow::on_pushButton_iniciararquivos_clicked(){
    rn.set_inicio_processo(true);
    system("gnome-terminal -x sh -c 'rosrun rviz rviz -d $HOME/laser_ws/src/scanner-laser/scanner_gui/resources/tgtsrc.rviz'");
}

/// Sliders sao liberados, a nuvem pode ser transformada
void MainWindow::on_horizontalSlider_x_sliderReleased(){
    double x = (double)ui.horizontalSlider_x->value();
    double y = ui.lineEdit_Y->text().toDouble();
    double z = ui.lineEdit_Z->text().toDouble();

    ui.lineEdit_X->setText(QString::number(x));

    rn.set_translacao(x, y, z);
}
void MainWindow::on_horizontalSlider_y_sliderReleased(){
    double x = ui.lineEdit_X->text().toDouble();
    double y = (double)ui.horizontalSlider_y->value();
    double z = ui.lineEdit_Z->text().toDouble();

    ui.lineEdit_Y->setText(QString::number(y));

    rn.set_translacao(x, y, z);
}
void MainWindow::on_horizontalSlider_z_sliderReleased(){
    double x = ui.lineEdit_X->text().toDouble();
    double y = ui.lineEdit_Y->text().toDouble();
    double z = (double)ui.horizontalSlider_z->value();

    ui.lineEdit_Z->setText(QString::number(z));

    rn.set_translacao(x, y, z);
}

void MainWindow::on_dial_x_sliderReleased(){
    double x = (double)ui.dial_x->value();
    double y = ui.lineEdit_rotacaoy->text().toDouble();
    double z = ui.lineEdit_rotacaoz->text().toDouble();

    ui.lineEdit_rotacaox->setText(QString::number(x));

    rn.set_rotacao(x, y, z);
}
void MainWindow::on_dial_y_sliderReleased(){
    double x = ui.lineEdit_rotacaox->text().toDouble();
    double y = (double)ui.dial_y->value();
    double z = ui.lineEdit_rotacaoz->text().toDouble();

    ui.lineEdit_rotacaoy->setText(QString::number(y));

    rn.set_rotacao(x, y, z);
}
void MainWindow::on_dial_z_sliderReleased(){
    double x = ui.lineEdit_rotacaox->text().toDouble();
    double y = ui.lineEdit_rotacaoy->text().toDouble();
    double z = (double)ui.dial_z->value();

    ui.lineEdit_rotacaoz->setText(QString::number(z));

    rn.set_rotacao(x, y, z);
}

/// Ajustes sobre os limites de translação a partir dos linedits
void MainWindow::on_lineEdit_limitex_returnPressed(){
    ui.horizontalSlider_x->setMaximum( ui.lineEdit_limitex->text().toInt());
    ui.horizontalSlider_x->setMinimum(-ui.lineEdit_limitex->text().toInt());
}
void MainWindow::on_lineEdit_limitey_returnPressed(){
    ui.horizontalSlider_y->setMaximum( ui.lineEdit_limitey->text().toInt());
    ui.horizontalSlider_y->setMinimum(-ui.lineEdit_limitey->text().toInt());
}
void MainWindow::on_lineEdit_limitez_returnPressed(){
    ui.horizontalSlider_z->setMaximum( ui.lineEdit_limitez->text().toInt());
    ui.horizontalSlider_z->setMinimum(-ui.lineEdit_limitez->text().toInt());
}

/// Refinando o valor a partir dos linedits do lado direito
void MainWindow::on_lineEdit_X_returnPressed(){
    double valor = ui.lineEdit_X->text().toDouble();
    if(valor >= ui.horizontalSlider_x->minimum() && valor <= ui.horizontalSlider_x->maximum()){
        ui.horizontalSlider_x->setValue(int(valor));

        double x = ui.lineEdit_X->text().toDouble();
        double y = ui.lineEdit_Y->text().toDouble();
        double z = ui.lineEdit_Z->text().toDouble();

        rn.set_translacao(x, y, z);
    }
}
void MainWindow::on_lineEdit_Y_returnPressed(){
    double valor = ui.lineEdit_Y->text().toDouble();
    if(valor >= ui.horizontalSlider_y->minimum() && valor <= ui.horizontalSlider_y->maximum()){
        ui.horizontalSlider_y->setValue(int(valor));

        double x = ui.lineEdit_X->text().toDouble();
        double y = ui.lineEdit_Y->text().toDouble();
        double z = ui.lineEdit_Z->text().toDouble();

        rn.set_translacao(x, y, z);
    }
}
void MainWindow::on_lineEdit_Z_returnPressed(){
    double valor = ui.lineEdit_Z->text().toDouble();
    if(valor >= ui.horizontalSlider_z->minimum() && valor <= ui.horizontalSlider_z->maximum()){
        ui.horizontalSlider_z->setValue(int(valor));

        double x = ui.lineEdit_X->text().toDouble();
        double y = ui.lineEdit_Y->text().toDouble();
        double z = ui.lineEdit_Z->text().toDouble();

        rn.set_translacao(x, y, z);
    }
}

/// Refinando o valor a partir dos linedits de rotacao
void MainWindow::on_lineEdit_rotacaox_returnPressed(){
    double valor = ui.lineEdit_rotacaox->text().toDouble();
    if(valor >= ui.dial_x->minimum() && valor <= ui.dial_x->maximum()){
        ui.dial_x->setValue(int(valor));

        double x = ui.lineEdit_rotacaox->text().toDouble();
        double y = ui.lineEdit_rotacaoy->text().toDouble();
        double z = ui.lineEdit_rotacaoz->text().toDouble();

        rn.set_rotacao(x, y, z);
    }
}
void MainWindow::on_lineEdit_rotacaoy_returnPressed(){
    double valor = ui.lineEdit_rotacaoy->text().toDouble();
    if(valor >= ui.dial_y->minimum() && valor <= ui.dial_y->maximum()){
        ui.dial_y->setValue(int(valor));

        double x = ui.lineEdit_rotacaox->text().toDouble();
        double y = ui.lineEdit_rotacaoy->text().toDouble();
        double z = ui.lineEdit_rotacaoz->text().toDouble();

        rn.set_rotacao(x, y, z);
    }
}
void MainWindow::on_lineEdit_rotacaoz_returnPressed(){
    double valor = ui.lineEdit_rotacaoz->text().toDouble();
    if(valor >= ui.dial_z->minimum() && valor <= ui.dial_z->maximum()){
        ui.dial_z->setValue(int(valor));

        double x = ui.lineEdit_rotacaox->text().toDouble();
        double y = ui.lineEdit_rotacaoy->text().toDouble();
        double z = ui.lineEdit_rotacaoz->text().toDouble();

        rn.set_rotacao(x, y, z);
    }
}

/// Botao para registrar as nuvens
void MainWindow::on_pushButton_registrar_clicked(){
    if(ui.checkBox_icp->isChecked())
        rn.set_profundidade_icp(ui.lineEdit_profundidadeicp->text().toDouble()/100.0);
    rn.registrar_nuvens(ui.checkBox_icp->isChecked());

    float x, y, z, rx, ry, rz;
    rn.get_TFinal(x, y, z, rx, ry, rz);

    ui.lineEdit_X->setText(QString::number(x));
    ui.lineEdit_Y->setText(QString::number(y));
    ui.lineEdit_Z->setText(QString::number(z));
    ui.lineEdit_rotacaox->setText(QString::number(rx));
    ui.lineEdit_rotacaoy->setText(QString::number(ry));
    ui.lineEdit_rotacaoz->setText(QString::number(rz));

    ui.dial_x->setValue(int(rx));
    ui.dial_y->setValue(int(ry));
    ui.dial_z->setValue(int(rz));
    ui.horizontalSlider_x->setValue(int(x));
    ui.horizontalSlider_y->setValue(int(y));
    ui.horizontalSlider_z->setValue(int(z));
}

/// Botao para salvar ler os arquivos nvm, criar os objetos de cameras e escrever o novo arquivo no lugar certo
void MainWindow::on_pushButton_salvarfinal_clicked(){
    rn.salvar_dados_finais(ui.lineEdit_pastafinal->text());
}

/// --------------------------------- ABA 3 ------------------------------------------- ///

/// Botao para carregar arquivo da nuvem que sera filtrada
void MainWindow::on_pushButton_nuvemacorrigir_clicked(){
    QString nome_alvo;
    nome_alvo = QFileDialog::getOpenFileName(this, "Nuvem a Filtrar", "", "PLY Files (*.ply)");
    ui.lineEdit_nuvemparafiltrar->setText(nome_alvo);

    rn.set_nuvem_filtrar(nome_alvo);
    // Habilitar o resto da aba
    ui.groupBox_filtrosforma->setEnabled(true);
    ui.frame_salvar->setEnabled(true);
    ui.pushButton_resetafiltro->setEnabled(true);
    ui.groupBox_polinomio->setEnabled(true);
}

/// Botao para visulizar enquanto a filtragem ocorre
void MainWindow::on_pushButton_visualizarcorrecao_clicked(){
    system("gnome-terminal -x sh -c 'rosrun rviz rviz -d $HOME/laser_ws/src/scanner-laser/scanner_gui/resources/filtrando.rviz'");
}

/// Botao para aplicar o filtro de voxel
void MainWindow::on_pushButton_voxel_clicked(){
    float voxel = ui.lineEdit_voxel->text().toFloat(); // aqui em centimetros, tem que mandar em metros
    ROS_INFO("Tamanho do voxel: %.4f cm", voxel);
    if(voxel != 0)
        rn.set_new_voxel(voxel/100.0);
}

/// Botao para aplicar filtros de outliers
void MainWindow::on_pushButton_outliers_clicked(){
    float mean = ui.lineEdit_outliersmean->text().toFloat();
    float dev  = ui.lineEdit_outliersstd->text().toFloat();
    ROS_INFO("Media: %.4f    Desvio: %.4f", mean, dev);
    if(mean != 0 && dev != 0)
        rn.set_new_outlier(mean, dev);
}

/// Botao para salvar a nuvem final na mesma pasta que a original
void MainWindow::on_pushButton_salvarnuvemfiltrada_clicked(){
    rn.salvar_nuvem_filtrada(ui.lineEdit_nuvemfiltradasalvar->text());
}

// Resetar os filtros aplicados
void MainWindow::on_pushButton_resetafiltro_clicked(){
    rn.reseta_filtros();
}

// Aplicar suavizacao polinomial na nuvem
void MainWindow::on_pushButton_filtropolinomio_clicked(){
  rn.aplica_filtro_polinomio(ui.lineEdit_graupolinomio->text().toInt(), ui.lineEdit_raiopolinomio->text().toFloat());
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

