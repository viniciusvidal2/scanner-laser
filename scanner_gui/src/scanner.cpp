#include "../include/scanner_gui/scanner.hpp"
#include <QTime>
#include <iostream>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <sstream>
#include <string>
#include <QStringListModel>
#include <QFuture>
#include <QtConcurrentRun>

#include <boost/lexical_cast.hpp>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>


namespace scanner_gui{
///////////////////////////////////////////////////////////////////////////////////////////
Scanner::Scanner(int argc, char **argv, QMutex *nmutex):init_argc(argc),
    init_argv(argv),mutex(nmutex)
{
    QFuture<void> future = QtConcurrent::run(this, &Scanner::init);

}
///////////////////////////////////////////////////////////////////////////////////////////
Scanner::~Scanner(){
    dynamixel_workbench_msgs::JointCommand central;
    central.request.pan_pos  = 60; // proximo do inicio do curso
    central.request.tilt_pos = 0;
    central.request.unit     = "raw";
    if(comando_motor.call(central))
        ROS_WARN("Centralizando o robo...");
    sleep(5);
    if(ros::isStarted()){
        ros::shutdown();
        ros::waitForShutdown();
    }
    wait();
}
///////////////////////////////////////////////////////////////////////////////////////////
void Scanner::init(){

    ros::init(init_argc, init_argv, "Scanner");
    if ( ! ros::master::check() )  {
        cout << "check ros master not good" << endl;
        return;
    }
    ros::start();
    ros::NodeHandle nh_;

    // FOV da ASTRA segundo fabricante no sentido PAN
    FOV_astra = 60;
    overlap = 0; // Overlap entre fotos [DEGREES]

    // Inicia variaveis do motor
    raw_min = 50; raw_max = 3988;
    deg_min =  4; deg_max =  350;
    raw_atual = 0; // Seguranca
    deg_raw = (deg_max - deg_min) / (raw_max - raw_min); raw_deg = 1.0 / deg_raw;
    dentro = 2;
    //    inicio_curso = raw_min; fim_curso = raw_max;
    this->set_course(deg_min, deg_max);

    viagens = 1; // Comecando default valor de viagens total
    viagem_atual = 0; // Qual viagem esta sendo realizada

    // Comecar a aquisicao
    comecar = false;

    // Capturar dados da camera
    capturar_camera = 0;

    // Aloca o ponteiro da nuvem acumulada
    acc = (PointCloud<PointXYZ>::Ptr) new PointCloud<PointXYZ>();
    acc->header.frame_id = "map";
    acc_cor = (PointCloud<PointXYZRGB>::Ptr) new PointCloud<PointXYZRGB>();
    acc_cor->header.frame_id = "map";

    // Matriz intrinseca da astra, rotação de eixos entre laser e astra e deslocamento no novo eixo
    Eigen::Matrix3f K;
    K << 525.1389,    1.4908,  324.1741,
                0,  521.6805,  244.8827,
                0,         0,    1.0000;
    Eigen::Matrix3f matrix;
    matrix = Eigen::AngleAxisf(-M_PI/2, Eigen::Vector3f::UnitZ()) *
             Eigen::AngleAxisf(-M_PI/2, Eigen::Vector3f::UnitX()) *
             Eigen::AngleAxisf(-M_PI/2, Eigen::Vector3f::UnitZ());
    Eigen::Matrix4f T_eixos = Eigen::Matrix4f::Identity();
    T_eixos.block<3,3>(0, 0) << matrix;
    T_eixos.block<3,1>(0, 3) << 0.004, 0.105, 0; // [m]

    // Objeto de trabalho e salvamento das nuvens
    saw = new SaveAndWork();

    // Inicia o subscriber sincronizado para as mensagens de laser e motor
    message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub(nh_, "/scan"                           , 100);
    message_filters::Subscriber<nav_msgs::Odometry>     motor_sub(nh_, "/dynamixel_angulos_sincronizados", 100);
    sync.reset(new Sync(syncPolicy(100), laser_sub, motor_sub));
    conexao_filter = sync->registerCallback(boost::bind(&Scanner::callback, this, _1, _2));

    // Inicia o subscriber sincronizado para motor e dados da astra - ja pega la o motor_sub
    message_filters::Subscriber<nav_msgs::Odometry      > mot_sub(nh_, "/dynamixel_angulos_sincronizados", 100);
    message_filters::Subscriber<sensor_msgs::Image      > img_sub(nh_, "/astra2"         , 100);
    message_filters::Subscriber<sensor_msgs::PointCloud2> ptc_sub(nh_, "/astra_projetada", 100);
    message_filters::Subscriber<sensor_msgs::PointCloud2> pix_sub(nh_, "/pixels"         , 100);
    sync2.reset(new Sync2(syncPolicy2(100), mot_sub, img_sub, ptc_sub, pix_sub));
    conexao_filter2 = sync2->registerCallback(boost::bind(&Scanner::callback2, this, _1, _2, _3, _4));

    // Inicio do publicador da nuvem acumulada e mensagem ros
    ros::Publisher pub_acc = nh_.advertise<sensor_msgs::PointCloud2>("/laser_acumulada", 100);
    sensor_msgs::PointCloud2 msg_acc;
    msg_acc.header.frame_id = "map";

    // Aqui o publicador de tf2 e inicio da mensagem
    tf2_ros::TransformBroadcaster broadcaster;
    tf_msg.header.frame_id = "map";
    tf_msg.child_frame_id = "laser";
    tf_msg.transform.translation.x = 0.0148;
    tf_msg.transform.translation.y = 0;
    tf_msg.transform.translation.z = 0;
    tf_msg.transform.rotation.x = 0;
    tf_msg.transform.rotation.y = 0;
    tf_msg.transform.rotation.z = 0;
    tf_msg.transform.rotation.w = 1;

    // Inicia o serviço - variavel global, para usar dentro do callback
    comando_motor = nh_.serviceClient<dynamixel_workbench_msgs::JointCommand>("/joint_command");

    // Rodar o no
    ros::Rate rate(2);
    while(ros::ok()){

        // Publicar a nuvem, no final quando a nuvem ficar colorida vai publicar essa nuvem para o usuario
        if(acc->size() >= acc_cor->size()){
            toROSMsg(*acc, msg_acc);
        } else {
            toROSMsg(*acc_cor, msg_acc);
        }
        msg_acc.header.stamp = ros::Time::now();
        pub_acc.publish(msg_acc);
        // Publicar a tf
        broadcaster.sendTransform(tf_msg);
        // Rodar o ros - MUITO IMPORTANTE
        ros::spinOnce();
        // Dormir
        rate.sleep();

    }

}
///////////////////////////////////////////////////////////////////////////////////////////
void Scanner::set_course(double min, double max){
    inicio_curso = deg2raw(min); fim_curso = deg2raw(max);

    /// Preencher os vetores conforme angulos de captura, inicio e fim das nuvens
    angulos_captura.clear(); inicio_nuvens.clear(); final_nuvens.clear();

    if(max - min < FOV_astra - overlap){ // Se so cabe uma captura por causa do range pequeno
        angulos_captura.push_back((min + max)/2);
        inicio_nuvens.push_back(min);
        final_nuvens.push_back(max);
    } else { // Calcular o espaçamento entre as capturas
        float ac = min + FOV_astra/2; // - overlap;
        float in = min, fn = min + FOV_astra - overlap;
        ac = (ac >= fn) ? (in + fn)/2 : ac;

        while(ac < max){ // Equanto nao varremos todo o range com o angulo central de captura
            angulos_captura.push_back(ac); inicio_nuvens.push_back(in); final_nuvens.push_back(fn);
            ac += FOV_astra - overlap;
            in  = (ac - FOV_astra/2 < min) ? min : ac - FOV_astra/2;
            fn  = (ac + FOV_astra/2 > max) ? max : ac + FOV_astra/2;
            ac  = (ac >= fn) ? (in + fn)/2 : ac;
        }
    }

    nuvens_parciais.clear() ; nuvens_parciais.resize(angulos_captura.size());
    imagens_parciais.clear(); imagens_parciais.resize(angulos_captura.size());
}
///////////////////////////////////////////////////////////////////////////////////////////
void Scanner::set_overlap(float o_pct){
    overlap = FOV_astra * o_pct/100;
    // Ajusta os intervalos la entre as nuvens nessa funçao de uma vez tambem
    set_course(raw2deg(inicio_curso), raw2deg(fim_curso));
}
///////////////////////////////////////////////////////////////////////////////////////////
void Scanner::set_trips(int t){
    viagens = t; viagem_atual = 1;
}
///////////////////////////////////////////////////////////////////////////////////////////
void Scanner::set_acquisition(bool start){
    comecar = start;
}
///////////////////////////////////////////////////////////////////////////////////////////
bool Scanner::get_acquisition(){
    return comecar;
}
///////////////////////////////////////////////////////////////////////////////////////////
bool Scanner::begin_reached(int &r){
    if(abs(int(raw_atual - inicio_curso)) > dentro){
        r = int(raw2deg(raw_atual));
        comecar = false;
        return false;
    } else {
        return true;
    }
}
///////////////////////////////////////////////////////////////////////////////////////////
bool Scanner::save_data(){
    if(acc->size() > 10){
        // Salvar a nuvem final de forma correta
        saw->process_color_and_save(imagens_parciais, nuvens_parciais, angulos_captura, acc, acc_cor);
        // Salvar o arquivo final de angulos para pos processamento
        saw->save_angles_file(inicio_nuvens, final_nuvens, angulos_captura);
    } else {
        ROS_WARN("Nao tem nuvem ainda seu imbecil !");
        return false;
    }
}
///////////////////////////////////////////////////////////////////////////////////////////
void Scanner::get_limits(int &minm, int &maxm){
    minm = int(raw2deg(inicio_curso)); maxm = int(raw2deg(fim_curso));
}
///////////////////////////////////////////////////////////////////////////////////////////
int Scanner::get_current_position(){
    return int(raw2deg(raw_atual));
}
///////////////////////////////////////////////////////////////////////////////////////////
int Scanner::get_current_trip(){
    return viagem_atual;
}
///////////////////////////////////////////////////////////////////////////////////////////
double Scanner::deg2raw(double deg){
    return (deg - deg_min)*raw_deg + raw_min;
}
///////////////////////////////////////////////////////////////////////////////////////////
double Scanner::raw2deg(double raw){
    return (raw - raw_min)*deg_raw + deg_min;
}
///////////////////////////////////////////////////////////////////////////////////////////
void Scanner::send_begin_course(){
    dynamixel_workbench_msgs::JointCommand inicio_curso_cmd;
    inicio_curso_cmd.request.pan_pos  = inicio_curso;
    inicio_curso_cmd.request.tilt_pos = 0;
    inicio_curso_cmd.request.unit     = "raw";
    if(comando_motor.call(inicio_curso_cmd))
        ROS_INFO("Enviamos ao inicio de curso, aguarde....");
    comecar = false; // Enquanto nao chegar nao aquisitamos
}
///////////////////////////////////////////////////////////////////////////////////////////
void Scanner::start_course(){
    comecar = true; // Podemos aquisitar
    viagem_atual = 0; // Garantir que estamos na primeira viagem ainda
}
///////////////////////////////////////////////////////////////////////////////////////////
void Scanner::send_to_opposite_edge(int t){
    dynamixel_workbench_msgs::JointCommand cmd;
    if(abs(remainder(t, 2)) == 1){ // Estamos indo ao fim de curso
        cmd.request.pan_pos  = fim_curso; // Primeiro ponta pe
        cmd.request.tilt_pos = 0;
        cmd.request.unit     = "raw";
        if(comando_motor.call(cmd))
            ROS_WARN("Mandamos para o fim de curso %d, viagem # %d !", int(fim_curso), viagem_atual);
    } else { // Estamos voltando ao inicio
        cmd.request.pan_pos  = inicio_curso; // Primeiro ponta pe
        cmd.request.tilt_pos = 0;
        cmd.request.unit     = "raw";
        if(comando_motor.call(cmd))
            ROS_WARN("Mandamos para o inicio de curso %d, viagem # %d !", int(inicio_curso), viagem_atual);
    }
}
///////////////////////////////////////////////////////////////////////////////////////////
void Scanner::accumulate_parcial_cloud(PointCloud<PointXYZ>::Ptr cloud, double ang_raw){
    double theta_y = raw2deg(ang_raw - raw_ref);

    PointCloud<PointXYZ>::Ptr temp (new PointCloud<PointXYZ>());
    for(size_t i=0; i < angulos_captura.size(); i++){
        if(theta_y >= inicio_nuvens[i] && theta_y <= final_nuvens[i]){
            *temp = nuvens_parciais[i];
            *temp += *cloud;
            nuvens_parciais[i] = *temp;
            ROS_INFO("Adicionamos nuvem %zu, limites %.1f e %.1f, com tamanho agora de %zu.", i, inicio_nuvens[i], final_nuvens[i], nuvens_parciais[i].size());
        }
    }
}
///////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix4f Scanner::transformFromRaw(double raw){
    // Valor em graus que girou, tirando a referencia, convertido para frame global ODOM:
    // theta_y = degrees (a favor da mao direita no rviz)
    double theta_y = raw2deg(raw - raw_ref);

    // Constroi matriz de rotacao
    Eigen::Matrix3f R;
    R = Eigen::AngleAxisf(               0, Eigen::Vector3f::UnitX()) *
            Eigen::AngleAxisf(DEG2RAD(theta_y), Eigen::Vector3f::UnitY()) *
            Eigen::AngleAxisf(               0, Eigen::Vector3f::UnitZ());
    // Constroi matriz homgenea e retorna
    Eigen::MatrixXf t(3, 1);
    t << 0.0148,
            0,
            0;
    Eigen::Matrix4f T;
    T << R, R*t,
            0, 0, 0, 1;

    // Prepara a mensagem atual de tf2 para ser transmitida e enviar
    tf_msg.header.stamp = ros::Time::now();
    tf_msg.header.frame_id = "map";
    tf_msg.child_frame_id = "laser";
    tf_msg.transform.translation.x = T(0,3);
    tf_msg.transform.translation.y = T(1,3);
    tf_msg.transform.translation.z = T(2,3);
    q.setRPY(0, DEG2RAD(theta_y), 0);
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();

    return T;
}
///////////////////////////////////////////////////////////////////////////////////////////
void Scanner::callback(const sensor_msgs::LaserScanConstPtr &msg_laser, const nav_msgs::OdometryConstPtr &msg_motor){
    // Nocao global da posicao raw atual do motor
    raw_atual = msg_motor->pose.pose.position.x;

    /// DAQUI PRA FRENTE VAI VALER ///
    if(comecar){

        // Controle para enviar comandos de acordo com a viagem
        if(viagem_atual <= viagens){
            if(abs(int(raw_atual - inicio_curso)) < dentro && abs(remainder(viagem_atual, 2)) == 0){
                viagem_atual++;
                if(viagem_atual <= viagens)
                    send_to_opposite_edge(viagem_atual);
            }
            if(abs(int(raw_atual - fim_curso   )) < dentro && abs(remainder(viagem_atual, 2)) == 1){
                viagem_atual++;
                if(viagem_atual <= viagens)
                    send_to_opposite_edge(viagem_atual);

            }
        } else { // Finalizar processo se chegar ao fim do curso
            ROS_INFO("Chegou ao fim da aquisicao, salvando todos os dados ....");
            this->save_data();
            ROS_INFO("Nuvem salva, conferir la na boa.");
            // Negando a flag novamente
            comecar = false;
        }

        // Começando, converter leitura para nuvem PCL
        sensor_msgs::PointCloud2 msg_cloud;
        projector.projectLaser(*msg_laser, msg_cloud);
        PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>());
        fromROSMsg(msg_cloud, *cloud);
        // Aplicar transformada de acordo com o angulo
        Eigen::Matrix4f T = transformFromRaw(msg_motor->pose.pose.position.x);
        transformPointCloud(*cloud, *cloud, T);
        // Acumular nuvem global - vista no rviz
        *acc += *cloud;
        // Acumular nuvem parcial correta
        accumulate_parcial_cloud(cloud, msg_motor->pose.pose.position.x);
        // Falar o ponto atual para a progressBar
        new_step();

    }
}
///////////////////////////////////////////////////////////////////////////////////////////
void Scanner::callback2(const nav_msgs::OdometryConstPtr& msg_motor,
                        const sensor_msgs::ImageConstPtr& msg_imagem,
                        const sensor_msgs::PointCloud2ConstPtr& msg_nuvem,
                        const sensor_msgs::PointCloud2ConstPtr& msg_pixels){
    // Salvar dados no caso de estarmos na primeira viagem
    if(viagem_atual == 1){

        // Se estiver proximo a algum dos angulos de captura ligamos
        for(size_t i=0; i < angulos_captura.size(); i++){
            if((abs(raw2deg(msg_motor->pose.pose.position.x) - angulos_captura[i]) < dentro) && capturar_camera == 0){
                // Absorver imagem
                cv_bridge::CvImagePtr imptr;
                imptr = cv_bridge::toCvCopy(msg_imagem, sensor_msgs::image_encodings::BGR8);
                imagens_parciais[i] = imptr->image;

                // Absorver nuvens
                PointCloud<PointXYZRGB>::Ptr nuvem_astra  (new PointCloud<PointXYZRGB>());
                PointCloud<PointXYZ>::Ptr    nuvem_pixels (new PointCloud<PointXYZ>()   );
                fromROSMsg(*msg_nuvem , *nuvem_astra );
                fromROSMsg(*msg_pixels, *nuvem_pixels);

                // Salvar imagens e nuvens
                saw->save_image_and_clouds_partial(imptr->image, nuvem_astra, nuvem_pixels, i);

                // Acerta o contador de dados capturados para nao repetir
                capturar_camera++;
            } else {
                capturar_camera = 0;
            }
        }

    }

}
///////////////////////////////////////////////////////////////////////////////////////////
} // Fim do namespace
