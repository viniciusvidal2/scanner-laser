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
    dynamixel_workbench_msgs::JointCommand deitar;
    deitar.request.pan_pos  = 0;
    deitar.request.tilt_pos = 1965;
    deitar.request.unit     = "raw";
    if(comando_motor.call(deitar))
        ROS_WARN("Deitando o robo...");
    sleep(3);
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

    // Inicia variaveis do motor
    raw_min = 152; raw_max = 3967;
    deg_min =  13; deg_max =  348;
    raw_tilt_hor = 2100;
    deg_raw = (deg_max - deg_min) / (raw_max - raw_min); raw_deg = 1.0 / deg_raw;
    dentro = 2;
    inicio_curso = raw_min; fim_curso = raw_max;

    intervalo = 3; // Passo do motor para cada aquisicao
    ponto_final_temp = inicio_curso; // A cada iteracao vai incrementar aqui para novo ponto final

    // Comecar a aquisicao
    comecar = false;

    // Aloca o ponteiro da nuvem acumulada
    acc = (PointCloud<PointXYZ>::Ptr) new PointCloud<PointXYZ>();
    acc->header.frame_id = "map";

    // Inicia o subscriber sincronizado para as mensagens
    message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub(nh_, "/scan"                           , 100);
    message_filters::Subscriber<nav_msgs::Odometry>     motor_sub(nh_, "/dynamixel_angulos_sincronizados", 100);
    sync.reset(new Sync(syncPolicy(100), laser_sub, motor_sub));
    conexao_filter =  sync->registerCallback(boost::bind(&Scanner::callback, this, _1, _2));

    // Inicio do publicador da nuvem acumulada e mensagem ros
    ros::Publisher pub_acc = nh_.advertise<sensor_msgs::PointCloud2>("/laser_acumulada", 100);
    sensor_msgs::PointCloud2 msg_acc;
    msg_acc.header.frame_id = "map";

    ROS_INFO("Ligamos subscribers e publishers.");

    // Aqui o publicador de tf2 e inicio da mensage
    tf2_ros::TransformBroadcaster broadcaster;
    tf_msg.header.frame_id = "map";
    tf_msg.child_frame_id = "laser";
    tf_msg.transform.translation.x = 0;
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

        // Publicar a nuvem
        toROSMsg(*acc, msg_acc);
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
    ponto_final_temp = inicio_curso;
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
    if(abs(raw_atual - inicio_curso) > dentro){
        r = int(raw2deg(raw_atual));
        comecar = false;
        return false;
    } else {
        return true;
    }
}
///////////////////////////////////////////////////////////////////////////////////////////
bool Scanner::save_cloud(){
    // Ver o tempo para diferenciar bags gravadas automaticamente
    time_t t = time(0);
    struct tm * now = localtime( & t );
    std::string year, month, day, hour, minutes, home;
    char const* tmp = getenv("HOME");
    if(tmp)
        home = std::string(tmp);
    //    year    = boost::lexical_cast<std::string>(now->tm_year + 1900);
    //    month   = boost::lexical_cast<std::string>(now->tm_mon + 1);
    //    day     = boost::lexical_cast<std::string>(now->tm_mday);
    hour    = boost::lexical_cast<std::string>(now->tm_hour);
    minutes = boost::lexical_cast<std::string>(now->tm_min );
    std::string filename = home + "/Desktop/laser_" + hour + "h_" + minutes + "m.ply";

    // Checar se tudo certo para salvar a nuvem
    if(acc->size() > 10){
        if(pcl::io::savePLYFileASCII(filename, *acc))
            return true;
        else
            return false;
    } else {
        ROS_WARN("Nao tem nuvem ainda seu imbecil !");
    }
}
///////////////////////////////////////////////////////////////////////////////////////////
void Scanner::get_limits(int &minm, int &maxm){
    minm = int(deg_min); maxm = int(deg_max);
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
    inicio_curso_cmd.request.tilt_pos = raw_tilt_hor;
    inicio_curso_cmd.request.unit     = "raw";
    if(comando_motor.call(inicio_curso_cmd))
        ROS_INFO("Enviamos ao inicio de curso, aguarde....");
    comecar = false; // Enquanto nao chegar nao aquisitamos
}
///////////////////////////////////////////////////////////////////////////////////////////
void Scanner::start_course(){
    dynamixel_workbench_msgs::JointCommand comecar_cmd;
    comecar_cmd.request.pan_pos  = ponto_final_temp + intervalo; // Primeiro ponta pe
    comecar_cmd.request.tilt_pos = raw_tilt_hor;
    comecar_cmd.request.unit     = "raw";
    if(comando_motor.call(comecar_cmd))
        ROS_WARN("Mandamos comecar a aquisicao !");
    comecar = true; // Podemos aquisitar
}
///////////////////////////////////////////////////////////////////////////////////////////
void Scanner::send_next_point(int end_point){
    dynamixel_workbench_msgs::JointCommand avancar;
    avancar.request.pan_pos  = end_point;
    avancar.request.tilt_pos = raw_tilt_hor;
    avancar.request.unit     = "raw";
    if(comando_motor.call(avancar))
        ROS_WARN("Mandamos para o ponto %d !", end_point);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix4f Scanner::transformFromRaw(double raw){
    // Valor em graus que girou, tirando a referencia, convertido para frame global ODOM:
    // theta_y = -degrees (contrario a mao direita)
    double theta_y = -raw2deg(raw - raw_ref);
    // Prepara a mensagem atual de tf2 para ser transmitida e enviar
    tf_msg.header.stamp = ros::Time::now();
    tf_msg.header.frame_id = "map";
    tf_msg.child_frame_id = "laser";
    tf_msg.transform.translation.x = 0;
    tf_msg.transform.translation.y = 0;
    tf_msg.transform.translation.z = 0;
    q.setRPY(0, DEG2RAD(theta_y), 0);
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();
    // Constroi matriz de rotacao
    Eigen::Matrix3f R;
    R = Eigen::AngleAxisf(               0, Eigen::Vector3f::UnitX()) *
        Eigen::AngleAxisf(DEG2RAD(theta_y), Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(               0, Eigen::Vector3f::UnitZ());
    // Constroi matriz homgenea e retorna
    Eigen::Vector3f t;
    t << 0, 0, 0;
    Eigen::Matrix4f T;
    T << R, t,
         0, 0, 0, 1;

    return T;
}
///////////////////////////////////////////////////////////////////////////////////////////
void Scanner::callback(const sensor_msgs::LaserScanConstPtr &msg_laser, const nav_msgs::OdometryConstPtr &msg_motor){
    // Nocao global da posicao raw atual do motor
    raw_atual = msg_motor->pose.pose.position.x;

    /// DAQUI PRA FRENTE VAI VALER ///
    if(comecar){

        if(abs(raw_atual - ponto_final_temp) < dentro){
            ROS_INFO("Aquisitando ....");
            // Começando, converter leitura para nuvem PCL
            sensor_msgs::PointCloud2 msg_cloud;
            projector.projectLaser(*msg_laser, msg_cloud);
            PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>());
            fromROSMsg(msg_cloud, *cloud);
            // Aplicar transformada de acordo com o angulo
            Eigen::Matrix4f T = transformFromRaw(msg_motor->pose.pose.position.x);
            transformPointCloud(*cloud, *cloud, T);
            // Acumular nuvem
            *acc += *cloud;
            // Setar novo ponto final da aquisicao
            ponto_final_temp += intervalo;
            send_next_point(ponto_final_temp);

            // Finalizar processo se chegar ao fim do curso
            if(abs(raw_atual - fim_curso) < dentro){
                ROS_INFO("Chegou no fim de curso, salvando nuvem na area de trabalho....");
                this->save_cloud();
                ROS_INFO("Nuvem salva, conferir la na boa.");
                // Negando a flag novamente
                comecar = false;
                // Resetando o ponto final
                ponto_final_temp = inicio_curso;
            }// fim if fim de curso

        }
//        } else {
//            if(permitido_avancar){
//                send_next_point(ponto_final_temp);
//                permitido_avancar = false;
//            }
//        }

    }
}
///////////////////////////////////////////////////////////////////////////////////////////
} // Fim do namespace
