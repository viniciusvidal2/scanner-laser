#include "../include/scanner_gui/saveandwork.hpp"
#include <QStringListModel>
#include <QFuture>
#include <QtConcurrentRun>

///////////////////////////////////////////////////////////////////////////////////////////
SaveAndWork::SaveAndWork(Eigen::Matrix3f K_, Eigen::Matrix4f Tla, std::string p):
    K_astra(K_), T_laser_astra(Tla), pasta(p)
{
    QFuture<void> future = QtConcurrent::run(this, &SaveAndWork::init);
}
///////////////////////////////////////////////////////////////////////////////////////////
SaveAndWork::~SaveAndWork(){
    if(ros::isStarted()){
        ros::shutdown();
        ros::waitForShutdown();
    }
    wait();
}
///////////////////////////////////////////////////////////////////////////////////////////
void SaveAndWork::init(){

    // Criando a pasta
    bool criado = mkdir(pasta.c_str(), 0777);
    if(!criado)
        ROS_INFO("Diretorio para salvar dados criado.");
    else
        ROS_ERROR("NAO FOI POSSIVEL CRIAR O DIRETORIO");

}
///////////////////////////////////////////////////////////////////////////////////////////
void SaveAndWork::save_image_and_clouds_partial(cv::Mat imagem, cv::Mat imzed, PointCloud<PointC>::Ptr nuvem_astra, PointCloud<PointXYZ>::Ptr nuvem_pixels, size_t indice){
    // Salvar a imagem na pasta certa
    std::string nome_imagem = "im_astra_"+std::to_string(int(indice))+".png";
    std::vector<int> opcoes;
    opcoes.push_back(cv::IMWRITE_PNG_COMPRESSION);
    cv::imwrite(pasta+nome_imagem, imagem, opcoes);

    // Salvar a imagem da ZED na pasta certa
    std::string nome_zed = "im_zed_"+std::to_string(int(indice))+".png";
    cv::imwrite(pasta+nome_zed, imzed, opcoes);

    // Salvar a nuvem na pasta certa da astra
    std::string nome_nuvem_ast = "nv_astra_"+std::to_string(int(indice))+".ply";
    savePLYFileASCII(pasta+nome_nuvem_ast, *nuvem_astra);

    // Salvar a nuvem de pixels na pasta certa
    std::string nome_nuvem_pix = "nv_pixel_"+std::to_string(int(indice))+".ply";
    savePLYFileASCII(pasta+nome_nuvem_pix, *nuvem_pixels);
}
///////////////////////////////////////////////////////////////////////////////////////////
void SaveAndWork::process_color_and_save(std::vector<cv::Mat> imagens, std::vector<PointCloud<PointC>> nuvens, std::vector<float> angulos, PointCloud<PointC>::Ptr acumulada_colorida){
    /// --- Projetar cada nuvem parcial na foto correspondente e colorir todo mundo ali para o resultado final --- ///
    // Cria vetor de nuvens coloridas parciais
    std::vector<PointCloud<PointC>> nuvens_coloridas(nuvens.size());
    // Para cada nuvem
    PointCloud<PointC>::Ptr color (new PointCloud<PointC>());
    #pragma omp parallel for
    for(size_t i=0; i < nuvens_coloridas.size(); i++){
        // Obter transformaçao entre camera e laser
        Eigen::Matrix4f T = this->calculate_transformation(angulos[i]);
        // Projetar e obter nuvem com cor
        *color = nuvens[i];
        this->project_cloud_to_image(color, imagens[i], T);
        nuvens[i] = *color;
    }

    // Alterar total a nuvem que se ve no programa principal para nuvem colorida sem normais
    acumulada_colorida->clear();
    for(size_t i=0; i < nuvens.size(); i++){
        *acumulada_colorida += nuvens[i];
    }

    // Calcular normais de cada nuvem e salvar
    // Criar nuvem final somando as parciais novamente
    PointCloud<PointT>::Ptr final (new PointCloud<PointT>());
    PointCloud<PointT>::Ptr temp  (new PointCloud<PointT>());
    #pragma omp parallel for
    for(size_t i=0; i < nuvens.size(); i++){
        calculate_normals(nuvens[i], temp);
        *final += *temp;
        savePLYFileASCII(pasta+"parcial_"+std::to_string(i+1)+".ply", *temp);
    }

    // Salvar a nuvem acumulada colorida com normais
    this->process_and_save_final_cloud(final);
}
///////////////////////////////////////////////////////////////////////////////////////////
void SaveAndWork::save_angles_file(std::vector<float> in, std::vector<float> fn, std::vector<float> ac){
    // Abrir arquivo
    std::string nome_arquivo = pasta+"angulos.txt";
    std::string linha;
    ofstream arquivo(nome_arquivo);
    if(arquivo.is_open()){
        // Escrever linha a linha
        for(size_t i=0; i < in.size(); i++){
            // ANGULO_INICIO ANGULO_CAPTURA ANGULO_FINAL
            linha = std::to_string(in[i]) + " " + std::to_string(ac[i]) + " " + std::to_string(fn[i]) + "\n";
            std::replace(linha.begin(), linha.end(), ',', '.');
            arquivo << linha;
        }
    }
    arquivo.close();

    // Salvar arquivos NVM
    std::string nome_nvm;
    std::string linha2;
    for(size_t i=0; i<ac.size(); i++){
        // Nome do arquivo nvm atual
        nome_nvm = pasta+"im_astra_"+std::to_string(i)+".nvm";
        // Calcular transformacao a partir do angulo e transformar em quaternion
        Eigen::Matrix4f T_temp;
        T_temp = calculate_transformation(ac[i]);
        Eigen::Matrix3f rot;
        rot << T_temp.block<3,3>(0, 0);
        Eigen::Quaternion<float> q(T_temp.block<3,3>(0, 0));
        // Formar a linha
        linha2 = this->write_line_for_nvm(550, nome_nvm, q);
        // Escrever no arquivo com nome correspondente ao da fracao de nuvem para imagem astra
        ofstream arquivo_astra_nvm(nome_nvm);
        if(arquivo_astra_nvm.is_open()){
            arquivo_astra_nvm << "NVM_V3\n\n";
            arquivo_astra_nvm << "1\n"; // Quantas imagens, sempre uma aqui
            arquivo_astra_nvm << linha2;
        }
        arquivo_astra_nvm.close();
        // Nome do arquivo nvm atual
        nome_nvm = pasta+"im_zed_"+std::to_string(i)+".nvm";
        // Formar a linha
        linha2 = this->write_line_for_nvm(1460, nome_nvm, q);
        // Escrever no arquivo com nome correspondente ao da fracao de nuvem para imagem astra
        ofstream arquivo_zed_nvm(nome_nvm);
        if(arquivo_zed_nvm.is_open()){
            arquivo_zed_nvm << "NVM_V3\n\n";
            arquivo_zed_nvm << "1\n"; // Quantas imagens, sempre uma aqui
            arquivo_zed_nvm << linha2;
        }
        arquivo_zed_nvm.close();
    }
}
///////////////////////////////////////////////////////////////////////////////////////////
void SaveAndWork::process_and_save_final_cloud(PointCloud<PointT>::Ptr entrada){
    // Limpar outliers aqui de uma vez
    pcl::RadiusOutlierRemoval<PointT> out;
    out.setRadiusSearch(0.1);
    out.setMinNeighborsInRadius(10);
    out.setInputCloud(entrada);
    out.filter(*entrada);

    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(entrada);
    sor.setMeanK(1);
    sor.setStddevMulThresh(1);
    sor.filter(*entrada);

    // Salvar a nuvem final com cor e normais certas
    std::string nome_nuvem = "nuvem_final.ply";
    savePLYFileASCII(pasta+nome_nuvem, *entrada);
}
///////////////////////////////////////////////////////////////////////////////////////////
void SaveAndWork::calculate_normals(PointCloud<PointC> entrada, PointCloud<PointT>::Ptr acc_normal){

    Eigen::Vector3f C = Eigen::Vector3f::Zero();
    search::KdTree<PointC>::Ptr tree (new search::KdTree<PointC>());

    // Calculando as normais
    NormalEstimationOMP<PointC, Normal> ne;
    PointCloud<PointC>::Ptr entradaptr (new PointCloud<PointC>());
    *entradaptr = entrada;
    ne.setInputCloud(entradaptr);

    ne.setSearchMethod(tree);
    PointCloud<Normal>::Ptr cloud_normals (new PointCloud<Normal>());
    ne.setKSearch(20);
    ne.setNumberOfThreads(8);

    ne.compute(*cloud_normals);

    concatenateFields(*entradaptr, *cloud_normals, *acc_normal);

    std::vector<int> indicesnan;
    removeNaNNormalsFromPointCloud(*acc_normal, *acc_normal, indicesnan);

    // Forcar virar as normais na marra
    for(unsigned long i=0; i < acc_normal->size(); i++){
        Eigen::Vector3f normal, cp;
        normal << acc_normal->points[i].normal_x, acc_normal->points[i].normal_y, acc_normal->points[i].normal_z;
        cp << C(0)-acc_normal->points[i].x, C(1)-acc_normal->points[i].y, C(2)-acc_normal->points[i].z;
        float cos_theta = (normal.dot(cp))/(normal.norm()*cp.norm());
        if(cos_theta <= 0){ // Esta apontando errado, deve inverter
            acc_normal->points[i].normal_x = -acc_normal->points[i].normal_x;
            acc_normal->points[i].normal_y = -acc_normal->points[i].normal_y;
            acc_normal->points[i].normal_z = -acc_normal->points[i].normal_z;
        }
    }

}
///////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix4f SaveAndWork::calculate_transformation(float thetay_deg){
    // Constroi matriz de rotacao
    Eigen::Matrix3f R;
    R = Eigen::AngleAxisf(                  0, Eigen::Vector3f::UnitX()) *
        Eigen::AngleAxisf(DEG2RAD(thetay_deg), Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(                  0, Eigen::Vector3f::UnitZ());
    // Constroi matriz homgenea e retorna
    Eigen::MatrixXf t(3, 1);
    t << 0.0148,
              0,
              0;
    Eigen::Matrix4f Trot;
    Trot << R, R*t,
            0, 0, 0, 1;

    return Trot*T_laser_astra;
}
///////////////////////////////////////////////////////////////////////////////////////////
void SaveAndWork::project_cloud_to_image(PointCloud<PointC>::Ptr in, cv::Mat img, Eigen::Matrix4f T){
    // Matriz de projecao
    Eigen::MatrixXf P(3, 4);
    P = K_astra*T.block<3,4>(0, 0);

    #pragma omp parallel for num_threads(20)
    for(size_t i=0; i < in->size(); i++){
        Eigen::MatrixXf X_(4,1);
        X_ << in->points[i].x, in->points[i].y, in->points[i].z, 1;
        // Projeta e tira a escala
        Eigen::MatrixXf X = P*X_;
        X = X/X(2,0);
        // Atribui a cor se for possivel
        if(floor(X(0,0)) >= 0 && floor(X(0,0)) < img.cols && floor(X(1,0)) >= 0 && floor(X(1,0)) < img.rows){
            // Colore com a cor correspondente
            in->points[i].b = img.at<cv::Vec3b>(int(X(1, 0)), int(X(0, 0)))[0];
            in->points[i].g = img.at<cv::Vec3b>(int(X(1, 0)), int(X(0, 0)))[1];
            in->points[i].r = img.at<cv::Vec3b>(int(X(1, 0)), int(X(0, 0)))[2];
        } else {
            // Cria ponto preto para mostrar onde nao esta chegando
            in->points[i].b = 0;
            in->points[i].g = 0;
            in->points[i].r = 0;
        }
    }

    // Removendo outliers aqui para mostrar melhor no RViz
//    pcl::StatisticalOutlierRemoval<PointC> sor;
//    sor.setInputCloud(in);
//    sor.setMeanK(1);
//    sor.setStddevMulThresh(0.5);
//    sor.filter(*in);
}
///////////////////////////////////////////////////////////////////////////////////////////
std::string SaveAndWork::write_line_for_nvm(float f, std::string nome, Eigen::Quaternion<float> q){
    std::string linha = nome;
    // Adicionar foco
    linha = linha + " " + std::to_string(f);
    // Adicionar quaternion
    linha = linha + " " + std::to_string(q.w()) + " " + std::to_string(q.x()) + " " + std::to_string(q.y()) + " " + std::to_string(q.z());
    // Adicionar centro da camera
    linha = linha + " 0 0 0";
    // Adicionar distorcao radial (crendo 0) e 0 final
    linha = linha + " 0 0\n"; // IMPORTANTE pular linha aqui, o MeshRecon precisa disso no MART
    // Muda as virgulas por pontos no arquivo
    std::replace(linha.begin(), linha.end(), ',', '.');
    return linha;
}
