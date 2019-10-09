#include "../include/scanner_gui/otimizacameraimagens.hpp"
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

///////////////////////////////////////////////////////////////////////////////////////////
OtimizaCameraImagens::OtimizaCameraImagens()
{
    QFuture<void> future = QtConcurrent::run(this, &OtimizaCameraImagens::init);
}
///////////////////////////////////////////////////////////////////////////////////////////
OtimizaCameraImagens::~OtimizaCameraImagens(){
    if(ros::isStarted()){
        ros::shutdown();
        ros::waitForShutdown();
    }
    wait();
}
///////////////////////////////////////////////////////////////////////////////////////////
void OtimizaCameraImagens::init(){
    fzed = 1462.0;
    fastra = 525.0;
    T_astra_zed_original << 1, 0, 0,  0.052,
                            0, 1, 0,  0.01 ,
                            0, 0, 1, -0.01 ,
                            0, 0, 0,  1    ;
}
///////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix4f OtimizaCameraImagens::process_pipeline(cv_bridge::CvImagePtr imagem_zed,
                                                       cv_bridge::CvImagePtr imagem_ast,
                                                       PointCloud<PointXYZ>::Ptr nuvem_pix_total,
                                                       PointCloud<PointC>::Ptr nuvem_bat,
                                                       Eigen::Matrix4f T_,
                                                       float &foco){
    // Aproximacao inicial
    T_projecao_atual = T_astra_zed_original*T_.inverse();

    comparaSift(imagem_ast, imagem_zed, nuvem_bat);
    resolveAstraPixeis(nuvem_pix_total, nuvem_bat, imagem_zed);

    // Chamar aqui o bat para otimizar em cima de toda a matriz de Transformacao
    // Otimizar foco, rotacao e posicao para ZED
    camera co; // camera com resultados para otimizar
    Eigen::Vector2f s(imagem_zed->image.cols/2.0, imagem_zed->image.rows/2.0);
    bool funcionou;
    co = bat(imagePointsZed, objectPointsZed, Tazo, fzed, s, funcionou);

    // Uma vez otimizado pelo bat a partir das correspondencias de pontos
    if(funcionou){
        T_projecao_atual = co.T;
        foco = co.foco;
    } else {
        foco = fzed;
    }
}
///////////////////////////////////////////////////////////////////////////////////////////
void OtimizaCameraImagens::comparaSift(cv_bridge::CvImagePtr astra, cv_bridge::CvImagePtr zed, PointCloud<PointC>::Ptr cloud){
    /// Calculando descritores SIFT ///
    // Keypoints e descritores para astra e zed
    std::vector<cv::KeyPoint> keypointsa, keypointsz;
    cv::Mat descriptorsa, descriptorsz;
    /// Comparando e filtrando matchs ///
    cv::Ptr<cv::DescriptorMatcher> matcher;
    matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    std::vector<std::vector< cv::DMatch > > matches;
    std::vector< cv::DMatch > good_matches;

    int tent = 0; // tentativas de achar X correspondencias bacanas
    float min_hessian = 2000;

    // Achando limites na imagem em X e Y onde podemos ter pontos 3D, estimativa inicial pra ter correspondencias
    Eigen::Matrix3f Kz;
    Kz << fzed,   0 , zed->image.cols/2.0,
            0 , fzed, zed->image.rows/2.0,
            0 ,   0 ,          1         ;
    Eigen::MatrixXf Pz = Kz*T_projecao_atual.block<3, 4>(0, 0);
    Eigen::Vector4f limites(1000, -1000, 1000, -1000), ponto;
    Eigen::Vector3f ponto_proj;
    for(size_t i=0; i<cloud->size(); i++){
        ponto << cloud->points[i].x, cloud->points[i].y, cloud->points[i].z, 1.0;
        ponto_proj = Pz*ponto;
        ponto_proj = ponto_proj/ponto_proj[2];
        if(ponto_proj[0] > 0 && ponto_proj[0] < zed->image.cols && ponto_proj[1] > 0 && ponto_proj[1] < zed->image.rows){
            if(ponto_proj[0] < limites[0])
                limites[0] = ponto_proj[0];
            if(ponto_proj[0] > limites[1])
                limites[1] = ponto_proj[0];
            if(ponto_proj[1] < limites[2])
                limites[2] = ponto_proj[1];
            if(ponto_proj[1] > limites[3])
                limites[3] = ponto_proj[1];
        }
    }

//    cout << "\nLimites x: " << limites[0] << " " << limites[1] << endl;
//    cout << "Limites y: "   << limites[2] << " " << limites[3] << endl << endl;
    goodKeypointsLeft.clear(); goodKeypointsRight.clear();

    while(goodKeypointsLeft.size() < 7 && tent < 20){
        good_matches.clear();
        Ptr<xfeatures2d::SURF> f2d = xfeatures2d::SURF::create(min_hessian);
        // Astra
        f2d->detectAndCompute(astra->image, Mat(), keypointsa, descriptorsa);
        // Zed
        f2d->detectAndCompute(zed->image  , Mat(), keypointsz, descriptorsz);

        matcher->knnMatch(descriptorsa, descriptorsz, matches, 2);

        for (size_t i = 0; i < matches.size(); i++)
        {
            if (matches.at(i).size() >= 2)
            {
                if (matches.at(i).at(0).distance < 0.75*matches.at(i).at(1).distance)
                {
                    good_matches.push_back(matches.at(i).at(0));
                }
            }
        }

        tent += 1;
        min_hessian = 0.7*min_hessian;
        //    } // alterar aqui

        // Daqui para baixo temos astra->left e zed->right

        std::vector<cv::Point2f> imgLeftPts;
        std::vector<cv::Point2f> imgRightPts;

        goodKeypointsLeft.clear();
        goodKeypointsRight.clear();

//        float scaleazx = zed->image.cols/astra->image.cols, scaleazy = zed->image.rows/astra->image.rows, window = 200.0;
        cout << "antes do meu filtro por janela: " << good_matches.size() << endl;
        for (size_t i = 0; i < good_matches.size(); i++)
        {
//            KeyPoint kp_as = keypointsa[good_matches[i].queryIdx], kp_zed = keypointsz[good_matches[i].trainIdx];
            //-- Get the keypoints from the good matches
//            if(kp_zed.pt.x > kp_as.pt.x*scaleazx-window && kp_zed.pt.x < kp_as.pt.x*scaleazx+window &&
//               kp_zed.pt.y > kp_as.pt.y*scaleazy-window && kp_zed.pt.y < kp_as.pt.y*scaleazy+window   ){
                goodKeypointsLeft.push_back(keypointsa[good_matches[i].queryIdx]);
                goodKeypointsRight.push_back(keypointsz[good_matches[i].trainIdx]);
                imgLeftPts.push_back(keypointsa[good_matches[i].queryIdx].pt);
                imgRightPts.push_back(keypointsz[good_matches[i].trainIdx].pt);
//            }
        }

        cout << "Aqui quantos keypoints bons depois da minha moda? " << goodKeypointsLeft.size() << endl;
        if(imgLeftPts.size() > 0){
            cv::Mat inliers;
            cv::Mat Ka = (cv::Mat_<double>(3, 3) << fastra, 0, astra->image.cols / 2.0, 0, fastra, astra->image.rows / 2.0, 0, 0, 1);
            cv::Mat E = findEssentialMat(imgLeftPts, imgRightPts, Ka, CV_RANSAC, 0.99999, 1.0, inliers);

            std::vector<cv::KeyPoint> goodKeypointsLeftTemp;
            std::vector<cv::KeyPoint> goodKeypointsRightTemp;
            bool dx = false, dy = false;
            for (size_t i = 0; i < inliers.rows; i++)
            {
                // Filtrando aqui pelos limites provaveis da nuvem tambem
                dx = false; dy = false;
                if(goodKeypointsLeft.at(i).pt.x > limites[0] && goodKeypointsLeft.at(i).pt.x < limites[1])
                    dx = true;
                if(goodKeypointsLeft.at(i).pt.y > limites[2] && goodKeypointsLeft.at(i).pt.y < limites[3])
                    dy = true;
                if (inliers.at<uchar>(i, 0) == 1 && dx && dy)
                {
                    goodKeypointsLeftTemp.push_back(goodKeypointsLeft.at(i));
                    goodKeypointsRightTemp.push_back(goodKeypointsRight.at(i));
                }
            }
            goodKeypointsLeft = goodKeypointsLeftTemp;
            goodKeypointsRight = goodKeypointsRightTemp;
            cout << "Aqui quantos keypoints bons depois do teste inliers e limites? " << goodKeypointsLeft.size() << endl;
        }

    } // fim do while

    if(goodKeypointsLeft.size()){
        char* home;
        home = getenv("HOME");
        std::string pasta = std::string(home)+"/Desktop/teste/";
        Mat a, z;
        astra->image.copyTo(a); zed->image.copyTo(z);
        for(size_t i=0; i<goodKeypointsLeft.size(); i++){
            cv::Scalar color = cv::Scalar(rand() % 255, rand() % 255, rand() % 255);
            circle(a, goodKeypointsLeft[i].pt, 5, color, 2);
            circle(z, goodKeypointsRight[i].pt, 5, color, 2);
        }
        std::string foto_zed = pasta+"fotozed.jpeg", foto_astra = pasta+"fotoastra.jpeg";
        imwrite(foto_astra, a);
        imwrite(foto_zed,   z);
    }
}
///////////////////////////////////////////////////////////////////////////////////////////
void OtimizaCameraImagens::resolveAstraPixeis(PointCloud<PointXYZ>::Ptr pixeis, PointCloud<PointC>::Ptr nuvem_total_bat, cv_bridge::CvImagePtr zed){
    if(goodKeypointsLeft.size() > 0){

        cout << "goodkeypointsleft aqui: " << goodKeypointsLeft.size() << endl;

        // Jeito meu, a principio burro, mas logico
        std::vector<int> indices_nuvem;
        indices_nuvem.resize(goodKeypointsLeft.size(), -1);
        std::vector<float> melhores_distancias;
        melhores_distancias.resize(indices_nuvem.size(), 1000);
        int lim_coord = 10; // Limite de distancia ao quadrado em pixels para cada coordenada

        #pragma omp parallel for num_threads(int(goodLeftKeypoints.size()))
        for(size_t i=0; i < goodKeypointsLeft.size(); i++) {
            for(unsigned long j=0; j < pixeis->size(); j++){
                float dx2 = (pixeis->points[j].x - goodKeypointsLeft[i].pt.x)*(pixeis->points[j].x - goodKeypointsLeft[i].pt.x);
                float dy2 = (pixeis->points[j].y - goodKeypointsLeft[i].pt.y)*(pixeis->points[j].y - goodKeypointsLeft[i].pt.y);

                if(sqrt(dx2) < lim_coord && sqrt(dy2) < lim_coord && sqrt(dx2+dy2) < melhores_distancias[i]){
                    melhores_distancias[i] = sqrt(dx2+dy2);
                    indices_nuvem[i] = j;
                }
            }
        }

        // Relacionando pontos 3D com o SIFT da Zed
        imagePointsZed.clear();
        objectPointsZed.clear();
        for (unsigned long i=0; i<indices_nuvem.size(); i++)
        {
            if (indices_nuvem[i] == -1)
                continue;
            imagePointsZed.push_back(goodKeypointsRight[i].pt);
            PointC pnuvem = nuvem_total_bat->points[indices_nuvem[i]];
            cv::Point3f p(pnuvem.x, pnuvem.y, pnuvem.z);
            objectPointsZed.push_back(p);
        }

        cout << "imagePointszed aqui: " << imagePointsZed.size() << endl;

//        updateRTFromSolvePNP(imagePointsZed, objectPointsZed, zed);
    }
}
///////////////////////////////////////////////////////////////////////////////////////////
void OtimizaCameraImagens::updateRTFromSolvePNP(std::vector<cv::Point2f> imagePoints, std::vector<cv::Point3f> objectPoints, cv_bridge::CvImagePtr zed)
{
    if(imagePoints.size() > 4){
        cv::Mat R, t, distCoef;
        cv::Mat Ka = (cv::Mat_<double>(3, 3) << fzed, 0, zed->image.cols/2.0, 0, fzed, zed->image.rows/2.0, 0, 0, 1);
        cv::solvePnPRansac(objectPoints, imagePoints, Ka, distCoef, R, t);
        cv::Mat R_3_3;
        cv::Rodrigues(R, R_3_3);
        for (unsigned int i = 0; i < 3; i++)
        {
            for (unsigned int j = 0; j < 3; j++)
            {
                T_astra_zed(i, j) = R_3_3.at<double>(i, j);
            }
        }
        for (unsigned int j = 0; j < 3; j++)
        {
            T_astra_zed(j, 3) = t.at<double>(j, 0);
        }
        T_astra_zed(3, 0) = T_astra_zed(3, 1) = T_astra_zed(3, 2) = 0; T_astra_zed(3, 3) = 1;
    } else {
        // Nao ha pontos suficientes, permanece com o chute de pose e foco vindos da zed+icp
        T_astra_zed = T_projecao_atual;
    }
}
///////////////////////////////////////////////////////////////////////////////////////////
OtimizaCameraImagens::camera OtimizaCameraImagens::bat(std::vector<Point2f> xy_zed, std::vector<Point3f> X_zed, Eigen::Matrix4f T_est, float foco_est, Eigen::Vector2f c_img, bool &valid){
    // Variavel de saida
    camera c;

    cout << "\nquantos pixels correspondentes? " << xy_zed.size() << endl;

    // Restriçoes do espaço de busca aqui - a principio somente somente translacao e foco, depois adicionar a rotacao tambem
    Eigen::MatrixXf rest(2, 4);
    float libt = 0.1, libf = 150; // Metros de liberdade em translaçao / unidade de foco, para espaço de busca
    rest << foco_est-libf, T_est(0, 3)-libt, T_est(1, 3)-libt, T_est(2, 3)-libt,
            foco_est+libf, T_est(0, 3)+libt, T_est(1, 3)+libt, T_est(2, 3)+libt;

    /// Parametros para os bats ///
    int nbats = 2000;
    float alfa = 0.5, lambda = 0.6, beta = 0.2, e = -0.1;

    // Caracteristicas dos bats (velocidade, taxa de emissao de pulso e amplitude sonora
    Eigen::MatrixXf v(nbats, rest.cols()), r(nbats, rest.cols());
    Eigen::MatrixXf As = Eigen::MatrixXf::Constant(nbats, rest.cols(), 1);
    Eigen::VectorXf F(nbats); // Vetor contendo a FOB de cada morcego
    float fob_temp = 0;

    // Iteraçoes
    int t = 0, t_max = 40, t_lim = 7;

    /// Iniciando os bats
    Eigen::MatrixXf bats(nbats, rest.cols());
    for(int i=0; i<nbats; i++){
        bats.row(i) << Eigen::MatrixXf::Random(1, rest.cols());
        F(i) = fob(xy_zed, X_zed, T_est, bats.row(i), c_img, rest);
    }
    // Variavel para o indice e menor valor do vetor de fob
    Eigen::VectorXf::Index indice_melhor_bat;
    float melhor_valor = F.minCoeff(&indice_melhor_bat);

    /// Rodando o algoritmo, realmente, vamos la ///
    float valor_anterior = melhor_valor;
    int contador_repeticoes = 0;
    while(t < t_max){
        // Controle de repeticao
        if(valor_anterior - melhor_valor <= 1e-2){
            contador_repeticoes += 1;
        } else {
            contador_repeticoes = 0;
        }
        valor_anterior = melhor_valor;

        // Se nao estourou repeticao, rodam os morcegos
        if(contador_repeticoes < t_lim){
            for(int i=0; i<nbats; i++){

                // Calculo da velocidade do morcego
                v.row(i) << v.row(i) + (bats.row(indice_melhor_bat)-bats.row(i))*beta;
                Eigen::MatrixXf bat_temp(1, 4);
                bat_temp << bats.row(i) + v.row(i);
                // Etapa de busca local
                if((double)rand()/(RAND_MAX) < r(i))
                    bat_temp << bats.row(indice_melhor_bat) + Eigen::MatrixXf::Constant(1, rest.cols(), e*As.mean());
                // Etapa de avaliacao da fob
                for(int j=0; j<bat_temp.cols(); j++){
                    if(bat_temp(j) < -1) bat_temp(j) = -1;
                    if(bat_temp(j) >  1) bat_temp(j) =  1;
                }
                fob_temp = fob(xy_zed, X_zed, T_est, bat_temp, c_img, rest);
                // Atualizando o morcego ou nao por busca global
                if((double)rand()/(RAND_MAX) < As(i) || fob_temp < F(i)){
                    bats.row(i) << bat_temp;
                    r(i)  = 1 - exp(-lambda*t);
                    As(i) = alfa*As(i);
                    F(i)  = fob_temp;
                }
                // Busca novamente pelo melhor valor dentre os morcegos pela fob
                melhor_valor = F.minCoeff(&indice_melhor_bat);

            } // fim do for de bats
            // Aumenta o contador de t iteracoes corridas
            t += 1;
            cout << "\nValor da FOB do melhor bat por pixel: " << melhor_valor/xy_zed.size() << endl;
        } else {
            break; // Ja acabou a busca aqui entao
        }

    } // fim do while t<tmax

    // Traz os valores de volta para o range original a partir do melhor bat e guarda na camera
    float f_bom  = (rest(1,0) - rest(0,0))*(bats.row(indice_melhor_bat)(0) + 1)/2 + rest(0, 0);
    float tx_bom = (rest(1,1) - rest(0,1))*(bats.row(indice_melhor_bat)(1) + 1)/2 + rest(0, 1);
    float ty_bom = (rest(1,2) - rest(0,2))*(bats.row(indice_melhor_bat)(2) + 1)/2 + rest(0, 2);
    float tz_bom = (rest(1,3) - rest(0,3))*(bats.row(indice_melhor_bat)(3) + 1)/2 + rest(0, 3);
    c.foco = f_bom;
    T_est.block<3, 1>(0, 3) << tx_bom, ty_bom, tz_bom;
    c.T << T_est;

    // Limpar os vetores
    imagePointsZed.clear(); objectPointsZed.clear();

    // Ver se o algoritmo foi capaz de otimizar
    valid = false;
    if(melhor_valor/xy_zed.size() <= 15.0)
        valid = true;

    return c;
}
///////////////////////////////////////////////////////////////////////////////////////////
float OtimizaCameraImagens::fob(std::vector<Point2f> xy_zed, std::vector<Point3f> X_zed, Eigen::Matrix4f T_est, Eigen::MatrixXf bat, Eigen::Vector2f c_img, Eigen::MatrixXf range){
    // Somatorio da fob final aqui
    float fob_final = 0;
    // Trazendo os valores de volta ao range original
    float f  = (range(1,0) - range(0,0))*(bat(0,0) + 1)/2 + range(0, 0);
    float tx = (range(1,1) - range(0,1))*(bat(0,1) + 1)/2 + range(0, 1);
    float ty = (range(1,2) - range(0,2))*(bat(0,2) + 1)/2 + range(0, 2);
    float tz = (range(1,3) - range(0,3))*(bat(0,3) + 1)/2 + range(0, 3);
    // Alterando a matriz de transformaçao
    T_est(0,3) = tx; T_est(1,3) = ty; T_est(2,3) = tz;
    // Matriz intrinseca
    Eigen::Matrix3f K_est;
    K_est << f, 0, c_img(0),
             0, f, c_img(1),
             0, 0,     1   ;
    // Ajustando matriz de transformaçao para 3x4
    Eigen::MatrixXf T(3, 4);
    T << T_est.block(0, 0, 3, 4);
    // Conferindo o tamanho dos vetores para evitar erro
    size_t pontos = (X_zed.size() <= xy_zed.size()) ? X_zed.size() : xy_zed.size();
    // Passando por todos os pontos 3D e 2D para calcular a fob final
    for(size_t i=0; i<pontos; i++){
        Eigen::Vector4f X_(X_zed[i].x, X_zed[i].y, X_zed[i].z, 1);
        Eigen::Vector3f x_goal(xy_zed[i].x, xy_zed[i].y, 1);
        Eigen::Vector3f X = T*X_;
        X = X/X(2); // Normalizando pela escala na ultima casa
        Eigen::Vector3f x = K_est*X;

        fob_final += sqrt( (x(0)-x_goal(0))*(x(0)-x_goal(0)) + (x(1)-x_goal(1))*(x(1)-x_goal(1)) );//(x - x_goal).norm();
    }

    return fob_final;
}
///////////////////////////////////////////////////////////////////////////////////////////
void OtimizaCameraImagens::printT(Eigen::Matrix4f T){
    cout << endl << endl;
    ROS_INFO("%.4f  %.4f  %.4f  %.4f", T(0,0), T(0,1), T(0,2), T(0,3));
    ROS_INFO("%.4f  %.4f  %.4f  %.4f", T(1,0), T(1,1), T(1,2), T(1,3));
    ROS_INFO("%.4f  %.4f  %.4f  %.4f", T(2,0), T(2,1), T(2,2), T(2,3));
    ROS_INFO("%.4f  %.4f  %.4f  %.4f", T(3,0), T(3,1), T(3,2), T(3,3));
    cout << endl << endl;
}
///////////////////////////////////////////////////////////////////////////////////////////
