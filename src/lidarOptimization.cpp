// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#include "lidarOptimization.h"

ReprojectionAnalyticCostFunction::ReprojectionAnalyticCostFunction(Eigen::Vector3d curr_point_, Eigen::Vector2d predict_xy_, Eigen::Matrix3d K_, Eigen::Quaterniond q_w_last_, Eigen::Vector3d t_w_last_)
        : curr_point(curr_point_), predict_xy(predict_xy_), K(K_), q_w_last(q_w_last_), t_w_last(t_w_last_){
}

bool ReprojectionAnalyticCostFunction::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    // cout++;
    Eigen::Map<const Eigen::Quaterniond> q_curr(parameters[0]);
    Eigen::Map<const Eigen::Vector3d> t_curr(parameters[0] + 4);

    Eigen::Vector3d lp;
    // lp = q_curr * curr_point + t_curr;
    // lp = q_relative * curr_point + t_relative;
    lp = curr_point;

    double x = lp[0];
    double y = lp[1];
    double z = lp[2];
    double x_2 = lp[0] * lp[0];
    double y_2 = lp[1] * lp[1];
    double z_2 = lp[2] * lp[2];
    double fx = K(0, 0);
    double fy = K(1, 1);

    Eigen::Vector3d image_xy = K * lp;

    image_xy[0] = image_xy[0] / image_xy[2];
    image_xy[1] = image_xy[1] / image_xy[2];

    // double control = 1;
    double control = 0;
    if( abs(predict_xy[0] - image_xy[0]) >= 1 || abs(predict_xy[1] - image_xy[1]) >= 1)
        control = 0;
    else
        control = 1;
    
    residuals[0] = control * (predict_xy[0] - image_xy[0]);
    residuals[1] = control * (predict_xy[1] - image_xy[1]);
 
    // std::cout << "image resual[0] = " << residuals[0] << std::endl;
    // std::cout << "image resual[1] = " << residuals[1] << std::endl;
    
    if(jacobians != NULL)
    {
        if(jacobians[0] != NULL)
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor> > J_se3(jacobians[0]);
            J_se3.setZero();
            J_se3.coeffRef(0, 0) = fx/z;
            J_se3.coeffRef(0, 1) = 0;
            J_se3.coeffRef(0, 2) = -fx*x/z_2;
            J_se3.coeffRef(0, 3) = -fx*x*y/z_2;
            J_se3.coeffRef(0, 4) = fx + (fx*x_2/z_2);
            J_se3.coeffRef(0, 5) = -fx*y/z;
            J_se3.coeffRef(1, 0) = 0;
            J_se3.coeffRef(1, 1) = fy/z;
            J_se3.coeffRef(1, 2) = -fy*y/z_2;
            J_se3.coeffRef(1, 3) = -fy-(fy*y_2/z_2);
            J_se3.coeffRef(1, 4) = fy*x*y/z_2;
            J_se3.coeffRef(1, 5) = fy*x/z;
            // J_se3.setZero();
            // J_se3.coeffRef(0, 0) = -fx*x*y/z_2;
            // J_se3.coeffRef(0, 1) = fx + (fx*x_2/z_2);
            // J_se3.coeffRef(0, 2) = -fx*y/z;
            // J_se3.coeffRef(0, 3) = fx/z;
            // J_se3.coeffRef(0, 4) = 0;
            // J_se3.coeffRef(0, 5) = -fx*x/z_2;
            // J_se3.coeffRef(1, 0) = -fy-(fy*y_2/z_2);
            // J_se3.coeffRef(1, 1) = fy*x*y/z_2;
            // J_se3.coeffRef(1, 2) = fy*x/z;
            // J_se3.coeffRef(1, 3) = 0;
            // J_se3.coeffRef(1, 4) = fy/z;
            // J_se3.coeffRef(1, 5) = -fy*y/z_2;
            J_se3 = control * J_se3;
        }
    }

    return true;
 
}

EdgeAnalyticCostFunction::EdgeAnalyticCostFunction(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_a_, Eigen::Vector3d last_point_b_)
        : curr_point(curr_point_), last_point_a(last_point_a_), last_point_b(last_point_b_){

}

bool EdgeAnalyticCostFunction::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    
    Eigen::Map<const Eigen::Quaterniond> q_last_curr(parameters[0]);
    Eigen::Map<const Eigen::Vector3d> t_last_curr(parameters[0] + 4);
    Eigen::Vector3d lp;
    lp = q_last_curr * curr_point + t_last_curr; 

    Eigen::Vector3d nu = (lp - last_point_a).cross(lp - last_point_b);
    Eigen::Vector3d de = last_point_a - last_point_b;
    double de_norm = de.norm();
    residuals[0] = nu.norm()/de_norm;
    
    if(jacobians != NULL)
    {
        if(jacobians[0] != NULL)
        {
            Eigen::Matrix3d skew_lp = skew(lp);
            Eigen::Matrix<double, 3, 6> dp_by_se3;
            dp_by_se3.block<3,3>(0,0) = -skew_lp;
            (dp_by_se3.block<3,3>(0, 3)).setIdentity();
            Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor> > J_se3(jacobians[0]);
            J_se3.setZero();
            Eigen::Matrix3d skew_de = skew(de);
            J_se3.block<1,6>(0,0) = - nu.transpose() / nu.norm() * skew_de * dp_by_se3/de_norm;
      
        }
    }  

    return true;
 
}   


SurfNormAnalyticCostFunction::SurfNormAnalyticCostFunction(Eigen::Vector3d curr_point_, Eigen::Vector3d plane_unit_norm_, double negative_OA_dot_norm_) 
                                                        : curr_point(curr_point_), plane_unit_norm(plane_unit_norm_), negative_OA_dot_norm(negative_OA_dot_norm_){

}

bool SurfNormAnalyticCostFunction::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    Eigen::Map<const Eigen::Quaterniond> q_w_curr(parameters[0]);
    Eigen::Map<const Eigen::Vector3d> t_w_curr(parameters[0] + 4);
    Eigen::Vector3d point_w = q_w_curr * curr_point + t_w_curr;
    residuals[0] = plane_unit_norm.dot(point_w) + negative_OA_dot_norm;

    if(jacobians != NULL)
    {
        if(jacobians[0] != NULL)
        {
            Eigen::Matrix3d skew_point_w = skew(point_w);
            Eigen::Matrix<double, 3, 6> dp_by_se3;
            dp_by_se3.block<3,3>(0,0) = -skew_point_w;
            (dp_by_se3.block<3,3>(0, 3)).setIdentity();
            Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor> > J_se3(jacobians[0]);
            J_se3.setZero();
            J_se3.block<1,6>(0,0) = plane_unit_norm.transpose() * dp_by_se3;
        }
    }
    return true;

}   


bool PoseSE3Parameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const
{
    Eigen::Map<const Eigen::Vector3d> trans(x + 4);

    Eigen::Quaterniond delta_q;
    Eigen::Vector3d delta_t;
    getTransformFromSe3(Eigen::Map<const Eigen::Matrix<double,6,1>>(delta), delta_q, delta_t);
    Eigen::Map<const Eigen::Quaterniond> quater(x);
    Eigen::Map<Eigen::Quaterniond> quater_plus(x_plus_delta);
    Eigen::Map<Eigen::Vector3d> trans_plus(x_plus_delta + 4);

    quater_plus = delta_q * quater;
    trans_plus = delta_q * trans + delta_t;

    return true;
}

bool PoseSE3Parameterization::ComputeJacobian(const double *x, double *jacobian) const
{
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
    (j.topRows(6)).setIdentity();
    (j.bottomRows(1)).setZero();

    return true;
}

void getTransformFromSe3(const Eigen::Matrix<double,6,1>& se3, Eigen::Quaterniond& q, Eigen::Vector3d& t){
    Eigen::Vector3d omega(se3.data());
    Eigen::Vector3d upsilon(se3.data()+3);
    Eigen::Matrix3d Omega = skew(omega);

    double theta = omega.norm();
    double half_theta = 0.5*theta;

    double imag_factor;
    double real_factor = cos(half_theta);
    if(theta<1e-10)
    {
        double theta_sq = theta*theta;
        double theta_po4 = theta_sq*theta_sq;
        imag_factor = 0.5-0.0208333*theta_sq+0.000260417*theta_po4;
    }
    else
    {
        double sin_half_theta = sin(half_theta);
        imag_factor = sin_half_theta/theta;
    }

    q = Eigen::Quaterniond(real_factor, imag_factor*omega.x(), imag_factor*omega.y(), imag_factor*omega.z());


    Eigen::Matrix3d J;
    if (theta<1e-10)
    {
        J = q.matrix();
    }
    else
    {
        Eigen::Matrix3d Omega2 = Omega*Omega;
        J = (Eigen::Matrix3d::Identity() + (1-cos(theta))/(theta*theta)*Omega + (theta-sin(theta))/(pow(theta,3))*Omega2);
    }

    t = J*upsilon;
}

Eigen::Matrix<double,3,3> skew(Eigen::Matrix<double,3,1>& mat_in){
    Eigen::Matrix<double,3,3> skew_mat;
    skew_mat.setZero();
    skew_mat(0,1) = -mat_in(2);
    skew_mat(0,2) =  mat_in(1);
    skew_mat(1,2) = -mat_in(0);
    skew_mat(1,0) =  mat_in(2);
    skew_mat(2,0) = -mat_in(1);
    skew_mat(2,1) =  mat_in(0);
    return skew_mat;
}