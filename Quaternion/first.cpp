#include <Eigen/Dense>
#include <iostream>

int main(int, char *[]) {
  // AngleAxisf aa = Quaternionf(..);
  Eigen::VectorXd vq(4);
  vq[0] = 1.0;
  vq[1] = 0.0;
  vq(2) = 0.0;
  vq(3) = 0.0;
  Eigen::Matrix<float, 4, 1> coe;
  Eigen::Matrix<float, 3, 3> R;
  Eigen::Quaternionf q(2.0, 0.1, 0.3, 4.0);  // wxyz
  coe = q.coeffs();
  std::cout << coe << std::endl;  // xyzw

  Eigen::Vector3f vec = q.vec();  // xyz
  std::cout << vec << std::endl;

  Eigen::AngleAxisf aa;
  Eigen::Matrix3f mat;
  Eigen::Quaternionf qm(mat);
  Eigen::Quaternionf qa(aa);

  Eigen::Quaternionf qw;
  qw.setIdentity();  // important
  R = qw.toRotationMatrix();
  qw = qw * q;
  qw = qw.normalized();  //规范化
  std::cout << R << std::endl;
  qw = qw.Identity();  // 1 0 0 0 or qw.setIdentity();
  std::cout << qw.w() << "  " << qw.x() << "  " << qw.y() << "  " << qw.z()
            << std::endl;
  qw = qw.inverse();
  std::cout << qw.w() << "  " << qw.x() << "  " << qw.y() << "  " << qw.z()
            << std::endl;

  Eigen::Vector3f w1;
  w1 << 1, 2, 3;
  Eigen::Vector3f w2;
  w2 << 2, 3, 4;
  qw = qw.Identity();
  qw = qw.setFromTwoVectors(w1, w2);  //出来的norm=1--姿态四元数
  std::cout << qw.squaredNorm() << std::endl;  // norm^2  .norm()
  q.setIdentity();
  qw = qw.normalized();
  q = q.normalized();
  q = q.inverse();
  std::cout << q.angularDistance(qw) << std::endl;  //必须先规范化
  std::cout << qw.dot(q) << std::endl;              // dot product 内积
  std::cout << qw.w() << "  " << qw.x() << "  " << qw.y() << "  " << qw.z()
            << std::endl;
  std::cout << q.w() << "  " << q.x() << "  " << q.y() << "  " << q.z()
            << std::endl;

  // std::cout<<q;
  std::cout << std::endl;
  for (int size = 1; size <= 4; ++size) {
    Eigen::MatrixXi m(size, size + 1);    // a (size)x(size+1)-matrix of int's
    for (int j = 0; j < m.cols(); ++j)    // loop over columns
      for (int i = 0; i < m.rows(); ++i)  // loop over rows
        m(i, j) = i + j * m.rows();       // to access matrix coefficients,
    // use operator()(int,int)
    std::cout << m << "\n\n";
  }
  Eigen::VectorXf v(4);  // a vector of 4 float's
  // to access vector coefficients, use either operator () or operator []
  v[0] = 1;
  v[1] = 2;
  v(2) = 3;
  v(3) = 4;
  std::cout << "\nv:\n" << v << std::endl;
}
