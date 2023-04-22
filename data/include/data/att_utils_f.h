#ifndef ATT_UTILS_H
#define ATT_UTILS_H

#include <Eigen/Dense>

using namespace Eigen;


  Vector4f q2aa(const Vector4f &q)
  {
    Vector4f aa;
    if (q(3) == 1) {
      aa << 1.0, 0.0, 0.0, 0.0;
    }
    else {
      float alpha = 2*atan2(sqrt(pow(q(0),2) + pow(q(1),2) + pow(q(2),2)), q(3));
      float ex = q(0)/sqrt(pow(q(0),2) + pow(q(1),2) + pow(q(2),2));
      float ey = q(1)/sqrt(pow(q(0),2) + pow(q(1),2) + pow(q(2),2));
      float ez = q(2)/sqrt(pow(q(0),2) + pow(q(1),2) + pow(q(2),2));
      aa(0) = ex;
      aa(1) = ey;
      aa(2) = ez;
      aa(3) = alpha;
    }
    return aa;
  }

  Vector4f aa2q(const Vector4f &aa)
  {
    float alpha = aa(3);
    float ex = aa(0);
    float ey = aa(1);
    float ez = aa(2);
    Vector4f q;
    q(0) = ex*sin(alpha/2);
    q(1) = ey*sin(alpha/2);
    q(2) = ez*sin(alpha/2);
    q(3) = cos(alpha/2);
    return q;
  }

  Matrix3f q2dcm(const Vector4f &q)
  {
    Matrix3f dcm;
    dcm(0,0) = pow(q(3),2) + pow(q(0),2) - pow(q(1),2) - pow(q(2),2);
    dcm(0,1) = 2*(q(0)*q(1) + q(3)*q(2));
    dcm(0,2) = 2*(q(0)*q(2) - q(3)*q(1));
    dcm(1,0) = 2*(q(0)*q(1) - q(3)*q(2));
    dcm(1,1) = pow(q(3),2) - pow(q(0),2) + pow(q(1),2) - pow(q(2),2);
    dcm(1,2) = 2*(q(1)*q(2) + q(3)*q(0));
    dcm(2,0) = 2*(q(0)*q(2) + q(3)*q(1));
    dcm(2,1) = 2*(q(1)*q(2) - q(3)*q(0));
    dcm(2,2) = pow(q(3),2) - pow(q(0),2) - pow(q(1),2) + pow(q(2),2);
    return dcm.transpose();
  }

  Matrix3f R_x(float angle)
  {
    Matrix3f R;
    R << 1.0, 0.0, 0.0,
         0.0, cos(angle), -sin(angle),
         0.0, sin(angle), cos(angle);
    return R;
  }

  Matrix3f R_y(float angle)
  {
    Matrix3f R;
    R << cos(angle), 0.0, sin(angle),
         0.0, 1.0, 0.0,
         -sin(angle), 0.0, cos(angle);
    return R;
  }

  Matrix3f R_z(float angle)
  {
    Matrix3f R;
    R << cos(angle), -sin(angle), 0.0,
         sin(angle), cos(angle), 0.0,
         0.0, 0.0, 1.0;
    return R;
  }

  Vector4f vec2q(const Vector3f &vec)
  {
    float alpha = sqrt(pow(vec(0),2) + pow(vec(1),2) + pow(vec(2),2));
    float ex = vec(0) / alpha;
    float ey = vec(1) / alpha;
    float ez = vec(2) / alpha;
    Vector4f aa;
    aa << ex, ey, ez, alpha;
    Vector4f q;
    q = aa2q(aa);
    return q;
  }

  Vector3f q2vec(const Vector4f &q)
  {
    Vector4f aa;
    aa = q2aa(q);
    float ex = aa(0);
    float ey = aa(1);
    float ez = aa(2);
    float alpha = aa(3);
    Vector3f vec;
    vec(0) = ex*alpha;
    vec(1) = ey*alpha;
    vec(2) = ez*alpha;
    return vec;
  }

  Vector3f q2eul313(const Vector4f &q)
  {
    Vector3f eul;
    Matrix3f dcm;
    dcm = q2dcm(q);
    eul(0) = atan2(dcm(2,0), dcm(2,1));
    eul(1) = acos(dcm(2,2));
    eul(2) = -atan2(dcm(0,2), dcm(1,2));
    return eul;
  }

  Matrix3f eul2dcm313(const Vector3f &eul_angles)
  {
    Matrix3f dcm;
    dcm = R_z(eul_angles(2))*R_x(eul_angles(1))*R_z(eul_angles(0));
    return dcm;
  }

  Vector4f dcm2q(const Matrix3f &dcm)
  {
    Vector4f q;
    float tr = dcm(0,0) + dcm(1,1) + dcm(2,2);
    if(tr > 0)
    {
      float S = sqrt(tr + 1) * 2;
      q(3) = 0.25 * S;
      q(0) = (dcm(2,1) - dcm(1,2)) / S;
      q(1) = (dcm(0,2) - dcm(2,0)) / S;
      q(2) = (dcm(1,0) - dcm(0,1)) / S;

    } else if(dcm(0,0) > dcm(1,1) && dcm(0,0) > dcm(2,2))
    {
      float S = sqrt(1.0 + dcm(0,0) - dcm(1,1) - dcm(2,2)) * 2;
      q(3) = (dcm(2,1) - dcm(1,2)) / S;
      q(0) = 0.25 * S;
      q(1) = (dcm(0,1) + dcm(1,0)) / S;
      q(2) = (dcm(0,2) + dcm(2,0)) / S;

    } else if(dcm(1,1) > dcm(2,2))
    {
      float S = sqrt(1.0 + dcm(1,1) - dcm(0,0) - dcm(2,2)) * 2;
      q(3) = (dcm(0,2) - dcm(2,0)) / S;
      q(0) = (dcm(0,1) + dcm(1,0)) / S;
      q(1) = 0.25 * S;
      q(2) = (dcm(1,2) + dcm(2,1)) / S;

    } else
    {
      float S = sqrt(1.0 + dcm(2,2) - dcm(0,0) - dcm(1,1)) * 2;
      q(3) = (dcm(1,0) - dcm(0,1)) / S;
      q(0) = (dcm(0,2) + dcm(2,0)) / S;
      q(1) = (dcm(1,2) + dcm(2,1)) / S;
      q(2) = 0.25 * S;
    }
    return q;
  }

  Vector4f qmult(const Vector4f &q1, const Vector4f &q2)
  {
    Vector4f q_prod;
    float x1 = q1(0); float y1 = q1(1); float z1 = q1(2); float w1 = q1(3);
    float x2 = q2(0); float y2 = q2(1); float z2 = q2(2); float w2 = q2(3);
    q_prod(0) =  x1*w2 + y1*z2 - z1*y2 + w1*x2;
    q_prod(1) = -x1*z2 + y1*w2 + z1*x2 + w1*y2;
    q_prod(2) =  x1*y2 - y1*x2 + z1*w2 + w1*z2;
    q_prod(3) = -x1*x2 - y1*y2 - z1*z2 + w1*w2;
    return q_prod.normalized();
  }

  float eigen_error(const Vector4f &q_real, const Vector4f &q_nom)
  {
    Vector4f q_real_conj;
    q_real_conj << -q_real(0), -q_real(1), -q_real(2), q_real(3);
    Vector4f q_error;
    q_error = qmult(q_nom, q_real_conj);
    Vector3f q_error_3f;
    q_error_3f << q_error(0), q_error(1), q_error(2);
    float a_error = 2.0 * asin(q_error_3f.norm());
    return a_error;
  }

#endif // ATT_UTILS_H
