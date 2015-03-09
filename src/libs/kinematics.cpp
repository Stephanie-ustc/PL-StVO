/*****************************************************************************
**   Stereo Visual Odometry by combining point and line segment features	**
******************************************************************************
**																			**
**	Copyright(c) 2015, Ruben Gomez-Ojeda, University of Malaga              **
**	Copyright(c) 2015, MAPIR group, University of Malaga					**
**																			**
**  This program is free software: you can redistribute it and/or modify	**
**  it under the terms of the GNU General Public License (version 3) as		**
**	published by the Free Software Foundation.								**
**																			**
**  This program is distributed in the hope that it will be useful, but		**
**	WITHOUT ANY WARRANTY; without even the implied warranty of				**
**  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the			**
**  GNU General Public License for more details.							**
**																			**
**  You should have received a copy of the GNU General Public License		**
**  along with this program.  If not, see <http://www.gnu.org/licenses/>.	**
**																			**
*****************************************************************************/

using namespace Eigen;

Matrix3f skew(Vector3f v)
{
    Matrix3f skew;

    skew(0,0) = 0; skew(1,1) = 0; skew(2,2) = 0;

    skew(0,1) = -v(2);
    skew(0,2) =  v(1);
    skew(1,2) = -v(0);

    skew(1,0) =  v(2);
    skew(2,0) = -v(1);
    skew(2,1) =  v(0);

    return skew;
}

Matrix3f fast_skewexp(Vector3f v)
{
    Matrix3f M, s, I = Matrix3f::Identity();
    double theta = v.norm();

    if(theta==0.0)
        M = I;
    else
    {
        s = skew(v)/theta;
        M << I + s * sin(theta) + s * s * (1-cos(theta));
    }

    return M;
}

Vector3f skewcoords(Matrix3f M)
{
    Vector3f skew;

    skew << M(2,1), M(0,2), M(1,0);

    return skew;
}

Matrix3f skewlog(Matrix3f M)
{
    Matrix3f skew;

    double val = (M.trace() - 1.0)/2.0;

    if(val > 1.0)
        val = 1.0;
    else if (val < -1.0)
        val = -1.0;

    double theta = acos(val);

    if(theta == 0.0)
        skew << 0,0,0,0,0,0,0,0,0;
    else
        skew << (M-M.transpose())/(2.0*sin(theta))*theta;

    return skew;
}

MatrixXf kroen_product(MatrixXf A, MatrixXf B)
{

    unsigned int Ar = A.rows(), Ac = A.cols(), Br = B.rows(), Bc = B.cols();

    MatrixXf AB(Ar*Br,Ac*Bc);

    for (unsigned int i=0; i<Ar; ++i)
        for (unsigned int j=0; j<Ac; ++j)
            AB.block(i*Br,j*Bc,Br,Bc) = A(i,j)*B;

    return AB;
}

Matrix3f v_logmap(VectorXf x)
{
    Vector3f w;
    double theta, theta2, theta3;
    Matrix3f W, I, V;

    w << x(0), x(1), x(2);
    theta = w.norm();   theta2 = theta*theta; theta3 = theta2*theta;
    W = skew(w);
    I << 1, 0, 0, 0, 1, 0, 0, 0, 1;

    if(theta>0.00001)
        V << I + ((1-cos(theta))/theta2)*W + ((theta-sin(theta))/theta3)*W*W;
    else
        V << I;

    return V;

}

MatrixXf diagonalMatrix(MatrixXf M, unsigned int N)
{
    MatrixXf A = MatrixXf::Zero(N,N);

    for(unsigned int i = 0; i < N; i++ )
    {
        A(i,i) = M(i,i);
    }

    return A;
}

Matrix4f transformation_expmap(VectorXf x){

    Matrix3f R, V, s, I = Matrix3f::Identity();
    Vector3f t, w;
    Matrix4f T = Matrix4f::Identity();

    t << x(0), x(1), x(2);
    w << x(3), x(4), x(5);

    double theta = w.norm();

    if(theta==0.0f)
        R = I;
        //t = t;
    else
    {
        s = skew(w)/theta;
        R << I + s * sin(theta) + s * s * (1.0f-cos(theta));
        V << I + s * (1.0f - cos(theta)) / theta + s * s * (theta - sin(theta)) / theta;
        t << V * t;
    }

    T.block(0,0,3,4) << R, t;

    return T;

}

VectorXf logarithm_map(Matrix4f T){

    Matrix3f R, Id3 = Matrix3f::Identity();
    Vector3f Vt, t, w;
    Matrix3f V = Matrix3f::Identity(), w_hat = Matrix3f::Zero();
    VectorXf x(6);

    Vt << T(0,3), T(1,3), T(2,3);
    w << 0.f, 0.f, 0.f;
    R = T.block(0,0,3,3);

    float val = (R.trace() - 1.f)/2.f;
    if(val > 1.f)
        val = 1.f;
    else if (val < -1.f)
        val = -1.f;

    float theta  = acos(val);
    float seno   = sin(theta);
    float coseno = cos(theta);

    if(theta != 0.f){
        w_hat << (R-R.transpose())/(2.f*seno)*theta;
        w << -w_hat(1,2), w_hat(0,2), -w_hat(0,1);
        Matrix3f s;
        s << skew(w) / theta;
        V << Id3 + s * (1.f-coseno) / theta + s * s * (theta - seno) / theta;
    }

    t = V.inverse() * Vt;
    x << t, w;
    return x;

}

Matrix4f transformation_expmap_approximate(VectorXf x){

    Matrix4f T = Matrix4f::Identity();

    T << 1.f, -x(5), x(4), x(0), x(5), 1.f, -x(3), x(1), -x(4), x(3), 1.f, x(2), 0.f, 0.f, 0.f, 1.f;

    return T;

}

VectorXf logarithm_map_approximate(Matrix4f T){

    VectorXf x(6);

    x(0) = T(0,3);
    x(1) = T(1,3);
    x(2) = T(2,3);
    x(3) = T(2,1);
    x(4) = T(0,2);
    x(5) = T(1,0);

    return x;

}

float    diffManifoldError(Matrix4f T1, Matrix4f T2){
    return (logarithm_map(T1)-logarithm_map(T2)).norm();
}

bool is_finite(const MatrixXf x){
    return ((x - x).array() == (x - x).array()).all();
}

float angDiff(float alpha, float beta){
    float theta = alpha - beta;
    if(theta>CV_PI)
        theta -= 2.f * CV_PI;
    if(theta<-CV_PI)
        theta += 2.f * CV_PI;
    return theta;
}


















