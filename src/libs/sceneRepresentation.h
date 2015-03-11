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

class sceneRepresentation{

public:

    sceneRepresentation();
    sceneRepresentation(string configFile);
    ~sceneRepresentation();
    void initializeScene(Matrix4f x_0);
    bool updateScene();

    void setText(int frame_, float time_, int nPoints_, int nPointsH_, int nLines_, int nLinesH_);
    void setCov(MatrixXf cov_);
    void setPose(Matrix4f x_);
    void setGT(Matrix4f xgt_);
    void setComparison(Matrix4f xcomp_);
    void setImage(Mat image_);
    void setImage(string image_);
    void setLegend();
    void setHelp();
    void setPoints(CMatrixFloat pData_);
    void setLines(CMatrixFloat lData_);

    bool waitUntilClose();
    bool isOpen();
    bool getYPR(float &yaw, float &pitch, float &roll);
    bool getPose(Matrix4f &T);

private:

    CMatrixDouble getPoseFormat(Matrix4f T);
    CMatrixDouble33 getCovFormat(MatrixXf cov_);
    CPose3D getPoseXYZ(VectorXf x);

    CDisplayWindow3D*           win;
    COpenGLScenePtr             theScene;
    COpenGLViewportPtr          image, legend, help;
    opengl::CSetOfObjectsPtr    bbObj, bbObj1, srefObj, srefObj1, gtObj, srefObjGT;
    opengl::CEllipsoidPtr       elliObj;
    opengl::CSetOfLinesPtr      lineObj;
    opengl::CPointCloudPtr      pointObj;
    opengl::CFrustumPtr         frustObj, frustObj1;
    opengl::CAxisPtr            axesObj;


    float           sbb, saxis, srad, sref, sline, sfreq, szoom, selli, selev, sazim, sfrust, slinef;
    CVectorDouble   v_aux, v_aux_, v_aux1, v_aux1_, v_auxgt, gt_aux_, v_auxgt_;
    CPose3D         pose, pose_0, pose_gt, pose_ini, ellPose, pose1,  change, frustumL_, frustumR_;
    Matrix4f        x_ini;
    mrptKeyModifier kmods;
    int             key;
    CMatrixDouble33 cov3D;
    bool            hasText, hasCov, hasGT, hasChange, hasImg, hasLines, hasPoints, hasFrustum, hasComparison, hasLegend, hasHelp, hasAxes, hasTraj, isKitti;

    Matrix4f        x, xgt, xcomp;
    MatrixXf        cov, W;
    unsigned int    frame, nPoints, nPointsH, nLines, nLinesH;
    float           time;
    string          img, img_legend, img_help;
    CMatrixFloat    lData, pData;
    CImage          img_mrpt_legend, img_mrpt_image, img_mrpt_help;

};

