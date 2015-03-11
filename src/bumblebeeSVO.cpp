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

#include "bumblebeeSVO.h"

Mat         imgL, imgR;
mutex       bbMutex, algMutex;
Matrix3f    K_calib;
float       base_;
string      configFile  = "svoConfig.ini";

void bumblebeeThread(){

    // Initialize the camera
    bumblebeeGrabber bbGrabber(configFile);
    bbGrabber.getCalib(K_calib,base_);

    // Grab the first stereo frame
    bbGrabber.grabStereo(imgL,imgR);
    algMutex.unlock();

    // Start grabbing continuously
    while(true){
        bbMutex.lock();
        bbGrabber.grabStereo(imgL,imgR);
        algMutex.unlock();
    }

}

void stereoVO(){
    // Define Variables
    utils::CTicTac          clock;
    float                   plerrNorm;
    unsigned int            pMatches, pMatchesH, lMatches, lMatchesH, iter = 0;
    Matrix4f                x_0 = Matrix4f::Identity(), x_cw, x = Matrix4f::Identity();
    MatrixXf                cov = MatrixXf::Zero(6,6);
    x_cw << 1, 0, 0, 0, 0, 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1;
    // Define SVO object
    plSVO                 svo(configFile);
    // Initialize Stereo VO Parameters
    algMutex.lock();
    bbMutex.unlock();
    cout << endl << "Initializing..." ;
    svo.setStereo(K_calib,base_);
    svo.initialize(imgL,imgR);
    svo.setInitialValue(x_0);
    sceneRepresentation   scene(configFile);
    scene.initializeScene(x_cw);
    cout << " done." << endl << endl;
    // Start the stereo visual odometry algorithm
    while(true){
        // Point-Line Tracking
        algMutex.lock();
        clock.Tic();
        svo.readImages(imgL,imgR);
        bbMutex.unlock();
        float t1 = clock.Tac() * 1000.f;
        svo.detectLR();
        svo.stereoMatching();
        svo.f2fTracking();
        // Point-Line SVO
        svo.setInitialValue(x.inverse());
        svo.svoOptim(x);
        svo.getCovariance(cov);
        svo.getErrorNorm(plerrNorm);
        svo.getMatches(pMatches,pMatchesH,lMatches,lMatchesH);
        if(!is_finite(x)){
            x   = Matrix4f::Identity();
            cov = MatrixXf::Zero(6,6);
        }
        float t2 = clock.Tac() * 1000.f;
        // Console output
        cout.setf(ios::fixed,ios::floatfield); cout.precision(8);
        cout << "Frame: " << iter << " \t Residual error: " << plerrNorm;
                cout.setf(ios::fixed,ios::floatfield); cout.precision(3);
        cout << " \t Proc. time: " << t2-t1 << " ms\t ";
        cout << "\t Points: " << pMatches << " (" << pMatchesH << ") \t Lines: " << lMatches << " (" << lMatchesH << ")" << endl;
        // Update the scene
        scene.setText(iter,t2-t1,pMatches,pMatchesH,lMatches,lMatchesH);
        scene.setCov(cov);
        scene.setPose(x);
        scene.setPoints(svo.getPOptimData());
        scene.setLines(svo.getLOptimData());
        scene.setImage( svo.imageInliersWeights(x.inverse()) );
        if(scene.updateScene())
            svo.initialize(imgL, imgR);
        svo.updateTracking();
        iter++;
    }
}

int main(int argc, char **argv)
{

    algMutex.lock();
    bbMutex.lock();

    thread bb(bumblebeeThread);
    thread alg(stereoVO);
    bb.join();
    alg.join();

    return 0;

}


