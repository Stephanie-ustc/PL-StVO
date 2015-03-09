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

#include "testSVO.h"

int main(int argc, char **argv){

    // Define Variables
    float                   plerrNorm;
    unsigned int            pMatches, pMatchesH, lMatches, lMatchesH;
    Matrix4f                x_0 = Matrix4f::Identity(), x = Matrix4f::Identity();
    MatrixXf                cov = MatrixXf::Zero(6,6);

    // Define SVO object
    plSVO                 svo;

    // Initialize the images and the stereo camera parameters
    Mat                     imgL, imgR;
    Matrix3f                K = Matrix3f::Identity();
    float                   baseline = 0.f;
    // ********************* PUT YOUR CODE HERE ********************* //
    // Read the images (load them into imgL and imgR)
    // Read the stereo camera parameters (load them into K and baseline)
    // ************************************************************** //

    // Initialize SVO algorithm
    cout << endl << "Initializing..." ;
    svo.setStereo(K,baseline);
    svo.initialize(imgL,imgR);
    svo.setInitialValue(x_0);
    cout << " done." << endl << endl;

    // Start the SVO algorithm
    while(true){

        // ************* PUT YOUR CODE HERE *********** //
        // Read the images (load them into imgL and imgR)
        // ******************************************** //

        // SVO
        x = svo.SVO(imgL,imgR,x.inverse());
        svo.getCovariance(cov);
        svo.getErrorNorm(plerrNorm);
        svo.getMatches(pMatches, pMatchesH, lMatches, lMatchesH);

        // ************* PUT YOUR CODE HERE *********** //
        // Process the output
        // ******************************************** //

    }

    return 0;

}


