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

class bumblebeeGrabber{

public:

    bumblebeeGrabber();
    bumblebeeGrabber(string configFile);
    ~bumblebeeGrabber();
    void grabStereo(Mat &imgLeft, Mat &imgRight);
    void getCalib(Matrix3f &K, float &baseline);
    IplImage *grabIplImage();

private:

    float f, cx, cy;
    Matrix3f K;

    TCaptureOptions_FlyCapture2 bbOptions;
    CImageGrabber_FlyCapture2*  bb;
    CObservationStereoImages    stereoObservation;
    CImage                      imgLeft_, imgRight_;

};

