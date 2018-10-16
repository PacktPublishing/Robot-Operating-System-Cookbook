0.2.5 (2016-04-23)
------------------
* fixes for Kinetic
* do not compile RGBD for OpenCV3 (as it is in there)
* proper error handling in OpenCV3
  fixes `#5 <https://github.com/wg-perception/opencv_candidate/issues/5>`_
* Contributors: Vincent Rabaud

0.2.4 (2015-03-29)
------------------
* get features2d to compile with OpenCV3
* do not compile RGBD for OpenCV3 as there should be an opencv_contrib module anyway
* remove Python bindings of data matrix
* remove LSH as it is now upstream
* get datamatrix to compile with OpenCV3
* Updating namespaces
* Contributors: Vincent Rabaud, edgarriba

0.2.3 (2014-07-20)
------------------
* no need for CMake hacks (plus, cholmod is not needed anymore)
* Contributors: Vincent Rabaud

0.2.2 (2014-07-19)
------------------
* do not build reconst3d as it is not used anywhere
  It's also a way to remove a package that depends on g2o for less maintenance
* update url and maintainer's email address
* add basic docs
* compile under trusty
* Contributors: Vincent Rabaud

0.2.1 (2014-04-13)
------------------
* get distro agnostic OpenCv2 dependency
* Contributors: Vincent Rabaud

0.2.0 (2014-04-13)
------------------
* disable tests as the Ubuntu version of OpenCV has a bug with TS for now (cvconfig.h not present)
* drop Fuerte support
* Contributors: Vincent Rabaud

0.1.9 (2013-08-28 20:14:21 -0800)
----------------------------------
- fix dependencies with PCL
