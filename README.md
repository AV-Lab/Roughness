# Roughness


This project is a part of the perception engine responsible for scoring the roughness of the in-front heading drivable surface. A stereo camera sensor is used to obtain a 3D depth map of the captured scene. Then, the RANSAC algorithm is employed on a 2D segmented drivable area, [obtained from our semantic segmentation model](https://github.com/AV-Lab/RoadSceneUnderstanding-ModifiedUNet), with the 3D depth information to obtain 3D drivable plane. Eventually, the plane is divided into small grids with a roughness score, where the roughness score of a grid is the mean of Ecludiean distances between grid points and the estimated plane by RANSAC. 

![Roughness Procedures](https://user-images.githubusercontent.com/20774864/173551758-108f5b7e-b98e-4e2e-8ee7-0d09989d350a.png)


