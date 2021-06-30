详细介绍如何实现ORB-SLAM2与OpenMVS的对接过程，打通上下游实现三位重建，二话不说先干图
<div align=center><img src="https://img-blog.csdnimg.cn/20210630143615963.gif" width="40%"/>

对于学SLAM的同学对ORB-SLAM2可能并不陌生，系统框架清晰明了，代码风格清新脱俗，几乎都快成为入门SLAM的必修课了。三维重建结构分为SFM和MVS这两个部分，常见的三维重建方法如图所示

<div align = center><img src="https://img-blog.csdnimg.cn/2021063015112246.png"/> <img src="https://img-blog.csdnimg.cn/20210630151351157.png"/> 

对于学过SLAM的同学看到这可能就觉得不舒服了，对于SLAM和三位重建这俩同宗师兄弟竟然都不给介绍介绍，这那说的过去，确实对与SLAM，我们所理解的实时定位与建图其实他同样可以充当Sturt form Motion的角色，为MVS提供初始的相机位姿与稀疏点云。

本文以ORB-SLAM2与OpenMVS为例，详细介绍对接过程：
**ORB-SLAM2位姿导出**
为与OpenMVS进行对接本次进对ORB-SLAM2进行部分修改，使之可以为OpenMVS提供稀疏点云、关键帧的位姿、内参，以及稀疏点云在各个View中的可见性。

主要更改如下

 - 在Map文件下增添如下函数

```cpp
public:    
    void Save(const string &filename,const cv::MatSize image_size);
    void SaveMapPoint(ofstream &f, MapPoint* mp);
    void SaveKeyFrame(ofstream &f, KeyFrame* kf);
protected:
std::vector<int> KeyId;
```
- 在System下增加：

```cpp
void System::SaveMap(const string &filename,const cv::MatSize image_size);
```
- 在mono_tum.cc或者orbslam的其他Examples中对System::SaveMap(const string &filename,const cv::MatSize image_size)这个函数进行调用即可。

```cpp
SLAM.SaveMap("../Examples/output/sfm.txt",im.size);
```

通过ORB-SLAM2 我们最终生成了sfm.txt这个保存相机位姿以及稀疏点云的文档，他的文件结构如下图所示




数据集地址： https://vision.in.tum.de/data/datasets/rgbd-dataset/download#freiburg1_plant
以下是我们运行数据集的效果
<div align=center><img src="https://img-blog.csdnimg.cn/20210630153553571.png"/>

sfm.txt的文件内容为：
<div align=center><img src="https://img-blog.csdnimg.cn/20210630153946500.png"/> <img src="https://img-blog.csdnimg.cn/20210630153925356.png"/>

**ORB-SLAM2导出位姿验证**

在与OpenMVS 进行对接之前，一定确保自己导出的信息是准确的，可以将相机三位空间坐标点以及相机在空间中的位置保存成ply、obj等三维格式，在meshlab中进行查看，或者如果你用的rgb-d相机的话，同样可以将深度图、rgb图一同投影下来，在meshlab下进行查看

<div align=center>
<img src="https://img-blog.csdnimg.cn/20210630154443598.png"  width="47%"/>
<img src="https://img-blog.csdnimg.cn/2021063015450612.png" width="40%"/>

下面则是我们根据得到的数据开始对OpenMVS进行初始化
**OpenMVS初始化**
为与SLAM进行对接，我们加入了read_pose.cpp、read_pose.h这两个c++文件，目的是对SLAM导出的位姿和稀疏点云进行读取，并对OpenMVS进行初始化。
主要核心函数有
```cpp
bool load_scene(string file,Scene &scene);
bool read_mvs_pose(string file,MVSPOSE &mvs_pose);
bool save_pointcloud_obj(string name, vector<POINT3F> points,int num_keyframes,RGB color)
```
我们只需要在DensifyPointCloud.cpp下 对加入函数如下
```cpp
// load and estimate a dense point-cloud
#define use_custom_pose
#ifdef use_custom_pose
    if(!load_scene(string(MAKE_PATH_SAFE(OPT::strInputFileName)),scene))
		return EXIT_FAILURE;
#else
	if (!scene.Load(MAKE_PATH_SAFE(OPT::strInputFileName)))
		return EXIT_FAILURE;
#endif
```
即可实现我们的初始化操作
**对接结果**
一下分别为稠密重建后的深度图融合、mesh重构、mesh优化以及问题贴图效果
<div align=center>
<img src="https://img-blog.csdnimg.cn/20210630160616501.png"  width="40%"/>
<img src="https://img-blog.csdnimg.cn/20210630160655543.png" width="46%"/>
<img src="https://img-blog.csdnimg.cn/20210630160703299.png"  width="44%"/>
<img src="https://img-blog.csdnimg.cn/20210630160715576.png" width="43%"/>

**InterfaceDensifyPointCloud接口理解**
Scene.image中包含 尺度 分辨率 name 以及相关连的相机(这个platforms里面放置的是相机的内参和位姿)
在platforms中包含相机的内参 位姿
Camera类中包含相机ID 分辨率 相机内参，通过Read函数对stream进行读取自身参数
Image类这里包含图片ID，外参，相关连的相机，图片name以及稀疏点在当前图片的投影点
```cpp
IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME
```
Point类中 点的ID，位置，颜色，以及当前点的可见图像id
首先明确一点，我们SLAM的内容要如实的传入到**Scene**这个类中
<img src="https://img-blog.csdnimg.cn/20210630160038451.png"  />
首先将数据传入imgaes中
<img src="https://img-blog.csdnimg.cn/20210630160134593.png"  />
然后将数据传入platforms中
<img src="https://img-blog.csdnimg.cn/20210630160154828.png"  />
以及poses
<img src="https://img-blog.csdnimg.cn/20210630160208450.png"  />

