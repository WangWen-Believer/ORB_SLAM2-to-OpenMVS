#include "read_pose.h"
#include <OpenMVS/MVS/Common.h>
using namespace MVS;
struct POINT3F
{
	float x;
	float y;
	float z;
	POINT3F(float x0, float y0, float z0) :x(x0), y(y0), z(z0){}
	POINT3F(){}
};
struct RGB
{
    int r;
    int g;
    int b;

    RGB(int r0, int g0,int b0) :r(r0), g(g0),b(b0){}
    RGB(){}
};

struct POINTCLOUD
{
	vector<POINT3F> points;
	vector<RGB> colors;
};
struct POSE
{
	float K[9];// camere intri
	float rot[9];// rotation (world to camera)
	float trans[3];// translation (world to camera)
};
struct MVSPOSE
{
	vector<POSE> poses;
	vector<POINT3F> spare_points;
	vector<vector<int>> views;   //针对稀疏点的共视views
	vector<string> images_name;
	int width;// image size
	int height;// image size
};

//save pointcloud
bool save_pointcloud_obj(string name, vector<POINT3F> points,int num_keyframes,RGB color){
    FILE* fpt=fopen(name.c_str(),"w");
    //判断保存路径是否有效
    if(!fpt)
    {
        printf("Error:%s path is not exis!!!", name.c_str());
        return false;
    }
    for(int i=0; i<points.size(); i++)
    {
        //to do use fprintf()
        // save points
        if(i<num_keyframes)
            fprintf(fpt,"v %f %f %f %d %d %d\n",points[i].x,points[i].y,points[i].z,color.r,color.g,color.b);
        else
            fprintf(fpt,"v %f %f %f %d %d %d\n",points[i].x,points[i].y,points[i].z,color.g,color.r,color.b);
    }
    fclose(fpt);
    return true;
}

bool read_mvs_pose(string file,MVSPOSE &mvs_pose)
{
	std::ifstream fin;
	fin.open(file, std::ios::in);
	if (fin.bad())
	{
		cout<<"can't open pose file"<<endl;
		return false;
	}
	//vector<string>imageLists;
	//vector<Cali> camerapara;
	int num_images = 0;
	char bufLine[1024];

	fin.getline(bufLine, sizeof(bufLine));
    istringstream isbufLine(bufLine);
	string s;
	int width;
	int height;
	isbufLine >> s>>width>>height;
	mvs_pose.height=height;
	mvs_pose.width=width;
//    std::cout << "width " << width<< std::endl;
	fin.getline(bufLine, sizeof(bufLine));
    istringstream isbuf_size(bufLine);
    isbuf_size >> num_images;
//    std::cout << "num_images " << num_images<< std::endl;
	//numimage = std::stoi(tmp);
    vector<POINT3F> points;
	for (int i = 0; i < num_images; i++)
	{
		fin.getline(bufLine, sizeof(bufLine));
        istringstream isbuf_image(bufLine);
		int id;
		string imag;
		double qw, qx, qy, qz, tx, ty, tz,fx,fy,cx,cy;
        isbuf_image >> id >> fx >>fy >>cx >>cy >> qw >> qx >> qy >> qz >> tx >> ty >> tz >>imag;
        mvs_pose.images_name.push_back(imag);
		POSE pose;
		pose.K[0]=fx;
		pose.K[2]=cx;
		pose.K[4]=fy;
		pose.K[5]=cy;
		pose.trans[0]=tx;
		pose.trans[1]=ty;
		pose.trans[2]=tz;
		// 四元数转旋转矩阵
		Eigen::Quaterniond q(qw,qx,qy,qz);

		Eigen::Matrix3d R = q.normalized().toRotationMatrix();
        for(int row=0;row<3;row++)
		{
			for(int col=0;col<3;col++)
			{
				pose.rot[3*row+col]=R(row,col);
			}
		}
        Eigen::Vector3d t(pose.trans[0],pose.trans[1],pose.trans[2]);
        Eigen::Vector3d Ow = -R.inverse()* t;

        POINT3F  Q(Ow[0],Ow[1],Ow[2]);
        points.push_back(Q);
		mvs_pose.poses.push_back(pose);
	}
	RGB color(0,255,0);
//    save_pointcloud_obj("/home/wangwen/Desktop/三维重建/MVS/data/track_platform.obj", points,mvs_pose.poses.size(),color);
    save_pointcloud_obj(string(WORKING_FOLDER) + "/track_platform.obj", points,mvs_pose.poses.size(),color);
	// 读稀疏点
	fin.getline(bufLine, sizeof(bufLine));
    istringstream isbuf_npoint(bufLine);
	int num_pts;
    isbuf_npoint >> num_pts;
	mvs_pose.spare_points.resize(num_pts);
	mvs_pose.views.resize(num_pts);
//    std::cout << "num_pts" <<num_pts << std::endl;
	for (int i = 0; i < num_pts; i++)
	{
		fin.getline(bufLine, sizeof(bufLine));
        istringstream isbuf_points(bufLine);
		double x, y, z;
		int num_keys;

        isbuf_points >> x >> y >> z >> num_keys;
		mvs_pose.spare_points[i].x = x;
		mvs_pose.spare_points[i].y = y;
		mvs_pose.spare_points[i].z = z;

		/*features->vvImagePoints[i].resize(num_keys);
		features->vvFrameIndex[i].resize(num_keys);
		features->vvKeyIndex[i].resize(num_keys);*/
//		mvs_pose.views[i].reserve(num_keys);
//		mvs_pose.views[i].reserve(num_keys);
		mvs_pose.views[i].reserve(num_keys);

		for (int j = 0; j < num_keys; j++)
		{
			int img_idx;
            isbuf_points >> img_idx ;
			mvs_pose.views[i].push_back(img_idx);	
		}
        POINT3F  Q(x,y,z);
        points.push_back(Q);
		
	}
    save_pointcloud_obj(string(WORKING_FOLDER) + "/track.obj", points,mvs_pose.poses.size(),color);
	return true;
}
bool load_scene(string file,Scene &scene)
{
    std::cout << "We are using Keyframe from SLAM to achieve 3D reconstruction" << std::endl;
    MVSPOSE mvs_pose;
	if(!read_mvs_pose(file,mvs_pose))
	{
		return false;
	}
    std::cout << "load mvs ok "  << std::endl;
    int numViews =mvs_pose.poses.size();
	scene.platforms.Reserve(numViews);
	scene.images.Reserve(numViews);
	scene.nCalibratedImages = 0;
    cout << "numViews " << mvs_pose.poses.size() << endl;
    vector<POINT3F> points;
	for (int count = 0; count < numViews; count++)
	{
		int idx = count; //true idx
		MVS::Image& image = scene.images.AddEmpty();

        string name=string(WORKING_FOLDER)+"images/"+mvs_pose.images_name[idx]+".png";
        image.name=name;
        image.platformID = scene.platforms.GetSize();
        image.cameraID = 0;
        image.ID = idx;
		image.scale = 1.0;
		image.width = mvs_pose.width;
		image.height = mvs_pose.height;

		MVS::Platform& platform = scene.platforms.AddEmpty();
		MVS::Platform::Camera& camera = platform.cameras.AddEmpty();

		camera.K = MVS::Platform::Camera::ComposeK<REAL, REAL>(mvs_pose.poses[idx].K[0], mvs_pose.poses[idx].K[4] , 2.0*mvs_pose.poses[idx].K[2] , 2.0*mvs_pose.poses[idx].K[5]);
		camera.R = RMatrix::IDENTITY;
		camera.C = CMatrix::ZERO;
//        cout << "camera.K" << camera.K << endl;
		// normalize camera intrinsics
		const REAL fScale(REAL(1) / MVS::Camera::GetNormalizationScale(image.width, image.height));
		camera.K(0, 0) *= fScale;
		camera.K(1, 1) *= fScale;
		camera.K(0, 2) *= fScale;
		camera.K(1, 2) *= fScale;
		// set pose
		image.poseID = platform.poses.GetSize();
		MVS::Platform::Pose& pose = platform.poses.AddEmpty();

		for (int n = 0; n < 9; n++)
		{
			pose.R.val[n] = mvs_pose.poses[idx].rot[n];
		}

		for (int j = 0; j < 3; ++j)
		{
			pose.C.ptr()[j] = -float(double(mvs_pose.poses[idx].rot[j])*double(mvs_pose.poses[idx].trans[0]) + double(mvs_pose.poses[idx].rot[3 + j])*double(mvs_pose.poses[idx].trans[1]) + double(mvs_pose.poses[idx].rot[6 + j])*double(mvs_pose.poses[idx].trans[2]));
		}

//		cout <<"image.poseID " << image.poseID <<"image.platformID"<<image.platformID<<endl;
//		cout << "camera.R\n"<<camera.R << "camera.C" << camera.C << endl;

        POINT3F point_tmp(pose.C.x,pose.C.y,pose.C.z);
		points.push_back(point_tmp);

		image.UpdateCamera(scene.platforms);

		++scene.nCalibratedImages;

//		cout <<"nCalibratedImages" << scene.nCalibratedImages << endl;
	}
    std::cout << "deal with feature points" << std::endl;
	//deal with feature points
	scene.pointcloud.points.Reserve(mvs_pose.spare_points.size());
	scene.pointcloud.pointViews.Resize(mvs_pose.spare_points.size());
	for (size_t idx = 0; idx<mvs_pose.spare_points.size(); ++idx) 
	{
		POINT3F X = mvs_pose.spare_points[idx];
		scene.pointcloud.points.AddConstruct(X.x, X.y, X.z);
		MVS::PointCloud::ViewArr& views = scene.pointcloud.pointViews[idx];
		int numview = mvs_pose.views[idx].size();
		for (int viewId = 0; viewId<numview; viewId++)
		{
			views.InsertSort(mvs_pose.views[idx][viewId]);
		}
	}
//    RGB color(255,0,0);
//    save_pointcloud_obj(file + "/track_1.obj", points,mvs_pose.poses.size(),color);
	return true;
}
//从关联文件中提取这些需要加载的图像的路径和时间戳
void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
    //输入文件流
    ifstream fAssociation;
    //打开关联文件
    fAssociation.open(strAssociationFilename.c_str());
    //一直读取,知道文件结束
    while(!fAssociation.eof())
    {
        string s;
        //读取一行的内容到字符串s中
        getline(fAssociation,s);
        //如果不是空行就可以分析数据了
        if(!s.empty())
        {
            //字符串流
            stringstream ss;
            ss << s;
            //字符串格式:  时间戳 rgb图像路径 时间戳 深度图像路径
            double t;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);

        }
    }
}
// }
// // 将彩色点云保存为obj格式
// bool save_pointcloud_obj(string name, POINTCLOUD pointcloud)
// {
// 	FILE* fpt = fopen(name.c_str(), "w");
// 	// make sure file is ok
// 	if (!fpt)
// 	{
// 		printf("Error: %s path is not exist!!!", name);
// 		return false;
// 	}
// 	if(pointcloud.points.size()!=pointcloud.colors.size())
// 	{
// 		printf("Error: colors not equal to points!!!", name);
// 		return false;
// 	}
// 	for (int i = 0; i < pointcloud.points.size(); i++)
// 	{
// 		// to do,use fprintf()
// 		POINT3F temp = pointcloud.points[i];
// 		RGB rgb = pointcloud.colors[i];
		
// 	}
// 	fclose(fpt);
// 	return true;
// }
