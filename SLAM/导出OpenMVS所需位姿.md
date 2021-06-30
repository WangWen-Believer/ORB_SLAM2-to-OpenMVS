主要更改如下

- 在Map文件下增添如下函数

```c++
public:    
    void Save(const string &filename,const cv::MatSize image_size);
    void SaveMapPoint(ofstream &f, MapPoint* mp);
    void SaveKeyFrame(ofstream &f, KeyFrame* kf);
protected:
std::vector<int> KeyId;
```

- 在System下增加：

```c++
void System::SaveMap(const string &filename,const cv::MatSize image_size);
```

- 在mono_tum.cc或者orbslam的其他Examples中对 **System::SaveMap(const string &filename,const cv::MatSize image_size)**这个函数进行调用即可。

```c++
SLAM.SaveMap("../Examples/output/sfm.txt",im.size);
```

